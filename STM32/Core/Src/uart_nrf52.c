#include <stdbool.h>
#include <string.h>
#include "uart_nrf52.h"
#include "debug_printf.h"
#include "cmsis_os.h"
#include "device_config.h"
#include "bugpack_data_manager.h"
#include "gps_driver.h"

extern UART_HandleTypeDef huart2;

volatile bool tx_lock = false;
volatile uint64_t tx_lock_time = 0;
uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
uint8_t tx_queue_buffer[UART_TX_BUFFER_SIZE*TX_QUEUE_BUFFER_ITEM_COUNT];
uint8_t tx_queue_item_count = 0;
uint32_t tx_queue_buffer_head = 0;
uint32_t tx_queue_buffer_tail = 0;
uint8_t packet_id_counter = 1;

uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t rx_buffer_index = 0;
uint8_t preamble_rx_buffer[4];
uint8_t preamble_rx_buffer_index = 0;
enum uart_state_t uart_rx_state = WAITING_FOR_PREAMBLE;
uint8_t remaining_data_length = 0;
uint8_t remaining_crc_length = 0;
volatile bool uart_rx_processing_ready = false;
extern DeviceConfig device_config;

uint8_t get_next_packet_id() {
    packet_id_counter++;
    if(packet_id_counter == 0) packet_id_counter = 1;
    return packet_id_counter;
}

void append_to_tx_queue(uart_packet_t packet) {
    if(tx_queue_item_count >= TX_QUEUE_BUFFER_ITEM_COUNT) {
        debugPrint("TX queue buffer overflow, ignoring the packet.\n");
        return;
    }
    tx_queue_buffer[tx_queue_buffer_head] = packet.command_type;
    tx_queue_buffer[tx_queue_buffer_head+1] = packet.packet_id;
    tx_queue_buffer[tx_queue_buffer_head+2] = packet.data_length;
    memcpy(&tx_queue_buffer[tx_queue_buffer_head+3], packet.data, packet.data_length);
    tx_queue_buffer_head += TX_QUEUE_BUFFER_STEP;
    tx_queue_item_count++;
    if(tx_queue_buffer_head >= sizeof(tx_queue_buffer)) tx_queue_buffer_head = 0; // Wrap around if at the end
}

uart_packet_t pop_from_tx_queue() {
    uart_packet_t packet = {
            .command_type = tx_queue_buffer[tx_queue_buffer_tail],
            .packet_id = tx_queue_buffer[tx_queue_buffer_tail+1],
            .data_length = tx_queue_buffer[tx_queue_buffer_tail+2],
            .data = &tx_queue_buffer[tx_queue_buffer_tail+3]
    };
    tx_queue_buffer_tail += TX_QUEUE_BUFFER_STEP;
    if(tx_queue_buffer_tail >= sizeof(tx_queue_buffer)) tx_queue_buffer_tail = 0; // Wrap around if at the end
    tx_queue_item_count--;
    return packet;
}


void send_uart_packet(uart_packet_t packet) {
    // start the packet with a preamble
    for(int i = 0; i < 4; i++) {
        tx_buffer[i] = 0x02;
    }
    // Copy the packet to the tx buffer
    tx_buffer[4] = packet.command_type;
    tx_buffer[5] = packet.packet_id;
    tx_buffer[6] = packet.data_length;

    if(packet.data_length > 0) memcpy(&tx_buffer[7], packet.data, packet.data_length);

    uint32_t crc = calculate_crc32(&tx_buffer[4], 3 + packet.data_length);
    memcpy(&tx_buffer[4 + 3 + packet.data_length], &crc, sizeof(crc));

    // Enable the tx_lock to prevent other sends
    tx_lock = true;
    tx_lock_time = HAL_GetTick();

    // Send the packet over UART with interrupt
    //LOG_PRINT(LOG_LEVEL_DBG, "Sending packet over UART...\n");
    uint8_t tx_send_length = 4 + 3 + packet.data_length + 5; // 4 preamble, 3 header, data, 4 crc
    HAL_UART_Transmit_IT(&huart2, tx_buffer, tx_send_length);
}

void process_uart_tx() {
    if(tx_lock && HAL_GetTick() - tx_lock_time > 8000) {
        //LOG_PRINT(LOG_LEVEL_DBG, "TX lock timeout, resetting the lock.\n");
        tx_lock = false;
    }
    if(tx_lock) return;
    if(tx_queue_item_count == 0) return;
    uart_packet_t packet = pop_from_tx_queue();
    //LOG_PRINT(LOG_LEVEL_DBG, "Sending packet from queue...\n");
    send_uart_packet(packet);
}

void init_nrf_uart_comm() {
    uart_rx_state = WAITING_FOR_PREAMBLE;
    clear_rx();
}

void clear_rx() {
    rx_buffer_index = 0;
    memset(rx_buffer, 0, UART_RX_BUFFER_SIZE);
    memset(preamble_rx_buffer, 0, 4);
    HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}

bool detect_preamble() {
    for(int i = 0; i < 4; i++) {
        if(preamble_rx_buffer[i] != 0x02) {
            return false;
        }
    }
    return true;
}

void on_nrf_uart_receive() {
    //LOG_PRINT(LOG_LEVEL_DBG, "R: %d\n", rx_buffer[rx_buffer_index]);

    // Copy the received byte to the preamble buffer
    preamble_rx_buffer[preamble_rx_buffer_index] = rx_buffer[rx_buffer_index];
    preamble_rx_buffer_index++;
    if (preamble_rx_buffer_index >= sizeof(preamble_rx_buffer)) preamble_rx_buffer_index = 0;
    //Verify the preamble buffer does not signal new packet
    if(detect_preamble()) {
        // New packet is being received
        uart_rx_state = WAITING_FOR_COMMAND;
        debugPrint("Preamble detected.\n");
        clear_rx(); // Clear the rx buffer, the preamble is not needed anymore
        return;
    }

    switch (uart_rx_state) {
        case WAITING_FOR_PREAMBLE:
            // Preamble is not detected yet, wait for it
            HAL_UART_Receive_IT(&huart2, (uint8_t *) &rx_buffer[rx_buffer_index], 1);
            return;
        case WAITING_FOR_COMMAND:
            // Verify that the command type is valid, else we can throw away the packet as it may be corrupted
            if(rx_buffer[rx_buffer_index] == 0 || rx_buffer[rx_buffer_index] >= CONF_MAX_VALUE) {
                //LOG_PRINT(LOG_LEVEL_DBG, "Received invalid command type: %d\n", rx_buffer[rx_buffer_index]);
                clear_rx();
                uart_rx_state = WAITING_FOR_PREAMBLE;
                break;
            }
            //LOG_PRINT(LOG_LEVEL_DBG, "Received command type: %d\n", rx_buffer[rx_buffer_index]);
            uart_rx_state = WAITING_FOR_ID;
            break;
        case WAITING_FOR_ID:
            //LOG_PRINT(LOG_LEVEL_DBG, "Received packet ID: %d\n", rx_buffer[rx_buffer_index]);
            uart_rx_state = WAITING_FOR_DATA_LENGTH;
            break;
        case WAITING_FOR_DATA_LENGTH:
            //LOG_PRINT(LOG_LEVEL_DBG, "Received data length: %d\n", rx_buffer[rx_buffer_index]);
            remaining_data_length = rx_buffer[rx_buffer_index];
            if(remaining_data_length > 0) uart_rx_state = WAITING_FOR_DATA;
            else uart_rx_state = WAITING_FOR_CRC;
            remaining_crc_length = 4;
            break;
        case WAITING_FOR_DATA:
            remaining_data_length--;
            if(remaining_data_length == 0) {
                uart_rx_state = WAITING_FOR_CRC;
            }
            break;
        case WAITING_FOR_CRC:
            remaining_crc_length--;
            if(remaining_crc_length == 0) { // Packet has been received, mark it as ready to
                // verify the CRC, send ACK and process the packet
                uart_rx_processing_ready = true;
            }
    }

    rx_buffer_index++;
    if(rx_buffer_index > sizeof(rx_buffer)-1) {
        uart_rx_state = WAITING_FOR_PREAMBLE;
        clear_rx();
        //LOG_PRINT(LOG_LEVEL_DBG, "UART RX buffer overflow, clearing the buffer.\n");
        return;
    }
    HAL_UART_Receive_IT(&huart2, (uint8_t *) &rx_buffer[rx_buffer_index], 1);
}

void process_uart_rx() { // Should be run in uart task loop
    if(!uart_rx_processing_ready) return;

    uart_packet_t packet = { // Move it to a structure to make it easier to work with
            .command_type = rx_buffer[0],
            .packet_id = rx_buffer[1],
            .data_length = rx_buffer[2],
            .data = &rx_buffer[3]
    };

    // Verify the CRC
    uint32_t crc = calculate_crc32(rx_buffer, 3 + packet.data_length);
    uint32_t received_crc;
    memcpy(&received_crc, &rx_buffer[3 + packet.data_length], sizeof(received_crc));
    if(crc != received_crc) {
        debugPrint("Received packet with invalid CRC, ignoring...\n");
        debugPrintf("Received CRC: %lu, Calculated CRC: %lu\n", received_crc, crc);
        clear_rx();
        uart_rx_processing_ready = false;
        uart_rx_state = WAITING_FOR_PREAMBLE;
        return;
    }

    switch (packet.command_type) {
        case SET_LORA_FREQ:
            if(packet.data_length != 4) {
                debugPrint("Invalid data length for SET_LORA_FREQ command.\n");
                break;
            }
            uint32_t frequency;
            memcpy(&frequency, packet.data, 4);
            if(set_lora_frequency(frequency)) {
                debugPrintf("LoRa frequency set to: %d\n", frequency);
            } else {
                debugPrintf("Invalid LoRa frequency: %d\n", frequency);
            }
            uart_send_lora_frequency();
            break;
        case GET_LORA_FREQ:
            debugPrint("GET_LORA_FREQ command received.\n");
            uart_send_lora_frequency();
            break;
        case SET_LORA_BW:
            if(packet.data_length != 4) {
                debugPrint("Invalid data length for SET_LORA_BW command.\n");
                break;
            }
            uint32_t bandwidth;
            memcpy(&bandwidth, packet.data, 4);
            if(set_lora_bandwidth(bandwidth)) {
                debugPrintf("LoRa bandwidth set to: %d\n", bandwidth);
            } else {
                debugPrintf("Invalid LoRa bandwidth: %d\n", bandwidth);
            }
            uart_send_lora_bandwidth();
            break;
        case GET_LORA_BW:
            debugPrint("GET_LORA_BW command received.\n");
            uart_send_lora_bandwidth();
            break;
        case SET_LORA_SYNC_WORD:
            if(packet.data_length != 2) {
                debugPrint("Invalid data length for SET_LORA_SYNC_WORD command.\n");
                break;
            }
            uint16_t sync_word;
            memcpy(&sync_word, packet.data, 2);
            if(set_lora_sync_word(sync_word)) {
                debugPrintf("LoRa sync word set to: %d\n", sync_word);
            } else {
                debugPrintf("Invalid LoRa sync word: %d\n", sync_word);
            }
            uart_send_lora_sync_word();
            break;
        case GET_LORA_SYNC_WORD:
            debugPrint("GET_LORA_SYNC_WORD command received.\n");
            uart_send_lora_sync_word();
            break;
        case SET_LORA_SPREADING_FACTOR:
            if(packet.data_length != 1) {
                debugPrint("Invalid data length for SET_LORA_SPREADING_FACTOR command.\n");
                break;
            }
            uint8_t spreading_factor;
            memcpy(&spreading_factor, packet.data, 1);
            if(set_lora_spreading_factor(spreading_factor)) {
                debugPrintf("LoRa spreading factor set to: %d\n", spreading_factor);
            } else {
                debugPrintf("Invalid LoRa spreading factor: %d\n", spreading_factor);
            }
            uart_send_lora_spreading_factor();
            break;
        case GET_LORA_SPREADING_FACTOR:
            debugPrint("GET_LORA_SPREADING_FACTOR command received.\n");
            uart_send_lora_spreading_factor();
            break;
        case SET_LORA_TX_POWER:
            if(packet.data_length != 1) {
                debugPrint("Invalid data length for SET_LORA_TX_POWER command.\n");
                break;
            }
            uint8_t tx_power;
            memcpy(&tx_power, packet.data, 1);
            if(set_lora_tx_power(tx_power)) {
                debugPrintf("LoRa TX power set to: %d\n", tx_power);
            } else {
                debugPrintf("Invalid LoRa TX power: %d\n", tx_power);
            }
            uart_send_lora_tx_power();
            break;
        case GET_LORA_TX_POWER:
            debugPrint("GET_LORA_TX_POWER command received.\n");
            uart_send_lora_tx_power();
            break;
        case STM_REBOOT:
            debugPrint("STM32 reboot command received.\n");
            osDelay(15);
            HAL_NVIC_SystemReset();
            break;
        case BUGPACK_DATA:
            debugPrint("BUGPACK_DATA command received.\n");
            if(packet.data_length != sizeof(BugPackData)) {
                debugPrint("Invalid data length for BUGPACK_DATA command.\n");
                break;
            }
            set_bugpack_data((BugPackData *) packet.data);
            break;
        case GET_BUGSAT_STATUS:
            debugPrint("GET_BUGSAT_STATUS command received.\n");
            uart_send_bugsat_status();
            break;
        default:
            debugPrintf("Unknown command type: %d\n", packet.command_type);
            break;
    }
    uart_rx_processing_ready = false;
    uart_rx_state = WAITING_FOR_PREAMBLE;
    clear_rx();
}

void uart_send_lora_frequency() {
    uint32_t frequency = device_config.lora_frequency;
    uart_packet_t packet = {
            .command_type = GET_LORA_FREQ,
            .packet_id = get_next_packet_id(),
            .data_length = 4,
            .data = (uint8_t *) &frequency
    };
    append_to_tx_queue(packet);
}

void uart_send_lora_bandwidth() {
    uint32_t bandwidth = device_config.lora_bandwidth;
    uart_packet_t packet = {
            .command_type = GET_LORA_BW,
            .packet_id = get_next_packet_id(),
            .data_length = 4,
            .data = (uint8_t *) &bandwidth
    };
    append_to_tx_queue(packet);
}

void uart_send_lora_sync_word() {
    uint16_t sync_word = device_config.lora_sync_word;
    uart_packet_t packet = {
            .command_type = GET_LORA_SYNC_WORD,
            .packet_id = get_next_packet_id(),
            .data_length = 2,
            .data = (uint8_t *) &sync_word
    };
    append_to_tx_queue(packet);
}

void uart_send_lora_spreading_factor() {
    uint8_t spreading_factor = device_config.lora_spreading_factor;
    uart_packet_t packet = {
            .command_type = GET_LORA_SPREADING_FACTOR,
            .packet_id = get_next_packet_id(),
            .data_length = 1,
            .data = &spreading_factor
    };
    append_to_tx_queue(packet);
}

void uart_send_lora_tx_power() {
    uint8_t tx_power = device_config.lora_tx_power;
    uart_packet_t packet = {
            .command_type = GET_LORA_TX_POWER,
            .packet_id = get_next_packet_id(),
            .data_length = 1,
            .data = &tx_power
    };
    append_to_tx_queue(packet);
}


extern GPSData gps_buffer;
extern bool rocket_launch_detected;
extern bool bugs_deployed;

void uart_send_bugsat_status() {
    uint8_t bugsat_status = 0;

    if (gps_buffer.altitude != 0) {
        bugsat_status |= 0x01;
    }
    if(rocket_launch_detected) {
        bugsat_status |= 0x02;
    }
    if(bugs_deployed) {
        bugsat_status |= 0x04;
    }

    uart_packet_t packet = {
            .command_type = GET_BUGSAT_STATUS,
            .packet_id = get_next_packet_id(),
            .data_length = 1,
            .data = &bugsat_status
    };
    append_to_tx_queue(packet);
}