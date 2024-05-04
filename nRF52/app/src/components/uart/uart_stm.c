#include "uart_stm.h"
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(uart_stm, LOG_LEVEL_DBG);

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

static uint8_t rx_buf[UART_BUFF_SIZE];
static uint8_t rx_processing_buf[UART_BUFF_SIZE];
uint8_t rx_buffer_index = 0;
static uint8_t tx_buf[UART_BUFF_SIZE];
//uint8_t tx_queue_buffer[UART_BUFF_SIZE*TX_QUEUE_BUFFER_ITEM_COUNT];
//uint8_t tx_queue_item_count = 0;
//uint32_t tx_queue_buffer_head = 0;
//uint32_t tx_queue_buffer_tail = 0;
static uint8_t pre_rx_buf[PRE_RX_BUFF_SIZE];
uint8_t preamble_rx_buffer[4];
uint8_t preamble_rx_buffer_index = 0;
enum uart_state_t uart_rx_state = WAITING_FOR_PREAMBLE;
uint8_t remaining_data_length = 0;
uint8_t remaining_crc_length = 0;
uint8_t packet_id_counter = 1;


extern uint32_t lora_frequency;
extern uint32_t lora_bandwidth;
extern uint16_t lora_sync_word;
extern uint8_t lora_spreading_factor;
extern uint8_t lora_tx_power;


static K_THREAD_STACK_DEFINE(uart_stack_area, 2048);
struct k_work uart_rx_work;
struct k_work_q uart_rx_work_queue;

void rx_offload_function(struct k_work *work)
{
    process_uart_rx();
}


uint32_t calculate_crc32(const uint8_t *data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFF;
    if(data == NULL || length == 0){
        return 0;
    }

    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];

        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            } else {
                crc = (crc >> 1);
            }
        }
    }

    return crc;
}

uint8_t get_next_packet_id() {
    packet_id_counter++;
    if(packet_id_counter == 0) packet_id_counter = 1;
    return packet_id_counter;
}

//void append_to_tx_queue(uart_packet_t packet) {
//    if(tx_queue_item_count >= TX_QUEUE_BUFFER_ITEM_COUNT) {
//        LOG_INF("TX queue buffer overflow, ignoring the packet.\n");
//        return;
//    }
//    tx_queue_buffer[tx_queue_buffer_head] = packet.command_type;
//    tx_queue_buffer[tx_queue_buffer_head+1] = packet.packet_id;
//    tx_queue_buffer[tx_queue_buffer_head+2] = packet.data_length;
//    memcpy(&tx_queue_buffer[tx_queue_buffer_head+3], packet.data, packet.data_length);
//    tx_queue_buffer_head += TX_QUEUE_BUFFER_STEP;
//    tx_queue_item_count++;
//    if(tx_queue_buffer_head >= sizeof(tx_queue_buffer)) tx_queue_buffer_head = 0; // Wrap around if at the end
//}

//uart_packet_t pop_from_tx_queue() {
//    uart_packet_t packet = {
//            .command_type = tx_queue_buffer[tx_queue_buffer_tail],
//            .packet_id = tx_queue_buffer[tx_queue_buffer_tail+1],
//            .data_length = tx_queue_buffer[tx_queue_buffer_tail+2],
//            .data = &tx_queue_buffer[tx_queue_buffer_tail+3]
//    };
//    tx_queue_buffer_tail += TX_QUEUE_BUFFER_STEP;
//    if(tx_queue_buffer_tail >= sizeof(tx_queue_buffer)) tx_queue_buffer_tail = 0; // Wrap around if at the end
//    tx_queue_item_count--;
//    return packet;
//}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
    switch (evt->type) {

        case UART_RX_RDY:
            for(uint8_t i = 0; i < PRE_RX_BUFF_SIZE; i++) {
                rx_buf[rx_buffer_index] = pre_rx_buf[i];
                on_stm_uart_receive();
            }
//            rx_buf[rx_buffer_index] = pre_rx_buf[0];
//            on_stm_uart_receive();
            break;
        case UART_RX_DISABLED:
            uart_rx_enable(dev, &pre_rx_buf, PRE_RX_BUFF_SIZE, RECEIVE_TIMEOUT);
            break;

        default:
            break;
    }
}

void uart_stm_init() {
    int err;
    if (!device_is_ready(uart)){
        LOG_INF("UART device not ready!");
        return;
    }
    clear_rx();
    err = uart_callback_set(uart, uart_cb, NULL);
    if (err) {
        LOG_INF("Failed to set UART callback");
        return;
    }
    err = uart_rx_enable(uart, &pre_rx_buf, PRE_RX_BUFF_SIZE, RECEIVE_TIMEOUT);
    if (err) {
        LOG_INF("Failed to enable UART RX");
        return;
    }

//    k_work_queue_start(&uart_rx_work_queue, uart_stack_area,
//                       K_THREAD_STACK_SIZEOF(uart_stack_area), 4,
//                       NULL);
//    k_work_init(&uart_rx_work, rx_offload_function);
//    LOG_INF("UART initialized");

    // get the current configuration from the STM
    uart_get_lora_frequency();
    k_sleep(K_MSEC(100));
    uart_get_lora_bandwidth();
    k_sleep(K_MSEC(100));
    uart_get_lora_sync_word();
    k_sleep(K_MSEC(100));
    uart_get_lora_spreading_factor();
    k_sleep(K_MSEC(100));
    uart_get_lora_tx_power();

    LOG_INF("Fetching current LoRa configuration from STM...");
}

void send_uart_packet(uart_packet_t packet) {
    // start the packet with a preamble
    for(int i = 0; i < 4; i++) {
        tx_buf[i] = 0x02;
    }
    // Copy the packet to the tx buffer
    tx_buf[4] = packet.command_type;
    tx_buf[5] = packet.packet_id;
    tx_buf[6] = packet.data_length;

    if(packet.data_length > 0) memcpy(&tx_buf[7], packet.data, packet.data_length);
    else {
        packet.data_length = 1;
        tx_buf[6] = 1;
        tx_buf[7] = 0;
    }

    uint32_t crc = calculate_crc32(&tx_buf[4], 3 + packet.data_length);
    memcpy(&tx_buf[4 + 3 + packet.data_length], &crc, sizeof(crc));

    // Send the packet over UART with interrupt
    //LOG_PRINT(LOG_LEVEL_DBG, "Sending packet over UART...\n");
    uint8_t tx_send_length = 4 + 3 + packet.data_length + 4; // 4 preamble, 3 header, data, 4 crc
//    HAL_UART_Transmit_IT(&huart2, tx_buffer, tx_send_length);
    uint8_t ret = uart_tx(uart, tx_buf, tx_send_length, SYS_FOREVER_MS);
    if (ret) {
        LOG_INF("Failed to send data over UART\n");
    }
}

bool detect_preamble() {
    for(int i = 0; i < 4; i++) {
        if(preamble_rx_buffer[i] != 0x02) {
            return false;
        }
    }
    return true;
}

void on_stm_uart_receive() {
    //LOG_PRINT(LOG_LEVEL_DBG, "R: %d\n", rx_buffer[rx_buffer_index]);

    // Copy the received byte to the preamble buffer
    preamble_rx_buffer[preamble_rx_buffer_index] = rx_buf[rx_buffer_index];
    preamble_rx_buffer_index++;
    if (preamble_rx_buffer_index >= sizeof(preamble_rx_buffer)) preamble_rx_buffer_index = 0;
    //Verify the preamble buffer does not signal new packet
    if(detect_preamble()) {
        // New packet is being received
        uart_rx_state = WAITING_FOR_COMMAND;
        clear_rx(); // Clear the rx buffer, the preamble is not needed anymore
        return;
    }

    switch (uart_rx_state) {
        case WAITING_FOR_PREAMBLE:
            // Preamble is not detected yet, wait for it
            return;
        case WAITING_FOR_COMMAND:
            // Verify that the command type is valid, else we can throw away the packet as it may be corrupted
            if(rx_buf[rx_buffer_index] == 0 || rx_buf[rx_buffer_index] >= CONF_MAX_VALUE) {
                //LOG_PRINT(LOG_LEVEL_DBG, "Received invalid command type: %d\n", rx_buffer[rx_buffer_index]);
                clear_rx();
                uart_rx_state = WAITING_FOR_PREAMBLE;
                break;
            }
            //LOG_PRINT(LOG_LEVEL_DBG, "Received command type: %d\n", rx_buffer[rx_buffer_index]);
//            printk("Command type: %d\n", rx_buf[rx_buffer_index]);
            uart_rx_state = WAITING_FOR_ID;
            break;
        case WAITING_FOR_ID:
            //LOG_PRINT(LOG_LEVEL_DBG, "Received packet ID: %d\n", rx_buffer[rx_buffer_index]);
            uart_rx_state = WAITING_FOR_DATA_LENGTH;
//            printk("Packet ID: %d\n", rx_buf[rx_buffer_index]);
            break;
        case WAITING_FOR_DATA_LENGTH:
            //LOG_PRINT(LOG_LEVEL_DBG, "Received data length: %d\n", rx_buffer[rx_buffer_index]);
            remaining_data_length = rx_buf[rx_buffer_index];
            if(remaining_data_length > 0) uart_rx_state = WAITING_FOR_DATA;
            else uart_rx_state = WAITING_FOR_CRC;
            remaining_crc_length = 4;
            break;
        case WAITING_FOR_DATA:
            remaining_data_length--;
            if(remaining_data_length <= 0) {
                uart_rx_state = WAITING_FOR_CRC;
            }
            break;
        case WAITING_FOR_CRC:
            remaining_crc_length--;
            if(remaining_crc_length == 0) { // Packet has been received, mark it as ready to
                // verify the CRC, send ACK and process the packet
                memcpy(rx_processing_buf, rx_buf, sizeof(rx_buf));
//                k_work_submit_to_queue(&uart_rx_work_queue, &uart_rx_work);
                process_uart_rx();
            }
    }

    rx_buffer_index++;
    if(rx_buffer_index > sizeof(rx_buf)-1) {
        printk("Buffer overflow\n");
        uart_rx_state = WAITING_FOR_PREAMBLE;
        clear_rx();
        //LOG_PRINT(LOG_LEVEL_DBG, "UART RX buffer overflow, clearing the buffer.\n");
        return;
    }
}

void clear_rx() {
    rx_buffer_index = 0;
    memset(rx_buf, 0, UART_BUFF_SIZE);
    memset(preamble_rx_buffer, 0, 4);
}

void process_uart_rx() { // Should be run in separate task, not from IRQ!
    LOG_INF("Processing packet!");
    uart_packet_t packet = { // Move it to a structure to make it easier to work with
            .command_type = rx_processing_buf[0],
            .packet_id = rx_processing_buf[1],
            .data_length = rx_processing_buf[2],
            .data = &rx_processing_buf[3]
    };

    // Verify the CRC
    uint32_t crc = calculate_crc32(rx_processing_buf, 3 + packet.data_length);
    uint32_t received_crc;
    memcpy(&received_crc, &rx_processing_buf[3 + packet.data_length], sizeof(received_crc));
    if(crc != received_crc) {
        LOG_INF("Received packet with invalid CRC, ignoring...\n");
        clear_rx();
        uart_rx_state = WAITING_FOR_PREAMBLE;
        return;
    }

    switch (packet.command_type) {
        case GET_LORA_FREQ:
            lora_frequency = *((uint32_t*) packet.data);
            LOG_INF("Received LoRa frequency: %d\n", lora_frequency);
            break;
        case GET_LORA_BW:
            lora_bandwidth = *((uint32_t*) packet.data);
            LOG_INF("Received LoRa bandwidth: %d\n", lora_bandwidth);
            break;
        case GET_LORA_SYNC_WORD:
            lora_sync_word = *((uint16_t*) packet.data);
            LOG_INF("Received LoRa sync word: %d\n", lora_sync_word);
            break;
        case GET_LORA_SPREADING_FACTOR:
            lora_spreading_factor = *packet.data;
            LOG_INF("Received LoRa spreading factor: %d\n", lora_spreading_factor);
            break;
        case GET_LORA_TX_POWER:
            lora_tx_power = *packet.data;
            LOG_INF("Received LoRa TX power: %d\n", lora_tx_power);
            break;
        default:
            LOG_INF("Unknown command type");
            break;
    }
    uart_rx_state = WAITING_FOR_PREAMBLE;
    clear_rx();
    LOG_INF("Packet processed!");
}

void uart_set_lora_frequency() {
    uart_packet_t packet = {
            .command_type = SET_LORA_FREQ,
            .packet_id = get_next_packet_id(),
            .data_length = sizeof(lora_frequency),
            .data = (uint8_t*) &lora_frequency
    };
    send_uart_packet(packet);
}

void uart_set_lora_bandwidth() {
    uart_packet_t packet = {
            .command_type = SET_LORA_BW,
            .packet_id = get_next_packet_id(),
            .data_length = sizeof(lora_bandwidth),
            .data = (uint8_t*) &lora_bandwidth
    };
    send_uart_packet(packet);
}

void uart_set_lora_sync_word() {
    uart_packet_t packet = {
            .command_type = SET_LORA_SYNC_WORD,
            .packet_id = get_next_packet_id(),
            .data_length = sizeof(lora_sync_word),
            .data = (uint8_t*) &lora_sync_word
    };
    send_uart_packet(packet);
}

void uart_set_lora_spreading_factor() {
    uart_packet_t packet = {
            .command_type = SET_LORA_SPREADING_FACTOR,
            .packet_id = get_next_packet_id(),
            .data_length = sizeof(lora_spreading_factor),
            .data = &lora_spreading_factor
    };
    send_uart_packet(packet);
}

void uart_set_lora_tx_power() {
    uart_packet_t packet = {
            .command_type = SET_LORA_TX_POWER,
            .packet_id = get_next_packet_id(),
            .data_length = sizeof(lora_tx_power),
            .data = &lora_tx_power
    };
    send_uart_packet(packet);
}

void uart_get_lora_frequency() {
    uart_packet_t packet = {
            .command_type = GET_LORA_FREQ,
            .packet_id = get_next_packet_id(),
            .data_length = 0,
            .data = NULL
    };
    send_uart_packet(packet);
}

void uart_get_lora_bandwidth() {
    uart_packet_t packet = {
            .command_type = GET_LORA_BW,
            .packet_id = get_next_packet_id(),
            .data_length = 0,
            .data = NULL
    };
    send_uart_packet(packet);
}

void uart_get_lora_sync_word() {
    uart_packet_t packet = {
            .command_type = GET_LORA_SYNC_WORD,
            .packet_id = get_next_packet_id(),
            .data_length = 0,
            .data = NULL
    };
    send_uart_packet(packet);
}

void uart_get_lora_spreading_factor() {
    uart_packet_t packet = {
            .command_type = GET_LORA_SPREADING_FACTOR,
            .packet_id = get_next_packet_id(),
            .data_length = 0,
            .data = NULL
    };
    send_uart_packet(packet);
}

void uart_get_lora_tx_power() {
    uart_packet_t packet = {
            .command_type = GET_LORA_TX_POWER,
            .packet_id = get_next_packet_id(),
            .data_length = 0,
            .data = NULL
    };
    send_uart_packet(packet);
}

void uart_reboot_stm() {
    uart_packet_t packet = {
            .command_type = STM_REBOOT,
            .packet_id = get_next_packet_id(),
            .data_length = 0,
            .data = NULL
    };
    send_uart_packet(packet);
}