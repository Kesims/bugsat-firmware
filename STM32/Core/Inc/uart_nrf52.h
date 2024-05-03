#ifndef STM32_UART_NRF52_H
#define STM32_UART_NRF52_H

#include "main.h"

#define UART_TX_BUFFER_SIZE 256
#define UART_RX_BUFFER_SIZE 256
#define TX_QUEUE_BUFFER_STEP UART_TX_BUFFER_SIZE
#define TX_QUEUE_BUFFER_ITEM_COUNT 4

enum command_type_t {
    SET_LORA_FREQ = 0x01,
    GET_LORA_FREQ = 0x02,
    SET_LORA_BW = 0x03,
    GET_LORA_BW = 0x04,
    SET_LORA_SYNC_WORD = 0x05,
    GET_LORA_SYNC_WORD = 0x06,
    SET_LORA_SPREADING_FACTOR = 0x07,
    GET_LORA_SPREADING_FACTOR = 0x08,
    SET_LORA_TX_POWER = 0x09,
    GET_LORA_TX_POWER = 0x0A,
    STM_REBOOT = 0x0B,
    CONF_MAX_VALUE, // Used to determine the max value of the enum
};

enum uart_state_t {
    WAITING_FOR_PREAMBLE = 0x00,
    WAITING_FOR_COMMAND = 0x01,
    WAITING_FOR_ID = 0x02,
    WAITING_FOR_DATA_LENGTH = 0x03,
    WAITING_FOR_DATA = 0x04,
    WAITING_FOR_CRC = 0x05,
};

typedef struct uart_packet_t {
    enum command_type_t command_type;
    uint8_t packet_id;
    uint8_t data_length;
    uint8_t * data; // last 4 bytes after of data are CRC32
} uart_packet_t;

typedef struct uart_tx_queue_item_t {
    uart_packet_t *packet;
    struct uart_tx_queue_item_t *next;
} uart_tx_queue_item_t;

void clear_rx();
void init_nrf_uart_comm();
void on_nrf_uart_receive();
void process_uart_rx();
void process_uart_tx();

void uart_send_lora_frequency();
void uart_send_lora_bandwidth();
void uart_send_lora_sync_word();
void uart_send_lora_spreading_factor();
void uart_send_lora_tx_power();


#endif //STM32_UART_NRF52_H
