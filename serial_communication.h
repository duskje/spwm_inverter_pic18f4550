#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>

#include "usart.h"
#include "spwm_tables.h"

typedef enum communication_action {
   RECV_CONN = 0,
   SEND_ACK = 1,
   SEND_SYN = 2,
   RECV_ACK = 3,
   RECV_SYN = 4
} communication_action_t;

typedef enum communication_state {
    DISCONNECTED = 0,
    CONNECTED = 1
} communication_state_t;

typedef enum result {
   ERROR = 0,
   SUCCESS = 1,
   TIMEOUT = 2,
   NOT_READY = 3,
   WRONG_MESSAGE_ERROR = 4,
   EXIT_RECEIVED = 5
} communication_result_t; // cambiar nombre

typedef enum message_queue_result {
    MESSAGE_QUEUE_SUCCESS = 0,
    MESSAGE_QUEUE_FULL = 1,
    MESSAGE_QUEUE_EMPTY = 2
} message_queue_result_t;

typedef enum msg_type {
   CONN = 1,
   ACK = 2,
   NACK = 3,
   SYN = 4, // cambiar a SYN
   ALIVE = 5,
   FETCH = 6,
   READY = 7,
   EXIT = 8
} msg_type_t;

void usart_init();

communication_result_t usart_receive_byte(uint8_t *recv_byte);
communication_result_t usart_receive_n_bytes(uint8_t n_bytes, uint8_t *usart_buffer, uint8_t usart_buf_len);

void usart_transmit_byte(uint8_t byte);
void usart_transmit_n_bytes(uint8_t n_bytes, uint8_t *usart_buffer);

void recv(msg_type_t *msg_type, uint8_t *data_buf, uint8_t data_buf_len);

void recv_and_dispatch();

communication_result_t recv_connect_message();
communication_result_t send_syn_message(modulation_index_tables_enum current_modulation_index);
communication_result_t send_ack_message();
communication_result_t recv_syn_message(modulation_index_tables_enum *current_modulation_index);
communication_result_t recv_ack_message();


#endif