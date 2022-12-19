/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  

#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>

#include "usart.h"
#include "spwm_tables.h"

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

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
   WRONG_MESSAGE_ERROR = 4
} result_t; // cambiar nombre

typedef enum message_queue_result {
    MESSAGE_QUEUE_SUCCESS = 0,
    MESSAGE_QUEUE_FULL = 1,
    MESSAGE_QUEUE_EMPTY = 2
} message_queue_result_t;


typedef enum msg_type {
   CONN = 1,
   ACK = 2,
   NACK = 3,
   SYNCHRO = 4, // cambiar a SYN
   ALIVE = 5,
   FETCH = 6,
   READY = 7,
   EXIT = 8
} msg_type_t;

void usart_init();

result_t usart_receive_byte(uint8_t *recv_byte);
result_t usart_receive_n_bytes(uint8_t n_bytes, uint8_t *usart_buffer, uint8_t usart_buf_len);

void usart_transmit_byte(uint8_t byte);
void usart_transmit_n_bytes(uint8_t n_bytes, uint8_t *usart_buffer);

void recv(msg_type_t *msg_type, uint8_t *data_buf, uint8_t data_buf_len);

void recv_and_dispatch();

result_t recv_connect_message();
result_t send_syn_message(modulation_index_tables_enum current_modulation_index);
result_t send_ack_message();
result_t recv_syn_message(modulation_index_tables_enum *current_modulation_index);
result_t recv_ack_message();


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif