/* 
 * File:   usart.h
 * Author: duskje
 *
 * Created on 2 de diciembre de 2022, 15:37
 */

#ifndef USART_H
#define	USART_H

#include <stdint.h>

#define USART_BUF_LEN 8

typedef enum usart_result_t {
    USART_SUCCESS,
    BUFFER_FULL_ERROR,
    BUFFER_EMPTY_ERROR,
} usart_result_t;

bool is_usart_ring_buffer_full();
bool is_usart_ring_buffer_empty();

usart_result_t usart_ring_buffer_put(uint8_t byte);
usart_result_t usart_ring_buffer_pop(uint8_t *byte);
usart_result_t usart_ring_buffer_peek(uint8_t* byte);
void usart_ring_buffer_clear();

uint8_t usart_ring_buffer_get_size();

#endif	/* USART_H */

