/*
 * File:   usart.c
 * Author: duskje
 *
 * Created on 2 de diciembre de 2022, 15:58
 */


#include <xc.h>

#include <stdbool.h>
#include <stdint.h>

#include "usart.h"

static uint8_t usart_ring_buffer[USART_BUF_LEN];

static uint8_t usart_ring_buffer_end_index = 0;
static uint8_t usart_ring_buffer_start_index = 0;
static uint8_t usart_ring_buffer_size = 0;

bool is_usart_ring_buffer_full(){
    // Si el próximo índice final llega hasta el índice de inicio, el buffer está lleno

    return usart_ring_buffer_size == USART_BUF_LEN;
}

bool is_usart_ring_buffer_empty(){
    // Si el ínidice de inicio es el mismo que el índice final, el buffer está vacío
    
	return usart_ring_buffer_end_index == usart_ring_buffer_start_index;
}

usart_result_t usart_ring_buffer_get_element(uint8_t *byte, uint8_t i){
    if(i >= usart_ring_buffer_size){
        return OUT_OF_BOUNDS_ERROR;
    }
    
    *byte = usart_ring_buffer[(usart_ring_buffer_start_index + i) % USART_BUF_LEN];
    
    return USART_SUCCESS;
}

void usart_ring_buffer_clear(){
   usart_ring_buffer_end_index = 0;
   usart_ring_buffer_start_index = 0;
   usart_ring_buffer_size = 0;
}

usart_result_t usart_ring_buffer_put(uint8_t byte){
	if(is_usart_ring_buffer_full()){
		return BUFFER_FULL_ERROR; // El buffer está lleno
	}

	uint8_t last_end = usart_ring_buffer_end_index;
    
	usart_ring_buffer[last_end] = byte;
    
    usart_ring_buffer_end_index++;
    usart_ring_buffer_end_index %= USART_BUF_LEN;
    
    usart_ring_buffer_size++;

	return USART_SUCCESS;
}


usart_result_t usart_ring_buffer_pop(uint8_t *byte){
	if(is_usart_ring_buffer_empty()){
		return BUFFER_EMPTY_ERROR;
	}

	uint8_t last_start = usart_ring_buffer_start_index;
	
    *byte = usart_ring_buffer[last_start];
    
    usart_ring_buffer_start_index++;
	usart_ring_buffer_start_index %= USART_BUF_LEN;
    
    usart_ring_buffer_size--;
    
	return USART_SUCCESS;
}

usart_result_t usart_ring_buffer_peek(uint8_t* byte){
    if(is_usart_ring_buffer_empty()){
        return BUFFER_EMPTY_ERROR;
    }
    
    *byte = usart_ring_buffer[usart_ring_buffer_start_index];
    
    return USART_SUCCESS;
}

uint8_t usart_ring_buffer_get_size(){
    return usart_ring_buffer_size;
}

void usart_recv_byte();
