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
    // Si el pr�ximo �ndice final llega hasta el �ndice de inicio, el buffer est� lleno
    
	// return ( (usart_ring_buffer_end_index + 1) % USART_BUF_LEN ) == usart_ring_buffer_start_index;
    return usart_ring_buffer_size == USART_BUF_LEN;
}

bool is_usart_ring_buffer_empty(){
    // Si el �nidice de inicio es el mismo que el �ndice final, el buffer est� vac�o
    
	return usart_ring_buffer_end_index == usart_ring_buffer_start_index;
}

usart_result_t usart_ring_buffer_put(uint8_t byte){
	if(is_usart_ring_buffer_full()){
		return BUFFER_FULL_ERROR; // El buffer est� lleno
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
}

uint8_t get_usart_ring_buffer_size(){
    return usart_ring_buffer_size;
}

void usart_recv_byte();