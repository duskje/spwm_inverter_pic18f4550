/*
 * File:   serial_communication.c
 * Author: duskje
 *
 * Created on November 10, 2022, 3:12 PM
 */

#include <string.h>

#include "serial_communication.h"


void usart_init() {
    TRISCbits.TRISC6 = 0; // Configuración del pin RC6 como salida
    TRISCbits.TRISC7 = 1; // Configuración del pin RC7 como entrada

    volatile uint8_t USART_CONFIG_TX = 0;

    USART_CONFIG_TX |= 0 << 7; // CSRC: Modo configuración Master/Slave, en modo asincrónico no importa
    USART_CONFIG_TX |= 0 << 6; // TX9: Configuramos la cantidad de bits de datos a 8
    USART_CONFIG_TX |= 1 << 5; // TXEN: Habilitamos la transmisión USART
    USART_CONFIG_TX |= 0 << 4; // SYNC: Modo asincróno
    USART_CONFIG_TX |= 0 << 3; // SENDB: Envío de 'break' en la próxima transmisión desactivado
    USART_CONFIG_TX |= 0 << 2; // BRGH: Baud rate de alta velocidad desactivado
    // USART_CONFIG_TX |= 0 << 1 TRMT: Estado del envío SOLO LECTURA
    // USART_CONFIG_TX |= 0 << 0 RX9D (8th bit data) NO SE USA

    volatile uint8_t USART_CONFIG_RX = 0;

    USART_CONFIG_RX |= 1 << 7; // SPEN: Activa el puerto serial
    USART_CONFIG_RX |= 0 << 6; // RX9: Configuramos la cantidad de bits de datos a 8
    USART_CONFIG_RX |= 1 << 5; // SREN: Habilitamos la recepción USART
    USART_CONFIG_RX |= 1 << 4; // CREN: Modo asincróno
    USART_CONFIG_RX |= 0 << 3; // ADDEN: Detección de dirección
    // USART_CONFIG_TX |= 0 << 2 FERR: Bit de error de encuadre SÓLO LECTURA
    // USART_CONFIG_TX |= 0 << 1 OERR: Bit de error de sobreescritura SÓLO LECTURA
    // USART_CONFIG_TX |= 0 << 0 TX9D (8th bit data) NO SE USA

    volatile uint8_t BAUDRATE_CONFIG = 0;

    BAUDRATE_CONFIG |= 0 << 5; // RXDTP: Desactivamos la inversión de la polaridad para RX
    BAUDRATE_CONFIG |= 0 << 4; // TXDTP: Desactivamos la inversión de la polaridad para TX
    BAUDRATE_CONFIG |= 0 << 3; // BRG16: Configuramos el registro del baud rate para sólo operar en 8 bits
    BAUDRATE_CONFIG |= 0 << 1; // WUE (Wake-up Enable Bit): Desactivamos la detección

    TXSTA = USART_CONFIG_TX;
    RCSTA = USART_CONFIG_RX;
    
    BAUDCON = BAUDRATE_CONFIG;

    SPBRG = 51;
}

void usart_transmit_byte(uint8_t byte) {
    while (!TXIF);

    TXREG = byte;
}

void usart_transmit_n_bytes(uint8_t n_bytes, uint8_t *usart_buffer) {
    for (int i = 0; i < n_bytes; i++) {
        usart_transmit_byte(usart_buffer[i]);
    }
}

void send_with_data(
    uint8_t msg_len,
    msg_type_t msg_type,
    uint8_t *data_buf,
    uint8_t data_buf_len
) {
    usart_transmit_byte(msg_len);
    usart_transmit_byte(msg_type);

    usart_transmit_n_bytes(msg_len - 1, data_buf);
}

void send_message(msg_type_t msg_type) {
    usart_transmit_byte(1);
    usart_transmit_byte(msg_type);
}


communication_result_t usart_recv(uint8_t n_bytes, uint8_t *usart_result_buffer, uint8_t usart_buffer_length) {
    if(usart_ring_buffer_get_size() == 2){
        uint8_t msg_type = 0;
        
        usart_ring_buffer_get_element(&msg_type, 1);
        
        if(msg_type == EXIT){
            return EXIT_RECEIVED;
        }
    }
    
    if(usart_ring_buffer_get_size() < n_bytes){
        return NOT_READY;
    }
    
    for (int i = 0; i < n_bytes; i++) {
        usart_ring_buffer_pop(&usart_result_buffer[i]);
    }

    return SUCCESS;
}

communication_result_t recv_connect_message(void){
    const uint8_t message_length = 2;
    uint8_t result_buffer[2] = {0};
    
    communication_result_t result = usart_recv(message_length, result_buffer, message_length);
    // TODO: Errores
    if(result != SUCCESS){
        return result;
    }
    
    if(result_buffer[1] != CONN){
        return WRONG_MESSAGE_ERROR;
    }
    
    return SUCCESS;
}

communication_result_t send_syn_message(modulation_index_tables_enum current_modulation_index){
    const uint8_t buffer_length = 3;
    const uint8_t message_length = 2;
    const msg_type_t message_type = SYN;
    
    uint8_t transmit_buffer[3] = {0};
    
    transmit_buffer[0] = message_length;
    transmit_buffer[1] = message_type;
    transmit_buffer[2] = current_modulation_index;

    usart_transmit_n_bytes(buffer_length, transmit_buffer);
    
    return SUCCESS;
}

communication_result_t recv_syn_message(modulation_index_tables_enum *current_modulation_index){
   const uint8_t buffer_length = 3;
   const uint8_t message_length = 3;
   const msg_type_t message_type = SYN;
   
   uint8_t recv_buffer[3] = {0};
   
   communication_result_t result = usart_recv(message_length, recv_buffer, message_length);
   
   if(result != SUCCESS){
      return result;
    }
    
    if(recv_buffer[1] != SYN){
        return WRONG_MESSAGE_ERROR;
    }
    
    *current_modulation_index = recv_buffer[2];
    
    return SUCCESS;
}

communication_result_t send_ack_message(void){
    const uint8_t message_length = 1;
    usart_transmit_byte(message_length);
    
    const msg_type_t message_type = ACK;
    usart_transmit_byte(message_type);
    
    return SUCCESS;
}

communication_result_t recv_ack_message(void){
    const uint8_t message_length = 2;
    uint8_t result_buffer[2] = {0};
    
    communication_result_t result = usart_recv(message_length, result_buffer, message_length);
    
    if(result != SUCCESS){
        return result;
    }

    if(result_buffer[1] != ACK){
        return WRONG_MESSAGE_ERROR;
    }
    
    return SUCCESS;
}
