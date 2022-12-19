/*
 * File:   serial_communication.c
 * Author: duskje
 *
 * Created on November 10, 2022, 3:12 PM
 */

#include <string.h>

#include "serial_communication.h"

#define DATA_BUF_LEN 10
#define MAX_RETRIES 10
#define MAX_FETCH_RETRIES 100
#define TIME_DELAY_MS 10

#define _XTAL_FREQ 20e6

#define MESSAGE_QUEUE_LEN 8

msg_type_t message_queue[8] = {0};
msg_type_t message_queue_size = 0;

message_queue_result_t message_queue_push(msg_type_t message){
    if (message_queue_size == MESSAGE_QUEUE_LEN) {
        return MESSAGE_QUEUE_FULL;
    }
    
    message_queue[message_queue_size] = message;
    message_queue_size++;
    
    return MESSAGE_QUEUE_SUCCESS;
}

message_queue_result_t message_queue_pop(msg_type_t *message){
    if (!message_queue_size) {
        return MESSAGE_QUEUE_EMPTY;
    }
    
    *message = message_queue[message_queue_size - 1];
    message_queue_size--;
    
    return MESSAGE_QUEUE_SUCCESS;
}

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

result_t usart_receive_n_bytes(uint8_t n_bytes, uint8_t *usart_buffer, uint8_t usart_buf_len) {
    for (int i = 0; i < n_bytes; i++) {
        uint8_t recv_byte;

        result_t result = usart_receive_byte(&recv_byte);

        if (result == TIMEOUT) {
            return TIMEOUT;
        }

        usart_buffer[i] = recv_byte;
    }

    return SUCCESS;
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

result_t recv_with_data(
    uint8_t *received_msg_len,
    msg_type_t *received_msg_type, 
    uint8_t *data_buf,
    uint8_t data_buf_len
) {
    result_t result;

    result = usart_receive_byte(received_msg_len);

    if (result == TIMEOUT) {
        return TIMEOUT;
    }

    result = usart_receive_byte(received_msg_type);

    if (result == TIMEOUT) {
        return TIMEOUT;
    }

    result = usart_receive_n_bytes((*received_msg_len) - 1, data_buf, data_buf_len);

    if (result == TIMEOUT) {
        return TIMEOUT;
    }

    return SUCCESS;
}

void send_message(msg_type_t msg_type) {
    usart_transmit_byte(1);
    usart_transmit_byte(msg_type);
}

result_t recv_message(uint8_t *received_msg_len, msg_type_t *received_msg_type) {
    result_t result;

    result = usart_receive_byte(received_msg_len);

    if (result == TIMEOUT) {
        return TIMEOUT;
    }

    result = usart_receive_byte(&received_msg_type);

    if (result == TIMEOUT) {
        return TIMEOUT;
    }

    return SUCCESS;
}

bool connect() {
    uint8_t recv_len = 0;
    msg_type_t recv_msg_type = 0;

    send_message(CONN);

    if (recv_message(&recv_len, &recv_msg_type) == TIMEOUT) {
        return false;
    }

    if (recv_msg_type != ACK) {
        return false;
    }

    uint8_t data_buf[3];

    data_buf[0] = PR2;
    data_buf[1] = CCPR1L;
    data_buf[2] = CCP1CON;

    send_with_data(4, SYNCHRO, data_buf, sizeof (data_buf));

    if (recv_message(&recv_len, &recv_msg_type) == TIMEOUT) {
        return false;
    }

    if (recv_msg_type != ACK) {
        return false;
    }

    return true;
}


void setCCP1CON_5_4(uint8_t ccpxcon) {
    CCP1CON &= ~(0b11 << 4); // Limpiamos los bits 5:4 de CCP1CON
    CCP1CON |= (ccpxcon & 0b11) << 4; // Asignamos los 2 dos bits menos significativos de dutyCycleBits a los bits 5:4 de CCP1CON 
}

uint8_t getCCP1CON_5_4(void) {
    return (CCP1CON >> 4) & 0b11;
}


result_t usart_recv(uint8_t n_bytes, uint8_t *usart_result_buffer, uint8_t usart_buffer_length) {
    if(usart_ring_buffer_get_size() < n_bytes){
        return NOT_READY;
    }
    
    for (int i = 0; i < n_bytes; i++) {
        //usart_result_buffer[i] = usart_ring_buffer_peek();
        usart_ring_buffer_pop(&usart_result_buffer[i]);
    }

    return SUCCESS;
}

result_t recv_connect_message(void){
    const uint8_t message_length = 2;
    uint8_t result_buffer[2] = {0};
    
    result_t result = usart_recv(message_length, result_buffer, message_length);
    // TODO: Errores
    if(result != SUCCESS){
        return result;
    }
    
    if(result_buffer[1] != CONN){
        return WRONG_MESSAGE_ERROR;
    }
    
    return SUCCESS;
}
/*/
result_t recv_syn_message(float *modulation_index){
    const uint8_t message_length = 6
    const msg_type_y message_type = SYNCRHO;
    
    usart_recv
}*/

result_t send_syn_message(modulation_index_tables_enum current_modulation_index){
    const uint8_t buffer_length = 3;
    const uint8_t message_length = 2;
    const msg_type_t message_type = SYNCHRO;
    
    uint8_t transmit_buffer[3] = {0};
    
    transmit_buffer[0] = message_length;
    transmit_buffer[1] = message_type;
    transmit_buffer[2] = current_modulation_index;

    usart_transmit_n_bytes(buffer_length, transmit_buffer);
    
    return SUCCESS;
}

result_t recv_syn_message(modulation_index_tables_enum *current_modulation_index){
   const uint8_t buffer_length = 3;
   const uint8_t message_length = 3;
   const msg_type_t message_type = SYNCHRO;
   
   uint8_t recv_buffer[3] = {0};
   
   result_t result = usart_recv(message_length, recv_buffer, message_length);
   
   if(result != SUCCESS){
      return result;
    }
    
    if(recv_buffer[1] != SYNCHRO){
        return WRONG_MESSAGE_ERROR;
    }
    
    *current_modulation_index = recv_buffer[2];
    
    return SUCCESS;
}

result_t send_ack_message(void){
    const uint8_t message_length = 1;
    usart_transmit_byte(message_length);
    
    const msg_type_t message_type = ACK;
    usart_transmit_byte(message_type);
    
    return SUCCESS;
}

result_t recv_ack_message(void){
    const uint8_t message_length = 2;
    uint8_t result_buffer[2] = {0};
    
    result_t result = usart_recv(message_length, result_buffer, message_length);
    
    if(result != SUCCESS){
        return result;
    }

    if(result_buffer[1] != ACK){
        return WRONG_MESSAGE_ERROR;
    }
    
    return SUCCESS;
}

bool synchronize(void) {
    uint8_t recv_len = 0;
    msg_type_t recv_msg_type = 0;

    uint8_t data_buf[DATA_BUF_LEN] = {0};
    result_t result;

    uint8_t fetch_retries = MAX_FETCH_RETRIES;
    
    // Se reintentará recibir una respuesta al menos 3 veces
    while (fetch_retries) {
        send_message(FETCH);

        result = recv_with_data(&recv_len, &recv_msg_type, data_buf, sizeof(data_buf));
        
        if (result == TIMEOUT) {
            fetch_retries--;
            
            for(unsigned int i = 0; i < 1e3; i++)
                __delay_us(1);
            
        } else if (recv_msg_type == SYNCHRO) {
            break; // Si se recibe el mensaje SYNC, se deja de reintentar
        } else if (recv_msg_type != SYNCHRO) {
            /* Si se recibe un mensaje, pero no es SYNC asumimos que hay un
             *  error en el envío desde la interfaz y se desconecta */
            
            return false; 
        }
    }
    
    if (result == TIMEOUT) {
        return false;
    }

    PR2 = data_buf[0];
    setCCP1CON_5_4(data_buf[2]);
    CCPR1L = data_buf[1];

    msg_type_t msg = ACK;
    
    for(unsigned int i = 0; i < 1e3; i++)
        __delay_us(1);
    
    send_message(msg);

    return true;
}
