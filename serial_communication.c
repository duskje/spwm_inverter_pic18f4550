/*
 * File:   serial_communication.c
 * Author: duskje
 *
 * Created on November 10, 2022, 3:12 PM
 */

#include "serial_communication.h"

#define DATA_BUF_LEN 10
#define MAX_RETRIES 10
#define MAX_FETCH_RETRIES 100
#define TIME_DELAY_MS 10

#define _XTAL_FREQ 20e6

void usart_init() {
    TRISCbits.TRISC6 = 0; // Configuraci�n del pin RC6 como salida
    TRISCbits.TRISC7 = 1; // Configuraci�n del pin RC7 como entrada

    volatile uint8_t USART_CONFIG_TX = 0;

    USART_CONFIG_TX |= 0 << 7; // CSRC: Modo configuraci�n Master/Slave, en modo asincr�nico no importa
    USART_CONFIG_TX |= 0 << 6; // TX9: Configuramos la cantidad de bits de datos a 8
    USART_CONFIG_TX |= 1 << 5; // TXEN: Habilitamos la transmisi�n USART
    USART_CONFIG_TX |= 0 << 4; // SYNC: Modo asincr�no
    USART_CONFIG_TX |= 0 << 3; // SENDB: Env�o de 'break' en la pr�xima transmisi�n desactivado
    USART_CONFIG_TX |= 0 << 2; // BRGH: Baud rate de alta velocidad desactivado
    // USART_CONFIG_TX |= 0 << 1 TRMT: Estado del env�o SOLO LECTURA
    // USART_CONFIG_TX |= 0 << 0 RX9D (8th bit data) NO SE USA

    volatile uint8_t USART_CONFIG_RX = 0;

    USART_CONFIG_RX |= 1 << 7; // SPEN: Activa el puerto serial
    USART_CONFIG_RX |= 0 << 6; // RX9: Configuramos la cantidad de bits de datos a 8
    USART_CONFIG_RX |= 1 << 5; // SREN: Habilitamos la recepci�n USART
    USART_CONFIG_RX |= 1 << 4; // CREN: Modo asincr�no
    USART_CONFIG_RX |= 0 << 3; // ADDEN: Detecci�n de direcci�n
    // USART_CONFIG_TX |= 0 << 2 FERR: Bit de error de encuadre S�LO LECTURA
    // USART_CONFIG_TX |= 0 << 1 OERR: Bit de error de sobreescritura S�LO LECTURA
    // USART_CONFIG_TX |= 0 << 0 TX9D (8th bit data) NO SE USA


    volatile uint8_t BAUDRATE_CONFIG = 0;

    BAUDRATE_CONFIG |= 0 << 5; // RXDTP: Desactivamos la inversi�n de la polaridad para RX
    BAUDRATE_CONFIG |= 0 << 4; // TXDTP: Desactivamos la inversi�n de la polaridad para TX
    BAUDRATE_CONFIG |= 0 << 3; // BRG16: Configuramos el registro del baud rate para s�lo operar en 8 bits
    BAUDRATE_CONFIG |= 0 << 1; // WUE (Wake-up Enable Bit): Desactivamos la detecci�n

    TXSTA = USART_CONFIG_TX;
    RCSTA = USART_CONFIG_RX;
    
    BAUDCON = BAUDRATE_CONFIG;

    SPBRG = 77;
}

result_t usart_receive_byte(uint8_t *recv_byte) {
    unsigned long timeout = 0;

    while (!RCIF) {
        if (timeout > 1e3) {
            return TIMEOUT;
        }
        
        __delay_us(1);
        
        timeout++;
    }

    bool OERR = (bool) RCSTA & 0b10;

    if (OERR) {
        CREN = 0;
        NOP();
        CREN = 1;
    }

    *recv_byte = RCREG;

    return SUCCESS;
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

    result = usart_receive_byte(received_msg_type);

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

uint8_t getCCP1CON_5_4() {
    return (CCP1CON >> 4) & 0b11;
}

void softSetPWMRegisters(uint8_t pr2_to_set, uint8_t ccpr1l_to_set, uint8_t ccp1con_5_4_to_set) {
    uint8_t current_pr2 = PR2;

    while (current_pr2 - pr2_to_set) {
        if (current_pr2 < pr2_to_set) {
            PR2++;
        } else {
            PR2--;
        }

        current_pr2 = PR2;
    }

    uint8_t current_ccpr1l = CCPR1L;

    while (current_ccpr1l - ccpr1l_to_set) {
        if (current_ccpr1l < ccpr1l_to_set) {
            CCPR1L++;
        } else {
            CCPR1L--;
        }

        current_ccpr1l = CCPR1L;
    }

    /*
    for(uint8_t current_ccpr1l = CCPR1L; current_ccpr1l < ccpr1l_to_set; current_ccpr1l++){
       CCPR1L = current_ccpr1l;
       __delay_ms(5);
    //}
     */

    setCCP1CON_5_4(ccp1con_5_4_to_set);
    //for(uint8_t current_ccp1con = getCCP1CON_5_4(); current_ccp1con < ccp1con_5_4_to_set; current_ccp1con++){
    //   setCCP1CON_5_4(current_ccp1con);
    // __delay_ms(5);
    //}
}

bool synchronize() {
    uint8_t recv_len = 0;
    msg_type_t recv_msg_type = 0;

    uint8_t data_buf[DATA_BUF_LEN] = {0};
    result_t result;

    uint8_t fetch_retries = MAX_FETCH_RETRIES;
    
    // Se reintentar� recibir una respuesta al menos 3 veces
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
             *  error en el env�o desde la interfaz y se desconecta */
            
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
