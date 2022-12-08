/*
 * File:   main.c
 * Author: duskje
 *
 * Created on November 10, 2022, 3:11 PM
 */


#include <xc.h>

#include "bitconfig.h"
#include "serial_communication.h"

#include "spwm_tables.h"
#include "usart.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define _XTAL_FREQ 20e6

void setPWMFreq(float PWMFreq) {
    const uint8_t TMR2Prescaler = 1;
    const uint32_t F_OSC = 32e6;
    
    PR2 = ceil(((float) F_OSC / (PWMFreq * 4 * (TMR2Prescaler))) - 1);
}

void setDutyCycle(uint8_t dutyCycle) {
    if (dutyCycle > 100)
        dutyCycle = 100;

    uint16_t dutyCycleBits = trunc((PR2 + 1) * ((float) dutyCycle / 100) * 4);
    CCPR1L = dutyCycleBits >> 2; // Quitamos los dos bits menos significativos y agregamos padding en los dos más significativos

    CCP1CON &= ~(0b11 << 4); // Limpiamos los bits 5:4 de CCP1CON
    CCP1CON |= (dutyCycleBits & 0b11) << 4; // Asignamos los 2 dos bits menos significativos de dutyCycleBits a los bits 5:4 de CCP1CON 
}

void initPWMMode(void) {
    TRISCbits.TRISC2 = 0;
    PR2 = 35;
    CCPR1L = 0; // Duty cycle inicialziado a cero
    T2CON = 0b10; // Prescaler a 16
    CCP1CON = 0x0c; // Modo PWM
    TMR2 = 0;
    T2CONbits.TMR2ON = 1;
}

void initSPWM(void){
    TRISCbits.TRISC2 = 0; // Pin C2 como salida
    TRISDbits.TRISD5 = 0; // Pin D5 como salida
    
    T2CON = 0b00; // Prescaler a 1
    
    PR2 = 199;
    CCPR1L = 0; // Duty cycle inicialziado a cero

    // Modo PWM
    CCP1CON = 0x0c;
    //CCP1CON = 0b1101;
    CCP1CON |= 0b10 << 6; // Modo Half-Bridge
    ECCP1DEL = 0b111; // Tiempo muerto
    //CCP2CON = 0x0c;
    
    TMR2 = 0;
    T2CONbits.TMR2ON = 1;
}

void initTimer1(void){
    T1CONbits.RD16 = 1;
    // T1CONbits.T1RUN = 0;
    
    // T1CONbits.T1CKPS = 0b;
}

void setInterrupts(void){
    RCONbits.IPEN = 1; // Activar interruptores de alta/baja prioridad
    INTCONbits.GIEH = 1; // Activar interruptores globales
    INTCONbits.GIEL = 1; // Activar interruptores de periféricos
    
    PIE1bits.TMR2IE = 1; // Activar interruptor de Timer 2
    IPR1bits.TMR2IP = 1; // Hacer al interruptor de alta prioridad
    
    PIE1bits.RCIE = 1; // Activar el interruptor de recepción usart
    IPR1bits.RCIP = 0; // Hacer el interruptor de prioridad baja
    
    //PIE1bits.TXIE = 1; // Activar el interruptor de transmisión usart
    //IPR1bits.TXIP = 0; // Hacer el interruptor de prioridad baja
}

void updateCCP1CON_CCPR1L(void){
    static int table_index_ccp1 = 0;
    
    CCPR1L = ccprxl_values_on_init[table_index_ccp1];
            
    CCP1CON &= ~(0b11 << 4); // Limpiamos los bits 5:4 de CCP1CON
    CCP1CON |= (ccpxcon_values_on_init[table_index_ccp1] & 0b11) << 4; // Asignamos los 2 dos bits menos significativos de dutyCycleBits a los bits 5:4 de CCP1CON
        
    table_index_ccp1 = (table_index_ccp1 + 1) % table_size;
}
    
void __interrupt(high_priority) highPriorityISR(){
    if (PIR1bits.TMR2IF) {
        updateCCP1CON_CCPR1L();
        
        PIR1bits.TMR2IF = 0;
    }
}

void __interrupt(low_priority) lowPriorityISR(){
    if(PIR1bits.RCIF){
        uint8_t recv_byte = RCREG;
        
        if (RCSTAbits.OERR) {
            CREN = 0;
            NOP();
            CREN = 1;
        }
        
        if (usart_ring_buffer_put(recv_byte) == BUFFER_FULL_ERROR) {
            NOP();
        }
        
        PIR1bits.RCIF = 0;
    }
}

void serial_communication(communication_state_t *comm_state){
    switch(&comm_state){
        case CONNECT:
            break;
    }
}

int main(void) {
    initSPWM();
    
    const float F_PWM = 40e3; // Frecuencia de conmutación requerida
    usart_init();

    bool connected = false;
    bool debug = false; 
    
    setInterrupts();
    
    communication_state_t comm_state = CONNECT;
    
    /* event loop */
    while(true){
    }

    /*
    while (true) {
        if (!connected) {
            connected = connect();

            if (debug) {
                NOP();
            }
        } else {
            connected = synchronize();
            debug = true;
        }
    }
    */

    return 0;
}

