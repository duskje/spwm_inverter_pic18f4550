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
#include "SPWM.h"
#include "usart.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define _XTAL_FREQ 20e6

volatile modulation_index_tables_enum current_modulation_index = MODULATION_INDEX_95;

volatile bool has_communication_timed_out = false;


void SPWM_init(void){
    TRISCbits.TRISC2 = 0; // Pin C2 como salida
    TRISDbits.TRISD5 = 0; // Pin D5 como salida
    
    T2CON = 0b00; // Prescaler a 1
    
    PR2 = 199;
    CCPR1L = 0; // Duty cycle inicialziado a cero

    // Modo PWM
    CCP1CON = 0x0c;
    //CCP1CON = 0b1101;
    CCP1CON |= 0b10 << 6; // Modo Half-Bridge
    ECCP1DEL = 0b11; // Tiempo muerto
    //CCP2CON = 0x0c;
    
    TMR2 = 0;
    T2CONbits.TMR2ON = 1;
}

void timer0_init(void){
    T0CONbits.TMR0ON = 0; // Empezar apagado
    T0CONbits.T08BIT = 0; // Configurar timer a 16 bits
    T0CONbits.PSA = 0; // Usar prescaler
    T0CONbits.T0CS = 0; // Modo timer
    
    // Configurar prescaler a 1 / 128
    T0CONbits.T0PS0 = 0;
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 1;
}

void timer0_start(){
    timer0_init();
    
    TMR0H = 0;
    TMR0L = 0;
    T0CONbits.TMR0ON = 1;
}

void timer0_stop(){
    T0CONbits.TMR0ON = 0;
    TMR0H = 0;
    TMR0L = 0;
}

void timer0_restart(){
   TMR0H = 0;
   TMR0L = 0;
   
   timer0_init();
   T0CONbits.TMR0ON = 1;
}

void set_interrupts(void){
    RCONbits.IPEN = 1; // Activar interruptores de alta/baja prioridad
    INTCONbits.GIEH = 1; // Activar interruptores globales
    INTCONbits.GIEL = 1; // Activar interruptores de periféricos
    
    PIE1bits.TMR2IE = 1; // Activar interruptor de TMR2
    IPR1bits.TMR2IP = 1; // Hacer al interruptor de alta prioridad
    
    INTCONbits.TMR0IE = 1; // Activar interrupciones TMR0
    INTCON2bits.TMR0IP = 0; // Hacer al interrptor de prioridad baja
    
    PIE1bits.RCIE = 1; // Activar el interruptor de recepción usart
    IPR1bits.RCIP = 0; // Hacer el interruptor de prioridad baja
    
    //PIE1bits.TXIE = 1; // Activar el interruptor de transmisión usart
    //IPR1bits.TXIP = 0; // Hacer el interruptor de prioridad baja
}

void update_CCP1CON_CCPR1L(void){
    static int table_index_ccp1 = 0;
    
    // CCPR1L = ccprxl_values_on_init[table_index_ccp1];
    //uint8_t *table = ccprxl_tables[MODULATION_INDEX_95];
    CCPR1L = ccprxl_tables[current_modulation_index][table_index_ccp1];

    CCP1CON &= ~(0b11 << 4); // Limpiamos los bits 5:4 de CCP1CON
    //    CCP1CON |= (ccpxcon_tables[MODULATION_INDEX_95][table_index_ccp1] & 0b11) << 4; // Asignamos los 2 dos bits menos significativos de dutyCycleBits a los bits 5:4 de CCP1CON
    CCP1CON |= (ccpxcon_tables[current_modulation_index][table_index_ccp1] & 0b11) << 4; // Asignamos los 2 dos bits menos significativos de dutyCycleBits a los bits 5:4 de CCP1CON
    //CCP1CON |= (ccpxcon_values_on_init[table_index_ccp1] & 0b11) << 4; // Asignamos los 2 dos bits menos significativos de dutyCycleBits a los bits 5:4 de CCP1CON

    table_index_ccp1 = (table_index_ccp1 + 1) % (SPWM_TABLE_SIZE - 2);
}

void __interrupt(high_priority) high_priorityISR(){
    if (PIR1bits.TMR2IF) {
        update_CCP1CON_CCPR1L();
        
        PIR1bits.TMR2IF = 0;
    }
}

void __interrupt(low_priority) low_priority_ISR(){
    if(PIR1bits.RCIF){
        uint8_t recv_byte = RCREG;
        
        if(RCSTAbits.OERR) {
            CREN = 0;
            NOP();
            CREN = 1;
        }
        
        if(usart_ring_buffer_put(recv_byte) == BUFFER_FULL_ERROR) {
            NOP();
        }
        
        PIR1bits.RCIF = 0;
    } else if(INTCONbits.TMR0IF){
        has_communication_timed_out = true;
        
        INTCONbits.TMR0IF = 0;
    }
}

void reset_communication(
    communication_state_t *comm_state,
    communication_action_t *comm_action
){
    usart_ring_buffer_clear();

    *comm_action = RECV_CONN;
    *comm_state = DISCONNECTED;
}

void do_serial_communication(
    communication_state_t *comm_state,
    communication_action_t *comm_action
){
    if(has_communication_timed_out){
        reset_communication(comm_state, comm_action);
        
	    has_communication_timed_out = false;
    }
    
    switch(*comm_state){
        result_t result;
        
        case CONNECTED:
            TRISDbits.RD1 = 0;
            PORTDbits.RD1 = 1;
                    
            switch(*comm_action){
                case RECV_SYN:
                    result = recv_syn_message(&current_modulation_index);
		    
                    if(result == SUCCESS){
                        timer0_restart();
			
                        *comm_action = SEND_ACK;           
                    } else if(result == NOT_READY){
                        return;
                    } else {
                        reset_communication(comm_state, comm_action);
                    }
                    
                    break;
                case SEND_ACK:
                    result = send_ack_message();
		    
                    if(result == SUCCESS){
                        timer0_restart();
			
                        *comm_action = RECV_SYN;           
                    } else if(result == NOT_READY){
                        return;
                    } else {
                        reset_communication(comm_state, comm_action);
                    }
		    
                    break;
                default:
                    break;
            }
            break;
        case DISCONNECTED:
            TRISDbits.RD1 = 0;
            PORTDbits.RD1 = 0;
            
            switch(*comm_action){
                case RECV_CONN:
                    timer0_stop();
                    
                    result = recv_connect_message();
            
                    if(result == SUCCESS){
                        timer0_start();
                        
                        *comm_action = SEND_SYN;         
                    } else if(result == NOT_READY){
                        return;
                    }
                    
                    break;
                case SEND_SYN:
                    result = send_syn_message(current_modulation_index);
            
                    if(result == SUCCESS){
                        timer0_restart();
                        
                        *comm_action = RECV_ACK;          
                    } else if(result == NOT_READY){
                        return;
                    } else { 
                        reset_communication(comm_state, comm_action);
                    }
                    
                    break;
                case RECV_ACK:
                    result = recv_ack_message();
                    
                    if(result == SUCCESS){
                        timer0_restart();
                        
                        *comm_action = RECV_SYN;
                        *comm_state = CONNECTED;            
                    } else if(result == NOT_READY){
                        return;
                    } else {
                        reset_communication(comm_state, comm_action);  
                    }
                    
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

int main(void) {
    SPWM_init();
    
    usart_init();
   
    set_interrupts();
    
    communication_action_t comm_action = RECV_CONN;
    communication_state_t comm_state = DISCONNECTED;
    
    while(true){
        do_serial_communication(&comm_state, &comm_action); 
    }

    return 0;
}

