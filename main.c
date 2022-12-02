/*
 * File:   main.c
 * Author: duskje
 *
 * Created on November 10, 2022, 3:11 PM
 */


#include <xc.h>

#include "bitconfig.h"
#include "serial_communication.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define _XTAL_FREQ 20e6

const uint32_t F_OSC = 48e6;

void setPWMFreq(float PWMFreq) {
    const uint8_t TMR2Prescaler = 16;
    PR2 = trunc(((float) F_OSC / (PWMFreq * 4 * (TMR2Prescaler))) - 1);
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

void process(void) {
    for (int i = 0; i < 100; i++) {
        setDutyCycle(i);
        __delay_ms(5);
    }
}

/*
void main(void){
   initPWMMode();
   
   const float F_PWM = 20e3; // Frecuencia de conmutación requerida
   
   setPWMFreq(F_PWM);
   
   while(true){
      process();
   }
   
   return;
}
 */

void startUpDuty(uint8_t start_duty, uint8_t end_duty) {
    for (int i = start_duty; i < end_duty; i++) {
        __delay_ms(10);
        setDutyCycle(i);
    }
}

int main(void) {
    // (48e6 / 9600 / 64) - 1 = 77.125
    initPWMMode();

    const float F_PWM = 20e3; // Frecuencia de conmutación requerida

    setPWMFreq(F_PWM);
    // setDutyCycle(50);
    startUpDuty(0, 50);

    // PR2 = 36; // Frecuencia de operacion a 20khz
    // CCP1CON = 7;
    // CCPR1L = 1;

    usart_init();

    bool connected = false;

    bool debug = false;

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

    return 0;
}

