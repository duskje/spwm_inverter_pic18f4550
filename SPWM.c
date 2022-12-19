#include "SPWM.h"
/*
void init_tables(){
	// Copiar valores iniciales a la tabla 1

	for(int i = 0; i < SPWM_TABLE_SIZE; i++){
		table1_ccpxcon[i] = ccpxcon_values_on_init[i];
		table1_ccprxl[i] = ccprxl_values_on_init[i];
	}

	current_ccpxcon_table = table1_ccpxcon;
	current_ccprxl_table = table1_ccprxl;

	next_ccpxcon_table = table2_ccpxcon;
	next_ccprxl_table = table2_ccprxl;
}

int real_duty_cycle_to_pic_duty_cycle(float duty_cycle){
	return (int) ((PR2 + 1) * 4 * duty_cycle);
}

uint8_t get_ccprxl(float duty_cycle){
	return real_duty_cycle_to_pic_duty_cycle(duty_cycle) >> 2;
}

uint8_t get_ccpxcon(float duty_cycle){
	return real_duty_cycle_to_pic_duty_cycle(duty_cycle) & 0b11;
}

void do_change_tables(){
	uint8_t *ccpxcon_tmp = current_ccpxcon_table;
	uint8_t *ccprxl_tmp = current_ccprxl_table;

	current_ccpxcon_table = next_ccpxcon_table;
	current_ccprxl_table = next_ccprxl_table;
 
	next_ccpxcon_table = ccpxcon_tmp;
	next_ccprxl_table = ccprxl_tmp;
}

void calculate_spwm_table(float modulation_index){
	// Calcular la tabla de valores SPWM usando muestreo asimétrico

	static float last_M = INIT_MODULATION_INDEX;
    
    float M = modulation_index; // Variable para facilitar la comprensión de la fórmula

	if(last_M == M){ 
        // Si el indice de modulación es el mismo que ingresa a la función no hay 
        // que calcular nada
		return;
	}

	currently_calculating = true;

	const float SWITCHING_PERIOD = 1 / 40e3;

	for(int k = 0; k < SPWM_TABLE_SIZE; k++){
		float t_on1 = (SWITCHING_PERIOD / 4) * (1 + M * sin_samples[k]);
		float t_on2;
		
		if(k + 1 != SPWM_TABLE_SIZE){
			t_on2 = (SWITCHING_PERIOD / 4) * (1 + M * sin_samples[k + 1]);
		} else { 
			t_on2 = (SWITCHING_PERIOD / 4) * (1 + M * sin_samples[0]);
		}

		float t_on = t_on1 + t_on2;
		float duty_cycle = t_on / SWITCHING_PERIOD;

		next_ccpxcon_table[k] = get_ccprxl(duty_cycle);
		next_ccprxl_table[k] = get_ccpxcon(duty_cycle);
	}

	currently_calculating = false;
	change_tables = true;
}
*/