/*
 * Iniciando.c
 *
 * Created: 4/05/2018 11:37:10 a. m.
 * Author : zvp19
 * Estructura de un programa
 */ 
// --------------Configuración del microcontrolador ------------------------
// llamado de librerias
#include <avr/io.h>  // llama la libreria que contiene las palabras reservadas, para el control de puertos de entrada/salida
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>  //include libm

#include "mpu6050.h"

#ifndef F_CPU
/* prevent compiler error by supplying a default */
/*# warning "F_CPU not defined for "*/
# define F_CPU 16000000UL
#endif


//--------definición de los puertos I/O ------------------------------------

#define INIT_PORT1() DDRD |= (1<<PD3) | (1<<PD4) | (1<<PD5) | (0<<PD6) | (0<<PD7) | (0<<PD2) // los pines D3,D4 y D5 son salidas y los pines D6, D7, D1, D2 son entradas.
#define INIT_PORT2() DDRB |= (1<<PB0)| (1<<PB1)| (1<<PB2)| (1<<PB3)
	
//--------definición de variables y constantes------------------------------
#define CLK_HIGH()  PORTD |= _BV(PD5)  // Fija un 1 en la salida del pin D3
#define CLK_LOW()   PORTD &= ~_BV(PD5) // Fija un 0 en la salida del pin D3
#define CS_HIGH()   PORTD |= _BV(PD4)
#define CS_LOW()    PORTD &= ~_BV(PD4)
#define DATA_HIGH() PORTD |= _BV(PD3)
#define DATA_LOW()  PORTD &= ~_BV(PD3)

#define CONT_HIGH_1() PORTB |= _BV(PB1)
#define CONT_LOW_1()  PORTB &= ~_BV(PB1)
#define CONT_HIGH_2() PORTB |= _BV(PB2)
#define CONT_LOW_2()  PORTB &= ~_BV(PB2)
#define CONT_HIGH_6() PORTB |= _BV(PB3)
#define CONT_LOW_6()  PORTB &= ~_BV(PB3)
#define CONT_HIGH_0() PORTB |= _BV(PB0)
#define CONT_LOW_0()  PORTB &= ~_BV(PB0)

#define BUTTON6_MASK (1<<PD6)
#define BUTTON7_MASK (1<<PD7)
#define BUTTON2_MASK (1<<PD2)
#define BUTTON_PIN PIND

volatile uint8_t button6_down = 0b00000000;
volatile uint8_t button7_down = 0b00000000;
volatile uint8_t button2_down = 0b00000000;

int x = 8;
int y = 1;
int time = 0;
int menu = 1;
int state = 1;
uint8_t row;

uint8_t now[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000
};

uint8_t win[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00000011,
	0b00001100,
	0b00110000,
	0b11000000,
	0b11000000,
	0b00110000,
	0b00001100,
	0b00000011
};

uint8_t menuLevel1[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00000000,
	0b00000001,
	0b00000000,
	0b01010101,
	0b01010100,
	0b01010000,
	0b01000000,
	0b00000000
};

uint8_t menuLevel2[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00000000,
	0b00000100,
	0b00000000,
	0b01010101,
	0b01010100,
	0b01010000,
	0b01000000,
	0b00000000
};

uint8_t menuLevel3[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00000000,
	0b00010000,
	0b00000000,
	0b01010101,
	0b01010100,
	0b01010000,
	0b01000000,
	0b00000000
};

uint8_t menuLevel4[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00000000,
	0b01000000,
	0b00000000,
	0b01010101,
	0b01010100,
	0b01010000,
	0b01000000,
	0b00000000
};

uint8_t current_level[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000
};

uint8_t mazeLevel1[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000
};
uint8_t mazeLevel2[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00000000
};
uint8_t mazeLevel3[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b01100010,
	0b00001010,
	0b10111010,
	0b10100010,
	0b10101110,
	0b10101010,
	0b10101010,
	0b00100000
};

uint8_t mazeLevel4[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00000010,
	0b11011100,
	0b00000101,
	0b11010100,
	0b11010010,
	0b10001010,
	0b00111010,
	0b01000000
};

//--------definición funciones del sistema ---------------------------------

// ********** Funcion enviar por SPI ***************************************
void update_now(int x, int y){
	strncpy(now, current_level, 8);
	
	row = 0b10000000;
	if (x==1)
	{
		row = 0b10000000;
	}
	else if (x==2){
		row = 0b01000000;
	}
	else if (x==3){
		row = 0b00100000;
	}
	else if (x==4){
		row = 0b00010000;
	}
	else if (x==5){
		row = 0b00001000;
	}
	else if (x==6){
		row = 0b00000100;
	}
	else if (x==7){
		row = 0b00000010;
	}
	else if (x==8){
		row = 0b00000001;
	}

	now[y-1] = row | current_level[y-1];
}

void check_time(){
		if(time>18000){
			CONT_HIGH_0();
		}
		else if(time>16000){
			CONT_LOW_0();
			CONT_LOW_1();
			CONT_LOW_2();
			CONT_HIGH_6();
		}
		else if(time>14000){
			CONT_HIGH_0();
		}
		else if(time>12000){
			CONT_LOW_0();
			CONT_HIGH_1();
		}
		else if(time>10000){
			CONT_HIGH_0();
		}
		else if(time>8000){
			CONT_LOW_0();
			CONT_LOW_1();
			CONT_HIGH_2();
		}
		else if(time>6000){
			CONT_HIGH_0();
		}
		else if(time>4000){
			CONT_LOW_0();
			CONT_HIGH_1();
		}
		else if(time>2000){
			CONT_HIGH_0();
			
		}
}

void spi_send(uint8_t data) // se especifica el tipo de variable que va a entrar a la funcion y como se llamara dentro de ella
{
	uint8_t i;  // declaracion de variable local
	for (i = 0; i < 8; i++, data <<= 1)  // Realiza el barrido de las 8 posiciones del vector data
	{
		CLK_LOW();
		if (data & 0x80)
			DATA_HIGH();
		else
			DATA_LOW();
			CLK_HIGH();
	}
	
}
// ********** Escribir en la matriz ***************************************
void max7219_writec(uint8_t high_byte, uint8_t low_byte)
{
	CS_LOW();
	spi_send(high_byte);
	spi_send(low_byte);
	CS_HIGH();
}
// ********** Limpiar la matriz *******************************************

void max7219_clear(void)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		max7219_writec(i+1, 0);
	}
}
// ********** Inicializar la matriz ***************************************
void max7219_init(void)
{
	INIT_PORT1();
	INIT_PORT2();
	CONT_LOW_0();
	CONT_LOW_1();
	CONT_LOW_2();
	CONT_LOW_6();
	// Decode mode: none
	max7219_writec(0x04, 0);
	// Intensity: 3 (0-15)
	max7219_writec(0x0A, 1);
	// Scan limit: All "digits" (rows) on
	max7219_writec(0x0B, 7);
	// Shutdown register: Display on
	max7219_writec(0x0C, 1);
	// Display test: off
	max7219_writec(0x0F, 0);
	max7219_clear();
}
// ********** Actualizar la pantalla ***************************************
uint8_t display[8];
void update_display(void)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		max7219_writec(i+1, display[i]);
	}
}
// ************************** Imagen ***************************************

void image(const uint8_t im[8])
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	display[i] = im[i];
}
// ************************** fijar Pixel ***************************************
void set_pixel(uint8_t r, uint8_t c, uint8_t value)
{
	switch (value)
	{
		case 0: // Clear bit
				display[r] &= (uint8_t) ~(0x80 >> c);
				break;
		case 1: // Set bit
				display[r] |= (0x80 >> c);
				break;
		default: // XOR bit
				display[r] ^= (0x80 >> c);
				break;
	}
}

static inline void debouncebtn6(void)
{
	// Counter for number of equal states
	static uint8_t count = 0;
	// Keeps track of current (debounced) state
	static uint8_t button_state = 1;
	// Check if button is high or low for the moment
	uint8_t current_state = (~BUTTON_PIN & BUTTON6_MASK) != 0;
	if (current_state != button_state) {
		// Button state is about to be changed, increase counter
		count++;
		if (count >= 4) {
			// The button have not bounced for four checks, change state
			button_state = current_state;
			// If the button was pressed (not released), tell main so
			if (current_state != 0) {
				button6_down = 1;
			}
			count = 0;
		}
		} else {
		// Reset counter
		count = 0;
	}
}

static inline void debouncebtn7(void)
{
	// Counter for number of equal states
	static uint8_t count = 0;
	// Keeps track of current (debounced) state
	static uint8_t button_state = 1;
	// Check if button is high or low for the moment
	uint8_t current_state = (~BUTTON_PIN & BUTTON7_MASK) != 0;
	if (current_state != button_state) {
		// Button state is about to be changed, increase counter
		count++;
		if (count >= 4) {
			// The button have not bounced for four checks, change state
			button_state = current_state;
			// If the button was pressed (not released), tell main so
			if (current_state != 0) {
				button7_down = 1;
			}
			count = 0;
		}
		} else {
		// Reset counter
		count = 0;
	}
}




int collision(int x, int y, uint8_t maze[]){ //1 si hay colision entre la bola y una pared del laberinto, 0 de lo contrario
	uint8_t maze_row = maze[y-1];
	uint8_t hasCollided = maze_row & (1 << (8-x));
	if(hasCollided){return 1;}
	else{return 0;}
}

// ------------ Inicio del programa ----------------------------------------
int main(void)
{
	DDRB = 0x01;
	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	double axg = 0;
	double ayg = 0;
	double azg = 0;
	double gxds = 0;
	double gyds = 0;
	double gzds = 0;
	CONT_HIGH_1();
	CONT_HIGH_2();
	CONT_HIGH_6();
	CONT_HIGH_0();

	//init interrupt
	sei();
	//init mpu6050
	mpu6050_init();
	_delay_ms(50);
	int velocity_delay = 250;
	max7219_init(); // llamado de la funcion "max7219_init"
	image(win);  // carga la imagen a visualizar
	update_display();
	while(1)  // loop infinito
	{
		debouncebtn6();
		debouncebtn7();
		if(button6_down)
		{
			if(state==0){
				image(win);  // carga la imagen a visualizar
				state = 1;
			}
			else if(state == 1){
				image(menuLevel1);
				state = 2;
			}
			else if(state == 2){
				image(now);
				state = 3;
			}
			else if(state == 4){
				image(menuLevel1);
				state = 2;

			}
			update_display();
			button6_down = 0;
		}
		
		if(button7_down)
		{
			if(state!=2){
				image(win);
			}
			else if(menu<4){
				menu = menu+1;
			}
			else{
				menu = 1;
			}
			if(state==1){
			}
			else if(menu==1){image(menuLevel1);}
			else if(menu==2){image(menuLevel2);}
			else if(menu==3){image(menuLevel3);}
			else if(menu==4){image(menuLevel4);}
			update_display();
			button7_down = 0;
		}
		
		
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		if(state==3){
			if(menu == 1)
			{
				strncpy(current_level, mazeLevel1, 8);
			}
			if(menu == 2)
			{
				strncpy(current_level, mazeLevel2, 8);
			}
			if(menu == 3)
			{
				strncpy(current_level, mazeLevel3, 8);
			}
			if(menu == 4)
			{
				strncpy(current_level, mazeLevel4, 8);
			}
			if(ayg<-0.1 && collision(x,y-1,current_level)==0){
				if(ayg<-0.3){
					velocity_delay = 150;
				}
				else{
					velocity_delay = 250;
				}
				if(y>1){
					y=y-1;
				}
			}
			else if(ayg>0.1 && collision(x,y+1,current_level)==0){
				if(ayg>0.3){
					velocity_delay = 100;
				}
				else{
					velocity_delay = 250;
				}
				if(y<8){
					y=y+1;
				}
			}
			if(axg<-0.1 && collision(x-1,y,current_level)==0){
				if(axg<-0.3){
					velocity_delay = 100;
				}
				else{
					velocity_delay = 250;
				}
				if(x>1){
					x=x-1;
				}
			}
			else if(axg>0.1 && collision(x+1,y,current_level)==0){
				if(axg>0.3){
					velocity_delay = 100;
				}
				else{
					velocity_delay = 250;
				}
				if(x<8){
					x=x+1;
				}
				
			}
			_delay_ms(velocity_delay);
			time = time + velocity_delay;
			update_now(x,y);
			image(now);
			update_display();
			check_time();
			
			if(x==1 && y==8 ){
				state=4;
				image(win);
				update_display();
				x=8;
				y=1;
				time = 0;
				menu = 1;
				CONT_LOW_0();
				CONT_LOW_1();
				CONT_LOW_2();
				CONT_LOW_6();
			}
		}
	}
}
