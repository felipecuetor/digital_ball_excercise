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
#ifndef F_CPU
/* prevent compiler error by supplying a default */
/*# warning "F_CPU not defined for "*/
# define F_CPU 16000000UL
#endif


//--------definición de los puertos I/O ------------------------------------

#define INIT_PORT() DDRD |= (1<<PD3) | (1<<PD4) | (1<<PD5) | ~(1<<PD6) | ~(1<<PD7) | ~(1<<PD1) | ~(1<<PD2) | ~(1<<PD0) // los pines D3,D4 y D5 son salidas y los pines D6, D7, D1, D2 son entradas.
	
//--------definición de variables y constantes------------------------------
#define CLK_HIGH()  PORTD |= _BV(PD5)  // Fija un 1 en la salida del pin D3
#define CLK_LOW()   PORTD &= ~_BV(PD5) // Fija un 0 en la salida del pin D3
#define CS_HIGH()   PORTD |= _BV(PD4)
#define CS_LOW()    PORTD &= ~_BV(PD4)
#define DATA_HIGH() PORTD |= _BV(PD3)
#define DATA_LOW()  PORTD &= ~_BV(PD3)

#define BUTTON6_MASK (1<<PD6)
#define BUTTON7_MASK (1<<PD7)
#define BUTTON1_MASK (1<<PD1)
#define BUTTON2_MASK (1<<PD2)
#define BUTTON0_MASK (1<<PD0)
#define BUTTON_PIN PIND

volatile uint8_t button6_down;
volatile uint8_t button7_down;
volatile uint8_t button1_down;
volatile uint8_t button2_down;
volatile uint8_t button0_down;

int x = 4;
int y = 4;
int menu = 1;
int state = 0;
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

uint8_t start[8] = {  // definicion de vector de 8 posiciones con variables internas de 8bits
	0b00000000,
	0b11111111,
	0b10001001,
	0b10001001,
	0b10001001,
	0b11001001,
	0b01110110,
	0b00000000
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

//--------definición funciones del sistema ---------------------------------

// ********** Funcion enviar por SPI ***************************************
void update_now(int x, int y){
	uint8_t empty = 0b00000000;
	now[0] = empty;
	now[1] = empty;
	now[2] = empty;
	now[3] = empty;
	now[4] = empty;
	now[5] = empty;
	now[6] = empty;
	now[7] = empty;
	
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
	now[y-1] = row;
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
	INIT_PORT();
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
	static uint8_t button_state = 0;
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
	static uint8_t button_state = 0;
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

static inline void debouncebtn1(void)
{
	// Counter for number of equal states
	static uint8_t count = 0;
	// Keeps track of current (debounced) state
	static uint8_t button_state = 0;
	// Check if button is high or low for the moment
	uint8_t current_state = (~BUTTON_PIN & BUTTON1_MASK) != 0;
	if (current_state != button_state) {
		// Button state is about to be changed, increase counter
		count++;
		if (count >= 100) {
			// The button have not bounced for four checks, change state
			button_state = current_state;
			// If the button was pressed (not released), tell main so
			if (current_state != 0) {
				button1_down = 1;
			}
			count = 0;
		}
		} else {
		// Reset counter
		count = 0;
	}
}

static inline void debouncebtn2(void)
{
	// Counter for number of equal states
	static uint8_t count = 0;
	// Keeps track of current (debounced) state
	static uint8_t button_state = 0;
	// Check if button is high or low for the moment
	uint8_t current_state = (~BUTTON_PIN & BUTTON2_MASK) != 0;
	if (current_state != button_state) {
		// Button state is about to be changed, increase counter
		count++;
		if (count >= 100) {
			// The button have not bounced for four checks, change state
			button_state = current_state;
			// If the button was pressed (not released), tell main so
			if (current_state != 0) {
				button2_down = 1;
			}
			count = 0;
		}
		} else {
		// Reset counter
		count = 0;
	}
}

static inline void debouncebtn0(void)
{
	// Counter for number of equal states
	static uint8_t count = 0;
	// Keeps track of current (debounced) state
	static uint8_t button_state = 0;
	// Check if button is high or low for the moment
	uint8_t current_state = (~BUTTON_PIN & BUTTON0_MASK) != 0;
	if (current_state != button_state) {
		// Button state is about to be changed, increase counter
		count++;
		if (count >= 100) {
			// The button have not bounced for four checks, change state
			button_state = current_state;
			// If the button was pressed (not released), tell main so
			if (current_state != 0) {
				button0_down = 1;
			}
			count = 0;
		}
		} else {
		// Reset counter
		count = 0;
	}
}

// ------------ Inicio del programa ----------------------------------------
int main(void)
{
	
	max7219_init(); // llamado de la funcion "max7219_init"
	update_now(x,y);
	image(menuLevel1);  // carga la imagen a visualizar
	update_display();
	while(1)  // loop infinito
	{
		debouncebtn6();
		debouncebtn7();
		debouncebtn1();
		debouncebtn2();
		debouncebtn0();
		if(button6_down)
		{
			if(state==0){
				image(start);  // carga la imagen a visualizar
				state = 1;
			}
			else if(state == 1){
				image(menuLevel1);
				state = 2;
			}
			else if(state == 2){
				//iniciar nivel seleccionado
				//PAsar a estados de juego
			}
			update_display();
			button6_down = 0;
		}
		
		if(button7_down)
		{
			if(menu>1){
				menu = menu-1;
			}
			if(menu==1){image(menuLevel1);}
			else if(menu == 2){image(menuLevel2);}
			else if(menu == 3){image(menuLevel3);}
			else if(menu == 4){image(menuLevel4);}
			update_display();
			button7_down = 0;
		}		
		if(button1_down)
		{
			if(menu>1){
				menu = menu-1;
			}
			if(menu==1){image(menuLevel1);}
			else if(menu == 2){image(menuLevel2);}
			else if(menu == 3){image(menuLevel3);}
			else if(menu == 4){image(menuLevel4);}
			update_display();
			button1_down = 0;
		}
		
		if(button2_down)
		{
			if(menu<4){
				menu = menu+1;
			}
			if(menu==1){image(menuLevel1);}
			else if(menu == 2){image(menuLevel2);}
			else if(menu == 3){image(menuLevel3);}
			else if(menu == 4){image(menuLevel4);}
			update_display();
			button2_down = 0;
		}
		
		if(button0_down)
		{
			if(state==0){
				image(start);  // carga la imagen a visualizar
				state = 1;
			}
			else if(state == 1){
				image(menuLevel1);
				state = 2;
			}
			else if(state == 2){
				//iniciar nivel seleccionado
				//PAsar a estados de juego
			}
			update_display();
			button0_down = 0;
		}
	}
}
