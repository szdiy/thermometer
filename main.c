/**************************************************************************
* AVR temp sensor
*
* copyright 2008 Michael Spiceland (mikeNOSPAM@fuzzymonkey.org)
*                Anders Ma (xuejiao.ma@gmail.com)
* 
**************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* defines */
#define COMMON_POSTIVE 1
//#define DEBUG

#ifdef DEBUG

#define SERIAL_OUTPUT
#define UART_BAUD_RATE 1200
#define UART_BAUD_CALC(UART_BAUD_RATE,F_OSC) ((F_OSC)/((UART_BAUD_RATE)*16l)-1)

#define SERIAL_TX_MAX_WAIT_MS 1
#define SERIAL_TX_OK 0
#define SERIAL_TX_FAIL 1

#endif //#ifdef DEBUG

/*
	PORTB is connected with 7-segment LED tube
      -a- 
    e|   |b
      _f_ 

    d|   |c
      _c_  . dp
*/
#define PBDIGTAL0      0X3f
#define PBDIGTAL1      0X06
#define PBDIGTAL2      0X5b
#define PBDIGTAL3      0X4f
#define PBDIGTAL4      0X66
#define PBDIGTAL5      0X6d
#define PBDIGTAL6      0X7d
#define PBDIGTAL7      0X07
#define PBDIGTAL8      0X7f
#define PBDIGTAL9      0X6f
#define PBDIGTALDOT    0X80


/* globals */
uint8_t negtive_temperature = 0;
uint8_t temperature_string[] = "--.-";

uint8_t digitalarray[11] = {
	PBDIGTAL0,
	PBDIGTAL1,
	PBDIGTAL2,
	PBDIGTAL3,
	PBDIGTAL4,
	PBDIGTAL5,
	PBDIGTAL6,
	PBDIGTAL7,
	PBDIGTAL8,
	PBDIGTAL9,
	PBDIGTALDOT
};

/* function declaration */

#ifdef SERIAL_OUTPUT
void serial_init(void);
void serial_tx(uint8_t byte);
void serial_tx_hex(uint8_t byte);
void serial_tx_string(uint8_t* string);
#endif

void system_init(void);
void adc_init(void);
void timer_init(void);
void led_init(void);
void refresh_led(void);
void main_loop(void);

void delay_ms(unsigned int);

double getTempF(double v10bit, double pdRes);
void double2string (double actualTemp, uint8_t* string);


SIGNAL(SIG_OUTPUT_COMPARE0A) 
{
	refresh_led();
}

#ifdef SERIAL_OUTPUT
void serial_init(void)
{
	// set baud rate
	UBRR0H = (uint8_t)(UART_BAUD_CALC(UART_BAUD_RATE, F_CPU)>>8);
	UBRR0L = (uint8_t)UART_BAUD_CALC(UART_BAUD_RATE, F_CPU);

	// enable receiver and transmitter; enable RX interrupt
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

	//asynchronous 8N1
	UCSR0C = (3 << UCSZ00);
}

void serial_tx(uint8_t byte)
{	
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = byte;
}

void serial_tx_hex(uint8_t byte)
{
	uint8_t hh, hl;

	hh = (byte >> 4) & 0xF0;
	hl = byte & 0x0F;
	
	// send high four bits
	if (0 <= hh && hh <= 9)
		serial_tx('0'+hh);			
	else
		serial_tx('a'+hh-10);

	// send low four bits
	if (0 <= hl && hl <= 9) 
		serial_tx('0'+hl);			
	else
		serial_tx('a'+hl-10);
}

void serial_tx_string(uint8_t* string)
{
	int i = 0;

	while (string[i]) {
		serial_tx(string[i]);
		i++;
	}
}
#endif

void delay_ms(unsigned int ms)
{
	_delay_ms(1);
}

void system_init(void) 
{
		// set system clock to 1MHZ
		// Clock Division Factor : 8 (0011b)
        CLKPR = _BV(CLKPCE); // change enable
        CLKPR |= _BV(CLKPS1) | _BV(CLKPS0); 
}

void adc_init(void)
{
	// disable ADC1-5 digit input, only enable ADC0
	DIDR0 = _BV(ADC5D) | _BV(ADC4D) | _BV(ADC3D) | _BV(ADC2D) | _BV(ADC1D);
	// voltage Reference : AVCC with external capacitor at AREF pin
	ADMUX = _BV(REFS0);
	// enable ADC
	ADCSRA |= _BV(ADEN);
	// Division Factor : 64
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1);

}

void timer_init(void)
{
	// use timer0 to refresh LED
	// timer0 interrupt freq is 1/10kHZ (every 10ms)
	TCCR0A = _BV(WGM01); // clear Timer on Compare Match (CTC) mode
	TCCR0B = _BV(CS11) | _BV(CS10); // 1M/64 = 16KHz
	OCR0A = 160; // 16kHZ/160 = 1/10KHz
	TIMSK0 = _BV(OCIE0A); // turn on output compare match A interrupt

/*
	TCNT0 = 0; //reset the timer
	TIMSK0 |= _BV(TOIE0); // enable timer/counter1 overflow interrupt
	TCCR0B |= _BV(CS00) | _BV(CS01); 
*/
}

void led_init(void) {
		DDRB = 0XFF; // ALL pin are under output mode
		PORTB = 0X0;

		// D2~4 are under output mode
		DDRD |= _BV(DDD2) | _BV(DDD3) | _BV(DDD4); 

		PORTD &= ~_BV(PD3); //left seg
		PORTD &= ~_BV(PD2); //middle seg
		PORTD &= ~_BV(PD4); //right seg
}

void refresh_led(void)
{
	uint8_t i = 0;
	uint8_t index;


#if COMMON_POSTIVE // common negtive
	while (temperature_string[i])
	{
		switch (i) {
		case 0:
			PORTD &= ~(1<<PD3); //left
			PORTD &= ~(1<<PD4); //right
			PORTD |= (1<<PD2); //middle
			//PORTB = digitalarray[k%11];
			break;
		case 1:
			PORTD &= ~(1<<PD2); //middle
			PORTD &= ~(1<<PD4); //right
			PORTD |= (1<<PD3); //left
			//PORTB = digitalarray[k%11];
			break;
		case 2:
			i++;
			continue;
			break;
		case 3:
			PORTD &= ~(1<<PD2); //middle
			PORTD &= ~(1<<PD3); //left
			PORTD |= (1<<PD4); //right
			//PORTB = digitalarray[k%11];
			break;
		}
		index = temperature_string[i] - 0X30;
		if (index >= 0 && index <= 10) {
			PORTB = ~digitalarray[index];
			if ((i == 1) ||
			    (i == 3 && negtive_temperature == 1)){
				PORTB &= ~PBDIGTALDOT;
			}
		}
		i++;
		delay_ms(2);
	}

	PORTD &= ~(1<<PD3); //left
	PORTD &= ~(1<<PD2); //middle
	PORTD &= ~(1<<PD4); //right
#else
	while (temperature_string[i])
	{
		switch (i) {
		case 0:
			PORTD |= (1<<PD3); //left
			PORTD |= (1<<PD4); //right
			PORTD &= ~(1<<PD2); //middle
			//PORTB = digitalarray[k%11];
			break;
		case 1:
			PORTD |= (1<<PD2); //middle
			PORTD |= (1<<PD4); //right
			PORTD &= ~(1<<PD3); //left
			//PORTB = digitalarray[k%11];
			break;
		case 2:
			i++;
			continue;
			break;
		case 3:
			PORTD |= (1<<PD2); //middle
			PORTD |= (1<<PD3); //left
			PORTD &= ~(1<<PD4); //right
			//PORTB = digitalarray[k%11];
			break;
		}
		index = temperature_string[i] - 0X30;
		if (index >= 0 && index <= 10) {
			PORTB = ~digitalarray[index];
			if ((i == 1) ||
			    (i == 3 && negtive_temperature == 1)){
				PORTB |= PBDIGTALDOT;
			}
		}
		i++;
		delay_ms(2);
	}
	PORTD |= (1<<PD3); //left
	PORTD |= (1<<PD2); //middle
	PORTD |= (1<<PD4); //right
#endif

}

/***************************************************************************
* double2string
* convert a double to a string and place it in a pre-allocated space
***************************************************************************/
void double2string (double actualTemp, uint8_t* string)
{
	int temp;

	cli(); // atomic operation

	/* prep the string */
	string[2] = '.';
	string[4] = '\0';

	//to include decimal point for display
	temp=(int16_t)(actualTemp * 10.0);   
	if((actualTemp*10.0 - temp) >= 0.5) temp=temp+1;

	if(temp < 0) {
		temp *= -1;
		negtive_temperature = 1;
	} else {
		negtive_temperature = 0;	
	}
	
	string[3] = ((uint8_t)(temp%10)) | 0x30;
	temp=temp/10;
	
	string[1] = ((uint8_t)(temp%10)) | 0x30;
	temp=temp/10;
	
	string[0] = ((uint8_t)(temp%10)) | 0x30;
	temp=temp/10;
	
	sei();
}


/*************************************************************************
* getTempF - return the temp in Farenheight from 
*            Vishay thermistor NTCLE100E3103JB0
* v10bit - 10 bit value read from the A/D
* pdRes - value (in ohms) of the resistor that is in series with thermistor
*************************************************************************/
double getTempF(double v10bit, double pdRes)
{
	if (v10bit == 1024)
		return -1;

	double thermResistance = (pdRes * v10bit / 1024.0)/
				  (1.0 - (v10bit/1024.0));
	//return thermResistance;

	double thermRefResistance = 10000.0;

	// Steinhart and Hart constants for Vishay thermistor NTCLE100E3103JB0
	double a = 3.354016 * pow(10, -3);
	double b = 2.56985 * pow(10, -4);
	double c = 2.620131 * pow(10, -6);
	double d = 6.383091 * pow(10, -8);

	double celcius = 1.0/(a + b * log(thermResistance/thermRefResistance) +
		c * pow(log(thermResistance/thermRefResistance), 2) + 
		d * pow(log(thermResistance/thermRefResistance), 3)) - 272.15;

	double farenheit = 9.0/5.0 * celcius + 32.0;

	//return -78.9;
	return celcius;
	return farenheit;
}

void main_loop(void)
{
	double temperature_pre;	
	double temperature;	
	double current;
#ifdef SERIAL_OUTPUT
	uint8_t separator[] = " ";
#endif

	temperature_pre = temperature = 0.0;

	while(1)
	{
		ADCSRA |= (1 << ADSC); // start ADC conversion
		while (ADCSRA & (1 << ADSC)); // wait for the result
		current = (uint16_t)(ADCL | (ADCH << 8));
		temperature = getTempF(current, 9980);
		if ((temperature - temperature_pre) > 0.20 || 
			(temperature_pre - temperature) > 0.20) {
			double2string(temperature, temperature_string);
			temperature_pre = temperature;
#ifdef SERIAL_OUTPUT
			serial_tx_string(temperature_string);
			serial_tx_string(separator);		
#endif
		}
		//delay_ms(100);
	}
}

int main(void)
{
	// do device initialization
	system_init();
	adc_init();
	timer_init();
#ifdef SERIAL_OUTPUT
	serial_init();
#endif	
	led_init();
	
	// enable interrupt
	sei();

	main_loop();

	return 0;
} 


