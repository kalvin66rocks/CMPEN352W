
//----------------------------------------------
// Name:	Coulston
// Date:	Spring 2016
// Purp:	Pi shield attached to Raspberry Pi
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	THIS PROGRAM WILL ONLY WORK WHEN THE PI'16 DEV BOARD IS CONNECTED TO THE RPI AND
//	A MINTERM.PY TERMINAL IS RUN ON THE RPI.  THIS PROGRAM WILL NOT WORK IN STANDALONE
//	MODE BECAUSE ALL I/O IS DIRECTED TO SERIAL PORT #1.
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
//----------------------------------------------
#include <p18F26k22.h>          // So we can directly use "TMR3"
#include <stdio.h>
#include <xc.h>

/* Set up the configuration bits */
#pragma config FOSC = INTIO67
#pragma config PLLCFG = ON			// Multiply the internal oscillator speed by 4
#pragma config PRICLKEN = OFF		// Primary clock enabled
#pragma config FCMEN = OFF			// Fail-Safe Clock Monitor enabled
#pragma config IESO = OFF			// Oscillator Switchover mode disabled
#pragma config PWRTEN = OFF			// Power up timer disabled
#pragma config BOREN = OFF			// Brown-out Reset disabled in hardware and software 
#pragma config BORV = 285			// VBOR set to 2.85 V nominal 
#pragma config WDTEN = OFF			// Watch dog timer is always disabled. SWDTEN has no effect. 
#pragma config WDTPS = 1			// 1:1 Watchdog timer postscalar 			
#pragma config CCP2MX = PORTC1		// CCP2 input/output is multiplexed with RC1  
#pragma config PBADEN = OFF			// PORTB<5:0> pins are configured as digital I/O on Reset 
#pragma config CCP3MX = PORTB5		// P3A/CCP3 input/output is multiplexed with RB5  
#pragma config HFOFST = OFF			// HFINTOSC output and ready status are delayed by the oscillator stable status  
#pragma config T3CMX = PORTC0		// T3CKI is on RC0  
#pragma config P2BMX = PORTC0		// P2B is on RC0 
#pragma config MCLRE = EXTMCLR		// MCLR pin enabled, RE3 input pin disabled  
#pragma config STVREN = OFF			// Stack full/underflow will not cause Reset 
#pragma config LVP = OFF			// Single-Supply ICSP disabled 
#pragma config XINST = OFF			// Instruction set extension and Indexed Addressing mode disabled (Legacy mode) 
#pragma config DEBUG = ON			// Block 0 (000800-003FFFh) code-protected 

typedef unsigned char int8;
typedef unsigned int  int16;
typedef unsigned long int32;

void INIT_PIC (void);

#define	TRIS_RX1_PIN		TRISCbits.TRISC7
#define	ANSEL_RX1_PIN		ANSELCbits.ANSC7
#define ANALOG	1
#define DIGITAL	0
#define OUTPUT	0
#define INPUT	1

//----------------------------------------------
// Main "function"
//----------------------------------------------
void main (void) {

	int8 i;

	INIT_PIC();

    
    TMR3H = 0x00;	TMR3L = 0x00; while (!PIR2bits.TMR3IF); PIR2bits.TMR3IF = 0;
	
    for (i=0; i<20; i++) 	printf("\r\n");
	printf("Pi16 Board\r\n");
	printf("Test Program for RPi\r\n");
	printf("Spring 2016\r\n");


    
    
	while(1) {

		if (PIR1bits.RC1IF) {
			PIR1bits.RC1IF = 0;
			switch (RCREG1) {             

			//--------------------------------------------
			// Reply with help menu
			//--------------------------------------------
			case '?':
				for (i=0; i<20; i++) 	printf("\r\n");
				printf("?: help menu\r\n");
				printf("o: k\r\n");
				printf("m: display microphone level\r\n");
				printf("l: toggle LED\r\n");		
				break;

			//--------------------------------------------
			// Reply with "k", used for PC to PIC test
			//--------------------------------------------
			case 'o':
				printf("o:	ok\r\n");
				break;
				
			//--------------------------------------------
			// Display a microphone samples
			//--------------------------------------------
			case 'm':
                ADCON0bits.ADON = 0;
                ADCON1 = 0b00000000;	// AN0 and Vcc and GND as positive and negative reference voltages
                ADCON0 = 0b00010000;	// Select channel RA5/AN4
                ADCON2 = 0b00101110;	// 12Tad acquisition time.  Use Fosc/64 for 1uS sampling clock
                ADCON0bits.ADON = 1;	// Turn on ADC
				ADCON0bits.GO_NOT_DONE = 1;		// start a new conversion
				while (	ADCON0bits.GO_NOT_DONE);		
				printf("%d",ADRESH);
				break;
				
			//--------------------------------------------
			// Toggle LED on RB5 to say hello
			//--------------------------------------------
			case 'l':
				LATBbits.LATB5 ^= 1;
				break;

			//--------------------------------------------
			// Just in case you leave cap-locks on....
			//--------------------------------------------
			default:
				printf("unrecognized character %c\r\n",RCREG1);

}	}	}	}

//----------------------------------------------
// INIT_PIC
// Initializes the PIC to run at 64MHz
// Setup TMR0 to roll over without interrupts
// Setup some of the GPIO pins attached to devices
// Setup a terminal interface 9600 on COM1
//----------------------------------------------
void INIT_PIC (void) {

	// ---------------Configure Oscillator------------------
	OSCCONbits.IRCF2 = 1;		// Internal RC Oscillator Frequency Select bits
	OSCCONbits.IRCF1 = 1;		// Set to 16Mhz
	OSCCONbits.IRCF0 = 1;		//
	OSCTUNEbits.PLLEN = 1;		// enable the 4xPLL, wicked fast 64Mhz

	TRISBbits.TRISB5 = 0;

	//----------Initialize the MIC--------------------
	TRISAbits.TRISA5 = 1;
	ANSELAbits.ANSA5 = 1;
	ADCON0bits.ADON = 0;
	ADCON1 = 0b00000000;	// AN0 and Vcc and GND as positive and negative reference voltages
	ADCON0 = 0b00010000;	// Select channel RA5/AN4
	ADCON2 = 0b00101110;	// 12Tad acquisition time.  Use Fosc/64 for 1uS sampling clock
	ADCON0bits.ADON = 1;	// Turn on ADC
	
	// configure timer3 as a vanilla 1:1 prescaled 8-bit timer
	T3CONbits.TMR3CS1 = 0;		// Clock source is Fosc/4
	T3CONbits.TMR3CS0 = 0;		// Clock source is Fosc/4
	T3CONbits.T3CKPS1 = 1;		// Prescale at 1:1
	T3CONbits.T3CKPS0 = 1;
	T3CONbits.T3RD16 = 1;		// R/W in 16-bit TMR3H is buffered, TMR3L is real
	T3CONbits.TMR3ON = 1;		// Turn timer on
	TMR3H = 0xE0;
	TMR3L = 0x00;

	// ---------------Setup the serial port------------------
	// Aiming for a baud rate of 9600
	// BAUD = FOSC/[64*(SPBRGH+1)]
	// SPBRGH = 64Mhz/ 9.6k/64 =  104
	TRIS_RX1_PIN = INPUT;
	ANSEL_RX1_PIN = DIGITAL;
	TXSTA1bits.TXEN = 1;
	TXSTA1bits.SYNC = 0;
	TXSTA1bits.BRGH = 0;
	BAUDCON1bits.BRG16 = 0;
	RCSTA1bits.CREN = 1;
	SPBRG1 = 104;
	RCSTA1bits.SPEN = 1;

	
}

//-----------------------------------------------------------------------------
// Helper function needed to point PRINTF to the first USART connected to RPi
//-----------------------------------------------------------------------------
void putch(char c) {
    
    while( ! TX1IF)
        continue;
    TX1REG = c;

}