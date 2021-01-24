// &&& Cooking Timer &&&&&&&&&&&&&&&&&&&&&&&
/********************************************************
File name:		Cooking_timer.c
Author:			DTsebrii
Date:			04/OCT/2020
Modified:		DTsebrii 22/JAN/2021 
Description:	The source code for a cooking timer. During
				its routine microcontroller waits a required 
				amount of time and then provides a sizgnal 
				for a customer
********************************************************/
// *** Libraries ****************************************
#include <stdlib.h>
#include <stdio.h>
#include <p18f45k22.h>

// *** Constants ***************************************
#define TRUE	1
#define FALSE	0
// Timer Constants 
#define TMR0FLAG INTCONbits.TMR0IF
#define HIGHBIT	0x0B
#define LOWBIT	0xDC
#define MINUTE	60
#define HOUR	60
// Hardware Constants 
#define LED		PORTAbits.RA2 // Third pin is for LED
#define PUSHB	PORTAbits.RA1 // Second pin is for PB
#define BUZZER	PORTAbits.RA3 // Fourth pin is for buzzer 
// ADC constants
#define ADCMASK	0x83
#define STEP	34 // Maximum allowed time = 60 min; 1024/60 is 34
// Object constant
#define TIMECHAN	0x00

// Interrupt Routine
#define TWOBITSON	0xC0

void ISR (); //calling the prototype for an interrupt function
#pragma code int_vector = 0x008
//*** int_vector*********************************************
/*Author:	CTalbot
Date: 		14/MAR/2020
Description: Calling the ISR function
Input:		None
Output:		None
*************************************************************/
void int_vector (void)
{
	_asm
		GOTO ISR
	_endasm
}//eo int_vector::
#pragma code
// *** Global variables *********************************
/*********************
Structure to keep all 
variables connected.
min & sec - time vars;
ledState - turns on and
of the LED;
pbState - Saves the 
pushbutton state
********************/
typedef struct system
{
	char min;
	char sec;
	char ledState;  // LED to be on when while user inputs his value 
	char pbState; // Flag to start counting time/check user input
}sys_t;
sys_t cookTimer;

char timeFlag = FALSE;
char lastState = FALSE;  // To latch a PB
char time = 0; // Time to be counted 
// *** Functions ****************************************
/*** initObj *******************************************************************************
Author:			DTsebrii
Date:			23/APR/2020
Modified:		None
Description:	Function to initialize the data structure object
Input:			sptr pointer to data structre object
Output:			None
*********************************************************************************************/
void initObj( sys_t *cptr )
{
	cptr->min = FALSE;
	cptr->sec = FALSE;
	cptr->ledState = TRUE; // LED is turned on when 
	cptr->pbState = FALSE;
}  // eo initObj::

/*** configInterrupt *******************************************************************************
Author:		DTsebrii
Date:		14/MAR/2020
Modified:	23/APR/2020
Description:Function to set the INTCON register parameters
Input:		None
Output:		None
*********************************************************************************************/
void configInterrupt (void)
{
	//Config TMR0 interrupt
	INTCON |= 0x20;
	INTCON2 &= 0xFB;
	//Global Interrupts
	INTCON |= TWOBITSON;  // 0xC0 
}// eo configInterrupt::

/*****************************************************************
Name:			resetTMR0
Author:			CTalbot
Date:			26/MAR/2020
Description: 	Reset timer0 and writes the preset value into the 
					count register. 
Input:			int preset, the preset count as a 2 byte value
Output:			None
*****************************************************************/
void resetTMR0 ( char lowBit, char highBit )
{
	TMR0FLAG =	FALSE;
	TMR0H =		highBit;
	TMR0L =		lowBit;
}  // eo resetTMR0:: 

/*** timerSetup *******************************************************************************
Author:		DTsebrii
Date:		22/JAN/2021
Description:Function to set the TMR0 parameters
Input:		char lowBit and highBit - TMRH and TMRL bits value
Output:		None
*********************************************************************************************/
void timerSetup ( char lowBit, char highBit )
{
	T0CON = 	0x93; //t0 enabled, 16 bit, clkout, 16 psv
	TMR0H = 	highBit;
	TMR0L = 	lowBit;
	TMR0FLAG = 	FALSE; //Turn back to the zero after event happens
}// eo timerSetup::


/*** oscSetup *******************************************************************************
Author:		DTsebrii
Date:		22/JAN/2021
Description:Function to set the oscilograph parameters
Input:		None
Output:		None
*********************************************************************************************/
void oscSetup (void)
{
	OSCCON = 0x52;  // Fosc = 4MHz
	OSCCON2 = 0x04;
	OSCTUNE = 0x80;
	while( OSCCONbits.HFIOFS!=1 );
}// eo oscSetup::

/*** adcSetup *******************************************************************************
Author:		DTsebrii
Date:		12/MAR/2020
Description:Function to set the ADC parameters
Input:		None
Output:		None
*********************************************************************************************/
void adcSetup (void)
{
	ADCON0|= 0x01;// ADC enable 
	ADCON1 = 0x00;
	ADCON2 = 0xA9;// 12 TAD right just F/8
}// eo adcSetup::


/*** configADC *******************************************************************************
Author:		DTsebrii
Date:		30/MAR/2020
Description:Function to configurate the required ADC parameters 
Input:		unsigned char tad, actual aquisition time
				0 is 0 TAD, 20 is 
Output:		None
*********************************************************************************************/
void configADC ( unsigned char tad, unsigned char cnvrClckSlct )
{
	ADCON0 |= 0x01;  // ADC enabled
	ADCON1 = 0x00;
	ADCON2 = 0x80;  // Right justified
	switch ( tad )
	{
		case 0:
			break; // ADCON2 is already set for 0TAD
		case 2:
			ADCON2 |= 0x08;
			break;
		case 4:
			ADCON2 |= 0x10;
			break;
		case 6:
			ADCON2 |= 0x18;
			break;
		case 8:
			ADCON2 |= 0x20;
			break;
		case 12:
			ADCON2 |= 0x28;
			break;
		case 16:
			ADCON2 |= 0x30;
			break;
		case 20:
			ADCON2 |= 0x38;
			break;
		default:
			printf((const rom far char *)"Wrong TAD. Check ADCCON2\n\r");
			break;
	} // eo TAD switch
	switch ( cnvrClckSlct )
	{
		case 2:
			break; // ADCON2 is already set for Fosc/2
		case 8:
			ADCON2 |= 0x01;  // Fosc/8
			break;
		case 32:
			ADCON2 |= 0x02;
			break;
		case 128:
			ADCON2 |= 0x03;  // Internal osc 600 kHz nominal
			break;
		case 4:
			ADCON2 |= 0x04;
			break;
		case 16:
			ADCON2 |= 0x05;
			break;
		case 64:
			ADCON2 |= 0x06;
			break;
		default:
			printf((const rom far char *)"Wrong Fosc/N. Check ADCCON2\n\r");
			break;
	}  // eo cnvrClckSlct switch
}// eo configADC::

/*** configPORTs *******************************************************************************
Author:			DTsebrii
Date:			12/MAR/2020
Modified:		DTsebrii, 30/MAR/2020
Description:	Function to set the ports parameters
Input:			None
Output:			None
*********************************************************************************************/
void configPORTs()
{
	//PORTA
	ANSELA =	0x01;  // RA0 is temperature sensor (Analog) pb and LED is digital
	LATA =		0x00;  // No input voltage
	TRISA = 	0xF3;  //Buzzer and LED is output (RA2&RA3)
	//PORTB
	ANSELB = 	0x00;
	LATB = 		0x00;
	TRISB = 	0xFF;
	//PORTC
	ANSELC = 	0x00;
	LATC = 		0x00;
	TRISC = 	0xFF;
	//PORTD
	ANSELD = 	0x00;
	LATD = 		0x00;
	TRISD = 	0xFF;
	//PORTE
	ANSELE = 	0x00;  
	LATE = 		0x00;
	TRISE = 	0xFF;
}// eo configPORTs::

/*** configSys *******************************************************************************
Author:			DTsebrii
Date:			12/MAR/2020
Modified:		DTsebrii, 30/MAR/2020
Description:	Function to call all initialztional functions
Input:			None
Output:			None
*********************************************************************************************/
void sysSetup()
{
	// Hardware functions 
	oscSetup();  // 4 MHz 
	timerSetup( LOWBIT, HIGHBIT );  // 1sec, 16PSV
	configPORTs();
	adcSetup();
	configInterrupt();
	
	// Software functions 
	initObj( &cookTimer );
} // eo configSys::

/*****************************************************************
Name:			sampADC
Author:			DTsebrii
Date:			23/APR/2019
Description: 	Read the ADC value
Input:			chID, char that represents ADC channel
Output:			ADRES/STEP time (60 min amx)
*****************************************************************/
char sampADC( char chID )
{
	ADCON0&= ADCMASK;
	ADCON0|=  chID<<2;
	ADCON0bits.GO = TRUE;
	while( ADCON0bits.GO );
	
	return ADRES/STEP; // Getting a time value 
	
}  // eo sampADC::

#pragma interrupt ISR
/*** ISR *******************************************************************************
Author:		CTalbot
Date:		14/MAR/2020
Modifier:	DTsebrii, 23/APR/2020
Description:Reseting TMR0
Input:		None
Output:		None
*********************************************************************************************/
void ISR()
{
	if( TMR0FLAG )
	{
		resetTMR0( LOWBIT, HIGHBIT );
		timeFlag = TRUE;
	}
	INTCON |= TWOBITSON;
}  // eo ISR::

void main ()
{
	sysSetup();
	while (TRUE)
	{
		// Geting time 
		if (!cookTimer.pbState)  // When pushbutton is not pressed 
		{
			time = sampADC(TIMECHAN);
			LED = TRUE;
			BUZZER = FALSE; // Turn on buzzer when it is not needed
			if (PUSHB == FALSE && !lastState)
			{
				lastState = TRUE;
				cookTimer.pbState = !cookTimer.pbState; // Pushbutton is pressed and time is decided
			}
			else if (PUSHB) 
				lastState = FALSE; // Reset a lastState
		}
		else
		{
			// counting time 
			if (timeFlag)
			{
				timeFlag = !timeFlag;
				LED = FALSE; // Turn off LED 
				cookTimer.sec++;
				if ( cookTimer.sec == MINUTE )
				{
					cookTimer.sec = 0;
					cookTimer.min++;
				}
				if ( cookTimer.min >= time )
				{
					BUZZER = TRUE; // Timer is completed
				}
				if (BUZZER)
				{
					if (PUSHB == FALSE && !lastState)
					{
						cookTimer.pbState = !cookTimer.pbState;
						lastState = TRUE;
						BUZZER = FALSE;
						initObj( &cookTimer );
					}
					else if (PUSHB) 
						lastState = FALSE; // Reset a lastState
				}
			} // eo timeFlag if/else  
		}
	} // eo while  
}
