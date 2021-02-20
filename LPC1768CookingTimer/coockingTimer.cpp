/*******************************************************************************
Name:           CookingTimer.cpp
Author:         DTsebrii
Date:           02/18/2021
Description:    A project to read an user input, convert adc value to a specific 
time, and wait for it. The end of a timer will be signilized with a enabled 
buzzer.
Used pins:
Serial communication    - USBTX and USBRX (Made for troubleshooting)
Potentiometer           - Pin 15(Analog IN)
Push button             - Pin 16 (InterruptIn)
LED                     - LED1
Buzzer                  - Pin 17 (Digital OUT) 
*******************************************************************************/
/// LIBRARIES //////////////////////////////////////////////////////////////////
#include "mbed.h"
#include "platform/mbed_thread.h"
#include "Timer.h"

/// CONSTANTS //////////////////////////////////////////////////////////////////
#define TRUE    1
#define FALSE   0
#define SEC     60
#define MIN     60
#define DAY     24          // Added for future use
#define BAUD    9600        // Default baudrate

/// PINS ///////////////////////////////////////////////////////////////////////
Timer       timer;          // Time counter
Serial      pc(USBTX, USBRX, BAUD);
DigitalOut  buzzer(p17);
DigitalOut  led(LED1);
AnalogIn    adc(p15);       // ADC pin to read an user input
InterruptIn button(p16);    // Works as soon as decission is made
/// GLOBAL VARIABLES ///////////////////////////////////////////////////////////
volatile char decisionFlag = FALSE;  // Checks either user presses pushbotton

typedef struct tmr 
{
    float timeLim;      // Var to save the alarm time
    float currTime;     // Var to save the current time
    char sec;           // Secundo Counter
    char min;           // Minute Counter
    char hour;          // Hour Counter
    char buzzState;     // Buzzer controller
} tmr_t;
tmr_t alarmClock;
/// FUNCTIONS //////////////////////////////////////////////////////////////////
/*******************************************************************************
Name:           initTMR
Author:         DTsebrii
Date:           02/20/2021
Input:          tptr - tmr_t object pointer
Output:         None
Description:    Initialize the timer object
*******************************************************************************/
void initTMR(tmr_t* tptr)
{
    tptr->timeLim = FALSE;
    tptr->currTime = FALSE;
    tptr->sec = FALSE;
    tptr->min = FALSE;
    tptr->hour = FALSE;
    tptr->buzzState = FALSE;
    
} // eo initTMR::
/*******************************************************************************
Name:           initSys
Author:         DTsebrii
Date:           02/20/2021
Input:          None
Output:         None
Description:    Invoke all system configuration functions within the main ruitine
*******************************************************************************/
void initSys()
{
    timer.start();
    initTMR(&alarmClock);
}  // eo initSys::

/*******************************************************************************
Name:           pbPressed
Author:         DTsebrii
Date:           02/20/2021
Input:          None
Output:         None
Description:    ISR that occurs during a PB press. It handles the following 
                procedures:
                Controlls the buzzer state
                Controlls the decisionFlag state
*******************************************************************************/
void pbPressed()
{
    if(alarmClock.buzzState)
    {
        initTMR(&alarmClock);   // Reinitialize the timer parameters
        buzzer = FALSE;
    }
    decisionFlag = !decisionFlag;
}  // eo pbPressed::

/*******************************************************************************
Name:           countTime
Author:         DTsebrii
Date:           02/20/2021
Input:          None
Output:         None
Description:    Function to controll the time flow
*******************************************************************************/
void countTime(tmr_t* tptr)
{
    led = FALSE;
    tptr->sec++;
    if(tptr->sec >= SEC)
    {
        tptr->sec = 0;
        tptr->min++;
    }
    if(tptr->min >= MIN)
    {
        tptr->min = 0;
        tptr->hour >= DAY ? tptr->hour = 0: tptr->hour++;    
    }
    tptr->currTime = tptr->min;
    tptr->currTime += (float)tptr->sec/100;
    if(tptr->currTime >= tptr->timeLim) tptr->buzzState = TRUE;
    buzzer = tptr->buzzState;
}  // eo countTime::

/*******************************************************************************
Name:           readTime
Author:         DTsebrii
Date:           02/20/2021
Input:          None
Output:         None
Description:    Function to read and interpret the ADC value
*******************************************************************************/
void readTime()
{
    float floatPoint = 0;
    led = TRUE;    
    alarmClock.timeLim = (int)(adc*MIN);
    floatPoint = adc*MIN - (int)(adc*MIN);
    floatPoint *= SEC;
    floatPoint = floatPoint/100;
    alarmClock.timeLim += floatPoint; // Geting a sec part
}  // eo readTime

/*******************************************************************************
Name:           display
Author:         DTsebrii
Date:           02/20/2021
Input:          None
Output:         None
Description:    Function to dsipplay all variables. Needed for debuging. Should
                be commented during normal program work
*******************************************************************************/
void display(){
     float adcVal = adc*MIN;
     pc.printf("\033[2J\033[1;0HThe Clock Parameters...\n");
     pc.printf("\033[2;0HClock is in the following format MM:SS\n");
     pc.printf("\033[3;0HClock Counter:\t%02d:%02d\n", alarmClock.min, alarmClock.sec);
     pc.printf("\033[4;0HReal ADC Value:\t%f\n", adcVal);
     pc,printf("\033[5;0HTime to wait:\t%f\n", alarmClock.timeLim);
     pc.printf("\033[6;0HTime value:\t%f", alarmClock.currTime);
     pc.printf("\033[7;0HBuzzer State:\t%d\n", alarmClock.buzzState);
     pc.printf("\033[8;0HDecisionFlag:\t%d\n", decisionFlag);
     
} // eo display::

int main()
{
    initSys();
    button.rise(&pbPressed);
    while (TRUE) {
        if(timer >= 1.0)
        {
            display();
            timer.reset();
            if(decisionFlag)
            {
                countTime(&alarmClock);
               
            }
            else
            {
                readTime();
            }    
        }
    }
}
