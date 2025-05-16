#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "eeprom.h"
#include "uart0.h"
#include "uart4.h"
#include "stringf.h"
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "rgb_led.h"

// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))      // PF1
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))      // PF2
#define GREEN_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))     // PF3
#define DATA_ENABLE  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))      // PC6
//#define DATA         (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))    // PC5
//#define RECEIVER     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))    // PC4

// Macros
#define MAX_ADDRESS 512
#define MODE_ADDRESS 0
uint16_t DEFAULT_ADDRESS = (uint16_t) 1;
uint32_t two = (uint32_t) 2;

// PortF masks
#define RED_LED_MASK        2               // Activity LED
#define BLUE_LED_MASK       4               // Transmit LED
#define GREEN_LED_MASK      8
#define DATA_ENABLE_MASK    64
//#define DATA_MASK           32
//#define RECEIVER_MASK       16

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
uint32_t mode, defAdd, time;
uint16_t maxAdd, newAdd, devAdd, ok, add, start, stop, first, last;
uint16_t transmit[MAX_ADDRESS] = {0,}, receive[MAX_ADDRESS] = {0,}, i=0;
uint8_t proceed=0, count=0, increment, k=0, iterations;

// create functions for break, and start timer.
void callBreak()
{
    UART4_LCRH_R |= UART_LCRH_BRK;
}

// Turn Timer1 on function
void timer1On()
{
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on timer
}

// Turn Timer2 on function
void timer2On()
{
    TIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on timer
}

// Turn Timer3 on function
void timer3On()
{
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on timer
}

// Turn Timer1 Off
void timer1Off()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
}

// Periodic Timer Interrupt Service Routine #1
void timer1Isr()
{
    setRgbColor(255,0,0);
    UART4_LCRH_R &= ~(UART_LCRH_BRK);
    waitMicrosecond(12);
    int16_t j;
    while(UART4_FR_R & UART_FR_BUSY);
    UART4_DR_R = 0x00;
    BLUE_LED = 1;
    setRgbColor(255,0,0);
    for(j=0;j<maxAdd;j++)
    {
        while(UART4_FR_R & UART_FR_BUSY);
        UART4_DR_R = transmit[j];
    }
    setRgbColor(0,0,0);
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;      // clear interrupt flag
    if(j>=maxAdd)
    {
        callBreak();
        timer1On();
    }
}

// Periodic Timer Interrupt Service Routine #2
void timer2Isr()
{
    TIMER0_ICR_R = TIMER_ICR_TATOCINT;
    if(count<iterations)
    {
        if(transmit[add]<=stop)
        {
            setRgbColor(transmit[add],0,0);          // this line is just to test if ramp is working on the red board
            transmit[add] = transmit[add]+increment;
            timer2On();
        }
        else
        {
            transmit[add] = start;
            count++;
            timer2On();
        }
    }
    else
    {
        TIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    }
}

// Periodic Timer Interrupt Service Routine #3
void timer3Isr()
{
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
    transmit[add] = last;
    setRgbColor(transmit[add],0,0);
}

// Initialize timer1 hw
void initTimer1Hw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for 1-shot mode (count down)
    TIMER1_TAILR_R = 3680;//3680                          // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC
}

// Initialize timer0 hw, timer0 sub A in hardware but timer 2 notation in software, as per my assumptions
void initTimer2Hw(uint16_t interval)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for 1-shot mode (count down)
    TIMER0_TAILR_R = 40000*interval;                           // set load value to 40e6 for 1 Hz interrupt rate
    TIMER0_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    NVIC_EN0_R = 1 << (INT_TIMER0A-16);              // turn-on interrupt 35 (TIMER0A) in NVIC
}

// Initialize timer3 hw, timer2 sub A in hardware but timer 3 notation in software, as per my assumptions
void initTimer3Hw(uint16_t interval)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for 1-shot mode (count down)
    TIMER2_TAILR_R = 40000*interval;                           // set load value to 40e6 for 1 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt 39 (TIMER0A) in NVIC
}

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Initialize Timer
    initTimer1Hw();

    // Initialize EEPROM
    initEeprom();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    GPIO_PORTC_DIR_R |= DATA_ENABLE_MASK;
    GPIO_PORTC_DEN_R |= DATA_ENABLE_MASK;

    GPIO_PORTF_DIR_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_DEN_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
}

// Controller mode function
void controllerMode(void)
{
    USER_DATA data;
    while(true)
    {
        if(kbhitUart0())
        {
            getsUart0(&data);
            putsUart0(data.buffer);
            putcUart0('\n');
            // parse the fields
            parseFields(&data);
            if(isCommand(&data,"device",1))
            {
                mode = 0;
                setRgbColor(0,0,0);
                uint32_t var =(uint32_t) getFieldInteger(&data, 1);
                writeEeprom(devAdd, var);
                return;
            }
            else
            {
                uint16_t i = 0;
                char printer[50] = {0,};
                putsUart0("Parsed!\r\n");
                if(isCommand(&data, "on", 0))
                {
                    ok = 1;
                    putsUart0("data stream is ON!\n");
                    callBreak();
                    timer1On();
                }
                else if(isCommand(&data, "off", 0))
                {
                    setRgbColor(0,0,0);
                    timer1Off();
                    BLUE_LED = 0;
                    ok = 0;
                    putsUart0("data stream has been turned OFF!\n");
                    return;
                }
                if(ok == 1)
                {
                    if(isCommand(&data, "set", 2))
                    {
                        putsUart0("Set Successful!\r\n");
                        uint16_t add = getFieldInteger(&data, 1);
                        int8_t value = getFieldInteger(&data, 2);
                        transmit[add] = value;
                    }
                    else if(isCommand(&data, "clear", 0))
                    {
                        putsUart0("Clear successful!\r\n");
                        for(i=0;i<MAX_ADDRESS;i++)
                        {
                            transmit[i] = 0;
                        }
                    }
                    else if(isCommand(&data, "get", 1))
                    {
                        putsUart0("Got the address!\r\n");
                        uint16_t add = (uint16_t) getFieldInteger(&data, 1);
                        snprintf(printer, sizeof(printer), "%d", transmit[add]);
                        putsUart0(printer);
                        putcUart0('\n');
                    }
                    else if(isCommand(&data, "max", 1))
                    {
                        maxAdd = getFieldInteger(&data, 1);
                        snprintf(printer, sizeof(printer), "%d", maxAdd);
                        putsUart0("Max address set to:\t");
                        putsUart0(printer);
                        putcUart0('\n');
                    }
                    else if(isCommand(&data, "ramp", 5))
                    {
                        count = 0;
                        uint16_t interval;
                        add = getFieldInteger(&data, 1);
                        time = getFieldInteger(&data, 2);
                        start = getFieldInteger(&data, 3);
                        stop = getFieldInteger(&data, 4);
                        iterations = getFieldInteger(&data, 5);
                        interval = time/10;
                        initTimer2Hw(interval);
                        increment = (stop-start)/10;
                        transmit[add] = start;
                        timer2On();
                    }
                    else if(isCommand(&data, "pulse", 4))
                    {
                        uint16_t interval;
                        add = getFieldInteger(&data,1);
                        time = getFieldInteger(&data,2);
                        first = getFieldInteger(&data,3);
                        last = getFieldInteger(&data,4);
                        interval = time*0.5;
                        initTimer3Hw(interval);
                        transmit[add] = first;
                        setRgbColor(transmit[add],0,0);
                        timer3On();
                    }
                }
            }
        }
    }
}

// Device mode function
void deviceMode()
{

    USER_DATA data;
    while(true)
    {
        if(kbhitUart0())
        {
            getsUart0(&data);
            putsUart0(data.buffer);
            putcUart0('\n');
            // parse the fields
            parseFields(&data);
            if(isCommand(&data, "controller", 0))
            {
                mode = 1;
                writeEeprom(newAdd, mode);
                return;
            }
        }
    }
}

// Uart ISR
void uart4Isr()
{
    if(DATA_ENABLE == 0 && UART_RIS_RXRIS)
    {
        if(UART_DR_BE & UART4_DR_R)
        {
            proceed=1;
            i=0;
        }
        if(proceed==1 && UART4_DR_R & 0xFF == 0x00);
        {
            proceed=2;
        }
        if (proceed==2)
        {
            receive[i]=UART4_DR_R & 0xFF;
            uint32_t defAdd = (uint32_t) readEeprom(devAdd);
            setRgbColor(0,receive[defAdd + two],receive[defAdd + two]);
            i++;
            if(i>MAX_ADDRESS)
            {
               i=0;
               proceed=0;
            }
        }
    }
    UART4_ICR_R = UART_ICR_RXIC;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

void main(void)
{
    // Initialize hardware
    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    initUart4();
    initRgb();

    maxAdd = MAX_ADDRESS;
    newAdd = MODE_ADDRESS;
    devAdd = (uint16_t) DEFAULT_ADDRESS;
    mode = (uint32_t) readEeprom(newAdd);
    writeEeprom(devAdd,DEFAULT_ADDRESS);
    defAdd = (uint32_t)readEeprom(devAdd);
    while(true)
    {
        if(mode == 1)
        {
            putsUart0("Current mode -> 'CONTROLLER MODE'\n");
            writeEeprom(newAdd, mode);
            DATA_ENABLE = 1;
            UART4_CTL_R &= ~(UART_CTL_RXE);
            UART4_CTL_R |= UART_CTL_TXE;
            controllerMode();
        }
        else if(mode == 0)
        {
            putsUart0("Current mode -> 'DEVICE MODE'\n");
            writeEeprom(newAdd, mode);
            DATA_ENABLE = 0;
            UART4_CTL_R &= ~(UART_CTL_TXE);
            UART4_CTL_R |= UART_CTL_RXE;
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
            deviceMode();
        }
    }
}


