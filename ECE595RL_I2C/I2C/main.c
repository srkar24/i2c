/**
 * @file main.c
 * @brief Main source code for the I2C program.
 *
 * This file contains the main entry point for the I2C program.
 *
 * It interfaces the following peripherals using I2C to demonstrate light and wall detection:
 *  - OPT3001 Ambient Light Sensor (TI BP-BASSENSORSMKII BoosterPack)
 *  - OPT3101 3-Channel Wide FOV Time-of-Flight Distance Sensor
 *
 * The following Timer_A modules are used in this lab:
 *  - Timer_A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer_A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @author Srushti Wadekar, Arshia P, Aaron Nanas
 */

#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/LPF.h"
#include "inc/OPT3001.h"
#include "inc/OPT3101.h"

//#define CONTROLLER_1    1
//#define CONTROLLER_2    1
#define CONTROLLER_3    1

#define DEBUG_ACTIVE    1

// Initialize lux threshold. Must be modified based on flashlight intensity
#define LUX_THRESHOLD 5000

// Set initial duty cycle of the left wheel to 25%
uint16_t Duty_Cycle_Left            = 3750;

// Set initial duty cycle of the right wheel to 25%
uint16_t Duty_Cycle_Right           = 3750;

// Declare struct for the light sensor
OPT3001_Result Light;

// Declare global variable to store lux value
double lux;

// Declare global variable to indicate that lux has been measured by the light sensor
uint8_t lux_done = 0;

// Declare global arrays and variables for the distance sensor
uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
uint32_t LeftDistance = 10000;
uint32_t CenterDistance = 10000;
uint32_t RightDistance = 10000;
uint32_t channel;
int TxChannelIdx;

/**
 * @brief This function samples the OPT3001 Ambient Light Sensor and calculates the lux.
 *
 * @param None
 *
 * @return None
 */
void Sample_Light_Sensor()
{
    Light = OPT3001_Read_Light();
    lux = (0.01) * (pow(2, Light.Exponent)) * (Light.Result);

#ifdef DEBUG_ACTIVE
    printf("\nLight Sensor Exponent: %d\n", Light.Exponent);
    printf("Light Sensor Mantissa: %d\n", Light.Result);
    printf("Light Sensor Lux: %f\n\n", lux);
#endif
}

/**
 * @brief This function samples the OPT3101 distance sensor and applies a low-pass filter for each channel.
 *
 * @param None
 *
 * @return None
 */
void Sample_Distance_Sensor()
{
    if (TxChannel <= 2)
    {
        // TxChannel 0 is Left
        if (TxChannel == 0)
        {
            FilteredDistances[0] = LPF_Calc(Distances[0]);
            LeftDistance = FilteredDistances[0];
        }

        // TxChannel 1 is Center
        else if (TxChannel == 1)
        {
            FilteredDistances[1] = LPF_Calc2(Distances[1]);
            CenterDistance = FilteredDistances[1];
        }

        // TxChannel 2 is Right
        else
        {
            FilteredDistances[2] = LPF_Calc3(Distances[2]);
            RightDistance = FilteredDistances[2];
        }
#ifdef DEBUG_ACTIVE
        printf("TX Channel: %d | Distance: %d mm\n", TxChannel, FilteredDistances[TxChannel]);
#endif
        TxChannel = 3;
        channel = (channel + 1) % 3;
        OPT3101_StartMeasurementChannel(channel);
        TxChannelIdx++;
    }

    // Check if there is noise
    if (TxChannelIdx >= 300)
    {
        TxChannelIdx = 0;
    }
}

/**
 * @brief This function enables the motors when the measured lux is greater than the specified threshold.
 *
 * @param None
 *
 * @return None
 */
void Controller_1()
{
    if (lux >= LUX_THRESHOLD)
    {
        Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
    }
    else
    {
        Motor_Stop();
    }
}

/**
 * @brief This function disables the motors when the robot is close to a wall. Otherwise, the motors are enabled.
 *
 * @param None
 *
 * @return None
 */
void Controller_2()
{
    if (CenterDistance <= 400)
    {
        Motor_Stop();
    }
    else
    {
        Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
    }
}

/**
 * @brief This function turns the motors slightly right when the robot is close to a wall on the front or left side.
 * Otherwise, the motors are enabled.
 *
 * @param None
 *
 * @return None
 */
void Controller_3()
{
    // Your code goes here

    if (CenterDistance <= 300 || LeftDistance <= 300)
    {
        Motor_Forward(4000, 50);
    }
    else
    {
        Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
    }
}

/**
 * @brief User-defined function executed by Timer A1 using a periodic interrupt.
 *
 * @param None
 *
 * @return None
 */
void Timer_A1_Periodic_Task(void)
{
#if defined CONTROLLER_1

    Controller_1();

#elif defined CONTROLLER_2
    #if defined CONTROLLER_1 || CONTROLLER_3
        #error "Only CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3 can be active at the same time."
    #endif

    Controller_2();

#elif defined CONTROLLER_3
    #if defined CONTROLLER_1 || CONTROLLER_2
#error "Only CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3 can be active at the same time."
    #endif

    Controller_3();

#else
    #error "Define either one of the options: CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3."
#endif
}

int main(void)
{
    // Initialize variables for the distance sensor
    TxChannelIdx = 0;
    channel = 1;
    TxChannel = 3;

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the built-in red LED and the RGB LEDs
    LED1_Init();
    LED2_Init();

    // Initialize the chassis board LEDs
    Chassis_Board_LEDs_Init();

    // Initialize the user buttons
    Buttons_Init();

    // Initialize EUSCI_A0_UART to use the printf function
    EUSCI_A0_UART_Init_Printf();

    // Initialize Timer A1 with interrupts enabled and an interrupt rate of 1 kHz
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Initialize the DC motors
    Motor_Init();

    // Initialize I2C using EUSCI_B1 module
    EUSCI_B1_I2C_Init();

    // Debug signal to indicate the start of OPT3001 initialization
    P8->OUT |= 0x01;

    // Initialize the OPT3001 Ambient Light Sensor
    OPT3001_Init();

    // Debug signal to indicate the end of OPT3001 initialization
    // and the start of OPT3101 initialization
    P8->OUT &= ~0x01;

    // Initialize the OPT3101 Distance Sensor
    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();
    OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
    OPT3101_StartMeasurementChannel(channel);

    // Initialize low pass filter used for the distance sensor
    LPF_Init(100, 8);
    LPF_Init2(100, 8);
    LPF_Init3(100, 8);

    // Enable the interrupts used by the timer modules and other peripherals
    EnableInterrupts();

    while(1)
    {
#if defined CONTROLLER_1

        Sample_Light_Sensor();

#elif defined CONTROLLER_2
    #if defined CONTROLLER_1 || CONTROLLER_3
        #error "Only CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3 can be active at the same time."
    #endif

        Sample_Distance_Sensor();


#elif defined CONTROLLER_3
    #if defined CONTROLLER_1 || CONTROLLER_2
#error "Only CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3 can be active at the same time."
    #endif

        Sample_Light_Sensor();
        Sample_Distance_Sensor();

#else
    #error "Define either one of the options: CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3."
#endif
    }
}
