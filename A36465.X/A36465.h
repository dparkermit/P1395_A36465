// Header file for new AFC Board
#ifndef __A36465_H
#define __A36465_H

#include <xc.h>
#include <libpic30.h>
#include <adc10.h>
#include <timer.h>
#include <pwm.h>

#include "P1395_CAN_SLAVE.h"
#include "P1395_MODULE_CONFIG.h"
#include "ETM_ANALOG.h"

/*
  Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN 
  Timer2 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer3 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by LTC265X Module
  I2C    - Used/Configured by EEPROM Module


  Timer5 - Used for 10ms Generation
  
  ADC Module - See Below For Specifics
  Motor Control PWM Module - Used to control AFC stepper motor
  Timer1 - Used for timing motor steps

*/


// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// Pins to be configured as inputs
/*

  RA9  - ADC VREF-
  RA10 - ADC VREF+
  RA14 - Trigger Input - SMA
  RA15 - Sample Parameter    (PLC ONLY)
 
  RB0  - ICD - PROGRAM
  RB1  - ICD - PROGRAM
  RB2  - Analog Input - Reverse Pwr          (S/H #3)
  RB3  - Analog Input - AFT AFC "A" Output   (S/H #1)      (until we need quadrature encoder module)
  RB4  - Analog Input - AFT AFC "B" output   (S/H #2)      (until we need quadrature encoder module)
  RB5  - Digital Input - Quadrature Encoder Module
  RB6  - Analog Input - PLC Parameter Select (S/H #4)
  RB7  - Analog Input - PLC Parameter Value  (S/H #4)
  RB8  - Analog Input - Forward Pwr          (S/H #4)
  RB9  - Analog Input - ADC DAC C            (S/H #4)
  RB13 - Analog Input - 5V MON               (S/H #4)
  RB14 - Analog Input - 15V MON              (S/H #4)
  RB15 - Analog Input - -15V MON             (S/H #4)

  RC1  - DAC LDAC  (Configured by DAC module)
  RC3  - DAC CS/LD (Configured by DAC module)

  RD1  - Motor Drv Fault
  RD4  - Spare Digital Input (PLC ONLY)
  RD8  - AFC/Manual Select   (PLC ONLY)
  RD9  - Manual Down         (PLC ONLY)
  RD10 - Manul Up            (PLC ONLY)
  RD13 - Fiber Trigger In (connects to input capture module so we can use pic to calculate PRF)

  RE1 - PWM 1H (Configured By Pic Module)
  RE3 - PWM 2H (Configured By Pic Module)
  RE5 - PWM 3H (Configured By Pic Module)
  RE7 - PWM 4H (Configured By Pic Module)
  RE8  - Fiber Trigger In
  RE9  - Fiber Energy Select In

  RF0  - CAN 1 (Configured By Pic Module)
  RF1  - CAN 1 (Configured By Pic Module)
  RF6  - SPI 1 (Configured By Pic Module)
  RF7  - SPI 1 (Configured By Pic Module)
  RF8  - SPI 1 (Configured By Pic Module)

  RG2  - I2C   (Configured By Pic Module)
  RG3  - I2C   (Configured By Pic Module)
  RG7  - Reset Detect
  
  Assuming we use the quadrature encoder module the following changes happen
  RB0, RB1 become Analog Inputs
  RB3 and RB4 become Digital Inputs
  AFT AFC "A" Output is routed to RB0        (S/H #1)
  AFT AFC "B" Output is routed to RB1        (S/H #2)


*/



#define A36465_TRISA_VALUE 0b1100011000000000 
#define A36465_TRISB_VALUE 0b1110001111111111 
#define A36465_TRISC_VALUE 0b0000000000001010 
#define A36465_TRISD_VALUE 0b0010011100010010
#define A36465_TRISE_VALUE 0b0000001110101010 
#define A36465_TRISF_VALUE 0b0000000111000011 
#define A36465_TRISG_VALUE 0b0000000010001100


// ------------- PIN DEFINITIONS ------------------- ///

// MOTOR CONTROL PINS
#define PIN_MOTOR_DRV_RESET_NOT          _LATD3
#define PIN_MOTOR_DRV_SLEEP_NOT          _LATD2
#define PIN_MOTOR_DRV_DECAY_SELECY       _LATD12

#define PIN_MOTOR_DRV_ISET_A0            _LATC14
#define PIN_MOTOR_DRV_ISET_A1            _LATC13
#define PIN_MOTOR_DRV_ISET_B0            _LATD0
#define PIN_MOTOR_DRV_ISET_B1            _LATD11

#define PIN_MOTOR_DRV_INPUT_NOT_FAULT    _RD1


// Test Points
#define PIN_TEST_POINT_A                 _LATG1
#define PIN_TEST_POINT_B                 _LATG0
#define PIN_TEST_POINT_G                 _LATB10
#define PIN_TEST_POINT_H                 _LATB11
#define PIN_TEST_POINT_J                 _LATBF4
#define PIN_TEST_POINT_K                 _LATBF5

#define PIN_RESET_DETECT                 _RG7

// LEDs
#define PIN_LED_OPERATIONAL_GREEN        _LATG6
#define PIN_LED_A_RED                    _LATG8



// ---------------- Motor Configuration Values ------------- //
#define MOTOR_PWM_FREQ            20000        // Motor Drive Frequency is 10KHz
#define MOTOR_SPEED_FAST          400          // Motor Speed in Full Steps per Second in "Fast Mode"
#define MOTOR_SPEED_SLOW          400           // 
#define FCY_CLK                             10000000      // 10 MHz
#define DELAY_SWITCH_TO_LOW_POWER_MODE 200


// --------------------- T1 Configuration -----
// With 1:8 prescale the minimum 1/32 step time is 52ms or a minimum speed of .6 Steps/second

#define T1CON_SETTING     (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_8 & T1_SYNC_EXT_OFF & T1_SOURCE_INT)
#define PR1_FAST_SETTING  (unsigned int)(FCY_CLK / 32 / 8 / MOTOR_SPEED_FAST)
#define PR1_SLOW_SETTING  (unsigned int)(FCY_CLK / 32 / 8 / MOTOR_SPEED_SLOW)


/* 
   TMR5 Configuration
   Timer5 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/

#define T5CON_VALUE                    (T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_SOURCE_INT)
#define PR5_PERIOD_US                  10000   // 10mS
#define PR5_VALUE_10_MILLISECONDS      (unsigned int)(FCY_CLK_MHZ*PR5_PERIOD_US/8)


// Motor Drive Configuration
#define PTCON_SETTING     (PWM_EN & PWM_IPCLK_SCALE1 & PWM_MOD_FREE)
#define PTPER_SETTING     (unsigned int)(FCY_CLK/MOTOR_PWM_FREQ)
// Special Event Trigger - TBD
#define PWMCON1_SETTING  (PWM_MOD1_COMP & PWM_MOD2_COMP & PWM_MOD3_COMP & PWM_MOD4_COMP & PWM_PEN1H & PWM_PEN1L & PWM_PEN2H & PWM_PEN2L & PWM_PEN3H & PWM_PEN3L & PWM_PEN4H & PWM_PEN4L)
#define PWMCON2_SETTING  (PWM_SEVOPS1 & PWM_OSYNC_TCY & PWM_UEN) 
#define DTCON1_SETTING   (PWM_DTAPS1 & PWM_DTA0 & PWM_DTBPS1 & PWM_DTB0)
#define DTCON2_SETTING   (PWM_DTS1A_UA & PWM_DTS1I_UA & PWM_DTS2A_UA & PWM_DTS2I_UA  & PWM_DTS3A_UA & PWM_DTS3I_UA & PWM_DTS4A_UA & PWM_DTS4I_UA)
#define FLTACON_SETTING  (PWM_FLTA1_DIS & PWM_FLTA2_DIS & PWM_FLTA3_DIS & PWM_FLTA4_DIS)
#define FLTBCON_SETTING  (PWM_FLTB1_DIS & PWM_FLTB2_DIS & PWM_FLTB3_DIS & PWM_FLTB4_DIS)
#define OVDCON_SETTING   (PWM_GEN_1H & PWM_GEN_1L & PWM_GEN_2H & PWM_GEN_2L & PWM_GEN_3H & PWM_GEN_3L & PWM_GEN_4H & PWM_GEN_4L)

// User code needs to set PDC1, PDC2 (Duty cycle 1 and 2)
// Duty cycle is encoded at twice the resolution of the Period.
#define PDC1_SETTING     PTPER_SETTING // 50% duty cycle
#define PDC2_SETTING     PTPER_SETTING // 50% duty cycle
#define PDC3_SETTING     PTPER_SETTING // 50% duty cycle
#define PDC4_SETTING     PTPER_SETTING // 50% duty cycle



// -------------------  ADC CONFIGURATION ----------------- //

#define ADCON1_SETTING   (ADC_MODULE_ON & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_MANUAL & ADC_SAMPLE_SIMULTANEOUS & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING   (ADC_VREF_EXT_EXT & ADC_SCAN_OFF & ADC_CONVERT_CH_0ABC & ADC_SAMPLES_PER_INT_1 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING   (ADC_SAMPLE_TIME_10 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_2Tcy)
#define ADCHS_SETTING    (ADC_CHX_POS_SAMPLEA_AN3AN4AN5 & ADC_CHX_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN8 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CHX_POS_SAMPLEB_AN3AN4AN5 & ADC_CHX_NEG_SAMPLEB_VREFN & ADC_CH0_POS_SAMPLEB_AN8 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING   (ENABLE_AN2_ANA & ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN8_ANA)
#define ADCSSL_SETTING   (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 & SKIP_SCAN_AN14 & SKIP_SCAN_AN15)

typedef struct {
  unsigned int control_state;
  unsigned int manual_target_position;
  unsigned int sample_complete;
  //int          frequency_error_filtered;
  unsigned int fast_afc_done;
  unsigned int pulses_on_this_run;
  unsigned long pulse_off_counter;
  unsigned int afc_hot_position;
  AnalogOutput aft_control_voltage;
  AnalogInput  aft_A_sample;
  AnalogInput  aft_B_sample;
  unsigned int aft_A_sample_filtered;
  unsigned int aft_B_sample_filtered;
  //unsigned int sample_index;
  unsigned int aft_A_sample_history[16];
  unsigned int aft_B_sample_history[16];
  unsigned int aft_filtered_error_for_client;

} AFCControlData;

extern AFCControlData global_data_A36465;


typedef struct {
  unsigned int current_position;
  unsigned int target_position;
  unsigned int home_position;
  unsigned int max_position;
  unsigned int min_position;
  //unsigned int pwm_table_index;
  unsigned int time_steps_stopped;
} STEPPER_MOTOR;

extern STEPPER_MOTOR afc_motor;

#define _STATUS_AFC_AUTO_ZERO_HOME_IN_PROGRESS          _STATUS_0
#define _STATUS_AFC_MODE_MANUAL_MODE                    _STATUS_1


#define _FAULT_CAN_COMMUNICATION_LATCHED                _FAULT_0



#endif
