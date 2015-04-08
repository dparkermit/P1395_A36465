#include "A36465.h"
#include "FIRMWARE_VERSION.h"
#include "ETM_EEPROM.h"
#include "LTC265X.h"

// This is the firmware for the AFC BOARD


_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
//_FBORPOR(PWRT_OFF & BORV45 & PBOR_OFF & MCLR_EN);
_FBORPOR(PWRT_OFF & BORV45 & PBOR_ON & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(GWRP_OFF & GSS_OFF);
_FICD(PGD);



unsigned int ShiftIndex(unsigned int index, unsigned int shift);

void DoAFC(void);

void DoADCFilter(void);

LTC265X U23_LTC2654;

// Cooldown table is stored in 5.12 second intervals
#define COOL_DOWN_TABLE_VALUES 31130,31260,28985,27246,25880,24768,23833,23023,22303,21651,21052,20495,19972,19479,19012,18568,18146,17742,17357,16988,16635,16296,15972,15660,15361,15074,14798,14532,14277,14031,13794,13565,13345,13132,12927,12728,12536,12351,12171,11997,11828,11665,11506,11352,11202,11057,10916,10778,10644,10514,10387,10263,10142,10024,9908,9796,9686,9578,9473,9370,9269,9170,9073,8978,8884,8793,8703,8614,8528,8442,8359,8276,8195,8115,8037,7959,7883,7808,7734,7661,7589,7518,7448,7379,7311,7243,7177,7111,7047,6983,6919,6857,6795,6734,6674,6614,6555,6497,6439,6382,6326,6270,6215,6160,6106,6053,6000,5947,5895,5844,5793,5742,5693,5643,5594,5546,5498,5450,5403,5356,5310,5264,5219,5174,5130,5085,5042,4998,4956,4913,4871,4829,4788,4747,4706,4666,4626,4586,4547,4508,4470,4431,4394,4356,4319,4282,4245,4209,4173,4138,4102,4067,4033,3998,3964,3931,3897,3864,3831,3798,3766,3734,3702,3671,3639,3608,3578,3547,3517,3487,3457,3428,3399,3370,3341,3313,3285,3257,3229,3202,3174,3147,3121,3094,3068,3042,3016,2990,2965,2940,2915,2890,2865,2841,2817,2793,2769,2745,2722,2699,2676,2653,2631,2608,2586,2564,2542,2521,2499,2478,2457,2436,2416,2395,2375,2354,2334,2315,2295,2275,2256,2237,2218,2199,2180,2162,2143,2125,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

#define FULL_POWER_TABLE_VALUES 50,76,102,128,153,179,204,229,253,277,300,322,344,366,386,406,425,443,460,476,491,505,517,529,540,549,557,564,570,574,577,579,580,579,577,574,570,564,557,549,540,529,517,505,491,476,460,443,425,406,386,366,344,322,300,277,253,229,204,179,153,128,102,76,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50

#define LOW_POWER_TABLE_VALUES 50,65,79,94,109,123,137,151,165,178,191,204,217,229,240,251,262,272,282,291,299,307,315,321,327,332,337,341,344,347,349,350,350,350,349,347,344,341,337,332,327,321,315,307,299,291,282,272,262,251,240,229,217,204,191,178,165,151,137,123,109,94,79,65,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50


const unsigned int PWMHighPowerTable[128] = {FULL_POWER_TABLE_VALUES};
const unsigned int PWMLowPowerTable[128]  = {LOW_POWER_TABLE_VALUES};
const unsigned int CoolDownTable[256]     = {COOL_DOWN_TABLE_VALUES};

STEPPER_MOTOR afc_motor;

AFCControlData global_data_A36465;


void DoStateMachine(void);
void InitializeA36465(void);
void DoA36465(void);


#define STATE_STARTUP       0x10
#define STATE_AUTO_ZERO     0x20
#define STATE_AUTO_HOME     0x30
#define STATE_RUN_AFC       0x40
#define STATE_RUN_MANUAL    0x50
#define STATE_FAULT         0x60


int main(void) {
  global_data_A36465.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}

#define AFC_MOTOR_MIN_POSITION      1000
#define AFC_MOTOR_MAX_POSITION      60000



void DoStateMachine(void) {
  switch (global_data_A36465.control_state) {

  case STATE_STARTUP:
    InitializeA36465();
    afc_motor.min_position = 0;
    afc_motor.max_position = 0xFFF0;
    afc_motor.home_position = 34000;
    afc_motor.time_steps_stopped = 0;
    global_data_A36465.control_state = STATE_AUTO_ZERO;
    break;

  case STATE_AUTO_ZERO:
    afc_motor.current_position = 0xFFF0;
    afc_motor.target_position  = 0;
    _CONTROL_NOT_CONFIGURED = 1;
    while (global_data_A36465.control_state == STATE_AUTO_ZERO) {
      DoA36465();
      if ((afc_motor.current_position <= 100) && (_CONTROL_NOT_CONFIGURED == 0)) {
	global_data_A36465.control_state = STATE_AUTO_HOME;
      }
    }
    break;

  case STATE_AUTO_HOME:
    global_data_A36465.aft_control_voltage.enabled = 1;
    afc_motor.min_position = AFC_MOTOR_MIN_POSITION;
    afc_motor.max_position = AFC_MOTOR_MAX_POSITION;
    afc_motor.target_position = afc_motor.home_position;
    global_data_A36465.afc_hot_position = afc_motor.home_position;
    while (global_data_A36465.control_state == STATE_AUTO_HOME) {
      DoA36465();
      if (afc_motor.current_position == afc_motor.home_position) {
	global_data_A36465.control_state = STATE_RUN_AFC;
      }
    }
    break;
    
  case STATE_RUN_AFC:
    while (global_data_A36465.control_state == STATE_RUN_AFC) {
      DoA36465();
      if (global_data_A36465.sample_complete) {
	global_data_A36465.sample_complete = 0;
	DoADCFilter();
	DoAFC();
	if (_SYNC_CONTROL_HIGH_SPEED_LOGGING) {
	  ETMCanSlaveLogCustomPacketC();
	  ETMCanSlaveLogCustomPacketD();
	}
      }

      if (_STATUS_AFC_MODE_MANUAL_MODE) {
	global_data_A36465.control_state = STATE_RUN_MANUAL;
      }
    }
    break;
    
    
  case STATE_RUN_MANUAL:
    while (global_data_A36465.control_state == STATE_RUN_MANUAL) {
      DoA36465();
      afc_motor.target_position = global_data_A36465.manual_target_position;
      if (global_data_A36465.sample_complete) {
	global_data_A36465.sample_complete = 0;
	DoADCFilter();
	if (_SYNC_CONTROL_HIGH_SPEED_LOGGING) {
	  ETMCanSlaveLogCustomPacketC();
	  ETMCanSlaveLogCustomPacketD();
	}
      }
      if (!_STATUS_AFC_MODE_MANUAL_MODE) {
	global_data_A36465.control_state = STATE_RUN_AFC;
      }
    }
    break;
    
 case STATE_FAULT:
    while (global_data_A36465.control_state == STATE_FAULT) {
      DoA36465();
    }
    break;


  default:
    global_data_A36465.control_state = STATE_RUN_AFC;
    break;

  }
}

#define NO_PULSE_TIME_TO_INITITATE_COOLDOWN 100    // 1 second
#define LIMIT_RECORDED_OFF_TIME             120000 // 1200 seconds, 20 minutes // 240 elements

void DoAFCCooldown(void);

void DoAFCCooldown(void) {
  unsigned int position_difference;
  unsigned int shift_position;

  if (global_data_A36465.pulse_off_counter >= 120000) {
    global_data_A36465.pulse_off_counter = 120000;
  }

  if (afc_motor.home_position > global_data_A36465.afc_hot_position) {
    position_difference = afc_motor.home_position - global_data_A36465.afc_hot_position;
    shift_position = ETMScaleFactor2(position_difference, CoolDownTable[global_data_A36465.pulse_off_counter >> 9], 0);
    afc_motor.target_position = afc_motor.home_position - shift_position;
  } else {
    position_difference = global_data_A36465.afc_hot_position - afc_motor.home_position; 
    shift_position = ETMScaleFactor2(position_difference, CoolDownTable[global_data_A36465.pulse_off_counter >> 9], 0);
    afc_motor.target_position = afc_motor.home_position + shift_position;
  }
}

void DoA36465(void) {
  ETMCanSlaveDoCan();
  local_debug_data.debug_0 = afc_motor.target_position;
  local_debug_data.debug_1 = afc_motor.current_position;
  local_debug_data.debug_2 = afc_motor.home_position;
  local_debug_data.debug_3 = global_data_A36465.control_state;
  local_debug_data.debug_4 = global_data_A36465.pulses_on_this_run;
  local_debug_data.debug_5 = global_data_A36465.fast_afc_done;
  local_debug_data.debug_6 = global_data_A36465.aft_A_sample.filtered_adc_reading;
  local_debug_data.debug_7 = global_data_A36465.aft_B_sample.filtered_adc_reading;

  local_debug_data.debug_8 = global_data_A36465.aft_A_sample.reading_scaled_and_calibrated;
  local_debug_data.debug_9 = global_data_A36465.aft_B_sample.reading_scaled_and_calibrated;
  local_debug_data.debug_A = global_data_A36465.aft_A_sample_filtered;
  local_debug_data.debug_B = global_data_A36465.aft_B_sample_filtered;
  local_debug_data.debug_C = 0;   
 

  if (_T5IF) {
    _T5IF = 0;

    
    if (_SYNC_CONTROL_CLEAR_DEBUG_DATA) {
      global_data_A36465.pulses_on_this_run = 0;
    }

    //update the AFT control voltage
    ETMAnalogScaleCalibrateDACSetting(&global_data_A36465.aft_control_voltage);
    WriteLTC265X(&U23_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36465.aft_control_voltage.dac_setting_scaled_and_calibrated);
  
    global_data_A36465.pulse_off_counter++;
    if (global_data_A36465.fast_afc_done == 1) {
      global_data_A36465.afc_hot_position = afc_motor.current_position;
    }

    if (global_data_A36465.control_state == STATE_RUN_AFC) {
      if (global_data_A36465.pulse_off_counter >= NO_PULSE_TIME_TO_INITITATE_COOLDOWN) {
	if (global_data_A36465.pulse_off_counter >= LIMIT_RECORDED_OFF_TIME) {
	  global_data_A36465.pulse_off_counter = LIMIT_RECORDED_OFF_TIME;
	}
	global_data_A36465.fast_afc_done = 0;
	global_data_A36465.pulses_on_this_run = 0;
	DoAFCCooldown();
      }
    }
  }
}



void InitializeA36465(void) {

  TRISA = A36465_TRISA_VALUE;
  TRISB = A36465_TRISB_VALUE;
  TRISC = A36465_TRISC_VALUE;
  TRISD = A36465_TRISD_VALUE;
  TRISE = A36465_TRISE_VALUE;
  TRISF = A36465_TRISF_VALUE;
  TRISG = A36465_TRISG_VALUE;

  PIN_MOTOR_DRV_RESET_NOT = 1;
  PIN_MOTOR_DRV_SLEEP_NOT = 1;

  PIN_MOTOR_DRV_ISET_A0   = 0;
  PIN_MOTOR_DRV_ISET_A1   = 0;
  PIN_MOTOR_DRV_ISET_B0   = 0;
  PIN_MOTOR_DRV_ISET_B1   = 0;

  PTPER   = PTPER_SETTING;
  PWMCON1 = PWMCON1_SETTING;
  PWMCON2 = PWMCON2_SETTING;
  DTCON1  = DTCON1_SETTING;
  DTCON2  = DTCON2_SETTING;
  FLTACON = FLTACON_SETTING;
  FLTBCON = FLTBCON_SETTING;
  OVDCON  = OVDCON_SETTING;
  PDC1    = PDC1_SETTING;
  PDC2    = PDC2_SETTING;
  PDC3    = PDC3_SETTING;
  PDC4    = PDC4_SETTING;
  PTCON   = PTCON_SETTING;

  PR1 = PR1_FAST_SETTING;
  _T1IF = 0;
  _T1IP = 6;
  _T1IE = 1;
  T1CON = T1CON_SETTING;

  PR5 = PR5_VALUE_10_MILLISECONDS;
  T5CON = T5CON_VALUE;
  _T5IF = 0;
  

  ADCON2 = ADCON2_SETTING;
  ADCON3 = ADCON3_SETTING;
  ADCHS  = ADCHS_SETTING;
  ADPCFG = ADPCFG_SETTING;
  ADCSSL = ADCSSL_SETTING;
  ADCON1 = ADCON1_SETTING;
  
  _INT1IF = 0;
  _INT1IP = 7;
  _INT1IE = 1;

  
  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;
  
  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;

  // Initialize the External EEprom
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);

  // Initialize LTC DAC
  //SetupLTC265X(&U23_LTC2654, ETM_SPI_PORT_1, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);
  SetupLTC265X(&U23_LTC2654, ETM_SPI_PORT_1, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RC1, _PIN_RC3);

#define AFT_CONTROL_VOLTAGE_MAX_PROGRAM  12000
#define AFT_CONTROL_VOLTAGE_MIN_PROGRAM  1000

  ETMAnalogInitializeOutput(&global_data_A36465.aft_control_voltage,
			    MACRO_DEC_TO_SCALE_FACTOR_16(3.98799),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_0,
			    AFT_CONTROL_VOLTAGE_MAX_PROGRAM,
			    AFT_CONTROL_VOLTAGE_MIN_PROGRAM,
			    0);

#define AFT_ANALOG_INPUT_UNDER_VOLTAGE 0
#define AFT_ANALOG_INPUT_OVER_VOLTAGE  10000
  
  ETMAnalogInitializeInput(&global_data_A36465.aft_A_sample,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.16583),
			   OFFSET_ZERO,
			   ANALOG_INPUT_3,
			   AFT_ANALOG_INPUT_OVER_VOLTAGE,
			   AFT_ANALOG_INPUT_UNDER_VOLTAGE,
			   0,
			   0,
			   0xFF00);

  ETMAnalogInitializeInput(&global_data_A36465.aft_B_sample,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.16583),
			   OFFSET_ZERO,
			   ANALOG_INPUT_4,
			   AFT_ANALOG_INPUT_OVER_VOLTAGE,
			   AFT_ANALOG_INPUT_UNDER_VOLTAGE,
			   0,
			   0,
			   0xFF00);

  // Initialize the Can module
  ETMCanSlaveInitialize();
}


unsigned int GetDirectionToMove(unsigned int samp_A, unsigned int samp_B);
//unsigned int GetIndexToMove(unsigned int samp_A, unsigned int samp_B);
unsigned int FastModeGetStepsToMove(unsigned int samp_A, unsigned int samp_B);
unsigned int FastModeGetStepsToMove(unsigned int samp_A, unsigned int samp_B);

unsigned int GetDirectionToMove(unsigned int samp_A, unsigned int samp_B) {
  // 1 will move the motor position down
  // 0 will move the motor position up
  if (samp_A >= samp_B) {
    return 0;  
  } else {
    return 1;  
  }
}


unsigned int FastModeGetStepsToMove(unsigned int samp_A, unsigned int samp_B) {
  unsigned int error;
  unsigned int steps;
  if (samp_A >= samp_B) {
    error = samp_A - samp_B;
  } else {
    error = samp_B - samp_A;
  }
  
  if (error > 0x7FFF) {
    error = 0x7FFF;
  }

  if (samp_A >= samp_B) {
    global_data_A36465.aft_filtered_error_for_client = error;
  } else {
    global_data_A36465.aft_filtered_error_for_client = error + 0x8000;
  }

  if (error >= 500) {
    // I am comfortable with our measurement.
    // Use the full scaling to get the steps to move
    steps = ETMScaleFactor2(error, MACRO_DEC_TO_CAL_FACTOR_2(.305344), 0);
  } else if (error >= 100) {
    // Our error is smaller and we don't want over correct due to signal error
    steps = ETMScaleFactor2(error, MACRO_DEC_TO_CAL_FACTOR_2(.152672), 0);
  } else {
    // We are within 32 micro steps (1 full step) don't bother moving at all in fast mode
    steps = 0;
  }

  if (steps >= 320) {
    // This is the largest possible movement from a single reading
    // Note at 200Hz the motor wont move more than 64 steps between pulses
    steps = 320;
  }
  
  return steps;
}


unsigned int SlowModeGetStepsToMove(unsigned int samp_A, unsigned int samp_B) {
  unsigned int error;
  unsigned int steps;
  if (samp_A >= samp_B) {
    error = samp_A - samp_B;
  } else {
    error = samp_B - samp_A;
  }
  
  if (error > 0x7FFF) {
    error = 0x7FFF;
  }

  if (samp_A >= samp_B) {
    global_data_A36465.aft_filtered_error_for_client = error;
  } else {
    global_data_A36465.aft_filtered_error_for_client = error + 0x8000;
  }

  if (error >= 200) {
    // I am comfortable with our measurement.
    // Use the full scaling to get the steps to move
    steps = ETMScaleFactor2(error, MACRO_DEC_TO_CAL_FACTOR_2(.305344), 0);
  } else if(error >= 26) {
    // Our error is smaller and we don't want over correct due to signal error
    steps = ETMScaleFactor2(error, MACRO_DEC_TO_CAL_FACTOR_2(.152672), 0);
  } else {
    // We are within 8 micro steps (1/4 full step) don't bother moving at all in slow mode
    steps = 0;
  }

  if (steps >= 32) {
    // This is the largest possible movement from a single reading in slow mode.
    // Note at 200Hz the motor wont move more than 64 steps between pulses
    steps = 32;
  }
  
  return steps;
}


#define MIN_ERROR_STEPS_FAST_AFC                   32          // We are within one motor full step
#define MAX_NUMBER_OF_PULSES_FOR_STARTUP_RESPONSE  256

void DoAFC(void) {
  unsigned int direction_move;
  //unsigned int index_move;
  unsigned int steps_to_move;
  unsigned int position_now;
  unsigned int new_target_position;
  
  
  position_now = afc_motor.current_position;
  direction_move = GetDirectionToMove(global_data_A36465.aft_A_sample_filtered, global_data_A36465.aft_B_sample_filtered);

  local_debug_data.debug_D = direction_move;
  
  if (!global_data_A36465.fast_afc_done) {
    /*
      The magnetron has just turned on after being off for a period of time.
      The tuner *could* be wildly out of position.
      We need to react to the incoming data from AFC very quickly
      This means less filtering and and high gain integral response - Max 4 Steps per sample
    */
    
    /*
      steps_to_move = fast_response_lookup_table[index_move];
    */
    //steps_to_move = index_move;
    steps_to_move = FastModeGetStepsToMove(global_data_A36465.aft_A_sample_filtered, global_data_A36465.aft_B_sample_filtered);
    local_debug_data.debug_F = steps_to_move;

    if ((global_data_A36465.pulses_on_this_run >= 4) && (steps_to_move <= MIN_ERROR_STEPS_FAST_AFC)) {
      global_data_A36465.fast_afc_done = 1;
      global_data_A36465.pulses_on_this_run &= 0xFFF0;  // WE need to set the lowest nibble to zero to guarantee that we get new data in the filter before the first "slow" move
    }
    if (global_data_A36465.pulses_on_this_run >= MAX_NUMBER_OF_PULSES_FOR_STARTUP_RESPONSE) {
      global_data_A36465.fast_afc_done = 1;
      global_data_A36465.pulses_on_this_run &= 0xFFF0; // WE need to set the lowest nibble to zero to guarantee that we get new data in the filter before the first "slow" move
    }
    
  } else {
    /*
      steps_to_move = slow_response_lookup_table[index_move];
    */
    //steps_to_move = index_move >> 1;
    steps_to_move = SlowModeGetStepsToMove(global_data_A36465.aft_A_sample_filtered, global_data_A36465.aft_B_sample_filtered);
    local_debug_data.debug_F = steps_to_move;
  
    if ((global_data_A36465.pulses_on_this_run & 0xF) != 0xF) {
      // We only want to adjust the position once every 16 pulses (because we are averaging the position to move for 16 pulses)
      steps_to_move = 0;
    }
  }
  
  // Adjust the target position based on steps to move;
  if (direction_move) {
    // decrease the target position
    if (position_now > steps_to_move) {
      new_target_position = position_now - steps_to_move;
    } else {
      new_target_position = 0;
    }
  } else {
    // increase the target position
    if ((0xFFFF - steps_to_move) > position_now) {
      new_target_position = position_now + steps_to_move;
    } else {
      new_target_position = 0xFFFF;
    }
  }
  afc_motor.target_position = new_target_position;
}


void DoADCFilter(void) {
  unsigned long temp;

  global_data_A36465.pulses_on_this_run++;
  global_data_A36465.pulse_off_counter = 0;

  if (_BUFS) {
    global_data_A36465.aft_A_sample.filtered_adc_reading = (ADCBUF1 << 6);
    global_data_A36465.aft_B_sample.filtered_adc_reading = (ADCBUF2 << 6);
  } else {
    global_data_A36465.aft_A_sample.filtered_adc_reading = (ADCBUF9 << 6);
    global_data_A36465.aft_B_sample.filtered_adc_reading = (ADCBUFA << 6);
  }
  ETMAnalogScaleCalibrateADCReading(&global_data_A36465.aft_A_sample);
  ETMAnalogScaleCalibrateADCReading(&global_data_A36465.aft_B_sample);

  global_data_A36465.aft_A_sample_history[(global_data_A36465.pulses_on_this_run & 0xF)] = global_data_A36465.aft_A_sample.reading_scaled_and_calibrated;
  global_data_A36465.aft_B_sample_history[(global_data_A36465.pulses_on_this_run & 0xF)] = global_data_A36465.aft_B_sample.reading_scaled_and_calibrated;
  
  if (global_data_A36465.pulses_on_this_run >= 0xF) {
    // Average the sample histories
    temp = global_data_A36465.aft_A_sample_history[0];
    temp += global_data_A36465.aft_A_sample_history[1];
    temp += global_data_A36465.aft_A_sample_history[2];
    temp += global_data_A36465.aft_A_sample_history[3];
    temp += global_data_A36465.aft_A_sample_history[4];
    temp += global_data_A36465.aft_A_sample_history[5];
    temp += global_data_A36465.aft_A_sample_history[6];
    temp += global_data_A36465.aft_A_sample_history[7];
    temp += global_data_A36465.aft_A_sample_history[8];
    temp += global_data_A36465.aft_A_sample_history[9];
    temp += global_data_A36465.aft_A_sample_history[10];
    temp += global_data_A36465.aft_A_sample_history[11];
    temp += global_data_A36465.aft_A_sample_history[12];
    temp += global_data_A36465.aft_A_sample_history[13];
    temp += global_data_A36465.aft_A_sample_history[14];
    temp += global_data_A36465.aft_A_sample_history[15];
    temp >>= 4;
    global_data_A36465.aft_A_sample_filtered = temp;


    // Average the sample histories
    temp = global_data_A36465.aft_B_sample_history[0];
    temp += global_data_A36465.aft_B_sample_history[1];
    temp += global_data_A36465.aft_B_sample_history[2];
    temp += global_data_A36465.aft_B_sample_history[3];
    temp += global_data_A36465.aft_B_sample_history[4];
    temp += global_data_A36465.aft_B_sample_history[5];
    temp += global_data_A36465.aft_B_sample_history[6];
    temp += global_data_A36465.aft_B_sample_history[7];
    temp += global_data_A36465.aft_B_sample_history[8];
    temp += global_data_A36465.aft_B_sample_history[9];
    temp += global_data_A36465.aft_B_sample_history[10];
    temp += global_data_A36465.aft_B_sample_history[11];
    temp += global_data_A36465.aft_B_sample_history[12];
    temp += global_data_A36465.aft_B_sample_history[13];
    temp += global_data_A36465.aft_B_sample_history[14];
    temp += global_data_A36465.aft_B_sample_history[15];
    temp >>= 4;
    global_data_A36465.aft_B_sample_filtered = temp;

  } else {
    global_data_A36465.aft_A_sample_filtered = global_data_A36465.aft_A_sample.reading_scaled_and_calibrated;
    global_data_A36465.aft_B_sample_filtered = global_data_A36465.aft_B_sample.reading_scaled_and_calibrated;
  }
}

unsigned int ShiftIndex(unsigned int index, unsigned int shift) {
  unsigned int value;
  value = index;
  value &= 0x007F;
  value += shift;
  value &= 0x007F;
  return value;
}



void __attribute__((interrupt, no_auto_psv, shadow)) _INT1Interrupt(void) {
  /* 
     The INT1 Interrupt is used to sample the Sigma/Delta signals durring a pulse 
  */

  /*
    We want a delay of 3.5 us
    Inherent delay is 1.2 us
    Need to add delay of 2.3us
  */

  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();  // 1uS

  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();  // 2uS

  Nop();
  Nop();
  Nop();  // 2.3uS


  _SAMP = 0;
  PIN_TEST_POINT_A = 1;
  _DONE = 0;
  while (!_DONE);
  PIN_TEST_POINT_A = 0;
  _INT1IF = 0;

  global_data_A36465.sample_complete = 1;
}


void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
  /*
    The T1 interrupt controls the motor movent
    The maximum speed of the motor is 1/32 step per _T1 interrupt
    The maximum speed of the motor is set by setting the time of the _T1 interrupt 
  */

  _T1IF = 0;

  // Ensure that the target position is a valid value
  if (afc_motor.target_position > afc_motor.max_position) {
    afc_motor.target_position = afc_motor.max_position;
  }
  if (afc_motor.target_position < afc_motor.min_position) {
    afc_motor.target_position = afc_motor.min_position;
  }
    
  if (afc_motor.current_position > afc_motor.target_position) {
    // Move the motor one position
    afc_motor.time_steps_stopped = 0;
    afc_motor.current_position--;
  } else if (afc_motor.current_position < afc_motor.target_position) {
    // Move the motor one position the other direction
    afc_motor.time_steps_stopped = 0;
    afc_motor.current_position++;
  } else {
    // We are at our target position
    afc_motor.time_steps_stopped++;
  }
  
  if (afc_motor.time_steps_stopped >= DELAY_SWITCH_TO_LOW_POWER_MODE) {
    // use the low power look up table
    afc_motor.time_steps_stopped = DELAY_SWITCH_TO_LOW_POWER_MODE;
    PDC1 = PWMLowPowerTable[ShiftIndex(afc_motor.current_position,0)];
    PDC2 = PWMLowPowerTable[ShiftIndex(afc_motor.current_position,64)];
    PDC3 = PWMLowPowerTable[ShiftIndex(afc_motor.current_position,32)];
    PDC4 = PWMLowPowerTable[ShiftIndex(afc_motor.current_position,96)];        
  } else {
    // use the high power lookup table
    PDC1 = PWMHighPowerTable[ShiftIndex(afc_motor.current_position,0)];
    PDC2 = PWMHighPowerTable[ShiftIndex(afc_motor.current_position,64)];
    PDC3 = PWMHighPowerTable[ShiftIndex(afc_motor.current_position,32)];
    PDC4 = PWMHighPowerTable[ShiftIndex(afc_motor.current_position,96)];        
  }
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}
