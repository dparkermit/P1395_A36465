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

LTC265X U23_LTC2654;

#define MAX_POWER_TABLE_VALUES 50,94,138,182,226,269,311,353,394,435,474,513,550,586,621,654,686,717,746,773,798,822,844,864,881,897,911,923,933,940,946,949,950,949,946,940,933,923,911,897,881,864,844,822,798,773,746,717,686,654,621,586,550,513,474,435,394,353,311,269,226,182,138,94,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50

#define FULL_POWER_TABLE_VALUES 50,76,102,128,153,179,204,229,253,277,300,322,344,366,386,406,425,443,460,476,491,505,517,529,540,549,557,564,570,574,577,579,580,579,577,574,570,564,557,549,540,529,517,505,491,476,460,443,425,406,386,366,344,322,300,277,253,229,204,179,153,128,102,76,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50

#define LOW_POWER_TABLE_VALUES 50,65,79,94,109,123,137,151,165,178,191,204,217,229,240,251,262,272,282,291,299,307,315,321,327,332,337,341,344,347,349,350,350,350,349,347,344,341,337,332,327,321,315,307,299,291,282,272,262,251,240,229,217,204,191,178,165,151,137,123,109,94,79,65,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50

const unsigned int PWMHighPowerTable[128] = {FULL_POWER_TABLE_VALUES};
const unsigned int PWMLowPowerTable[128] = {LOW_POWER_TABLE_VALUES};

#define FAST_RESPONSE_TABLE_VALUES 0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3
#define SLOW_RESPONSE_TABLE_VALUES 0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3

const unsigned int fast_response_lookup_table[16] = {FAST_RESPONSE_TABLE_VALUES};
const unsigned int slow_response_lookup_table[16] = {SLOW_RESPONSE_TABLE_VALUES};

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
    afc_motor.min_position = AFC_MOTOR_MIN_POSITION;
    afc_motor.max_position = AFC_MOTOR_MAX_POSITION;
    afc_motor.target_position = afc_motor.home_position;
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
	//DoAFC();
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


void DoA36465(void) {
  ETMCanSlaveDoCan();
  local_debug_data.debug_0 = afc_motor.target_position;
  local_debug_data.debug_1 = afc_motor.current_position;
  local_debug_data.debug_2 = afc_motor.home_position;
  local_debug_data.debug_3 = afc_motor.max_position;
  local_debug_data.debug_4 = afc_motor.min_position;
  local_debug_data.debug_5 = global_data_A36465.control_state;
  local_debug_data.debug_6 = global_data_A36465.manual_target_position;
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

  ADCON2 = ADCON2_SETTING;
  ADCON3 = ADCON3_SETTING;
  ADCHS  = ADCHS_SETTING;
  ADPCFG = ADPCFG_SETTING;
  ADCSSL = ADCSSL_SETTING;
  ADCON1 = ADCON1_SETTING;
  
  _INT1IF = 0;
  _INT1IP = 7;
  _INT1IE = 1;

  _INT1IE = 0;

  
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


  // Initialize the Can module
  ETMCanSlaveInitialize();


  // Initialize LTC DAC
  SetupLTC265X(&U23_LTC2654, ETM_SPI_PORT_1, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);

  
}










unsigned int GetDirectionToMove(int error_reading);
unsigned int GetIndexToMove(int error_reading);

unsigned int GetDirectionToMove(int error_reading) {
  if (error_reading > 0) {
    return 1;
  } else {
    return 0;
  }
}

unsigned int GetIndexToMove(int error_reading) {
  if (error_reading < 0) {
    error_reading = -error_reading;
  }
  error_reading >>= 3;
  if (error_reading >= 15) {
    error_reading = 15;
  }
  return error_reading;
}

#define MIN_ERROR_FAST_AFC                         16
#define MAX_NUMBER_OF_PULSES_FOR_STARTUP_RESPONSE  256

void DoAFC(void) {
  unsigned int direction_move;
  unsigned int index_move;
  unsigned int steps_to_move;
  unsigned int position_now;
  unsigned int new_target_position;

  position_now = afc_motor.current_position;
  
  direction_move = GetDirectionToMove(global_data_A36465.frequency_error_filtered);
  index_move = GetIndexToMove(global_data_A36465.frequency_error_filtered);

  if (!global_data_A36465.fast_afc_done) {
    /*
      The magnetron has just turned on after being off for a period of time.
      The tuner *could* be wildly out of position.
      We need to react to the incoming data from AFC very quickly
      This means less filtering and and high gain integral response - Max 4 Steps per sample
    */

    steps_to_move = fast_response_lookup_table[index_move];

    if ((global_data_A36465.pulses_on_this_run >= 4) && (global_data_A36465.frequency_error_filtered <= MIN_ERROR_FAST_AFC)) {
      global_data_A36465.fast_afc_done = 1;
    }
    if (global_data_A36465.pulses_on_this_run >= MAX_NUMBER_OF_PULSES_FOR_STARTUP_RESPONSE) {
      global_data_A36465.fast_afc_done = 1;
    }
 
  } else {
    steps_to_move = slow_response_lookup_table[index_move];
  }
  
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
