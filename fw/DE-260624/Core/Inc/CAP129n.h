/*
 *	This is the header file of the CAP1293/6/8 library.
 */



#ifndef __CAP129n_H__
#define __CAP129n_H__

#include <string.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h"
#include "CAP129n_registers.h"

//Default I2C address
#define DEFAULT_I2C_ADDR 0x28

//Define register values

// Product ID
#define PROD_ID_VALUE_1293 0x6F
#define PROD_ID_VALUE_1296 0x69
#define PROD_ID_VALUE_1298 0x71

#define MODEL_CAP1293 PROD_ID_VALUE_1293
#define MODEL_CAP1296 PROD_ID_VALUE_1296
#define MODEL_CAP1298 PROD_ID_VALUE_1298

// Capacitive sensor input 
#define OFF 0x00
#define ON 0x01


// Sensitivity for touch detection
#define SENSITIVITY_128X 0x00 //Most sensitive
#define SENSITIVITY_64X 0x01
#define SENSITIVITY_32X 0x02  //Default
#define SENSITIVITY_16X 0x03
#define SENSITIVITY_8X 0x04
#define SENSITIVITY_4X 0x05
#define SENSITIVITY_2X 0x06
#define SENSITIVITY_1X 0x07 //Least sensitive


//Max hold duration before recalibration options (in ms)
#define MAX_DURRATION_560 0x00
#define MAX_DURRATION_840 0x01
#define MAX_DURRATION_1120 0x02
#define MAX_DURRATION_1400 0x03
#define MAX_DURRATION_1680 0x04
#define MAX_DURRATION_2240 0x05
#define MAX_DURRATION_2800 0x06
#define MAX_DURRATION_3360 0x07
#define MAX_DURRATION_3920 0x08
#define MAX_DURRATION_4480 0x09
#define MAX_DURRATION_5600 0x0A	//Default
#define MAX_DURRATION_6720 0x0B
#define MAX_DURRATION_7840 0x0C
#define MAX_DURRATION_8906 0x0D
#define MAX_DURRATION_10080 0x0E
#define MAX_DURRATION_11200 0x0F

//begin function statuses
#define ERR_NO_DEVICE_AT_ADDRESS 1
#define ERR_WRONG_PROD_ID 2
#define BEGIN_SUCCESS 0

//Pattern detection options
#define MTP_MODE_SPECIFIC 1
#define MTP_MODE_MINIMAL_TOUCHES 2

#define MTP_TRESHOLD_12_5 1
#define MTP_TRESHOLD_25 2
#define MTP_TRESHOLD_37_5 3
#define MTP_TRESHOLD_100 4
// Sensitivity Control Register
typedef union {
  struct
  {
    uint8_t BASE_SHIFT : 4;
    uint8_t DELTA_SENSE : 3;
    uint8_t EMPTY_1 : 1;
  } SENSITIVITY_CONTROL_FIELDS;
  uint8_t SENSITIVITY_CONTROL_COMBINED;
} SENSITIVITY_CONTROL_REG;

// General Status Register
typedef union {
  struct
  {
    uint8_t TOUCH : 1;
    uint8_t MTP : 1;
    uint8_t MULT : 1;
    uint8_t EMPTY_1 : 1;
    uint8_t PWRS : 1;
    uint8_t ACAL_FAIL : 1;
    uint8_t BC_OUT : 1;
    uint8_t EMPTY_2 : 1;
  } GENERAL_STATUS_FIELDS;
  uint8_t GENERAL_STATUS_COMBINED;
} GENERAL_STATUS_REG;

// Sensor Input Status Register
typedef union {
  struct
  {
    uint8_t CS1 : 1;
    uint8_t CS2 : 1;
    uint8_t CS3 : 1;
    uint8_t CS4 : 1;
    uint8_t CS5 : 1;
    uint8_t CS6 : 1;
	uint8_t CS7 : 1;
	uint8_t CS8 : 1;
  } SENSOR_INPUT_STATUS_FIELDS;
  uint8_t SENSOR_INPUT_STATUS_COMBINED;
} SENSOR_INPUT_STATUS_REG;

// Sensor Input Enable Register
typedef union {
  struct
  {
    uint8_t CS1_EN : 1;
    uint8_t CS2_EN : 1;
    uint8_t CS3_EN : 1;
	uint8_t CS4_EN : 1;
	uint8_t CS5_EN : 1;
	uint8_t CS6_EN : 1;
	uint8_t CS7_EN : 1;
	uint8_t CS8_EN : 1;
  } SENSOR_INPUT_ENABLE_FIELDS;
  uint8_t SENSOR_INPUT_ENABLE_COMBINED;
} SENSOR_INPUT_ENABLE_REG;

// Main Control Register
typedef union {
  struct
  {
    uint8_t INT : 1;
    uint8_t COMBO : 1;
	uint8_t C_GAIN : 2;
    uint8_t DSLEEP : 1;
    uint8_t STBY : 1;
    uint8_t GAIN : 2;
  } MAIN_CONTROL_FIELDS;
  uint8_t MAIN_CONTROL_COMBINED;
} MAIN_CONTROL_REG;

// Interrupt Enable Register
typedef union {
  struct
  {
    uint8_t CS1_INT_EN : 1;
    uint8_t CS2_INT_EN : 1;
    uint8_t CS3_INT_EN : 1;
	uint8_t CS4_INT_EN : 1;
	uint8_t CS5_INT_EN : 1;
	uint8_t CS6_INT_EN : 1;
	uint8_t CS7_INT_EN : 1;
	uint8_t CS8_INT_EN : 1;
  } INTERRUPT_ENABLE_FIELDS;
  uint8_t INTERRUPT_ENABLE_COMBINED;
} INTERRUPT_ENABLE_REG;

//-----BEGIN CONFIGURATION REGISTERS-----
typedef union {
  struct
  {
    uint8_t EMPTY_1 : 3;
    uint8_t MAX_DUR_EN : 1;
    uint8_t DIS_ANA_NOISE : 1;
	uint8_t DIS_DIG_NOISE : 1;
	uint8_t EMPTY_2 : 1;
	uint8_t TIMEOUT : 1;
  } CONFIGURATION_FIELDS;
  uint8_t CONFIGURATION_COMBINED;
} CONFIGURATION_REG;
typedef union {
  struct
  {
    uint8_t INT_REL_n : 1;
    uint8_t ACAL_FAIL_INT : 1;
    uint8_t DIS_RF_NOISE : 1;
	uint8_t SHOW_RF_NOISE : 1;
	uint8_t BC_OUT_INT : 1;
	uint8_t BLK_PWR_CTRL : 1;
	uint8_t BC_OUT_RECAL : 1;
	uint8_t EMPTY_1 : 1;
  } CONFIGURATION_2_FIELDS;
  uint8_t CONFIGURATION_2_COMBINED;
} CONFIGURATION_2_REG;
typedef union {
  struct
  {
    uint8_t RPT_RATE : 4;
    uint8_t MAX_DUR : 4;
  } SENSOR_INPUT_CONFIGURATION_FIELDS;
  uint8_t SENSOR_INPUT_CONFIGURATION_COMBINED;
} SENSOR_INPUT_CONFIGURATION_REG;

typedef union {
  struct
  {
    uint8_t EMPTY_1 : 2;
	uint8_t B_MULT_T : 2;
	uint8_t EMPTY_2 : 3;	
    uint8_t MULT_BLK_EN : 1;
  } MULTIPLE_TOUCH_CONFIGURATION_FIELDS;
  uint8_t MULTIPLE_TOUCH_CONFIGURATION_COMBINED;
} MULTIPLE_TOUCH_CONFIGURATION_REG;

typedef union {
  struct
  {
	uint8_t MTP_ALERT : 1;
	uint8_t COMP_PTRN : 1;
	uint8_t MTP_TH : 2;		  
    uint8_t EMPTY_1 : 3;
	uint8_t MTP_EN : 1;
  } MULTIPLE_TOUCH_PATTERN_CONFIGURATION_FIELDS;
  uint8_t MULTIPLE_TOUCH_PATTERN_CONFIGURATION_COMBINED;
} MULTIPLE_TOUCH_PATTERN_CONFIGURATION_REG;

typedef union {
  struct
  {  
	uint8_t CS1_RPT_EN: 1;
	uint8_t CS2_RPT_EN: 1;
	uint8_t CS3_RPT_EN: 1;
	uint8_t CS4_RPT_EN: 1;
	uint8_t CS5_RPT_EN: 1;
	uint8_t CS6_RPT_EN: 1;
	uint8_t CS7_RPT_EN: 1;
	uint8_t CS8_RPT_EN: 1;	
  } REPEAT_RATE_ENABLE_FIELDS;
  uint8_t REPEAT_RATE_ENABLE_COMBINED;
} REPEAT_RATE_ENABLE_REG;

typedef union {
  struct
  {
	uint8_t CS1_PTRN : 1;
	uint8_t CS2_PTRN : 1;
	uint8_t CS3_PTRN : 1;		  
    uint8_t CS4_PTRN : 1;
	uint8_t CS5_PTRN : 1;
	uint8_t CS6_PTRN : 1;
	uint8_t CS7_PTRN : 1;
	uint8_t CS8_PTRN : 1;
  } MULTIPLE_TOUCH_PATTERN_FIELDS;
  uint8_t MULTIPLE_TOUCH_PATTERN_COMBINED;
} MULTIPLE_TOUCH_PATTERN_REG;

//-----END CONFIGURATION REGISTERS-----

typedef union {
  struct
  {
	uint8_t CS1_CAL : 1;
	uint8_t CS2_CAL : 1;
	uint8_t CS3_CAL : 1;		  
    uint8_t CS4_CAL : 1;
	uint8_t CS5_CAL : 1;
	uint8_t CS6_CAL : 1;
	uint8_t CS7_CAL : 1;
	uint8_t CS8_CAL : 1;
  } CALIBRATION_ACTIVATE_AND_STATUS_FIELDS;
  uint8_t CALIBRATION_ACTIVATE_AND_STATUS_COMBINED;
} CALIBRATION_ACTIVATE_AND_STATUS_REG;


// Signal Guard Enable Register
typedef union {
	struct
	{
		uint8_t CS1_SG_EN : 1;
		uint8_t CS2_SG_EN : 1;
		uint8_t CS3_SG_EN : 1;
		uint8_t CS4_SG_EN : 1;
		uint8_t CS5_SG_EN : 1;
		uint8_t CS6_SG_EN : 1;
		uint8_t CS7_SG_EN : 1;
		uint8_t CS8_SG_EN : 1;
	} SIGNAL_GUARD_ENABLE_FIELDS;
	uint8_t SIGNAL_GUARD_ENABLE_COMBINED;
} SIGNAL_GUARD_ENABLE_REG;



  
int begin(uint8_t deviceAddress, uint8_t sensitivity, bool interrupts, bool sgEnable);  
bool isConnected(void);
void setSensitivity(uint8_t sensitivity);
uint8_t getSensitivity(void);

// Check if a capacitive touch panel has been touched
bool isTouched(uint8_t id);
bool isAnyTouched(void);
bool isMTPTouched(void);

//-----BEGIN CONFIGURATION FUNCTIONS-----
void enableSMBusTimeout(void);
void disableSMBusTimeout(void);
void setMaximumHoldDuration(uint8_t duration);
void enableMaximumHoldDuration(void);
void disableMaximumHoldDuration(void);
void enableRFNoiseFilter(void);
void disableRFNoiseFilter(void);

void enableMultipleTouchLimit(void);
void disableMultipleTouchLimit(void);
void setMultipleTouchLimit(uint8_t touches);
void enableMTPDetection(void);
void disableMTPDetection(void);
void setMTPDetectionMode(uint8_t mode);
void setMTPPatternSpecificButtons(bool cs1_mtp, bool cs2_mtp, bool cs3_mtp, bool cs4_mtp, bool cs5_mtp, bool cs6_mtp, bool cs7_mtp, bool cs8_mtp);
void setMTPDetectionTreshold(uint8_t tresh);
void setMTPDetectionMinimalButtons(uint8_t btns);
void enableMTPInterrupt(void);
void disableMTPInterrupt(void);

void disableInterruptRepeatRate(void);
void enableInterruptRepeatRate(void);
void disableInterruptOnRelease(void);
void enableInterruptOnRelease(void);
//-----END CONFIGURATION FUNCTIONS-----

void calibrateTouch(uint8_t id);
void calibrateAll(void);

// Clears INT bit
void clearInterrupt(void);

//Signal guard
void enableSignalGuard(void);
void disableSignalGuard(void);

//Enable/disable specific buttons
void enableSensing(uint8_t id);
void disableSensing(uint8_t id);
bool isEnabledSensing(uint8_t id);

// Configures if interrupt triggers on touch
void setInterruptDisabled(void);
void setInterruptEnabled(void);
//bool isInterruptEnabled();

uint8_t checkMainControl(void);
uint8_t checkStatus(void);


uint8_t getInputStatus(void);  //dodano VM
uint8_t getGeneralStatus(void);  //dodano VM
uint8_t getMainControl(void);  //dodano VM
uint8_t readRegister(CAP129n_Register reg);


extern uint8_t _deviceAddress;   //Keeps track of I2C address. 
extern uint8_t _specifiedModel;
extern bool _singalGuardEnabled;

// Read and write to registers

void readRegisters(CAP129n_Register reg, uint8_t *buffer, uint8_t len);
void writeRegister(CAP129n_Register reg, uint8_t data);
void writeRegisters(CAP129n_Register reg, uint8_t *buffer, uint8_t len);
  

  
#endif
