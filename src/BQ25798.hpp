/**
 * @file
 *    BQ25798.h
 *  
 * @details 
 *  BQ25798 I2C Controlled, 1- to 4-Cell, 5-A Buck-Boost Battery Charger with Dual-Input
 *  Selector, MPPT for Solar Panels and Fast Backup Mode.
 *
 * @brief
 *    This header file contains register definitions and function declarations for controlling the BQ25798 charging IC.
 *    The BQ25798 is a versatile buck-boost charger that handles a wide range of input and output voltages and currents,
 *    making it suitable for various battery charging applications.
 *
 *    This library provides detailed control over the BQ25798's charging parameters, including system voltage regulation,
 *    charge current limits, and input voltage limits. It is ideal for applications requiring precise control over battery charging.
 *
 *    Functions include initializing the charger, reading and setting various register values, managing ADC functionality,
 *    and printing configuration settings for debugging and monitoring purposes.
 *
 * @example
 *    ```cpp
 *    #include <BQ25798.h>
 *    
 *    void setup() {
 *        Serial.begin(115200);
 *        initialize_BQ25798(4, 1400, 10000, 14600, 1400, true, true); // Example initialization
 *    }
 *
 *    void loop() {
 *        // Example usage
 *        BQ25798_printAllADCValues();
 *        delay(5000);
 *    }
 *    ```
 *
 * @version 1.0.0
 * @date 2023-12-25
 * @author
 *    Itay Nave, Embedded Software Engineer
 *
 * @license
 *    © 2023 Itay Nave. All rights reserved.
 *    This software is provided "as is", without warranty of any kind, express or implied.
 *    Redistribution and use in source and binary forms, with or without modification,
 *    are permitted provided that the above copyright notice and this permission notice
 *    appear in all copies.
 */

#pragma once

#include <Arduino.h>
#include "I2C.hpp"

#define BQ25798_SLAVE_ADDRESS 0x6B

// Enum for register addresses
enum BQ25798_Registers
{
    REG00_Minimal_System_Voltage = 0x00, // Sets the minimum system voltage. It resets to 3.5V upon adapter unplugging and is not reset by the REG_RST and WATCHDOG. The range is 3.5V to 24V with a fixed offset of 0V and a bit step size of 100mV.
    REG01_Charge_Voltage_Limit = 0x01,   // Sets the charge voltage limit. Its range is 3.5V to 19.2V, with a fixed offset of 0V and a bit step size of 16mV. It resets to 4.512V upon adapter unplugging and is not reset by the REG_RST and WATCHDOG.
    REG03_Charge_Current_Limit = 0x03,   // Sets the charge current limit. During power-on reset (POR), the device reads the resistance tied to the PROG pin to identify the default battery cell count and determine the default power-on battery charging current: 1A. The range is 50mA to 5000mA.
    REG05_Input_Voltage_Limit = 0x05,    // Sets the absolute VINDPM (Voltage Input Dynamic Power Management) threshold. It resets to 3600mV upon adapter unplugging and sets to the value based on the VBUS measurement when the adapter plugs in. The range is 3600mV to 22000mV.
    REG06_Input_Current_Limit = 0x06,    // Sets the input current limit. It resets to 3000mA upon adapter unplugging and is not reset by the REG_RST and WATCHDOG. The range is 100mA to 3200mA, with a fixed offset of 0mA and a bit step size of 50mA.
    REG08_Precharge_Control = 0x08,      // Controls the precharge current limit and battery voltage thresholds for the transition from precharge to fast charge, defined as a ratio of battery regulation limit (VREG).
    REG09_Termination_Control = 0x09,    // Controls the termination current. The range is 40mA to 1000mA with a fixed offset of 0mA and a bit step size of 40mA.
    REG0A_Re_charge_Control = 0x0A,      // Sets the battery recharge threshold offset (below VREG) and recharge deglitch time. The range for the recharge threshold offset is 50mV to 800mV with a fixed offset of 50mV and a bit step size of 50mV.
    REG0B_VOTG_regulation = 0x0B,        // Regulates the OTG (On-The-Go) output voltage.
    REG0D_IOTG_regulation = 0x0D,        // Regulates the OTG output current.
    REG0E_Timer_Control = 0x0E,          // Controls various timers including safety timers for charging.
    REG0F_Charger_Control_0 = 0x0F,      // General charger control register 0.
    REG10_Charger_Control_1 = 0x10,      // General charger control register 1.
    REG11_Charger_Control_2 = 0x11,      // General charger control register 2.
    REG12_Charger_Control_3 = 0x12,      // General charger control register 3.
    REG13_Charger_Control_4 = 0x13,      // General charger control register 4.
    REG14_Charger_Control_5 = 0x14,      // General charger control register 5.
    REG15_MPPT_Control = 0x15,           // Controls the MPPT (Maximum Power Point Tracking) functionality for solar charging.
    REG16_Temperature_Control = 0x16,    // Controls temperature monitoring and related safety mechanisms.
    REG17_NTC_Control_0 = 0x17,          // Controls NTC (Negative Temperature Coefficient) thermistor monitoring.
    REG18_NTC_Control_1 = 0x18,          // Additional control for NTC thermistor monitoring.
    REG19_ICO_Current_Limit = 0x19,      // Sets the ICO (Input Current Optimizer) current limit.
    REG1B_Charger_Status_0 = 0x1B,       // Provides the charger status information.
    REG1C_Charger_Status_1 = 0x1C,       // Additional charger status information.
    REG1D_Charger_Status_2 = 0x1D,       // Additional charger status information.
    REG1E_Charger_Status_3 = 0x1E,       // Additional charger status information.
    REG1F_Charger_Status_4 = 0x1F,       // Additional charger status information.
    REG20_FAULT_Status_0 = 0x20,         // Provides fault status information.
    REG21_FAULT_Status_1 = 0x21,         // Additional fault status information.
    REG22_Charger_Flag_0 = 0x22,         // Charger flag register 0.
    REG23_Charger_Flag_1 = 0x23,         // Charger flag register 1.
    REG24_Charger_Flag_2 = 0x24,         // Charger flag register 2.
    REG25_Charger_Flag_3 = 0x25,         // Charger flag register 3.
    REG26_FAULT_Flag_0 = 0x26,           // Fault flag register 0.
    REG27_FAULT_Flag_1 = 0x27,           // Fault flag register 1.
    REG28_Charger_Mask_0 = 0x28,         // Charger mask register 0.
    REG29_Charger_Mask_1 = 0x29,         // Charger mask register 1.
    REG2A_Charger_Mask_2 = 0x2A,         // Charger mask register 2.
    REG2B_Charger_Mask_3 = 0x2B,         // Charger mask register 3.
    REG2C_FAULT_Mask_0 = 0x2C,           // Fault mask register 0.
    REG2D_FAULT_Mask_1 = 0x2D,           // Fault mask register 1.
    REG2E_ADC_Control = 0x2E,            // Controls ADC (Analog-to-Digital Converter) functionality.
    REG2F_ADC_Function_Disable_0 = 0x2F, // Disables specific ADC functions.
    REG30_ADC_Function_Disable_1 = 0x30, // Additional control for disabling ADC functions.
    REG31_IBUS_ADC = 0x31,               // ADC result for IBUS (Input Bus Current).
    REG33_IBAT_ADC = 0x33,               // ADC result for IBAT (Battery Current).
    REG35_VBUS_ADC = 0x35,               // ADC result for VBUS (Input Bus Voltage).
    REG37_VAC1_ADC = 0x37,               // ADC result for VAC1 (Charger Input 1 Voltage).
    REG39_VAC2_ADC = 0x39,               // ADC result for VAC2 (Charger Input 2 Voltage).
    REG3B_VBAT_ADC = 0x3B,               // ADC result for VBAT (Battery Voltage).
    REG3D_VSYS_ADC = 0x3D,               // ADC result for VSYS (System Voltage).
    REG3F_TS_ADC = 0x3F,                 // ADC result for TS (Temperature Sense).
    REG41_TDIE_ADC = 0x41,               // ADC result for TDIE (Die Temperature).
    REG43_D_Plus_ADC = 0x43,             // ADC result for D+ (Data Line Positive).
    REG45_D_Minus_ADC = 0x45,            // ADC result for D- (Data Line Negative).
    REG47_DPDM_Driver = 0x47,            // Controls the DPDM (D+/D-) driver.
    REG48_Part_Information = 0x48        // Provides part information.
};


// Define an enum for the charge status states
typedef enum
{
    Not_Charging = 0,
    Trickle_Charge,
    Pre_charge,
    Fast_Charge_CC_Mode,
    Taper_Charge_CV_Mode,
    Reserved_State,
    Top_off_Timer_Active_Charging,
    Charge_Termination_Done
} ChargeStatus;

// ChargeStatus
extern ChargeStatus chargingStatus;

// Configuration external variables
extern uint8_t cellCount;
extern uint16_t systemVoltage;
extern uint16_t chargeVoltage;
extern uint16_t chargeCurrent;
extern uint16_t inputCurrent;
extern bool isIINDPMEnabled;
extern bool isEXTILIMEnabled;
extern bool isMPPTEnabled;
extern uint8_t solarPanel_OR_Grid;

// ADC external variables
extern int16_t vbusValue;
extern int16_t ibusValue;
extern int16_t vbatValue;

// Initialize BQ25798
void initialize_BQ25798(int cellCount, int inputCurrentLimit, int minimalSystemVoltage, int chargeVoltageLimit, int chargeCurrentLimit, bool enableCharging, bool enableMPPT);
uint8_t BQ25798_readRegister(uint8_t regAddress);
uint16_t BQ25798_readRegister16(uint8_t regAddress);

// Charge Status
ChargeStatus BQ25798_getChargeStatus();
const char *BQ25798_getChargeStatusString(ChargeStatus status);

// MPPT
void BQ25798_enableMPPT();
void BQ25798_disableMPPT();

// ADC Function declarations
int16_t BQ25798_readADCValue(uint8_t regAddress);
void BQ25798_enableADC();
void BQ25798_disableADC();

// Enable IBAT Discharge Sensing
void BQ25798_toggleIBATCurrentSensing();
void BQ25798_enableDischargeCurrentSensing();
void BQ25798_enableChargeCurrentSensing();

// Reset registers
void BQ25798_resetRegisters();
void BQ25798_doNotResetRegisters();

// Watchdog
void BQ25798_disableWatchdogTimer();
void BQ25798_setDefaultWatchdogTimer();

// Battery Cell Count
void BQ25798_setBatteryCellCount(uint8_t cellCount);

// INPUT BUS (Solar panel or power supply)
void BQ25798_enableInputCurrentLimit();
void BQ25798_disableInputCurrentLimit();
void BQ25798_setMinimalSystemVoltage(uint16_t voltage);
void BQ25798_setInputCurrentLimit(uint16_t current);

// Charger
void BQ25798_setChargeVoltageLimit(uint16_t voltage);
void BQ25798_setChargeCurrentLimit(uint16_t current);

// Enable/disable charging
void BQ25798_enableCharging();
void BQ25798_disableCharging();

// Read the ADC values and update the external variables
void BQ25798_updateADCValues();

// Determine Power Source Based On VBUS Changes
void determinePowerSourceBasedOnVBUSChanges();

// Print functions
void BQ25798_printConfigurationSettings(bool allow_print);
void BQ25798_printAllADCValues();
void BQ25798_printLocalADCValues();
