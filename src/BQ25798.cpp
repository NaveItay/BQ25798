#include "BQ25798.hpp"

ChargeStatus chargingStatus = Not_Charging;

// Configuration variables
uint8_t cellCount;
uint16_t systemVoltage;
uint16_t chargeVoltage;
uint16_t chargeCurrent;
uint16_t inputCurrent;
bool isIINDPMEnabled;
bool isEXTILIMEnabled;
bool isMPPTEnabled;

// ADC variables
int16_t vbusValue = 0;
int16_t ibusValue = 0;
int16_t vbatValue = 0;
int16_t ibatValue = 0;

#define VBUS_ARRAY_SIZE 1440
int16_t vbusValues[VBUS_ARRAY_SIZE]; // Declare the array to store VBUS values
uint16_t currentIndex = 0;           // Index to keep track of the current position in the array
uint8_t solarPanel_OR_Grid = 255;    // (255) -> Unknown yet (24Hours) | (10) -> Solar Panel | (20) -> GRID

static void checkVbusValuesArray();

void  initialize_BQ25798(int cellCount, int inputCurrentLimit, int minimalSystemVoltage, int chargeVoltageLimit, int chargeCurrentLimit, bool enableCharging, bool enableMPPT)
{
    BQ25798_resetRegisters(); // Reset all registers to default
    delay(100);
    
    BQ25798_disableWatchdogTimer(); // Disable watchdog timer
    BQ25798_enableADC();            // Enable ADC for monitoring

    // Battery
    BQ25798_setBatteryCellCount(cellCount); // Set cell count to 4S

    // Input BUS (Solar panel or power supply)
    BQ25798_setInputCurrentLimit(inputCurrentLimit);     // Set Input Current limit to 1.4A
    BQ25798_setMinimalSystemVoltage(minimalSystemVoltage); // Set minimal system voltage to 10.0V

    // Charger
    BQ25798_setChargeVoltageLimit(chargeVoltageLimit); // The charge voltage for LiFePO4 is typically 3.65V per cell, for 4 cells it's 14.6V
    BQ25798_setChargeCurrentLimit(chargeCurrentLimit);  // Set charge current limit to 1.4A

    // Enable / Disable charging based on flag
    if (enableCharging)
    {
        BQ25798_enableCharging();
    }
    else
    {
        BQ25798_disableCharging();
    }

    // Enable / Disable MPPT based on flag
    if (enableMPPT)
    {
        BQ25798_enableMPPT();
    }
    else
    {
        BQ25798_disableMPPT();
    }

    // IBATD is charge Sensing
    BQ25798_enableChargeCurrentSensing();

    // Print configuration
    BQ25798_printConfigurationSettings(true);
}

uint8_t BQ25798_readRegister(uint8_t regAddress)
{
    uint8_t regValue;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, regAddress, 1, &regValue);
    return regValue;
}

uint16_t BQ25798_readRegister16(uint8_t regAddress)
{
    uint8_t data[2];
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, regAddress, 2, data);
    return (uint16_t)(data[0] << 8 | data[1]);
}

/**
 * @brief Retrieves the current charge status from the BQ25798 charger.
 *
 * This function reads the Charger Status 1 Register (REG1C) from the BQ25798 battery charger
 * and extracts the current charging status encoded in the CHG_STAT bits (bits 5 to 7).
 * It returns the charging status as a value of the ChargeStatus enum, which provides
 * a clearer and more manageable representation of the charging state in the rest of the program.
 *
 * ChargeStatus enum values:
 * - 0: Not_Charging — The battery is not currently charging.
 * - 1: Trickle_Charge — The battery is charging at a trickle charge rate, used for very low battery levels.
 * - 2: Pre_charge — The battery is in pre-charge state, typically used when the battery voltage is below the normal range.
 * - 3: Fast_Charge_CC_Mode — The battery is charging in constant current mode at a fast rate.
 * - 4: Taper_Charge_CV_Mode — The battery is in constant voltage mode where the current gradually tapers down as the battery gets closer to full charge.
 * - 5: Reserved_State — A reserved state not currently used for normal operation.
 * - 6: Top_off_Timer_Active_Charging — The battery is topping off; this mode kicks in to fully optimize battery charge near full capacity.
 * - 7: Charge_Termination_Done — Charging is complete, and no further charging is occurring.
 *
 * @return ChargeStatus The current charging status as an enumerated type, making it
 *         more understandable and easier to handle in comparison to raw numeric codes.
 */
ChargeStatus BQ25798_getChargeStatus()
{
    // Read the Charger Status 1 Register (REG1C)
    uint8_t chargeStatus = BQ25798_readRegister(REG1C_Charger_Status_1);

    // Extract the CHG_STAT bits (5-7) and align them to the least significant bits
    uint8_t chg_stat = (chargeStatus & 0b11100000) >> 5;

    // Cast the numeric status to the ChargeStatus enum type and return it
    return (ChargeStatus)chg_stat;
}

const char *BQ25798_getChargeStatusString(ChargeStatus status)
{
    switch (status)
    {
    case Not_Charging:
        return "Not Charging";
    case Trickle_Charge:
        return "Trickle Charge";
    case Pre_charge:
        return "Pre-charge";
    case Fast_Charge_CC_Mode:
        return "Fast Charge (CC Mode)";
    case Taper_Charge_CV_Mode:
        return "Taper Charge (CV Mode)";
    case Reserved_State:
        return "Reserved State";
    case Top_off_Timer_Active_Charging:
        return "Top-off Timer Active Charging";
    case Charge_Termination_Done:
        return "Charge Termination Done";
    default:
        return "Unknown Status";
    }
}

void BQ25798_enableMPPT()
{
    uint8_t mpptControl;
    // Read the current value of the MPPT control register
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG15_MPPT_Control, 1, &mpptControl);
    // Set the EN_MPPT bit (bit 0) to 1 to enable MPPT
    mpptControl |= 0b00000001;
    // Write the updated value back to the MPPT control register
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG15_MPPT_Control, 1, &mpptControl);
}

void BQ25798_disableMPPT()
{
    uint8_t mpptControl;
    // Read the current value of the MPPT control register
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG15_MPPT_Control, 1, &mpptControl);
    // Clear the EN_MPPT bit (bit 0) to 0 to disable MPPT
    mpptControl &= ~0b00000001;
    // Write the updated value back to the MPPT control register
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG15_MPPT_Control, 1, &mpptControl);
}

int16_t BQ25798_readADCValue(uint8_t regAddress)
{
    uint8_t data[2];
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, regAddress, 2, data);
    return static_cast<int16_t>(data[0] << 8 | data[1]);
}

void BQ25798_enableADC()
{
    uint8_t adcControl;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG2E_ADC_Control, 1, &adcControl);
    adcControl |= 0b10000000; // Set the ADC_EN bit (bit 7) to 1
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG2E_ADC_Control, 1, &adcControl);
}

void BQ25798_disableADC()
{
    uint8_t adcControl;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG2E_ADC_Control, 1, &adcControl);
    adcControl &= ~0b10000000; // Clear the ADC_EN bit (bit 7)
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG2E_ADC_Control, 1, &adcControl);
}

void BQ25798_toggleIBATCurrentSensing()
{
    uint8_t chargerControl5;
    // Read the current state of Charger Control 5 register
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG14_Charger_Control_5, 1, &chargerControl5);
    // Toggle the EN_IBAT bit (bit 5)
    chargerControl5 ^= 0b00100000; // Use XOR to toggle bit 5
    // Write back the modified value to the register
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG14_Charger_Control_5, 1, &chargerControl5);
}

void BQ25798_enableDischargeCurrentSensing()
{
    uint8_t chargerControl5;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG14_Charger_Control_5, 1, &chargerControl5);
    // Set the EN_IBAT bit (bit 5) to 1 to enable discharge current sensing
    chargerControl5 |= 0b00100000; // Binary representation for setting bit 5
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG14_Charger_Control_5, 1, &chargerControl5);
}

void BQ25798_enableChargeCurrentSensing()
{
    uint8_t chargerControl5;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG14_Charger_Control_5, 1, &chargerControl5);
    // Clear the EN_IBAT bit (bit 5) to 0 to enable charge current sensing
    chargerControl5 &= ~0b00100000; // Binary representation for clearing bit 5
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG14_Charger_Control_5, 1, &chargerControl5);
}

void BQ25798_resetRegisters()
{
    uint8_t terminationControl;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG09_Termination_Control, 1, &terminationControl);
    terminationControl |= 0b01000000; // Set the REG_RST bit (bit 6) to 1 to reset
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG09_Termination_Control, 1, &terminationControl);
}

void BQ25798_doNotResetRegisters()
{
    uint8_t terminationControl;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG09_Termination_Control, 1, &terminationControl);
    terminationControl &= ~0b01000000; // Clear the REG_RST bit (bit 6) to not reset
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG09_Termination_Control, 1, &terminationControl);
}

void BQ25798_disableWatchdogTimer()
{
    uint8_t chargerControl1;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG10_Charger_Control_1, 1, &chargerControl1);

    // Clear the WATCHDOG_2:0 bits (bits 0-2) to disable the Watchdog timer
    chargerControl1 &= ~0b00000111; // Equivalent to clearing the lowest 3 bits
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG10_Charger_Control_1, 1, &chargerControl1);
}

void BQ25798_setDefaultWatchdogTimer()
{
    uint8_t chargerControl1;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG10_Charger_Control_1, 1, &chargerControl1);

    // Clear the WATCHDOG_2:0 bits (bits 0-2)
    chargerControl1 &= ~0b00000111;
    // Set WATCHDOG_2:0 bits to 0b101 for 40s default timer setting
    chargerControl1 |= 0b00000101;
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG10_Charger_Control_1, 1, &chargerControl1);
}

/**
 * Sets the battery cell count for the BQ25798.
 * The BQ25798 charger IC's behavior, including default charging parameters,
 * is influenced by the battery cell count. This count is typically determined
 * by the resistance connected to the PROG pin on the IC. It's crucial that
 * the resistance at the PROG pin matches the intended cell count setting
 * to ensure proper charging behavior.
 *
 * PROG Pin Resistance for Different Cell Counts:
 * - 1s cell: 3.0 kΩ (1.5 MHz) or 4.7 kΩ (750 kHz)
 * - 2s cell: 6.04 kΩ (1.5 MHz) or 8.2 kΩ (750 kHz)
 * - 3s cell: 10.5 kΩ (1.5 MHz) or 13.7 kΩ (750 kHz)
 * - 4s cell: 17.4 kΩ (1.5 MHz) or 27.0 kΩ (750 kHz)
 *
 * This function configures the cell count in the REG0A_Re_charge_Control register.
 *
 * @param cellCount The desired battery cell count (1s, 2s, 3s, or 4s).
 *                  The value must be between 1 and 4 inclusive.
 *                  Each cell count corresponds to a different voltage
 *                  and current configuration as follows:
 *
 *                  - 1s: ICHG = 1A, VSYSMIN = 3.5V, VREG = 4.2V, VREG Range = 3V - 4.99V
 *                  - 2s: ICHG = 1A, VSYSMIN = 7V, VREG = 8.4V, VREG Range = 5V - 9.99V
 *                  - 3s: ICHG = 1A, VSYSMIN = 9V, VREG = 12.6V, VREG Range = 10V - 13.99V
 *                  - 4s: ICHG = 1A, VSYSMIN = 12V, VREG = 16.8V, VREG Range = 14V - 18.8V
 *
 * The function reads the current value of the register, modifies the
 * CELL_1:0 bits (bits 7-6) to set the desired cell count, and writes
 * the updated value back to the register.
 *
 * Note: Ensure that the resistance at the PROG pin is set correctly
 * according to the desired cell count to maintain consistent and safe
 * charging behavior.
 */
void BQ25798_setBatteryCellCount(uint8_t cellCount)
{
    if (cellCount < 1 || cellCount > 4)
    {
        Serial.println("Invalid cell count!");
        return;
    }

    // Read the current value of the register
    uint8_t regValue;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG0A_Re_charge_Control, 1, &regValue);

    // Clear the CELL_1:0 bits (bits 7-6)
    regValue &= 0b00111111;

    // Set the CELL_1:0 bits according to the cellCount
    // 1s: 0b00, 2s: 0b01, 3s: 0b10, 4s: 0b11
    regValue |= ((cellCount - 1) << 6);

    // Write the updated value back to the register
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG0A_Re_charge_Control, 1, &regValue);
}

/**
 * Enables the input current limit function in the BQ25798.
 * This function manipulates the EN_EXTILIM and EN_IINDPM bits to enable
 * the input current limit set by the ILIM_HIZ pin and internal IINDPM register.
 */
void BQ25798_enableInputCurrentLimit()
{
    uint8_t chargerControl5;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG14_Charger_Control_5, 1, &chargerControl5);

    // Set the EN_IINDPM bit (bit 2) and EN_EXTILIM bit (bit 1)
    chargerControl5 |= 0b00000110;

    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG14_Charger_Control_5, 1, &chargerControl5);
}

/**
 * Disables the input current limit function in the BQ25798.
 * This function manipulates the EN_EXTILIM and EN_IINDPM bits to disable
 * the input current limit set by the ILIM_HIZ pin and internal IINDPM register.
 */
void BQ25798_disableInputCurrentLimit()
{
    uint8_t chargerControl5;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG14_Charger_Control_5, 1, &chargerControl5);

    // Clear the EN_IINDPM bit (bit 2) and EN_EXTILIM bit (bit 1)
    chargerControl5 &= ~0b00000110;

    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG14_Charger_Control_5, 1, &chargerControl5);
}

void BQ25798_setMinimalSystemVoltage(uint16_t voltage)
{
    // Ensure the voltage is within the valid range (2500mV to 16000mV)
    if (voltage < 2500 || voltage > 16000)
    {
        Serial.println("Voltage out of range!");
        return;
    }

    // Calculate the register value
    // The minimal system voltage has a fixed offset of 2500mV and a step size of 250mV
    uint8_t regValue = (voltage - 2500) / 250;

    // Write the value to the register
    // Only the lower 6 bits (VSYSMIN_5:0) of the register are used for setting the voltage
    // Ensure the upper bits (7-6) are set to 0 as they are RESERVED
    regValue &= 0b00111111; // Masking to keep only the lower 6 bits

    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG00_Minimal_System_Voltage, 1, &regValue);
}

/**
 * Sets the input current limit for the BQ25798.
 * The input current limit is used to control the maximum current the charger can draw from the input source.
 *
 * @param current The desired input current limit in mA. The value must be between 100mA and 3300mA.
 *                The register has a step size of 10mA, so the current value will be rounded to the nearest valid step.
 */
void BQ25798_setInputCurrentLimit(uint16_t current)
{
    // Ensure the current is within the valid range (100mA to 3300mA)
    if (current < 100 || current > 3300)
    {
        Serial.println("Current out of range!");
        return;
    }

    // Calculate the register value
    // The input current limit has a step size of 10mA
    uint16_t regValue = current / 10;

    // Prepare the data to be written (2 bytes)
    uint8_t data[2];
    data[0] = (regValue >> 8) & 0b00000001; // Upper 1 bit (bit 8)
    data[1] = regValue & 0b11111111;        // Lower 8 bits (bits 7-0)

    // Write the value to the register
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG06_Input_Current_Limit, 2, data);
}

void BQ25798_setChargeVoltageLimit(uint16_t voltage)
{
    // Ensure the voltage is within the valid range (3000mV to 18800mV)
    if (voltage < 3000 || voltage > 18800)
    {
        Serial.println("Voltage out of range!");
        return;
    }

    // Calculate the register value
    // The charge voltage limit has a step size of 10mV
    uint16_t regValue = voltage / 10;

    // Prepare the data to be written (2 bytes)
    uint8_t data[2];
    data[0] = (regValue >> 8) & 0b00000111; // Upper 3 bits (bits 10-8)
    data[1] = regValue & 0b11111111;        // Lower 8 bits (bits 7-0)

    // Write the value to the register
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG01_Charge_Voltage_Limit, 2, data);
}

void BQ25798_setChargeCurrentLimit(uint16_t current)
{
    // Ensure the current is within the valid range (50mA to 5000mA)
    if (current < 50 || current > 5000)
    {
        Serial.println("Current out of range!");
        return;
    }

    // Calculate the register value
    // The charge current limit has a step size of 10mA
    uint16_t regValue = current / 10;

    // Prepare the data to be written (2 bytes)
    uint8_t data[2];
    data[0] = (regValue >> 8) & 0b00000001; // Upper 1 bit (bit 8)
    data[1] = regValue & 0b11111111;        // Lower 8 bits (bits 7-0)

    // Write the value to the register
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG03_Charge_Current_Limit, 2, data);
}

void BQ25798_enableCharging()
{
    // Read the current value of the register
    uint8_t regValue;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG0F_Charger_Control_0, 1, &regValue);

    // Set the EN_CHG bit (bit 5)
    regValue |= 0b00100000; // Binary representation for setting bit 5

    // Write the updated value back to the register
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG0F_Charger_Control_0, 1, &regValue);
}

void BQ25798_disableCharging()
{
    // Read the current value of the register
    uint8_t regValue;
    i2cReadBytes(BQ25798_SLAVE_ADDRESS, REG0F_Charger_Control_0, 1, &regValue);

    // Clear the EN_CHG bit (bit 5)
    regValue &= 0b11011111; // Binary representation for clearing bit 5

    // Write the updated value back to the register
    i2cWriteBytes(BQ25798_SLAVE_ADDRESS, REG0F_Charger_Control_0, 1, &regValue);
}

void BQ25798_updateADCValues()
{
    // Read the ADC values and update the external variables
    vbusValue = BQ25798_readADCValue(REG35_VBUS_ADC); // VBUS ADC value
    ibusValue = BQ25798_readADCValue(REG31_IBUS_ADC); // IBUS ADC value
    vbatValue = BQ25798_readADCValue(REG3B_VBAT_ADC); // VBAT ADC value
    // ibatValue = BQ25798_readADCValue(REG33_IBAT_ADC); // IBAT ADC value

    // Check the charging status
    ChargeStatus chargingStatus = BQ25798_getChargeStatus();
    if (chargingStatus == Not_Charging || chargingStatus == Charge_Termination_Done)
    {
        BQ25798_enableDischargeCurrentSensing();
        ibatValue = BQ25798_readADCValue(REG33_IBAT_ADC); // IBAT ADC value
    }
    else
    {
        BQ25798_enableChargeCurrentSensing();
        ibatValue = BQ25798_readADCValue(REG33_IBAT_ADC); // IBAT ADC value
    }
}

void determinePowerSourceBasedOnVBUSChanges()
{
    // Store the value in the array
    vbusValues[currentIndex] = vbusValue;

    // Increment the index
    currentIndex++;

    // Check if the index has reached the end or exceeds the array size
    if (currentIndex >= VBUS_ARRAY_SIZE)
    {
        // Perform analysis on the array
        checkVbusValuesArray();

        // Reset the index to 0 for the next cycle
        currentIndex = 0;
    }
}

void BQ25798_printConfigurationSettings(bool allow_print)
{
    // Read register values
    uint8_t cellCountReg = BQ25798_readRegister(REG0A_Re_charge_Control);
    uint8_t systemVoltageReg = BQ25798_readRegister(REG00_Minimal_System_Voltage);
    uint16_t chargeVoltageReg = BQ25798_readRegister16(REG01_Charge_Voltage_Limit);
    uint16_t chargeCurrentReg = BQ25798_readRegister16(REG03_Charge_Current_Limit);
    uint16_t inputCurrentReg = BQ25798_readRegister16(REG06_Input_Current_Limit);
    uint8_t chargerControl5 = BQ25798_readRegister(REG14_Charger_Control_5);
    uint8_t mpptControl = BQ25798_readRegister(REG15_MPPT_Control);

    // Interpret the CELL_1:0 bits (bits 7-6) for battery cell count
    cellCount = ((cellCountReg >> 6) & 0x03) + 1; // Adding 1 because 0b00 represents 1s

    // Interpret the VSYSMIN_5:0 bits for minimal system voltage
    systemVoltage = ((systemVoltageReg & 0x3F) * 250) + 2500; // Range: 2500mV to 16000mV, Step: 250mV

    // Interpret the charge voltage limit (10mV steps)
    chargeVoltage = (chargeVoltageReg * 10);

    // Interpret the charge current limit (10mA steps)
    chargeCurrent = (chargeCurrentReg * 10);

    // Interpret the input current limit (10mA steps)
    // Extracting the lower 9 bits (bits 0-8)
    inputCurrent = (inputCurrentReg & 0x01FF) * 10;

    // Determine the status of the input current limit
    isIINDPMEnabled = chargerControl5 & 0b00000100;  // Check EN_IINDPM bit (bit 2)
    isEXTILIMEnabled = chargerControl5 & 0b00000010; // Check EN_EXTILIM bit (bit 1)

    // MPPT
    isMPPTEnabled = mpptControl & 0b00000001; // Check EN_MPPT bit (bit 0)

    // Print the values
    if (allow_print)
    {
        Serial.print("Battery Cell Count: ");
        Serial.println(cellCount);
        Serial.print("Minimal System Voltage: ");
        Serial.println(systemVoltage);
        Serial.print("Charge Voltage Limit: ");
        Serial.println(chargeVoltage);
        Serial.print("Charge Current Limit: ");
        Serial.println(chargeCurrent);
        Serial.print("Input Current Limit: ");
        Serial.println(inputCurrent);
        Serial.print("Internal IINDPM Register Input Current Regulation (EN_IINDPM): ");
        Serial.println(isIINDPMEnabled ? "Enabled" : "Disabled");
        Serial.print("External ILIM_HIZ Pin Input Current Regulation (EN_EXTILIM): ");
        Serial.println(isEXTILIMEnabled ? "Enabled" : "Disabled");
        Serial.print("Maximum Power Point Tracking (MPPT) Enable: ");
        Serial.println(isMPPTEnabled ? "Enabled" : "Disabled");
    }
}

void BQ25798_printAllADCValues()
{
    Serial.println();
    Serial.println("*************************************************************");
    Serial.println("ADC Values:");
    Serial.print("VBUS: ");
    Serial.println(BQ25798_readADCValue(REG35_VBUS_ADC)); // Bus voltage
    Serial.print("IBUS: ");
    Serial.println(BQ25798_readADCValue(REG31_IBUS_ADC)); // Input current
    Serial.print("VBAT: ");
    Serial.println(BQ25798_readADCValue(REG3B_VBAT_ADC)); // Battery voltage
    Serial.print("IBAT: ");
    Serial.println(BQ25798_readADCValue(REG33_IBAT_ADC)); // Battery current
    Serial.print("VAC1: ");
    Serial.println(BQ25798_readADCValue(REG37_VAC1_ADC)); // Charger voltage 1
    Serial.print("VAC2: ");
    Serial.println(BQ25798_readADCValue(REG39_VAC2_ADC)); // Charger voltage 2
    Serial.print("VSYS: ");
    Serial.println(BQ25798_readADCValue(REG3D_VSYS_ADC)); // System voltage
    Serial.print("TS: ");
    Serial.println(BQ25798_readADCValue(REG3F_TS_ADC)); // Temperature sense
    Serial.print("TDIE: ");
    Serial.println(BQ25798_readADCValue(REG41_TDIE_ADC)); // Die temperature
    Serial.print("D+: ");
    Serial.println(BQ25798_readADCValue(REG43_D_Plus_ADC)); // D+ line voltage
    Serial.print("D-: ");
    Serial.println(BQ25798_readADCValue(REG45_D_Minus_ADC)); // D- line voltage
}

void BQ25798_printLocalADCValues()
{
    BQ25798_updateADCValues();

    Serial.print("VBUS ADC Value: ");
    Serial.println(vbusValue);
    Serial.print("IBUS ADC Value: ");
    Serial.println(ibusValue);
    Serial.print("VBAT ADC Value: ");
    Serial.println(vbatValue);
    Serial.print("IBAT ADC Value: ");
    Serial.println(ibatValue);
}

static void checkVbusValuesArray()
{
    bool significantChangeDetected = false;

    for (int i = 1; i < VBUS_ARRAY_SIZE; i++)
    {
        if (abs(vbusValues[i] - vbusValues[i - 1]) > 10000)
        {
            significantChangeDetected = true;
            break;
        }
    }

    // Update the variable based on the analysis
    if (significantChangeDetected)
    {
        solarPanel_OR_Grid = 10; // Solar Panel connected
    }
    else
    {
        solarPanel_OR_Grid = 20; // GRID connected
    }
}
