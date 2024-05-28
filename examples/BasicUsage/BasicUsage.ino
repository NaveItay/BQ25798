/**
 * @file BasicUsage.ino
 * 
 * @brief Example sketch to demonstrate the usage of the BQ25798 library.
 * 
 * This example initializes the BQ25798 charger with predefined parameters and 
 * periodically reads and prints ADC values.
 * 
 * @version 1.0.0
 * @date 2023-12-25
 * 
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

#include <Arduino.h>
#include <BQ25798.hpp>

// BQ25798 Initialization Parameters
#define BATTERY_CELL_COUNT 4         // Number of cells in the battery
#define INPUT_CURRENT_LIMIT 1400     // Input current limit in mA
#define MINIMAL_SYSTEM_VOLTAGE 10000 // Minimal system voltage in mV
#define CHARGE_VOLTAGE_LIMIT 14600   // Charge voltage limit in mV
#define CHARGE_CURRENT_LIMIT 1400    // Charge current limit in mA
#define ENABLE_CHARGING true         // Enable charging
#define ENABLE_MPPT true             // Enable MPPT

void setup()
{
    // Initialize serial communication at 115200 baud rate
    Serial.begin(115200);
    
    // Give some time for the serial monitor to start
    delay(1000);

    // Initialize the BQ25798 charger with the defined parameters
    initialize_BQ25798(BATTERY_CELL_COUNT, INPUT_CURRENT_LIMIT, MINIMAL_SYSTEM_VOLTAGE, CHARGE_VOLTAGE_LIMIT, CHARGE_CURRENT_LIMIT, ENABLE_CHARGING, ENABLE_MPPT);

    // Print the initial configuration settings
    Serial.println("Initial BQ25798 Configuration:");
    BQ25798_printConfigurationSettings(true);
}

void loop()
{
    // Read and print all ADC values
    BQ25798_printAllADCValues();

    // Wait for 5 seconds before the next reading
    delay(5000);
}
