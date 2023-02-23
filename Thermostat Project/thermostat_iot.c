/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Standard library
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// TI drivers
#include <ti/drivers/Timer.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include "ti_drivers_config.h"

// Temperature result registers
#define TMP006_RESULT_REG          0x0001
#define TMP11X_RESULT_REG          0x0000

// I2C slave addresses
#define TMP_COUNT                   3
#define TMP006_LAUNCHPAD_ADDR      0x41
#define TMP11X_BASSENSORS_ADDR     0x48
#define TMP116_LAUNCHPAD_ADDR      0x49

// Custom defines
#define NUM_TASKS                   4

// Custom structs
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[TMP_COUNT] = {
    { 0x48, TMP11X_RESULT_REG, "11X" },
    { 0x49, TMP11X_RESULT_REG, "116" },
    { 0x41, TMP006_RESULT_REG, "006" }
};

typedef struct Task {
    int state;
    int (*taskFunction)(int);
    uint16_t period_milliseconds;
    uint16_t timeSinceLastRun_milliseconds;
} Task;

// Global flags
volatile unsigned char FLAG_PERIOD_ELAPSED = 0;     // period for the state machine
unsigned char FLAG_INCREASE_TARGET_TEMP = 0;        // increase target temperature
unsigned char FLAG_DECREASE_TARGET_TEMP = 0;        // decrease target temperature
unsigned char FLAG_HEAT_ON = 0;                     // turn heat on


// Global variables
const uint32_t STATE_PERIOD_MILLISECONDS = 1000;
Task task[NUM_TASKS];
UART2_Handle uart;
I2C_Handle i2c;
uint8_t rxBuffer[2];
uint8_t txBuffer[1];
I2C_Transaction i2cTransaction;
uint16_t targetTemperature = 21;    // Set default target temp to 21 celsius / 70 fahrenheit
uint16_t currentTemperature;


// Callbacks
// ButtonSW3 callback
void gpioButtonIncreaseTargetTemperature(uint_least8_t index) {
    FLAG_INCREASE_TARGET_TEMP = 1;
}


// ButtonSW2 callback
void gpioButtonDecreaseTargetTemperature(uint_least8_t index) {
    FLAG_DECREASE_TARGET_TEMP = 1;
}


// State period task callback
void statePeriodCallback(Timer_Handle myHandle, int_fast16_t status) {
    // Raise flag indicating that the state machine can run its tasks again
    FLAG_PERIOD_ELAPSED = 1;
}


void setupGPIO() {
    // Configure the LED and button pins
    GPIO_setConfig(LED_RED_D10, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    // Configure buttons
    GPIO_setConfig(BUTTON_SW3, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(BUTTON_SW3, gpioButtonIncreaseTargetTemperature);
    GPIO_enableInt(BUTTON_SW3);

    GPIO_setConfig(BUTTON_SW2, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(BUTTON_SW2, gpioButtonDecreaseTargetTemperature);
    GPIO_enableInt(BUTTON_SW2);
}


void setupTimers()
{
    Timer_Handle timer;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);

    // State task period timer (timer[0]) for .5 seconds
    params.period = STATE_PERIOD_MILLISECONDS * 1000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = statePeriodCallback;
    timer = Timer_open(CONFIG_TIMER_0, &params);

    // Error checking
    if (timer == NULL) {
        // Failed to initialized timer
        while (1) {}
    }

    // Start state task timer / error checking
    if (Timer_start(timer) == Timer_STATUS_ERROR) {
        // Failed to start state period timer
        while (1) {}
    }
}


void setupUART() {
    UART2_Params uartParams;

    // Create a UART where the default read and write mode is BLOCKING
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL) {
        // Failed to initialize the UART
        printf("Error! Failed to initialize the UART.\r\n");
        while (1) {}
    }
}


void setupI2C(void) {
    int8_t i;
    int8_t found;
    I2C_Params i2cParams;
    size_t bytesWritten;
    unsigned char MESSAGE_MAX = 64;

    // Static messages
    char initMessage[] = "Initializing I2C Driver - ";
    char failMessage[] = "Failed\r\n";
    char passMessage[] = "Passed\r\n";
    char foundMessage[] = "Found\r\n";
    char noMessage[] = "No\r\n";

    // Dynamic messages (contains variables)
    char whoMessage[MESSAGE_MAX];
    char detectMessage[MESSAGE_MAX];
    char noDetectMessage[MESSAGE_MAX];
    memset(whoMessage, 0, MESSAGE_MAX);
    memset(detectMessage, 0, MESSAGE_MAX);
    memset(noDetectMessage, 0, MESSAGE_MAX);

    UART2_write(uart, initMessage, sizeof(initMessage), &bytesWritten);

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(I2C_TP1, &i2cParams);
    if (i2c == NULL) {
        UART2_write(uart, failMessage, sizeof(failMessage), &bytesWritten);
        while (1);
    }

    UART2_write(uart, passMessage, sizeof(passMessage), &bytesWritten);

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    // Common I2C transaction setup
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;
    found = false;

    for(i = 0; i < TMP_COUNT; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        snprintf(whoMessage, 64, "Is this %s? ", sensors[i].id);
        UART2_write(uart, whoMessage, sizeof(whoMessage), &bytesWritten);
        if (I2C_transfer(i2c, &i2cTransaction)) {
            UART2_write(uart, foundMessage, sizeof(foundMessage), &bytesWritten);
            found = true;
            break;
        }
        UART2_write(uart, noMessage, sizeof(noMessage), &bytesWritten);
    }

    if(found) {
        snprintf(detectMessage, 64, "Detected TMP%s I2C address:%x\n\r", sensors[i].id, i2cTransaction.slaveAddress);
        UART2_write(uart, detectMessage, sizeof(detectMessage), &bytesWritten);
    }
    else {
        snprintf(noDetectMessage, 64, "Detected TMP%s I2C address:%x\n\r", sensors[i].id, i2cTransaction.slaveAddress);
        UART2_write(uart, noDetectMessage, sizeof(noDetectMessage), &bytesWritten);
    }
}

enum TAR_States {TAR_Initial, TAR_Set, TAR_Failed};
int setTargetTemperature (int state) {
    enum TAR_States TAR_State = (enum TAR_States)state;

    // State transitions
    switch(TAR_State) {
    case TAR_Initial:
        TAR_State = TAR_Set;
        break;
    case TAR_Set:
        break;
    case TAR_Failed:
        break;
    default:
        TAR_State = TAR_Failed;
        break;
    }

    // State actions
    switch(TAR_State) {
    case TAR_Initial:
        break;
    case TAR_Set:
        // Increase target temperature
        if (FLAG_INCREASE_TARGET_TEMP) {
            FLAG_INCREASE_TARGET_TEMP = 0;
            targetTemperature++;
        }
        // Decrease target temperature
        if (FLAG_DECREASE_TARGET_TEMP) {
            FLAG_DECREASE_TARGET_TEMP = 0;
            targetTemperature--;
        }
        break;
    case TAR_Failed:
        while(1);
    default:
        break;
    }

    return TAR_State;
}


enum TMP_States {TMP_Initial, TMP_Sample, TMP_Failed};
int sampleCurrentTemperature(int state) {
    enum TMP_States TMP_State = (enum TMP_States)state;
    unsigned char outputMaxSize = 64;
    char output[outputMaxSize];
    size_t bytesWritten;

    // State Transitions
    switch(TMP_State) {
    case TMP_Initial:
        TMP_State = TMP_Sample;
        break;
    case TMP_Sample:
        break;
    case TMP_Failed:
        break;
    default:
        TMP_State = TMP_Failed;
        break;
    }

    // State actions
    switch(TMP_State) {
    case TMP_Initial:
        break;
    case TMP_Sample:
        // Sample the current temperature
        if (I2C_transfer(i2c, &i2cTransaction)) {
            // Extract degrees C from the received data; see TMP sensor datasheet
            currentTemperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
            currentTemperature *= 0.0078125;

            // If the MSB is set '1', then we have a 2's complement negative value which needs to be sign extended
            if (rxBuffer[0] & 0x80) {
                currentTemperature |= 0xF000;
            }
        }
        else {
            // Output error messages to UART
            memset(output, 0, outputMaxSize);
            snprintf(output, outputMaxSize, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status);
            UART2_write(uart, output, sizeof(output), &bytesWritten);

            memset(output, 0, outputMaxSize);
            snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r");
            UART2_write(uart, output, sizeof(output), &bytesWritten);
        }
        break;
    case TMP_Failed:
        while(1);
    default:
        break;
    }

    return TMP_State;
}


enum LED_States {LED_Initial, LED_Off, LED_On, LED_Failed};
int updateLED(int state) {
    enum LED_States LED_State = (enum LED_States)state;
    unsigned char turnOff = 0;
    unsigned char turnOn = 1;

    // State transitions
    switch(LED_State) {
    case LED_Initial:
        LED_State = LED_Off;
        break;
    case LED_Off:
        // Don't switch if target is equal to current to prevent flapping interface
        if (currentTemperature >= targetTemperature) {
            LED_State = LED_Off;
        }
        else if (currentTemperature < targetTemperature) {
            LED_State = LED_On;
        }
    break;
    case LED_On:
        // Don't switch if target is equal to current to prevent flapping interface
        if (currentTemperature <= targetTemperature) {
            LED_State = LED_On;
        }
        else if (currentTemperature > targetTemperature) {
            LED_State = LED_Off;
        }
        break;
    case LED_Failed:
        break;
    default:
        LED_State = LED_Failed;
        break;
    }

    // State actions
    switch(LED_State) {
    case LED_Initial:
        break;
    case LED_Off:
        // Turn LED off
        FLAG_HEAT_ON = 0;
        GPIO_write(LED_RED_D10, turnOff);
        break;
    case LED_On:
        // Turn LED on
        FLAG_HEAT_ON = 1;
        GPIO_write(LED_RED_D10, turnOn);
        break;
    case LED_Failed:
        while(1);
    default:
        break;
    }

    return LED_State;
}


enum DAT_States {DAT_Initial, DAT_Send, DAT_Failed};
int sendDataUpdateToServer(int state) {
    enum DAT_States DTA_State = (enum DAT_States)state;
    static uint32_t uptime_milliseconds = STATE_PERIOD_MILLISECONDS;
    static uint32_t uptime_seconds = 0;
    size_t bytesWritten;
    int_fast16_t status;
    unsigned char OUTPUT_MAX = 64;
    char output[OUTPUT_MAX];

    // State transitions
    switch(DAT_State) {
    case DAT_Initial:
        DAT_State = DAT_Send;
        break;
    case DAT_Send:
        break;
    case DAT_Failed:
        break;
    default:
        DAT_State = DAT_Failed;
        break;
    }

    // State actions
    switch(DAT_State) {
    case DAT_Initial:
        break;
    case DTA_Send:
        // Track uptime
        uptime_milliseconds += STATE_PERIOD_MILLISECONDS;
        if (uptime_milliseconds >= 1000) {
            uptime_milliseconds = 0;
            uptime_seconds++;    // will overflow and wrap in 11k years...
        }

        // Send data to server through UART2 interface
        memset(output, 0, OUTPUT_MAX);
        snprintf(output, OUTPUT_MAX, "<%02d,%02d,%d,%04d>\r\n", currentTemperature, targetTemperature, FLAG_HEAT_ON, uptime_seconds);
        status = UART2_write(uart, output, sizeof(output), &bytesWritten);

        if (status != UART2_STATUS_SUCCESS || bytesWritten == 0) {
            printf("Error! Failed to write to the UART.\r\n");
        }
        break;
    case DAT_Failed:
        while(1);
    default:
        break;
    }

    return DAT_State;
}


void setupTasks() {
    unsigned int i;

    // Set target temperature every 200ms
    task[0].state = TAR_Initial;
    task[0].taskFunction = &setTargetTemperature;
    task[0].period_milliseconds = 200;

    // Sample the current temperature every 500ms
    task[1].state = TMP_Initial;
    task[1].taskFunction = &sampleCurrentTemperature;
    task[1].period_milliseconds = 500;

    // Update LED every 1s
    task[2].state = LED_Initial;
    task[2].taskFunction = &updateLED;
    task[2].period_milliseconds = 1000;

    // Send temperature data to server every 1s
    task[3].state = DAT_Initial;
    task[3].taskFunction = &sendDataUpdateToServer;
    task[3].period_milliseconds = 1000;

    // For all tasks...
    for (i = 0; i < NUM_TASKS; i++) {
        task[i].timeSinceLastRun_milliseconds = 0;
    }
}


void *mainThread(void *arg0) {
    // Variable setup
    unsigned int i;

    // Setup timers, GPIO buttons and LEDs, I2C, and UART
    GPIO_init();
    setupGPIO();
    setupUART();
    setupI2C();
    setupTimers();
    setupTasks();

    while(1) {
        // Wait for period timer to raise flag
        while (!FLAG_PERIOD_ELAPSED);
        FLAG_PERIOD_ELAPSED = 0;

        // Run all tasks according to their period -
        // 0. setTargetTemperature();       // Listens to buttons to raise/lower target temperature
        // 1. sampleCurrentTemperature();   // Samples the current temperature of the room
        // 2. updateLED();                  // Checks if heat should be turned on/off based on current heat
        // 3. sendDataUpdateToServer();     // Sends setting and reading data to server via wifi
        for (i = 0; i < NUM_TASKS; i++) {
            // Increment time since last run
            task[i].timeSinceLastRun_milliseconds += STATE_PERIOD_MILLISECONDS;

            // Run task if time since last run has reached period, and reset time
            if (task[i].timeSinceLastRun_milliseconds >= task[i].period_milliseconds) {
                task[i].state = task[i].taskFunction(task[i].state);
                task[i].timeSinceLastRun_milliseconds = 0;
            }
        }
    } // end task loop
}
