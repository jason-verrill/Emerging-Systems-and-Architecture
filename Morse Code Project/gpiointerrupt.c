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


//  ======== gpiointerrupt.c ========

// Standard library includes
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

// Driver includes
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"
#include <ti/drivers/Timer.h>

// Definitions
#define LED_RED 9               // red LED is D8 (P02) on board
#define LED_GREEN 11            // green LED is D10 (P64) on board
#define LED_OFF 0
#define LED_ON 1
#define BUTTON_SW3 22           // button is SW3 (P15) on board
#define NO_INDEX_REQUEST -1     // Used by timer state

// Task structure
typedef struct Task {
    int state;
    int (*taskFunction)(int);   // pass and return task's state
} Task;


// Global flags
unsigned char FLAG_MESSAGE_MODE = 0;                    // Indicates SOS or SOS_OK mode
volatile unsigned char FLAG_TIMER_STATE_PERIOD = 0;     // Measures time and moves state machine forward
volatile unsigned char FLAG_TIMER_FINISHED = 0;         // Checks if timer has completed
unsigned char FLAG_LED_RED = 0;                         // Tells RED_LED it should be on/off
unsigned char FLAG_LED_GREEN = 0;                       // Tells GREEN_LED it should be on/off

// Global variables
Timer_Handle timer[4];

enum TimerLength {TIMER_SHORT = 1, TIMER_LONG = 2, TIMER_LONGEST = 3};

// Callback for the long and short continuous timer
void timerContinuousCallback(Timer_Handle myHandle, int_fast16_t status) {
    // Raise state period flag
    FLAG_TIMER_STATE_PERIOD = 1;
}


// Callback for the period timer
void timerOneShotCallback(Timer_Handle myHandle, int_fast16_t status) {
    // Raise timer flag
    FLAG_TIMER_FINISHED = 1;
}


// Initialize and configure the timers for each letter
void setupTimers() {
    unsigned char numTimers = 4;
    unsigned char i;

    Timer_Params params;

    // Setup continuous timer
    Timer_Params_init(&params);
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.period = 500000;
    params.timerCallback = timerContinuousCallback;
    timer[0] = Timer_open(CONFIG_TIMER_0, &params);

    // Setup short one shot timer
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_ONESHOT_CALLBACK;
    params.period = 500000;
    params.timerCallback = timerOneShotCallback;
    timer[1] = Timer_open(CONFIG_TIMER_1, &params);

    // Setup long one shot timer
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_ONESHOT_CALLBACK;
    params.period = 1500000;
    params.timerCallback = timerOneShotCallback;
    timer[2] = Timer_open(CONFIG_TIMER_2, &params);

    // Setup very long one shot timer
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_ONESHOT_CALLBACK;
    params.period = 3500000;
    params.timerCallback = timerOneShotCallback;
    timer[3] = Timer_open(CONFIG_TIMER_3, &params);

    // Error checking
    for (i = 0; i < numTimers; i++) {
        if (timer[i] == NULL) {
            while (1) {
                printf("Error creating timer%c\r\n", i);
            }
        }
    }

    // Start continuous timer
    if (Timer_start(timer[0]) == Timer_STATUS_ERROR) {
        while (1) {
            printf("Error starting timer0");
        }
    }
}


// Callback function for the GPIO interrupt for BUTTON_SW3
void gpioButtonFxn(uint_least8_t index) {
    // Toggle message mode
    FLAG_MESSAGE_MODE = !FLAG_MESSAGE_MODE;
}


// Initialize and configure GPIO LEDs and Buttons
void setupGPIO() {
    GPIO_init();

    // LEDs
    GPIO_setConfig(LED_RED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(LED_GREEN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    // Buttons
    GPIO_setConfig(BUTTON_SW3, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(BUTTON_SW3, gpioButtonFxn);
    GPIO_enableInt(BUTTON_SW3);
}


// States for each message and letter
enum MSG_States {
    MSG_Initial,
    MSG_SOS_1, MSG_SOS_2, MSG_SOS_3,
    MSG_OK_1, MSG_OK_2,
    MSG_PAUSE,
    MSG_FAILED
};

// The message manager tracks what word, letter, and sequence we are on
int MSG_messageManager(int state) {
    enum MSG_States MSG_State = (enum MSG_States)state;
    static signed char letterIsCompleted = 0;
    static unsigned char currentSequence = 1;
    static unsigned char timerInProgress = 0;
    const unsigned char sequenceLength = 3;

    // State transitions
    // Checks to see if a letter is complete and if so, and moves to a pause, then to the next letter
    // Otherwise stay in the same state
    switch(MSG_State) {
    case MSG_Initial:
        MSG_State = MSG_SOS_1;
        break;
    case MSG_SOS_1:
        if (letterIsCompleted) {
            MSG_State = MSG_SOS_2;
        }
        else {
            MSG_State = MSG_SOS_1;
        }
        break;
    case MSG_SOS_2:
        if (letterIsCompleted) {
            MSG_State = MSG_SOS_3;
        }
        else {
            MSG_State = MSG_SOS_2;
        }
        break;
    case MSG_SOS_3:
        // Message mode is 'sos', go to pause
        if (letterIsCompleted && FLAG_MESSAGE_MODE == 0) {
            MSG_State = MSG_PAUSE;
        }
        // Message mode is 'sos ok', keep going
        else if (letterIsCompleted && FLAG_MESSAGE_MODE == 1) {
            MSG_State = MSG_OK_1;
        }
        else {
            MSG_State = MSG_SOS_3;
        }
        break;
    case MSG_OK_1:
        if (letterIsCompleted) {
            MSG_State = MSG_OK_2;
        }
        else {
            MSG_State = MSG_OK_1;
        }
        break;
    case MSG_OK_2:
        if (letterIsCompleted) {
            MSG_State = MSG_PAUSE;
        }
        else {
            MSG_State = MSG_OK_2;
        }
        break;
    case MSG_PAUSE:
        if (letterIsCompleted) {
            MSG_State = MSG_SOS_1;  // loop back
        }
        else {
            MSG_State = MSG_PAUSE;
        }
        break;
    case MSG_FAILED:
    default:
        while(1) {}
    } // End state transitions

    // Reset completion if this isn't the first run
    letterIsCompleted = 0;

    // State actions
    switch(MSG_State) {
    case MSG_Initial:
        break;
    case MSG_SOS_1:
    case MSG_SOS_3:
        // Flash 's' sequence - red short 3x
        // If there are more flash sequences...
        if (currentSequence <= sequenceLength) {
            // If no timer is started, start a timer and turn red led on
            if (!timerInProgress) {
                Timer_start(timer[TIMER_SHORT]);
                FLAG_LED_RED = 1;
                timerInProgress = 1;
            }
            // If a timer is in progress but hasn't finished
            else if (timerInProgress && !FLAG_TIMER_FINISHED) {
                // do nothing
            }
            // If a timer was in progress but has finished, go to next flash sequence
            else if (timerInProgress && FLAG_TIMER_FINISHED) {
                FLAG_TIMER_FINISHED = 0;
                timerInProgress = 0;
                FLAG_LED_RED = 0;
                currentSequence++;
            }
        }

        // If there are no more flash sequences...
        if (currentSequence > sequenceLength) {
            // Reset sequence and flag letter complete
            currentSequence = 1;
            letterIsCompleted = 1;
        }
        break;
    case MSG_SOS_2:
    case MSG_OK_1:
        // Flash 's' sequence - green long 3x
        // If there are more flash sequences...
        if (currentSequence <= sequenceLength) {
            // If no timer is started, start a timer and turn green led on
            if (!timerInProgress) {
                Timer_start(timer[TIMER_LONG]);
                FLAG_LED_GREEN = 1;
                timerInProgress = 1;
            }
            // If a timer is in progress but hasn't finished
            else if (timerInProgress && !FLAG_TIMER_FINISHED) {
                // do nothing
            }
            // If a timer was in progress but has finished, go to next flash sequence
            else if (timerInProgress && FLAG_TIMER_FINISHED) {
                FLAG_TIMER_FINISHED = 0;
                timerInProgress = 0;
                FLAG_LED_GREEN = 0;
                currentSequence++;
            }
        }

        // If there are no more flash sequences...
        if (currentSequence > sequenceLength) {
            // Reset sequence and flag letter complete
            currentSequence = 1;
            letterIsCompleted = 1;
        }
        break;
    case MSG_OK_2:
        // Flash 'o' sequence - green long x1, red short x1, green long x1
        // If there are more flash sequences...
        if (currentSequence <= sequenceLength) {
            // If no timer is started, start a timer and turn green led on
            if (!timerInProgress) {
                if (currentSequence == 2) {
                    Timer_start(timer[TIMER_SHORT]);
                    FLAG_LED_RED = 1;
                }
                else {
                    Timer_start(timer[TIMER_LONG]);
                    FLAG_LED_GREEN = 1;
                }
                timerInProgress = 1;
            }
            // If a timer is in progress but hasn't finished
            else if (timerInProgress && !FLAG_TIMER_FINISHED) {
                // do nothing
            }
            // If a timer was in progress but has finished, go to next flash sequence
            else if (timerInProgress && FLAG_TIMER_FINISHED) {
                FLAG_TIMER_FINISHED = 0;
                timerInProgress = 0;
                FLAG_LED_RED = 0;
                FLAG_LED_GREEN = 0;
                currentSequence++;
            }
        }

        // If there are no more flash sequences...
        if (currentSequence > sequenceLength) {
            // Reset sequence and flag letter complete
            currentSequence = 1;
            letterIsCompleted = 1;
        }
        break;
    case MSG_PAUSE:
        if (!timerInProgress) {
            // Start timer
            Timer_start(timer[TIMER_LONGEST]);
            timerInProgress = 1;
        }
        else if (timerInProgress && !FLAG_TIMER_FINISHED) {
            // do nothing
        }
        else if (timerInProgress && FLAG_TIMER_FINISHED) {
            // Reset flags
            FLAG_TIMER_FINISHED = 0;
            timerInProgress = 0;
            letterIsCompleted = 1;
        }
        break;
    case MSG_FAILED:
    default:
        while(1) {}
    } // End state actions

    return MSG_State;
}


// LED Red Control task
enum LDR_States {LDR_Initial, LDR_On, LDR_Off, LDR_Failed};
int LDR_redLEDControl(int state) {
    enum LDR_States LDR_State = (enum LDR_States)state;

    // State transitions
    switch(LDR_State) {
    case LDR_Initial:
        LDR_State = LDR_Off;
        break;
    case LDR_On:
    case LDR_Off:
        if (FLAG_LED_RED == 0) {
            LDR_State = LDR_Off;
        }
        else if (FLAG_LED_RED == 1) {
            LDR_State = LDR_On;
        }
        break;
    default:
        // Failed state
        LDR_State = LDR_Failed;
        break;
    }

    // State actions
    switch(LDR_State) {
    case LDR_Initial:
        break;
    case LDR_On:
        GPIO_write(LED_RED, LED_ON);
        break;
    case LDR_Off:
        GPIO_write(LED_RED, LED_OFF);
        break;
    case LDR_Failed:
        // Failed state
        while(1) {}
    }

    return LDR_State;
}


// LED Green Control task
enum LDG_States {LDG_Initial, LDG_On, LDG_Off, LDG_Failed};
int LDG_greenLEDControl(int state) {
    enum LDG_States LDG_State = (enum LDG_States)state;

    // State transitions
    switch(LDG_State) {
    case LDG_Initial:
        LDG_State = LDG_Off;
        break;
    case LDG_On:
    case LDG_Off:
        if (FLAG_LED_GREEN == 0) {
            LDG_State = LDG_Off;
        }
        else if (FLAG_LED_GREEN == 1) {
            LDG_State = LDG_On;
        }
        break;
    default:
        // Failed state
        LDG_State = LDG_Failed;
        break;
    }

    // State actions
    switch(LDG_State) {
    case LDG_Initial:
        break;
    case LDG_On:
        GPIO_write(LED_GREEN, LED_ON);
        break;
    case LDG_Off:
        GPIO_write(LED_GREEN, LED_OFF);
        break;
    case LDG_Failed:
        // Failed state
        while(1) {}
    }

    return LDG_State;
}


void setupTasks(Task *task) {
    // Setup message manager
    task[0].state = MSG_Initial;
    task[0].taskFunction = &MSG_messageManager;

    // Setup Red LED
    task[1].state = LDR_Initial;
    task[1].taskFunction = &LDR_redLEDControl;

    // Setup Green LED
    task[2].state = LDG_Initial;
    task[2].taskFunction = &LDG_greenLEDControl;
}


void *mainThread(void *arg0) {
    static Task task[3];
    unsigned char numTasks = 3;
    unsigned char i;

    // Setup
    setupGPIO();
    setupTasks((Task*)&task);
    setupTimers();

    // SyncSM tasks
    while(1) {
        // If the period timer flag has been raised...
        if (FLAG_TIMER_STATE_PERIOD) {
            // Lower timer flag
            FLAG_TIMER_STATE_PERIOD = 0;

            // Run tasks
            for (i = 0; i < numTasks; i++) {
                task[i].state = task[i].taskFunction(task[i].state);
            }
        }
    }
}
