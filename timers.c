/******************************************************************************
 * File:    timers.c
 * Device:  TMS320F280015x (or similar)
 *
 * Purpose:
 *   - Uses a single timer (Timer2) to count system time with a 1 ms tick.
 *   - Every 10 ms, Timer2 ISR triggers master transmission via uartMsgTxMaster().
 *   - Every 100 ms, Timer2 ISR triggers ACU transmission via uartMsgTxACU().
 *   - When systemTimeCounter reaches 60,000 ms (60 seconds), a flag is set.
 *
 * Note:
 *   - Timer0 and Timer1 functionalities have been removed to consolidate timing on Timer2.
 ******************************************************************************/

#include "timers.h"
#include "uart_operation_master.h"  // for uartMsgTxMaster()
#include "uart_operation_acu.h"     // for uartMsgTxACU()

volatile uint32_t systemTimeCounter = 0; // System time counter in milliseconds
volatile bool systemTimeElapsed = false;   // Flag indicating if the maximum time has elapsed

// Internal counters for TX timing
static volatile uint16_t masterCounter = 0;    // For master TX every 10 ms
static volatile uint16_t acuTxCounter = 0;       // For ACU TX every 100 ms

// Forward declaration of the ISR.
__interrupt void systemTimerISR(void);

//
// initSystemTimer()
// Initializes Timer2 to generate a 1 ms tick. In its ISR, it updates:
//   - The system time counter.
//   - A master counter that triggers uartMsgTxMaster() every 10 ms.
//   - An ACU counter that triggers uartMsgTxACU() every 100 ms.
//
void initSystemTimer(void)
{
    systemTimeElapsed = false;
    systemTimeCounter = 0;
    masterCounter = 0;
    acuTxCounter = 0;

    // Stop Timer2 during configuration
    CPUTimer_stopTimer(CPUTIMER2_BASE);

    // Calculate Timer2 period for 1 ms tick
    uint32_t timerPeriod = (uint32_t)((DEVICE_SYSCLK_FREQ / 1e6) * 1000); // 1 ms
    CPUTimer_setPeriod(CPUTIMER2_BASE, timerPeriod - 1);
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);

    // Set emulation mode
    CPUTimer_setEmulationMode(CPUTIMER2_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    // Register the Timer2 ISR
    Interrupt_register(INT_TIMER2, systemTimerISR);

    // Enable Timer2 interrupt and PIE group interrupt
    CPUTimer_enableInterrupt(CPUTIMER2_BASE);
    Interrupt_enable(INT_TIMER2);

    // Start Timer2
    CPUTimer_startTimer(CPUTIMER2_BASE);
}

//
// systemTimerISR()
// Timer2 ISR executed every 1 ms:
//   - Increments systemTimeCounter.
//   - Every 10 ms, triggers master transmission via uartMsgTxMaster().
//   - Every 100 ms, triggers ACU transmission via uartMsgTxACU().
//   - When systemTimeCounter reaches 60,000 ms (60 seconds), sets systemTimeElapsed.
//
__interrupt void systemTimerISR(void)
{
    // Increment the system time counter (1 ms tick)
    systemTimeCounter++;
    if(systemTimeCounter >= 60000) // 60 seconds
    {
        systemTimeElapsed = true;
    }

    // Increment software counters
    masterCounter++;
    acuTxCounter++;

    // Every 10 ms, trigger master transmission
    if(masterCounter >= 10)
    {
        masterCounter = 0;
        uartMsgTxMaster();
    }

    // Every 100 ms, trigger ACU transmission
    if(acuTxCounter >= 500)
    {
        acuTxCounter = 0;
        uartMsgTxACU();
    }

    // Clear Timer2 overflow flag and acknowledge the PIE group.
    CPUTimer_clearOverflowFlag(CPUTIMER2_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//
// resetSystemTimer()
// Resets the system time counter and elapsed flag.
//
void resetSystemTimer(void)
{
    systemTimeCounter = 0;
    systemTimeElapsed = false;
}

//
// getSystemTime()
// Returns the current system time in milliseconds.
//
uint32_t getSystemTime(void)
{
    return systemTimeCounter;
}

//
// hasSystemTimeElapsed()
// Returns true if the system time has reached the maximum (60 seconds).
//
bool hasSystemTimeElapsed(void)
{
    return systemTimeElapsed;
}
