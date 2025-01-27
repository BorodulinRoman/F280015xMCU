#include "timers.h"
#include "uart_operation_master.h" // Timer0 ISR calls uartMsgTxMaster()
#include "uart_operation_slave.h"  // Timer1 ISR calls uartMsgTxSlave() after 2 ms

//
// Forward declarations of ISRs
//
__interrupt void cpuTimer0ISR(void);
__interrupt void cpuTimer1ISR(void);

//
// For debug: track how many times Timer1 ISR fired
//
static volatile uint16_t cpuTimer1Count = 0;

//=============================================================================
// 1) Timer0 - for Master TX every period_us
//=============================================================================
void initCPUTimer0(float period_us)
{
    // 1) Stop Timer0 while configuring
    CPUTimer_stopTimer(CPUTIMER0_BASE);

    // 2) Calculate the timer period in CPU cycles
    uint32_t timerPeriod = (uint32_t)((DEVICE_SYSCLK_FREQ / 1e6) * period_us);
    CPUTimer_setPeriod(CPUTIMER0_BASE, timerPeriod - 1);

    // 3) No prescaler
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);

    // 4) Reload & set emulation mode
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    // 5) Register ISR for Timer0
    Interrupt_register(INT_TIMER0, cpuTimer0ISR);

    // 6) Enable Timer0 interrupt & PIE
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);

    // 7) Start Timer0
    CPUTimer_startTimer(CPUTIMER0_BASE);
}

__interrupt void cpuTimer0ISR(void)
{
    // Timer0 overflow => send master packet
    uartMsgTxMaster();

    // Clear overflow
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);

    // Acknowledge PIE group 1 for Timer0
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//=============================================================================
// 2) Timer1 - for Slave 2 ms delay, calls uartMsgTxSlave()
//=============================================================================
void initSlaveTimer2ms(void)
{
    // 1) Stop Timer1 while configuring
    CPUTimer_stopTimer(CPUTIMER1_BASE);

    // 2) Suppose 100 MHz => 2 ms => 200,000 cycles
    CPUTimer_setPeriod(CPUTIMER1_BASE, 200000UL - 1UL);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);

    // 3) Emulation mode
    CPUTimer_setEmulationMode(CPUTIMER1_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    // 4) Register ISR for Timer1
    Interrupt_register(INT_TIMER1, cpuTimer1ISR);

    // 5) Enable Timer1 interrupt
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    Interrupt_enable(INT_TIMER1);

    // Timer1 remains stopped until scibRxISR() sees a full 20-byte packet
    // Then it can do:
    //   CPUTimer_stopTimer(CPUTIMER1_BASE);
    //   CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    //   CPUTimer_resumeTimer(CPUTIMER1_BASE);
}

__interrupt void cpuTimer1ISR(void)
{
    // Debug increment
    cpuTimer1Count++;

    // Stop Timer1 so it doesn't keep repeating
    CPUTimer_stopTimer(CPUTIMER1_BASE);

    //
    // Now call the slave TX routine to send a random packet
    //
    uartMsgTxSlave();

    // Clear overflow
    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);

    // Timer1 is CPU INT13 (not in PIE group 1..12). No group ack needed.
}
