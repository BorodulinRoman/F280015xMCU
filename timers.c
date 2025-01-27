#include "timers.h"
#include "uart_operation_master.h"   // So we can call uartMsgTxMaster()

//
// Forward declaration of the ISR
//
__interrupt void cpuTimer0ISR(void);

//
// initCPUTimer0()
//   - Configures CPU Timer0 for the specified period (in microseconds).
//   - Registers the ISR and starts the timer.
//   - The ISR will call 'uartMsgTxMaster()' directly.
//
void initCPUTimer0(float period_us)
{
    //
    // 1) Stop Timer0 while configuring
    //
    CPUTimer_stopTimer(CPUTIMER0_BASE);

    //
    // 2) Calculate timer ticks for the given period in microseconds
    //    Assuming DEVICE_SYSCLK_FREQ is your CPU clock (e.g. 100 MHz).
    //
    uint32_t timerPeriod = (uint32_t)((DEVICE_SYSCLK_FREQ / 1e6) * period_us);

    // Set the timer period (minus 1)
    CPUTimer_setPeriod(CPUTIMER0_BASE, timerPeriod - 1);

    //
    // 3) No prescaler
    //
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);

    //
    // 4) Reload & set emulation mode
    //
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    //
    // 5) Register the ISR & enable in PIE
    //
    Interrupt_register(INT_TIMER0, cpuTimer0ISR);

    //
    // 6) Enable Timer0 interrupt in the peripheral & PIE
    //
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);

    //
    // 7) Start the timer
    //
    CPUTimer_startTimer(CPUTIMER0_BASE);
}

//
// cpuTimer0ISR()
//   - Called automatically when Timer0 overflows
//   - Directly calls uartMsgTxMaster() to send a packet.
//
__interrupt void cpuTimer0ISR(void)
{
    //
    // Call the UART transmit routine immediately
    // so the system sends data as soon as the timer hits.
    //
    uartMsgTxMaster();

    //
    // Clear the timer interrupt flag
    //
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);

    //
    // Acknowledge the PIE interrupt group for Timer0
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
