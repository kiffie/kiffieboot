/*
 * configuration registers
 * assumption: 8 MHz crystal for primary osciallator
 */

#if defined(__32MX440F256H__) || defined( __32MX250F128B__) || defined(__32MX270F256B__) || defined(__32MX274F256B__) || defined(__32MX220F032B__) || defined(__32MX470F512H__)

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#if __PIC32_FEATURE_SET == 470
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)
#else
#pragma config FUSBIDIO = OFF           // USB USID Selection
#ifndef __32MX274F256B__
    #pragma config FVBUSONIO = OFF      // USB VBUS ON Selection
#endif
#endif

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#if SYS_CLOCK == 40000000
    #pragma config FPLLMUL  = MUL_20    // PLL Multiplier
    #pragma config FPLLODIV = DIV_2     // PLL Output Divider
#elif SYS_CLOCK == 48000000
    #pragma config FPLLMUL  = MUL_24    // PLL Multiplier
    #pragma config FPLLODIV = DIV_2     // PLL Output Divider
#elif SYS_CLOCK == 72000000
    #pragma config FPLLMUL  = MUL_18    // PLL Multiplier
    #pragma config FPLLODIV = DIV_1     // PLL Output Divider
#elif SYS_CLOCK == 96000000
    #pragma config FPLLMUL  = MUL_24    // PLL Multiplier
    #pragma config FPLLODIV = DIV_1     // PLL Output Divider
#else
    #error "unexpected SYS_CLOCK"
#endif
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider (2x Divider)
#pragma config UPLLEN = ON              // USB PLL Enable (Enabled)

// DEVCFG1
#ifdef __32MX274F256B__
    #pragma config FNOSC    = SPLL      // Oscillator Selection
#else
    #pragma config FNOSC    = PRIPLL    // Oscillator Selection
#endif
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor Selection (Clock Switch Enable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
//#pragma config DEBUG = OFF              // Background Debugger Enable
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select
#pragma config PWP = OFF                // Program Flash Write Protect
#pragma config BWP = OFF                // Boot Flash Write Protect bit
#pragma config CP = OFF                 // Code Protect

#else
    #error Cannot define configuration bits.
#endif
