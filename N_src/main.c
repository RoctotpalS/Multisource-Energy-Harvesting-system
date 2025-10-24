/*
 * multisource_energy_harvester_n_sources.c
 *
 * Modified from original 2-source MSP430 project to support up to N sources using
 * a 3 x 8 analog MUX bank (up to 24 sources) and a SIPO shift-register chain
 * (74HC595 style) to control the signal (sws) and block (swb) switches for each
 * source. The evaluation-phase / harvesting-phase logic is kept the same as the
 * original: first evaluate each source charging the evaluation capacitor and
 * measuring the resulting V, pick the source with max V as primary, then
 * during harvesting loop add or block secondaries based on phase checks.
 *
 * HARDWARE SUMMARY (wiring):
 * - MUX bank: three 8:1 analog multiplexers (e.g., CD4051 or similar) for up to 24 sources.
 *   * Shared address pins S0,S1,S2 -> MCU: P2.2, P2.3, P2.4
 *   * Each MUX has an active-enable (/EN or OE pin) -> MCU P2.5 (bank0), P2.6 (bank1), P2.7 (bank2)
 *   * All three MUX outputs are wired together to the ADC input A0 (P1.0) but only one MUX's
 *     enable should be activated at a time (others disabled) so outputs don't conflict.
 *   * MUX channel mapping: sourceIndex 0..(N-1): bank = index/8, channel = index%8 -> connect that source to A0
 *
 * - ADC pins:
 *   * A0 (P1.0): MUX output for reading any source voltage (read with MUX selects)
 *   * A2 (P1.2): evaluation capacitor HIGH node (cap_high) - same as original
 *   * A3 (P1.3): evaluation capacitor LOW node (cap_low) - same as original
 *   * A4 (P1.4): common reference / ground sample used in phase detection - same as original
 *
 * - Shift register (SIPO, e.g., 74HC595): controls all sws / swb outputs.
 *   * SER (data) -> P1.5
 *   * SRCLK (shift clock) -> P1.6
 *   * RCLK (latch) -> P1.7
 *   * The shift register chain output bits are mapped as follows:
 *        bit 0  -> source 0 SWS (signal switch)
 *        bit 1  -> source 0 SWB (block switch)
 *        bit 2  -> source 1 SWS
 *        bit 3  -> source 1 SWB
 *        ... and so on (2 bits per source: [SWS, SWB])
 *   * You must ensure enough cascaded 74HC595 devices to cover 2*N outputs.
 *
 * - Capacitor switches and other control pins kept on same MCU pins as original:
 *   * CSW  -> P2.0 (evaluation capacitor switch)
 *   * CSWS -> P2.1 (storage capacitor switch)
 *
 * NOTES & LIMITS:
 * - MAX_SOURCES is currently set to 24 (3 x 8 MUX). Change it only if you change wiring.
 * - The code assumes the shift register's output bit = 1 activates the corresponding signal
 *   switch (SWS) and bit = 0 activates block switch (SWB) - but wiring at hardware (transistors
 *   or MOSFET gates) must be consistent. The actual polarity can be inverted in software if needed.
 * - All ADC readings are done sequentially via the ADC10 and the MUX selection.
 * - This code uses interrupts for ADC and Timer_A overflow similar to original code.
 * - Timing (delays) kept similar to the original code: 0.5s cap charge, 1s discharge, small evaluation delays.
 *
 * HOW TO USE / CONFIGURE:
 * - Set MAX_SOURCES to number of physical sources (<= 24)
 * - Make sure shift register chain length covers 2*MAX_SOURCES outputs.
 * - Wire MUX banks so each source's positive terminal goes to a channel input on a MUX.
 * - Ensure MUX bank enables are active-low (CD4051 has an active-low enable) — code assumes
 *   enabling a bank is done by pulling its EN pin LOW; disabling is HIGH. If your MUX has
 *   opposite polarity, swap logic in setMuxBankEnable().
 *
 * DISCLAIMER: This code has not been hardware-tested by me. Use it as a working template and
 * carefully validate on bench with safe voltages first.
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

// ======= USER-CONFIG =======
#define MAX_SOURCES 16  // <= 24 for 3x8 MUX. Change this to desired number of sources (<=24).
#define HARVEST_SECONDS 300UL

// ======= PIN / HARDWARE MAP =======
// ADC channels (pins on P1)
#define ADC_SOURCE_CH 0  // P1.0 (A0)  <- MUX output (selected source voltage)
#define ADC_CAP_HIGH_CH 2 // P1.2 (A2) evaluation capacitor higher potential
#define ADC_CAP_LOW_CH 3  // P1.3 (A3) evaluation capacitor lower potential
#define ADC_REF_CH 4      // P1.4 (A4) board reference

// Capacitor control (keep same as original)
#define CSW   BIT0  // P2.0 evaluation capacitor switch
#define CSWS  BIT1  // P2.1 storage capacitor switch

// MUX control pins (shared select lines and bank enables)
#define MUX_S0 BIT2  // P2.2  (address bit 0)
#define MUX_S1 BIT3  // P2.3  (address bit 1)
#define MUX_S2 BIT4  // P2.4  (address bit 2)
#define MUX_EN0 BIT5 // P2.5  (enable bank 0)
#define MUX_EN1 BIT6 // P2.6  (enable bank 1)
#define MUX_EN2 BIT7 // P2.7  (enable bank 2)

// Shift-register pins (74HC595 style) on P1
#define SR_SER   BIT5 // P1.5 (data)
#define SR_SRCLK BIT6 // P1.6 (shift clock)
#define SR_RCLK  BIT7 // P1.7 (latch)

// ======= GLOBALS =======
volatile unsigned int adc_results[8]; // store ADC results by target index
volatile unsigned int adcChannel = 0; // selected ADC channel (INCH_x)
volatile unsigned char adcReadTargetIndex = 0; // where to store ADC result in adc_results

volatile unsigned long overflowTicks = 0;  // Timer overflow counter

unsigned long startTime = 0, endTime = 0; // in microseconds/converted later
float totalTime = 0.0f;

// Evaluation diffs (signed) for each source
int evalDiffs[MAX_SOURCES];

// Shift register state (2 bits per source). Use 64-bit to support up to 32 sources (we cap to 24).
unsigned long long shiftState = 0ULL; // bit 0 = src0 SWS, bit1 = src0 SWB, bit2=src1 SWS, ...
int shiftBits = 2 * MAX_SOURCES;

// ======= FORWARD DECLARATIONS =======
void analogReadToIndex(unsigned int pin, unsigned char targetIndex);
unsigned int readMuxedSource(unsigned int sourceIndex);
void setMuxBankEnable(int bank, bool enable);
void setMuxAddress(uint8_t channel);
void updateShiftRegister(void);
void setSourceSwitches(unsigned int sourceIndex, bool sws_on, bool swb_on);
void blockAllSources(void);

// ADC read function (starts conversion and returns when ISR stores value into adc_results[targetIndex])
void analogReadToIndex(unsigned int pin, unsigned char targetIndex) {
    adcChannel = pin;
    adcReadTargetIndex = targetIndex;
    ADC10CTL0 &= ~ENC;           // Disable ADC before configuration
    ADC10CTL1 = (pin << 12);     // select channel INCH_x
    ADC10AE0 = (1 << pin);       // enable analog input
    P1DIR &= ~(1 << pin);        // set pin as input (ensure)
    ADC10CTL0 |= ENC + ADC10SC;  // start conversion
}

// ADC ISR
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
    // store ADC result in requested slot
    adc_results[adcReadTargetIndex] = ADC10MEM;
    __bic_SR_register_on_exit(CPUOFF);  // Exit low-power after ADC
}

// Timer overflow ISR - same as original
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void) {
    if (TAIV == 10) {
        overflowTicks++; // each overflow corresponds to full-count
    }
}

// small helper: enable/disable a MUX bank's output. NOTE: many analog MUX chips have active-LOW enable.
// This function assumes enabling a bank means pulling its EN pin LOW. If your MUX is opposite,
// invert the logic here.
void setMuxBankEnable(int bank, bool enable) {
    // enable = true -> drive EN pin LOW (active)
    if (bank == 0) {
        if (enable) P2OUT &= ~MUX_EN0; else P2OUT |= MUX_EN0;
    } else if (bank == 1) {
        if (enable) P2OUT &= ~MUX_EN1; else P2OUT |= MUX_EN1;
    } else if (bank == 2) {
        if (enable) P2OUT &= ~MUX_EN2; else P2OUT |= MUX_EN2;
    }
}

// set the 3-bit address lines S0..S2
void setMuxAddress(uint8_t channel) {
    // channel 0..7
    if (channel & 0x01) P2OUT |= MUX_S0; else P2OUT &= ~MUX_S0;
    if (channel & 0x02) P2OUT |= MUX_S1; else P2OUT &= ~MUX_S1;
    if (channel & 0x04) P2OUT |= MUX_S2; else P2OUT &= ~MUX_S2;
}

// Read a specific source voltage using the MUX chain; returns the raw ADC10 value (0..1023)
unsigned int readMuxedSource(unsigned int sourceIndex) {
    if (sourceIndex >= MAX_SOURCES) return 0;

    uint8_t bank = sourceIndex / 8;
    uint8_t channel = sourceIndex % 8;

    // disable all banks first
    setMuxBankEnable(0, false);
    setMuxBankEnable(1, false);
    setMuxBankEnable(2, false);

    // set address
    setMuxAddress(channel);

    // enable only required bank
    setMuxBankEnable(bank, true);

    // small settling delay for MUX switching
    __delay_cycles(5000);

    // read via ADC on ADC_SOURCE_CH -> store result into adc_results[0]
    analogReadToIndex(ADC_SOURCE_CH, 0);
    __bis_SR_register(CPUOFF + GIE);

    // disable bank (avoid bus conflicts if other circuits connected)
    setMuxBankEnable(bank, false);

    return adc_results[0];
}

// Shift-register (74HC595) update
void updateShiftRegister(void) {
    // We will shift MSB first so the highest bit ends up at the last output in the chain.
    // Latch low
    P1OUT &= ~SR_RCLK;

    unsigned int i;
    for (i = 0; i < shiftBits; ++i) {
        unsigned long long bitmask = 1ULL << (shiftBits - 1 - i); // MSB-first
        if (shiftState & bitmask) P1OUT |= SR_SER; else P1OUT &= ~SR_SER;

        // pulse SRCLK
        P1OUT |= SR_SRCLK;
        __delay_cycles(1);
        P1OUT &= ~SR_SRCLK;
        __delay_cycles(1);
    }

    // latch
    P1OUT |= SR_RCLK;
    __delay_cycles(1);
    P1OUT &= ~SR_RCLK;
}

// Set the sws and swb bits for a particular source in the shiftState, then update the chain
void setSourceSwitches(unsigned int sourceIndex, bool sws_on, bool swb_on) {
    if (sourceIndex >= MAX_SOURCES) return;
    unsigned int sws_bit = sourceIndex * 2;     // bit for SWS
    unsigned int swb_bit = sourceIndex * 2 + 1; // bit for SWB

    if (sws_on) shiftState |= (1ULL << sws_bit); else shiftState &= ~(1ULL << sws_bit);
    if (swb_on) shiftState |= (1ULL << swb_bit); else shiftState &= ~(1ULL << swb_bit);

    updateShiftRegister();
}

// Convenience: block all sources (SWS=0, SWB=1)
void blockAllSources(void) {
    unsigned int i;
    for (i = 0; i < MAX_SOURCES; ++i) {
        // block: sws_off, swb_on
        unsigned int sws_bit = i*2;
        unsigned int swb_bit = i*2 + 1;
        shiftState &= ~(1ULL << sws_bit);
        shiftState |=  (1ULL << swb_bit);
    }
    updateShiftRegister();
}

// ================= SETUP =================
void setup(void) {
    WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer
    BCSCTL1 = CALBC1_1MHZ;       // Set the DCO to 1 MHz
    DCOCTL = CALDCO_1MHZ;

    // ADC pins (P1.0..P1.4) left as inputs
    P1DIR &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4);
    P1OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4);

    // Shift register pins -> outputs
    P1DIR |= SR_SER | SR_SRCLK | SR_RCLK;
    P1OUT &= ~(SR_SER | SR_SRCLK | SR_RCLK);

    // MUX selects + enables + capacitor control pins on P2 -> outputs
    P2DIR |= MUX_S0 | MUX_S1 | MUX_S2 | MUX_EN0 | MUX_EN1 | MUX_EN2 | CSW | CSWS;
    // default: disable all MUX banks (pull EN HIGH), capacitor switches low
    P2OUT |= MUX_EN0 | MUX_EN1 | MUX_EN2;
    P2OUT &= ~(CSW | CSWS);

    // ADC setup similar to original
    ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ADC10IE;  // use Vcc/Vss ref, sample & hold time, enable ADC and interrupts
    __enable_interrupt();

    // Timer_A setup for time tracking (leave same as original)
    TACTL = TASSEL_2 + MC_1 + ID_3 + TAIE;  // SMCLK, Up mode, /8, enable overflow interrupt
    TACCR0 = 0xFFFF;

    // initialize shift register to all blocked
    blockAllSources();
}

// Convert timer ticks to seconds (original calculation: 1MHz/8 -> each tick = 8us)
float getCurrentTimeSeconds(void) {
    unsigned long totalTicks = (overflowTicks * 65536UL) + TA0R;
    return (float)(totalTicks * 8.0) / 1000000.0; // seconds
}

// ================= MAIN =================
int main(void) {
    setup();

    while (1) {
        // ---- Evaluation phase: for each source, charge evaluation capacitor and measure V ----
        unsigned int i;
        for (i = 0; i < MAX_SOURCES; ++i) {
            // Ensure all sources blocked
            blockAllSources();

            // Unblock and enable only this source
            setSourceSwitches(i, true, false); // SWS on, SWB off

            // Turn on evaluation capacitor switch to charge it from the selected source
            P2OUT |= CSW;
            __delay_cycles(500000); // 0.5s to charge (same as original)

            // Read cap high and cap low
            analogReadToIndex(ADC_CAP_HIGH_CH, 2); __bis_SR_register(CPUOFF + GIE);
            analogReadToIndex(ADC_CAP_LOW_CH, 3);  __bis_SR_register(CPUOFF + GIE);

            int ch = (int)adc_results[2];
            int cl = (int)adc_results[3];
            evalDiffs[i] = ch - cl;

            // disconnect evaluation cap and unblock the current source
            P2OUT &= ~CSW;
            setSourceSwitches(i, false, true); // SWS off, SWB on (block)

            __delay_cycles(1000000); // discharge wait 1s (same as original)
        }

        // Select primary = source with maximum evalDiff
        int primary = 0; int maxVal = evalDiffs[0];
        for (i = 1; i < MAX_SOURCES; ++i) {
            if (evalDiffs[i] > maxVal) { maxVal = evalDiffs[i]; primary = i; }
        }

        // configure switches for chosen primary: primary ON, others blocked initially
        for (i = 0; i < MAX_SOURCES; ++i) {
            if (i == (unsigned)primary) setSourceSwitches(i, true, false); // enable primary
            else setSourceSwitches(i, false, true); // block
        }

        // shift storage capacitor switch from evaluation to storage (same as original)
        P2OUT &= ~CSW;
        P2OUT |= CSWS;

        // start time
        startTime = (unsigned long)getCurrentTimeSeconds();

        // Harvesting loop for HARVEST_SECONDS seconds
        totalTime = 0.0f;
        while (totalTime <= (float)HARVEST_SECONDS) {
            // sample reference once per loop
            analogReadToIndex(ADC_REF_CH, 4); __bis_SR_register(CPUOFF + GIE);
            int vref = (int)adc_results[4];

            // read primary voltage
            unsigned int vp_raw = readMuxedSource(primary);
            int vp = (int)vp_raw;
            int vp_ref = vp - vref;

            // iterate through all other sources and add/block based on phase check
            unsigned int s;
            for (s = 0; s < MAX_SOURCES; ++s) {
                if (s == (unsigned)primary) continue;

                unsigned int vs_raw = readMuxedSource(s);
                int vs = (int)vs_raw;
                int vdiff = vs - vp; // similar to original x10

                // If primary is in the same polarity as (secondary - primary) -> in-phase -> add secondary
                if (((vp_ref > 0) && (vdiff > 0)) || ((vp_ref < 0) && (vdiff < 0))) {
                    // add secondary
                    setSourceSwitches(s, true, false); // sws on, swb off
                } else {
                    // block secondary
                    setSourceSwitches(s, false, true); // sws off, swb on
                }
            }

            __delay_cycles(5000);
            endTime = (unsigned long)getCurrentTimeSeconds();
            totalTime = endTime - startTime;
        }

        // Reset timer counters after harvest
        if (totalTime > (float)HARVEST_SECONDS) {
            overflowTicks = 0;
            TA0R = 0;
            startTime = 0;
            endTime = 0;
            totalTime = 0;
        }
    }

    return 0; // never reached
}
