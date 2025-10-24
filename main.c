#include <msp430.h>

unsigned int x1 = 0;  // ADC result for P1.0 (A0); Source 1 direct value w.r.t microcontroller  ground
unsigned int x2 = 0;  // ADC result for P1.1 (A1) ; Source 2 Voltage w.r.t. A0 or A4 direct value
unsigned int x3 = 0;  // ADC result for P1.2 (A2) ; Higher potential of capacitor 
unsigned int x4 = 0;  // ADC result for P1.3 (A3) ; Lower potential of capacitor 
int x5 = 0;  // Voltage difference accross capacitors when charged through source 1; 
int x6 = 0;  // Voltage difference accross capacitors when charged through source 2; 
unsigned int x7 = 0;  // ADC result for P1.4 (A4) 
int x8 = 0; // instant voltage of source 1 w.r.t.  own ground
int x9 = 0; // instant voltage of source 2 w.r.t.  own ground/Ckt. ground when signal switch of source 1 is off
int x10 = 0;
int x = 0; //instant voltage of source 2 w.r.t.  own ground when signal switch of source 1 is on.
float startTime = 0, endTime = 0, totalTime = 0; // end time= total code excusion duration from last evaluation;

unsigned long overflowTicks = 0;  // Timer overflow counter

#define sws1 BIT2      // P2.2 source 1 signal switch
#define swb1 BIT5      // P1.5 source 1 signal block switch
#define sws2 BIT6      // P1.6 source 2 signal switch
#define swb2 BIT7      // P1.7 source 2 signal block switch
#define csw BIT0       // P2.0 evaluation capacitor switch
#define csws BIT1      // P2.1 storage capacitor switch

unsigned int adcChannel = 0;  // Current ADC channel

// ADC read function
void analogRead(unsigned int pin) {
    adcChannel = pin;
    ADC10CTL0 &= ~ENC;             // Disable ADC before configuration
    ADC10CTL1 = pin << 12;         // Select channel (INCH_x)
    ADC10AE0 = 1 << pin;           // Enable analog input on the selected pin
    P1DIR &= ~(1 << pin);          // Set pin as input
    ADC10CTL0 |= ENC + ADC10SC;    // Start conversion
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
    switch (adcChannel) {
        case 0: x1 = ADC10MEM; break;
        case 1: x2 = ADC10MEM; break;
        case 2: x3 = ADC10MEM; break;
        case 3: x4 = ADC10MEM; break;
        case 4: x7 = ADC10MEM; break;
    }
    __bic_SR_register_on_exit(CPUOFF);  // Exit low-power mode after ADC ISR
}


#pragma vector = TIMER0_A1_VECTOR  //for calculating time starting from end of evaluation 
__interrupt void Timer_A1_ISR(void) {
    if (TAIV == 10) {
        overflowTicks++;  // Count each overflow which is of 1 sec
    }
}

void setup() {
    WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer
    BCSCTL1 = CALBC1_1MHZ;       // Set the DCO to 1 MHz
    DCOCTL = CALDCO_1MHZ;
    
    P1DIR &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4);  // Set P1.0 - P1.4 as inputs
    P1OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4);  //outputs are pulled down
    P1DIR |= sws2 + swb1 + swb2;  // Set switch pins as output
    P2DIR |= sws1 + csw + csws;  // Set switch pins as output
    P1OUT &= ~(swb1 + sws2 + swb2);  // Initialize all switch pins to LOW
    P2OUT &= ~(sws1 + csw + csws);  // Initialize control pins to LOW

    // ADC setup
    ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ADC10IE;  // Enable ADC, use internal reference, and enable interrupts
    __enable_interrupt();  // Enable global interrupts

    // Timer_A Setup for Time Tracking
    TACTL = TASSEL_2 + MC_1 + ID_3 + TAIE;  // SMCLK, Up mode, /8, enable overflow interrupt
    TACCR0 = 0xFFFF;  // Max 16-bit value
}

float getCurrentTimeSeconds() {
    // Each tick = 8 µs (1 MHz / 8), TA0R counts up to 65535
    unsigned long totalTicks = (overflowTicks * 65536UL) + TA0R;
    return (float)(totalTicks * 8.0) / 1000000.0;  // Time in seconds
}

int main(void) {
    setup();

    while (1) {
        // Source 1 evaluation mode: Control switches and pins
       // P2OUT &= ~csw; // Turning off the evaluation capacitor switch 
        P2OUT &= ~csws; // Turning off the storage capacitor switch
        P1OUT &= ~swb1; // turning off source 1 signal block switch
        P2OUT |= sws1; // turning on source 1 signal switch
        x = 1;
        P1OUT &= ~sws2; // turning off source 2 signal  switch
        P1OUT |= swb2; // turning on source 2 signal block switch
        P2OUT |= csw; // Turning on the evaluation capacitor switch
        __delay_cycles(500000);  // delay for allowing to charge evaluation capacitor (0.5 sec)

        analogRead(2);  __bis_SR_register(CPUOFF + GIE);  // Start ADC conversion on P1.2 (A2)
        analogRead(3);  __bis_SR_register(CPUOFF + GIE);  // Start ADC conversion on P1.3 (A3)
        x5 = x3 - x4;  // Calculate difference

        // allowing discharge of evaluation capacitor
        P2OUT &= ~csw;
        P2OUT &= ~sws1;
        x = 0;
        P1OUT &= ~swb2;
        __delay_cycles(1000000); 

        // Source 2 evaluation mode: Control switches and pins
        P2OUT |= csw; // Turning on the evaluation capacitor switch
        P1OUT |= swb1; // turning on source 1 signal block switch
        P1OUT |= sws2; // turning on source 2 signal switch
        __delay_cycles(500000);  // delay for allowing to charge evaluation capacitor (0.5 sec)


        analogRead(2);  __bis_SR_register(CPUOFF + GIE);
        analogRead(3);  __bis_SR_register(CPUOFF + GIE);
        x6 = x3 - x4;  // Calculate difference

        //derermining primary source (if x5 is higher, then select s1 as primary else s2 as primary)
        if (x5 > x6) {
            P1OUT &= ~swb1;
            P2OUT |= sws1; //enabling source 1 switch
            x = 1;
            P1OUT &= ~sws2; //disconnecting source 2 
            P1OUT |= swb2; 
            P2OUT &= ~csw;
            P2OUT |= csws;
        } else {
            P2OUT &= ~sws1;  //disconnecting source 1
            x = 0; 
            P1OUT &= ~swb2;
            P1OUT |= sws2; //enabling source 2 switch
            P1OUT |= swb1;
            P2OUT &= ~csw;
            P2OUT |= csws;
        }

        startTime = getCurrentTimeSeconds();  // Store the current time, beginning of energy extraction of primary source

        // Loop for 300 seconds
        while (totalTime <= 300) {
            
            analogRead(0); __bis_SR_register(CPUOFF + GIE);  // Start ADC conversion on P1.0 (A0)
            analogRead(1); __bis_SR_register(CPUOFF + GIE);  // Start ADC conversion on P1.1 (A1)
            analogRead(3); __bis_SR_register(CPUOFF + GIE);  // Start ADC conversion on P1.3 (A3)

            x8 = x1 - x7;
            x9 = x2 - x7;
            x10 = x2 - x1;

            if (x5 > x6) {
                if (((x8 > 0) && ( x10 > 0 )) || ((x8 < 0) && ( x10 < 0 ))) {
                    P1OUT &= ~swb2;
                    P1OUT |= sws2; // adding secondary source as well
                    P1OUT &= ~swb1;
                    P2OUT |= sws1;
                    x = 1;
                } else {
                    P1OUT &= ~sws2;
                    P1OUT |= swb2; // blocking secondary source as anti-phase
                    P1OUT &= ~swb1;
                    P2OUT |= sws1;
                    x = 1;
                }
            } else {
                if ( x == 1 ) { // determining if source 1 switch is on
                    if (((x10 > 0) && ( x8 > 0 )) || ((x10 < 0) && ( x8 < 0 ))) {
                        P1OUT &= ~swb1;
                        P2OUT |= sws1;
                        x = 1;
                        P1OUT &= ~swb2;
                        P1OUT |= sws2;
                    } else {
                        P2OUT &= ~sws1;
                        x = 0;
                        P1OUT |= swb1;
                        P1OUT &= ~swb2;
                        P1OUT |= sws2;
                    }
                } else {  // source 1 switch is off
                    if (((x9 > 0) && ( x8 > 0 )) || ((x9 < 0) && ( x8 < 0 ))) {
                        P1OUT &= ~swb1;
                        P2OUT |= sws1;
                        x = 1;
                        P1OUT &= ~swb2;
                        P1OUT |= sws2;
                    } else {
                        P2OUT &= ~sws1;
                        x = 0;
                        P1OUT |= swb1;
                        P1OUT &= ~swb2;
                        P1OUT |= sws2;
                    }
                }
            }

            __delay_cycles(5000);  // small delay to between evaluation for phase and anti phase among primary and secondary sources
            endTime = getCurrentTimeSeconds();
            totalTime = endTime - startTime;
        }

        // Reset the totalTime after crossing 10 seconds
        if (totalTime > 300) {
            overflowTicks = 0;
            TA0R = 0;
            startTime = 0;
            endTime = 0;
            totalTime = 0;  // Reset totalTime
        }
    }
}