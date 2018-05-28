/*
 * Control engineering of a DC motor using a PID control model
 * (using Hall effect sensors)
 *
 * Date: 25/05/2018
 * File: main.c
 * Author: Aubin Detrez
 * 	Jonathan Pascal
 * Target: MSP430
 */
#include <msp430.h>

#define DT 0.1f // Sampling period (timer delay), in seconds
#define PWM_MAX 10000 // Max PWM value (PWM frequency)


// Define PID coefficients, according to Bisection and Takahashi method
#ifndef TAKAHASHI
	// Coefficients using the Bisection method
	#define KP 0.001f
	#define KI 0.0045f
	#define KD 0.00005f
#else
	// Coefficients using the Takahashi method
	#define KP 5.9883E-5
	#define KI 0.0015
	#define KD 7.0175E-6
#endif


#ifndef TAKAHASHI

volatile float accu = 0;
volatile float last_error = 0;
/*
 * Standard PID implementation
 * @param cmd speed command (rotation per second)
 * @param sensor speed sensor (rotation per second)
 * @return max 1 min 0
 */
float pid(float cmd, float sensor)
{
	float error = sensor - cmd;
	accu += DT*error;
	float deriv = (error - last_error)/DT;
	last_error = error;
	return (error*KP) + (accu*KI) + (deriv*KD);
}

#else

volatile float old_pid = 0;
volatile float error_1 = 0;
volatile float error_2 = 0;

/*
 * PID implementation, with 'innovation' to be used with Takahashi method to
 * calcul coefficients
 * @param cmd speed command (rotation per second)
 * @param sensor speed sensor (rotation per second)
 * @return max 1, min 0
 */
float pid(float cmd, float sensor)
{
	float result;
	float error = sensor - cmd;

	result = old_pid + KP*(error - error_1) + KI*DT*error + KD/DT*(error - 2*error_1 + error_2);

	old_pid = result;
	error_2 = error_1;
	error_1 = error;
	return result;
}

#endif

volatile unsigned int cmpt1 = 0; // counter for hall sensor 1
volatile unsigned int cmpt2 = 0; // counter for hall sensor 2
volatile float sensor; // result of speed measurement, rotation per second

volatile float command = 0; // PID command, rotation per second

// Measured speed and PID 's output log, for debug purpose
volatile float log [10]; // measured speed log
volatile float pid_log [10]; // pid's output log
volatile int i = 0;

/*
 * Delay cycle
 * @param ms delay in milli second
 */
void wait(unsigned int ms)
{
	volatile unsigned int i, z;
	i=100;
	while(i--) {
		z=ms;
 		while(z--);
	}
}

/*
 * Entry point
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    	BCSCTL1 = CALBC1_1MHZ; // 1MHz clock frequency
	DCOCTL = CALDCO_1MHZ; // 1MHz clock frequency

	// Hall effect sensors
	// set pins as input
	P2SEL &= ~(BIT6 | BIT7);
	P2SEL2 &= ~(BIT6 | BIT7);
	P2DIR &= ~(BIT6 | BIT7);
	P2REN |= (BIT6 | BIT7);
	P2OUT |= (BIT6 | BIT7);

	// Set interrupt for Hall sensors
	P2IE |= (BIT6 | BIT7); // enable interrupt
	P2IES &= ~(BIT6 | BIT7); // raising edge
	P2IFG = 0; // reset flag

	// P2.1: H Bridge ENA
	// always 1
	P2DIR |= BIT1;
	P2SEL &= ~BIT1;
	P2SEL2 &= ~BIT1;
	P2OUT |= BIT1;

	// P2.0: H Bridge IN1
	// apply PWM1
	P2DIR |= BIT0;
	P2SEL = BIT0; // select TA1.0 function
	P2SEL2 &= ~BIT0; // select TA1.0 function

	// P2.2: H Bridge IN2
	// apply PWM2
	P2DIR |= BIT2;
	P2SEL = BIT2; // select TA1.1 function
	P2SEL2 &= ~BIT2; // select TA1.1 function

	// configure DT seconds timer
	TA0CTL = 0 | (TASSEL_2 | ID_3); // 8 divider
	TA0CTL |= TAIE;
	TA0CCR0 = (unsigned int)(62500*DT*2); // execute the timer every DT second
	TA0CTL |= MC_1; // up mode

	// DC motor control using PWM
	TA1CTL = TASSEL_2 | MC_1 | ID_0; // SMCLK clock, UP mod, 1 times divider
	TA1CCTL1 |= OUTMOD_7; // use mode 7 (PWM)
	TA1CCR0 = PWM_MAX;
	TA1CCR1 =  PWM_MAX; // start point: do not power the motor

	__enable_interrupt();

	command = 3800/60; // set the command to 38000 rpm
	while(1)
	{
		// Main loop, can change command, do other stuff...
	}
}


/*
 * Executed every DT seconds
 * Compute the actual speed using Hall effect sensors' output
 * Update motor's command by computing PID's value
 */
#pragma vector=TIMER0_A1_VECTOR
__interrupt void fnc_timer()
{
	TA0CTL &= ~TAIFG; // reset flag

 	// Compute the actual speed using Hall effect sensors' output
	sensor = (cmpt1+cmpt2)/(12.f*DT); // rotation per second

	// Reset counters
	cmpt1=0;
	cmpt2=0;

	float pid_result = pid(command, sensor);

	// log outputs for debug purpose
	log[i] = sensor;
	pid_log[i] = pid_result;
	i++;
	i %= 10;

	// PID's output clipping
	float pid_unsaturated = pid_result;
	pid_result = (pid_result > 1.f)? 1.f : pid_result; // saturation haute
	pid_result = (pid_result < 0.f)? 0.f : pid_result; // saturation basse

	// Change the motor's command (PWM)
	TA1CCR1 = PWM_MAX*pid_result;

	// Integral's term desaturation
	accu += pid_result - pid_unsaturated;
}

/*
 * Hall effect sensors interrupt
 */
#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
	if (P2IFG & BIT6 == BIT6) {
		P2IES ^= BIT6;

		cmpt1++;
		P2IFG &= ~(BIT6);
	}

	if (P2IFG & BIT7 == BIT7) {
		P2IES ^= BIT7;

		cmpt2++;
		P2IFG &= ~(BIT7);
	}
}
