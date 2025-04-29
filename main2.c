#include <p33Fxxxx.h>
// do not change the order of the following 3 definitions
#define FCY 12800000UL 
#include <stdio.h>
#include <libpic30.h>
#include <uart.h>
#include <math.h>

#include "lcd.h"
#include "types.h"
#include "motor.h"
#include "timer.h"

// Calibration constants for touchscreen and servo
#define MIN_X        410.0
#define MAX_X        2540.0
#define MID_X       ((MIN_X + MAX_X) * 0.5)
#define MIN_Y        300.0
#define MAX_Y       3000.0
#define MID_Y       ((MIN_Y + MAX_Y) * 0.5)
#define SERVO_MIN    900.0
#define SERVO_MAX   2100.0
#define SERVO_CENTER ((SERVO_MIN + SERVO_MAX) * 0.5)

// PID controller gains and parameters
static int small_switch = 200;
static double kp = 0.2, ki = 0.1, kd = 2.3;
static double kp_y = 0.2, ki_y = 0.1, kd_y = 2.3;
static double small_kp = 0.04, small_kd = 0.7;
static double small_kp_y = 0.04, small_kd_y = 0.9;

// State variables for PID control
static double prev_x_duty = 0.0, prev_y_duty = 0.0;
static double int_err_x = 0.0, int_err_y = 0.0;

// 3rd Order Butterworth Low-Pass Filter for noise reduction
static double butter_x[4] = {0.0, 0.0, 0.0, 0.0};    // Input history for X
static double butter_xout[4] = {0.0, 0.0, 0.0, 0.0}; // Output history for X
static double butter_y[4] = {0.0, 0.0, 0.0, 0.0};    // Input history for Y
static double butter_yout[4] = {0.0, 0.0, 0.0, 0.0}; // Output history for Y

// Filter coefficients (Fs=20Hz, Fc=15Hz Butterworth)
static const double b_coeffs[4] = {0.03568569, 0.10705706, 0.10705706, 0.03568569};
static const double a_coeffs[3] = {-1.31972830, 0.78046995, -0.15525615};

// Apply Butterworth filter (X-axis)
double apply_butterworth_filter_x(double input_value) {
    // Shift input history
    butter_x[3] = butter_x[2];
    butter_x[2] = butter_x[1];
    butter_x[1] = butter_x[0];
    butter_x[0] = input_value;
    // Shift output history
    butter_xout[3] = butter_xout[2];
    butter_xout[2] = butter_xout[1];
    butter_xout[1] = butter_xout[0];
    // Compute new output
    butter_xout[0] = b_coeffs[0]*butter_x[0] + b_coeffs[1]*butter_x[1] 
                   + b_coeffs[2]*butter_x[2] + b_coeffs[3]*butter_x[3]
                   - a_coeffs[0]*butter_xout[1] - a_coeffs[1]*butter_xout[2]
                   - a_coeffs[2]*butter_xout[3];
    return butter_xout[0];
}

// Apply Butterworth filter (Y-axis)
double apply_butterworth_filter_y(double input_value) {
    // Shift input history
    butter_y[3] = butter_y[2];
    butter_y[2] = butter_y[1];
    butter_y[1] = butter_y[0];
    butter_y[0] = input_value;
    // Shift output history
    butter_yout[3] = butter_yout[2];
    butter_yout[2] = butter_yout[1];
    butter_yout[1] = butter_yout[0];
    // Compute new output
    butter_yout[0] = b_coeffs[0]*butter_y[0] + b_coeffs[1]*butter_y[1] 
                   + b_coeffs[2]*butter_y[2] + b_coeffs[3]*butter_y[3]
                   - a_coeffs[0]*butter_yout[1] - a_coeffs[1]*butter_yout[2]
                   - a_coeffs[2]*butter_yout[3];
    return butter_yout[0];
}

// Configure Timer1 for 50ms period interrupts (20Hz)
void enable_timer1() {
    __builtin_write_OSCCONL(OSCCONL | 2);   // enable peripheral pin configuration (if needed)
    CLEARBIT(T1CONbits.TON);
    CLEARBIT(T1CONbits.TCS);
    CLEARBIT(T1CONbits.TSYNC);
    T1CONbits.TCKPS = 0b10;    // 1:64 prescaler
    TMR1 = 0x00;
    PR1 = 10000;              // period value for ~50ms (20Hz)
    IPC0bits.T1IP = 0x01;     // Timer1 interrupt priority
    IFS0bits.T1IF = 0;        // clear Timer1 interrupt flag
    IEC0bits.T1IE = 1;        // enable Timer1 interrupt
    T1CONbits.TON = 1;        // start Timer1
}

// ADC initialization (12-bit mode, manual sampling)
void init_adc1() {
    CLEARBIT(AD1CON1bits.ADON);
    SETBIT(AD1CON1bits.AD12B);     // 12-bit mode
    AD1CON1bits.FORM = 0;          // integer output
    AD1CON1bits.SSRC = 0x7;        // auto-convert after sampling
    AD1CON2 = 0;
    CLEARBIT(AD1CON3bits.ADRC);
    AD1CON3bits.SAMC = 0x1F;       // 31 Tad auto-sample time
    AD1CON3bits.ADCS = 0x2;        // Tad = 3*Tcy
    SETBIT(AD1CON1bits.ADON);      // enable ADC1
}

// Read touchscreen ADC value (single sample)
unsigned short read_touchscreen() {
    SETBIT(AD1CON1bits.SAMP);           // start sampling
    while(!AD1CON1bits.DONE);           // wait for conversion
    CLEARBIT(AD1CON1bits.DONE);
    return ADC1BUF0 & 0x0FFF;           // 12-bit result
}

// Timer1 ISR (runs every 50ms)
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void) {
    // Clear Timer1 interrupt flag
    IFS0bits.T1IF = 0;
    // Select X-axis measurement
    touch_select_dim(1);
    AD1CHS0bits.CH0SA = 0x0F;  // AN15 (X-axis analog input)

    int curr_x = read_touchscreen();
    double filtered_x = apply_butterworth_filter_x((double) curr_x);
    // Compute servo duty for current X position
    double curr_x_duty = ((filtered_x - MIN_X) * (SERVO_MAX - SERVO_MIN) / (MAX_X - MIN_X)) + SERVO_MIN;
    double err_x = curr_x_duty - SERVO_CENTER;
    double speed_x = curr_x_duty - prev_x_duty;
    prev_x_duty = curr_x_duty;
    double out_x;
    // Apply PID control for X-axis
    if(fabs(err_x) < small_switch) {
        // Use reduced gains for small error
        double pd_out = SERVO_CENTER - (small_kp * err_x + small_kd * speed_x);
        double pred_out = pd_out - (ki * (int_err_x + err_x * 0.05));
        if(pred_out > SERVO_MAX && err_x < 0) {
            // Prevent integrator wind-up (at max)
        } else if(pred_out < SERVO_MIN && err_x > 0) {
            // Prevent integrator wind-up (at min)
        } else {
            int_err_x += err_x * 0.05;
        }
        out_x = pd_out - (ki * int_err_x);
    } else {
        // Use normal PID gains
        double pd_out = SERVO_CENTER - (kp * err_x + kd * speed_x);
        double pred_out = pd_out - (ki * (int_err_x + err_x * 0.05));
        if(pred_out > SERVO_MAX && err_x < 0) {
            // Prevent integrator wind-up
        } else if(pred_out < SERVO_MIN && err_x > 0) {
            // Prevent integrator wind-up
        } else {
            int_err_x += err_x * 0.05;
        }
        out_x = pd_out - (ki * int_err_x);
    }
    // Clamp output to safe servo range
    if(out_x < SERVO_MIN) out_x = SERVO_MIN;
    if(out_x > SERVO_MAX) out_x = SERVO_MAX;
    motor_set_duty(1, (int) out_x);

    __delay_ms(10);
    AD1CHS0bits.CH0SA = 0x09;  // AN9 (Y-axis analog input)
    int curr_y = read_touchscreen();
    double filtered_y = apply_butterworth_filter_y((double) curr_y);
    // Compute servo duty for current Y position
    double curr_y_duty = ((filtered_y - MIN_Y) * (SERVO_MAX - SERVO_MIN) / (MAX_Y - MIN_Y)) + SERVO_MIN;
    double err_y = curr_y_duty - SERVO_CENTER;
    double speed_y = curr_y_duty - prev_y_duty;
    prev_y_duty = curr_y_duty;
    double out_y;
    // Apply PID control for Y-axis
    if(fabs(err_y) < small_switch) {
        // Use reduced gains for small error
        double pd_out_y = SERVO_CENTER - (small_kp_y * err_y + small_kd_y * speed_y);
        double pred_out_y = pd_out_y - (ki_y * (int_err_y + err_y * 0.05));
        if(pred_out_y > SERVO_MAX && err_y < 0) {
            // Prevent integrator wind-up (at max)
        } else if(pred_out_y < SERVO_MIN && err_y > 0) {
            // Prevent integrator wind-up (at min)
        } else {
            int_err_y += err_y * 0.05;
        }
        out_y = pd_out_y - (ki_y * int_err_y);
    } else {
        // Use normal PID gains
        double pd_out_y = SERVO_CENTER - (kp_y * err_y + kd_y * speed_y);
        double pred_out_y = pd_out_y - (ki_y * (int_err_y + err_y * 0.05));
        if(pred_out_y > SERVO_MAX && err_y < 0) {
            // Prevent integrator wind-up
        } else if(pred_out_y < SERVO_MIN && err_y > 0) {
            // Prevent integrator wind-up
        } else {
            int_err_y += err_y * 0.05;
        }
        out_y = pd_out_y - (ki_y * int_err_y);
    }
    // Clamp output to safe servo range
    if(out_y < SERVO_MIN) out_y = SERVO_MIN;
    if(out_y > SERVO_MAX) out_y = SERVO_MAX;
    motor_set_duty(0, (int) out_y);

    // Switch back to X-axis mode for next cycle
    touch_select_dim(1);
    // Throttle serial output to avoid overhead
    static int print_counter = 0;
    if(++print_counter >= 5) {
        print_counter = 0;
        printf("KpX=%.2f KiX=%.2f KdX=%.2f | posX=%d setX=%d | KpY=%.2f KiY=%.2f KdY=%.2f | posY=%d setY=%d\r\n",
               kp, ki, kd, (int)filtered_x, (int)MID_X, kp_y, ki_y, kd_y, (int)filtered_y, (int)MID_Y);
    }
}

int main(){
    // LCD Initialization Sequence 
    __C30_UART=1;   
    lcd_initialize();
    lcd_clear();
    // Initialize UART2 for PC serial communication
    TRISFbits.TRISF4 = 1; // U2RX as input (not used here)
    TRISFbits.TRISF5 = 0; // U2TX as output
    U2MODEbits.BRGH = 0; // low speed mode
    U2BRG = (FCY / (16 * 9600UL)) - 1; // Baud = 9600
    U2MODE = 0;
    U2MODEbits.UARTEN = 1; // enable UART2
    U2STA = 0;
    U2STAbits.UTXEN = 1; // enable UART2 TX
    __C30_UART = 2; // redirect printf to UART2

    touch_init();
    motor_init(0);
    motor_init(1);
    init_adc1();
    // Configure touchscreen analog pins (B15 = X, B9 = Y)
    SETBIT(TRISBbits.TRISB15); 
    CLEARBIT(AD1PCFGLbits.PCFG15); // RB15 to analog (X-position)
    SETBIT(TRISBbits.TRISB9);
    CLEARBIT(AD1PCFGLbits.PCFG9);  // RB9 to analog (Y-position)

    // Start in X-axis mode and allow settling
    touch_select_dim(1);
    __delay_ms(10);
    enable_timer1();

    while(1){
        __delay_ms(10); // idle loop (everything handled in ISR)
    }
    return 0;
}
