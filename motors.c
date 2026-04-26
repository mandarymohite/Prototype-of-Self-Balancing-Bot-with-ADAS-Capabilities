/* ========================================================================== */
/*  motors.c  –  Motor driver for Self-Balancing Robot with ADAS              */
/*                                                                            */
/*  Original: Akash Patil and Vidhya Palaniappan  (5/5/2024)                 */
/*  ADAS Extension: Added differential-speed steering for ADAS manoeuvres    */
/* ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

#define RANGE           250     /* max value for PWM range denominator        */
#define INITIAL_VALUE   0       /* start PWM at 0 % duty cycle               */

/* ---- TB6612FNG driver pin assignments (wiringPi numbering) -------------- */
#define AIN1    0               /* direction control – left motor             */
#define PWMA    26              /* speed control     – left motor             */
#define BIN1    3               /* direction control – right motor            */
#define PWMB    23              /* speed control     – right motor            */
#define STBY    6               /* STBY / enable pin for driver IC            */

/* ---- motor state -------------------------------------------------------- */
double left_speed;
double right_speed;


/* Initialise GPIO for motors (digital outputs + hardware PWM) */
void init_motors(void)
{
    wiringPiSetup();

    pinMode(PWMA, PWM_OUTPUT);
    pinMode(PWMB, PWM_OUTPUT);

    pinMode(AIN1, OUTPUT);
    digitalWrite(AIN1, HIGH);

    pinMode(BIN1, OUTPUT);
    digitalWrite(BIN1, HIGH);

    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    pwmSetRange(RANGE);     /* 250 steps                                     */
    pwmSetClock(192);       /* 19.2 MHz / 192 = 100 kHz PWM carrier          */
}


/* Bring both motors to a hard stop */
void stop_motors(void)
{
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    digitalWrite(PWMA, LOW);

    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    digitalWrite(PWMB, LOW);
}


/* Drive both motors straight-forward at a fixed calibration speed (70/250) */
void move_forward(void)
{
    pwmWrite(PWMA, 70);
    pwmWrite(PWMB, 70);
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, HIGH);
}


/*
 * motors() – core motor command
 *   speed        : base speed from PID (-GUARD_GAIN … +GUARD_GAIN)
 *   left_offset  : trim / steering bias for left motor
 *   right_offset : trim / steering bias for right motor
 *
 * Positive speed → forward;  negative speed → reverse.
 * ADAS manoeuvres (avoid, steer) call this with asymmetric offsets.
 */
void motors(double speed, double left_offset, double right_offset)
{
    left_speed  = speed + left_offset;
    right_speed = speed + right_offset;

    /* --- Left motor --- */
    if (left_speed < 0)
    {
        pwmWrite(PWMA, (int)(-left_speed));
        digitalWrite(AIN1, HIGH);          /* reverse polarity */
    }
    else if (left_speed > 0)
    {
        pwmWrite(PWMA, (int)left_speed);
        digitalWrite(AIN1, LOW);           /* forward polarity */
    }
    else
    {
        pwmWrite(PWMA, 0);
    }

    /* --- Right motor --- */
    if (right_speed < 0)
    {
        pwmWrite(PWMB, (int)(-right_speed));
        digitalWrite(BIN1, HIGH);
    }
    else if (right_speed > 0)
    {
        pwmWrite(PWMB, (int)right_speed);
        digitalWrite(BIN1, LOW);
    }
    else
    {
        pwmWrite(PWMB, 0);
    }
}


/*
 * steer_left() / steer_right()
 * Convenience wrappers for ADAS lateral avoidance.
 * Reduces the inside wheel speed while keeping the outside wheel at 'speed'.
 * 'ratio' (0.0 – 1.0) controls how sharp the turn is.
 */
void steer_left(double speed, double ratio)
{
    double inner = speed * (1.0 - ratio);
    motors(0.0, inner, speed);      /* left inner, right outer */
}

void steer_right(double speed, double ratio)
{
    double inner = speed * (1.0 - ratio);
    motors(0.0, speed, inner);      /* left outer, right inner */
}
