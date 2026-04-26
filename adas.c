/* ========================================================================== */
/*  adas.c  –  ADAS subsystem implementation                                  */
/*                                                                            */
/*  Compiled against wiringPi; link with -lwiringPi -lm                      */
/* ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <syslog.h>

#include <wiringPi.h>
#include "adas.h"

/* ---- module-private helpers --------------------------------------------- */

/*
 * measure_distance_pin()
 * Generic HC-SR04 ranging function. Uses any trig/echo GPIO pair.
 * Returns distance in centimetres; returns 999.0 on timeout.
 */
float measure_distance_pin(int trig, int echo)
{
    /* generate 10 µs trigger pulse */
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    /* wait for echo to go HIGH (pulse start) – timeout 30 ms */
    long timeout = micros() + 30000;
    while (digitalRead(echo) == LOW)
        if (micros() > timeout) return 999.0f;

    long start_us = micros();

    /* wait for echo to go LOW (pulse end) – timeout 30 ms */
    timeout = start_us + 30000;
    while (digitalRead(echo) == HIGH)
        if (micros() > timeout) return 999.0f;

    long end_us = micros();

    /* distance = pulse_duration_µs / 58  (sound round-trip at ~343 m/s) */
    return (float)(end_us - start_us) / 58.0f;
}


/* ---- public API --------------------------------------------------------- */

/*
 * adas_init()
 * Configure all ADAS GPIO pins. Call AFTER wiringPiSetup() in main.
 */
void adas_init(void)
{
    /* Front sensor */
    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);
    digitalWrite(TRIG_FRONT, LOW);

    /* Left sensor */
    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    digitalWrite(TRIG_LEFT, LOW);

    /* Right sensor */
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);
    digitalWrite(TRIG_RIGHT, LOW);

    /* Buzzer */
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    /* LEDs */
    pinMode(LED_FCW, OUTPUT);
    digitalWrite(LED_FCW, LOW);
    pinMode(LED_LCK, OUTPUT);
    digitalWrite(LED_LCK, LOW);

    /* brief settling delay after pin init */
    delay(100);

    openlog("adas_robot", LOG_PID | LOG_CONS, LOG_USER);
    syslog(LOG_INFO, "ADAS subsystem initialised");
}


/*
 * adas_alert_on() / adas_alert_off()
 * Toggle the buzzer and FCW LED together.
 */
void adas_alert_on(void)
{
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_FCW,    HIGH);
}

void adas_alert_off(void)
{
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_FCW,    LOW);
}


/*
 * adas_log_state()
 * Emit current ADAS state to syslog for offline analysis.
 */
void adas_log_state(const adas_data_t *a)
{
    static const char *state_names[] = {
        "NOMINAL", "WARN", "SLOW", "STEER_L", "STEER_R", "E_STOP", "FALLOVER"
    };
    syslog(LOG_INFO,
           "ADAS state=%-9s front=%.1fcm left=%.1fcm right=%.1fcm "
           "spd_limit=%.1f FCW=%d AES=%d LCK=%d",
           state_names[a->state],
           a->dist_front_cm, a->dist_left_cm, a->dist_right_cm,
           a->speed_limit, a->fcw_active, a->aes_active, a->lck_active);
}


/*
 * adas_update()
 * ─────────────
 * Central ADAS decision loop. Called from service_4 at ~200 Hz.
 *
 * Reads the three ultrasonic sensors and the robot's current tilt angle,
 * then evaluates the following feature hierarchy (highest priority first):
 *
 *   1. FALLOVER    – robot has fallen past 60°, stop and log
 *   2. E_STOP      – obstacle < 12 cm ahead, hard stop
 *   3. FCW / SLOW  – obstacle 12–40 cm ahead, warn + reduce speed
 *   4. LCK_L/R     – side wall closer than 15 cm, steer away
 *   5. NOMINAL     – no threats, full speed allowed
 */
void adas_update(adas_data_t *adas, double current_tilt_deg)
{
    /* ------------------------------------------------------------------ */
    /* 1. Sample all three sensors                                         */
    /* ------------------------------------------------------------------ */
    adas->dist_front_cm = measure_distance_pin(TRIG_FRONT, ECHO_FRONT);
    delay(10);   /* short inter-sensor gap to avoid acoustic cross-talk   */
    adas->dist_left_cm  = measure_distance_pin(TRIG_LEFT,  ECHO_LEFT);
    delay(10);
    adas->dist_right_cm = measure_distance_pin(TRIG_RIGHT, ECHO_RIGHT);

    /* ------------------------------------------------------------------ */
    /* 2. Tilt / Fall-Over Guard (FTG) – highest priority                 */
    /* ------------------------------------------------------------------ */
    if (fabs(current_tilt_deg) >= TILT_FALLOVER_DEG)
    {
        adas->state         = ADAS_FALLOVER;
        adas->speed_limit   = SPEED_STOP;
        adas->steer_offset_L = 0.0;
        adas->steer_offset_R = 0.0;
        adas->aes_active    = 1;
        adas->fcw_active    = 0;
        adas->lck_active    = 0;
        adas_alert_on();
        adas_log_state(adas);
        return;
    }

    /* ------------------------------------------------------------------ */
    /* 3. Autonomous Emergency Stop (AES)                                 */
    /* ------------------------------------------------------------------ */
    if (adas->dist_front_cm <= DIST_EMERGENCY_STOP_CM)
    {
        adas->state         = ADAS_E_STOP;
        adas->speed_limit   = SPEED_STOP;
        adas->steer_offset_L = 0.0;
        adas->steer_offset_R = 0.0;
        adas->aes_active    = 1;
        adas->fcw_active    = 1;
        adas->lck_active    = 0;
        adas_alert_on();
        adas_log_state(adas);
        return;
    }
    adas->aes_active = 0;

    /* ------------------------------------------------------------------ */
    /* 4. Forward Collision Warning + Adaptive Speed Limiter              */
    /* ------------------------------------------------------------------ */
    if (adas->dist_front_cm <= DIST_FCW_ALERT_CM)
    {
        adas->state       = ADAS_WARN;
        adas->speed_limit = SPEED_SLOW_LIMIT * 0.5;   /* extra cautious   */
        adas->fcw_active  = 1;
        adas_alert_on();
    }
    else if (adas->dist_front_cm <= DIST_SLOW_CM)
    {
        adas->state       = ADAS_SLOW;
        adas->speed_limit = SPEED_SLOW_LIMIT;
        adas->fcw_active  = 0;
        adas_alert_off();
    }
    else
    {
        adas->speed_limit = SPEED_NORMAL_LIMIT;
        adas->fcw_active  = 0;
        adas_alert_off();
    }

    /* ------------------------------------------------------------------ */
    /* 5. Lane / Corridor Keeping (LCK)                                   */
    /* ------------------------------------------------------------------ */
    int steer_needed = 0;

    if (adas->dist_left_cm < DIST_LANE_KEEP_CM)
    {
        /* Too close to left wall → steer right */
        adas->steer_offset_L =  15.0;   /* boost right (outer) motor       */
        adas->steer_offset_R = -15.0;   /* reduce left (inner) motor       */
        steer_needed = 1;
        adas->state  = ADAS_STEER_R;
        digitalWrite(LED_LCK, HIGH);
    }
    else if (adas->dist_right_cm < DIST_LANE_KEEP_CM)
    {
        /* Too close to right wall → steer left */
        adas->steer_offset_L = -15.0;
        adas->steer_offset_R =  15.0;
        steer_needed = 1;
        adas->state  = ADAS_STEER_L;
        digitalWrite(LED_LCK, HIGH);
    }
    else
    {
        adas->steer_offset_L = 0.0;
        adas->steer_offset_R = 0.0;
        digitalWrite(LED_LCK, LOW);
    }

    adas->lck_active = steer_needed;

    /* If nothing triggered, mark nominal */
    if (adas->state != ADAS_WARN  &&
        adas->state != ADAS_SLOW  &&
        adas->state != ADAS_STEER_L &&
        adas->state != ADAS_STEER_R)
    {
        adas->state = ADAS_NOMINAL;
    }

    adas_log_state(adas);
}
