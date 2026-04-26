/* ========================================================================== */
/*  adas.h  –  Advanced Driver Assistance System for Self-Balancing Robot     */
/*                                                                            */
/*  Features                                                                  */
/*    • Forward Collision Warning  (FCW)  – audible / LED alert               */
/*    • Autonomous Emergency Stop  (AES)  – overrides PID speed to 0         */
/*    • Lane / Corridor Keeping    (LCK)  – left HC-SR04 + right HC-SR04      */
/*    • Adaptive Speed Limiter     (ASL)  – reduces top speed near obstacles  */
/*    • Fall / Tilt Guard          (FTG)  – already in original; formalized   */
/*    • Syslog telemetry                  – every ADAS event logged           */
/* ========================================================================== */

#ifndef ADAS_H
#define ADAS_H

#include <stdint.h>

/* ---- GPIO pin assignments (wiringPi numbering) -------------------------- */
/* Front ultrasonic (HC-SR04) */
#define TRIG_FRONT      16
#define ECHO_FRONT      15

/* Left-side ultrasonic */
#define TRIG_LEFT       21
#define ECHO_LEFT       22

/* Right-side ultrasonic */
#define TRIG_RIGHT      24
#define ECHO_RIGHT      25

/* Buzzer for FCW alert */
#define BUZZER_PIN      28

/* LED indicators */
#define LED_FCW         29      /* red  – front collision warning  */
#define LED_LCK         27      /* amber – lane-keep active        */

/* ---- Distance thresholds (centimetres) ---------------------------------- */
#define DIST_EMERGENCY_STOP_CM   12.0f   /* AES: hard stop below this       */
#define DIST_FCW_ALERT_CM        25.0f   /* FCW: warn below this            */
#define DIST_SLOW_CM             40.0f   /* ASL: reduce max speed           */
#define DIST_LANE_KEEP_CM        15.0f   /* LCK: steer away if < this       */

/* ---- Tilt thresholds (degrees) ------------------------------------------ */
#define TILT_FALLOVER_DEG        60.0    /* FTG: matches original code       */
#define TILT_WARN_DEG            45.0    /* FTG: pre-fall warning            */

/* ---- ADAS speed limits -------------------------------------------------- */
#define SPEED_NORMAL_LIMIT       100.0   /* normal GUARD_GAIN                */
#define SPEED_SLOW_LIMIT         50.0    /* ASL reduced-speed zone           */
#define SPEED_STOP               0.0

/* ---- ADAS state machine ------------------------------------------------- */
typedef enum {
    ADAS_NOMINAL   = 0,   /* all clear, PID runs normally              */
    ADAS_WARN      = 1,   /* FCW active, speed capped, buzzer on       */
    ADAS_SLOW      = 2,   /* ASL active, speed reduced                 */
    ADAS_STEER_L   = 3,   /* LCK steering left                        */
    ADAS_STEER_R   = 4,   /* LCK steering right                       */
    ADAS_E_STOP    = 5,   /* AES: motors hard-stopped                  */
    ADAS_FALLOVER  = 6    /* FTG: robot tipped, motors stopped         */
} adas_state_t;

/* ---- Shared ADAS data structure (written by service_4, read by others) -- */
typedef struct {
    float         dist_front_cm;
    float         dist_left_cm;
    float         dist_right_cm;
    adas_state_t  state;
    double        speed_limit;      /* dynamic cap fed back into PID       */
    double        steer_offset_L;   /* lateral offset for left  motor      */
    double        steer_offset_R;   /* lateral offset for right motor      */
    int           fcw_active;
    int           aes_active;
    int           lck_active;
} adas_data_t;

/* ---- Function prototypes ------------------------------------------------ */
void   adas_init(void);
void   adas_update(adas_data_t *adas, double current_tilt_deg);
float  measure_distance_pin(int trig, int echo);
void   adas_alert_on(void);
void   adas_alert_off(void);
void   adas_log_state(const adas_data_t *adas);

#endif /* ADAS_H */
