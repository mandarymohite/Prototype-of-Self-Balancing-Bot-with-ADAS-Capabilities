/* ========================================================================== */
/*  mySegway.c  –  Self-Balancing Robot  ·  with ADAS Extension              */
/*                                                                            */
/*  Architecture                                                              */
/*  ──────────────────────────────────────────────────────────────────────    */
/*  Sequencer (SCHED_FIFO, max prio) @ 800 Hz                                */
/*    ├─ Service 1 @ 400 Hz  –  IMU read + complementary filter              */
/*    ├─ Service 2 @ 400 Hz  –  PID compute                                  */
/*    ├─ Service 3 @ 400 Hz  –  Motor command (PID + ADAS offsets)           */
/*    └─ Service 4 @ 200 Hz  –  ADAS sensor scan + state machine             */
/* ========================================================================== */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/time.h>

#include <errno.h>
#include <signal.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <assert.h>

#include "adas.h"          /* ADAS feature module                            */

/* ── timing constants ────────────────────────────────────────────────────── */
#define USEC_PER_MSEC       (1000)
#define NANOSEC_PER_SEC     (1000000000)
#define NSEC_PER_SEC        (1000000000)
#define NUM_CPU_CORES       (1)
#define U32_T               unsigned int

/* ── thread / semaphore count ────────────────────────────────────────────── */
#define NUM_THREADS         (5)     /* Sequencer + S1 S2 S3 S4               */

/* ── IMU scaling ─────────────────────────────────────────────────────────── */
#define RAD_TO_DEG          57.29578
#define GYRO_SCALE_X        131.0
#define GYRO_SCALE_Y        131.0

/* ── PID gains ───────────────────────────────────────────────────────────── */
double Kp = 14.0;
double Ki = 8.0;
double Kd = 210.0;
double K  = 1.9 * 1.12;

/* ── Complementary filter weights ────────────────────────────────────────── */
double K0 = 0.98;      /* gyroscope weight  */
double K1 = 0.02;      /* accelerometer weight */

/* ── Feasibility test tables ─────────────────────────────────────────────── */
U32_T ex1_period[]   = {2500, 2500, 2500, 5000};
U32_T ex1_wcet[]     = {1407,   44,   63,  200};
U32_T ex1_deadline[] = {2323, 2396, 2500, 5000};

/* ── Global IMU state ────────────────────────────────────────────────────── */
int fd;
int acclX, acclY, acclZ;
int gyroX, gyroY, gyroZ;
double accl_scaled_x, accl_scaled_y, accl_scaled_z;
double gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;

/* ── Filter / orientation state ──────────────────────────────────────────── */
double gyro_offset_x, gyro_offset_y;
double gyro_total_x,  gyro_total_y;
double gyro_x_delta,  gyro_y_delta;
double rotation_x,    rotation_y;
double last_x,        last_y;

/* ── PID state ───────────────────────────────────────────────────────────── */
double GUARD_GAIN    = 100.0;
double error, last_error, integrated_error;
double pTerm, iTerm, dTerm;
double angle;
double angle_offset  = 0.0;
double speed;

/* ── Timing ──────────────────────────────────────────────────────────────── */
double deltaT;
struct timeval tv, tv2;
struct timeval start_time_val;
unsigned long long timer, t, bootup;

/* ── Thread control ──────────────────────────────────────────────────────── */
int abortTest = 0;
int abortS1   = 0,  abortS2 = 0,  abortS3 = 0,  abortS4 = 0;
sem_t semS1, semS2, semS3, semS4;

/* ── Shared ADAS data (written by S4, read by S2/S3) ────────────────────── */
adas_data_t g_adas;
pthread_mutex_t adas_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    int                threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;

/* ── Forward declarations ────────────────────────────────────────────────── */
double getTimeMsec(void);
void   print_scheduler(void);
void   testAxesData(void);
void   testMotorControl(void);
int    completion_time_feasibility(U32_T n, U32_T p[], U32_T w[], U32_T d[]);
int    scheduling_point_feasibility(U32_T n, U32_T p[], U32_T w[], U32_T d[]);

void *Sequencer(void *threadp);
void *service_1(void *threadp);
void *service_2(void *threadp);
void *service_3(void *threadp);
void *service_4(void *threadp);

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Low-level sensor helpers                                                   */
/* ─────────────────────────────────────────────────────────────────────────── */

/* Read a 16-bit 2's-complement word from the MPU-6050 via wiringPi I2C */
int read_word_2c(int addr)
{
    int val;
    val  = wiringPiI2CReadReg8(fd, addr);
    val  = val << 8;
    val += wiringPiI2CReadReg8(fd, addr + 1);
    if (val >= 0x8000)
        val = -(65536 - val);
    return val;
}

double dist_3d(double a, double b)
{
    return sqrt((a * a) + (b * b));
}

double get_y_rotation(double x, double y, double z)
{
    return atan2(y, dist_3d(x, z)) * (180.0 / M_PI);
}

double get_x_rotation(double x, double y, double z)
{
    return -(atan2(x, dist_3d(y, z)) * (180.0 / M_PI));
}

/* Read all 6 axes from MPU-6050 and scale to SI units */
void read_all(void)
{
    acclX = read_word_2c(0x3D);
    acclY = read_word_2c(0x3B);
    acclZ = read_word_2c(0x3F);

    accl_scaled_x = acclX / 16384.0;
    accl_scaled_y = acclY / 16384.0;
    accl_scaled_z = acclZ / 16384.0;

    gyroX = read_word_2c(0x45);
    gyroY = read_word_2c(0x43);
    gyroZ = read_word_2c(0x47);

    gyro_scaled_x = gyroX / GYRO_SCALE_X;
    gyro_scaled_y = gyroY / GYRO_SCALE_Y;
    gyro_scaled_z = gyroZ / GYRO_SCALE_X;
}

unsigned long long getTimestamp(void)
{
    gettimeofday(&tv, NULL);
    return (unsigned long long)tv.tv_sec * 1000000ULL + tv.tv_usec;
}

double constrain(double v, double min_v, double max_v)
{
    if (v <= min_v) return min_v;
    if (v >= max_v) return max_v;
    return v;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  PID – speed is capped by the dynamic ADAS speed limit                     */
/* ─────────────────────────────────────────────────────────────────────────── */
void pid(void)
{
    error = last_y - angle_offset;

    pTerm = Kp * error;

    integrated_error = 0.95 * integrated_error + error;
    iTerm = Ki * integrated_error;

    dTerm = Kd * (error - last_error);
    last_error = error;

    /* Fetch ADAS speed limit (thread-safe read) */
    double adas_limit;
    pthread_mutex_lock(&adas_mutex);
    adas_limit = g_adas.speed_limit;
    pthread_mutex_unlock(&adas_mutex);

    /* Replace fixed GUARD_GAIN with dynamic ADAS limit */
    speed = constrain(K * (pTerm + iTerm + dTerm), -adas_limit, adas_limit);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Signal handler – clean motor stop on Ctrl-C                               */
/* ─────────────────────────────────────────────────────────────────────────── */
void sigintHandler(int sig)
{
    (void)sig;
    stop_motors();
    adas_alert_off();
    syslog(LOG_INFO, "SIGINT received – motors stopped, exiting.");
    exit(EXIT_SUCCESS);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Startup self-tests                                                         */
/* ─────────────────────────────────────────────────────────────────────────── */
void testAxesData(void)
{
    acclX = read_word_2c(0x3D);
    acclY = read_word_2c(0x3B);
    acclZ = read_word_2c(0x3F);

    accl_scaled_x = acclX / 16384.0;
    accl_scaled_y = acclY / 16384.0;
    accl_scaled_z = acclZ / 16384.0;

    gyroX = read_word_2c(0x45);
    gyroY = read_word_2c(0x43);
    gyroZ = read_word_2c(0x47);

    gyro_scaled_x = gyroX / GYRO_SCALE_X;
    gyro_scaled_y = gyroY / GYRO_SCALE_Y;
    gyro_scaled_z = gyroZ / GYRO_SCALE_X;

    const double MAX_ACCL = 16.0,  MIN_ACCL = -16.0;
    const double MAX_GYRO = 2000.0, MIN_GYRO = -2000.0;

    assert(acclX >= MIN_ACCL && acclX <= MAX_ACCL);
    assert(acclY >= MIN_ACCL && acclY <= MAX_ACCL);
    assert(acclZ >= MIN_ACCL && acclZ <= MAX_ACCL);
    assert(gyroX >= MIN_GYRO && gyroX <= MAX_GYRO);
    assert(gyroY >= MIN_GYRO && gyroY <= MAX_GYRO);
    assert(gyroZ >= MIN_GYRO && gyroZ <= MAX_GYRO);

    printf("[TEST] IMU axes data within range – PASS\n");
}

void testMotorControl(void)
{
    init_motors();
    move_forward();
    motors(50.0,  0.0,  0.0);
    motors(-50.0, 0.0,  0.0);
    motors(0.0,  20.0, -20.0);
    motors(0.0, -20.0,  20.0);
    stop_motors();
    printf("[TEST] Motor control sequence – PASS\n");
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  main()                                                                     */
/* ─────────────────────────────────────────────────────────────────────────── */
void main(void)
{
    wiringPiSetup();
    signal(SIGINT, sigintHandler);

    /* Initialise ADAS GPIO and syslog */
    adas_init();

    /* Initialise motor driver */
    init_motors();

    /* Set ADAS default state – full speed allowed until first sensor sweep */
    g_adas.state         = ADAS_NOMINAL;
    g_adas.speed_limit   = SPEED_NORMAL_LIMIT;
    g_adas.steer_offset_L = 0.0;
    g_adas.steer_offset_R = 0.0;
    g_adas.fcw_active    = 0;
    g_adas.aes_active    = 0;
    g_adas.lck_active    = 0;

    bootup = getTimestamp();
    delay(200);

    /* ── I2C MPU-6050 setup ─────────────────────────────────────────────── */
    fd = wiringPiI2CSetupInterface("/dev/i2c-1", 0x68);
    wiringPiI2CWriteReg8(fd, 0x6B, 0x00);   /* wake up MPU-6050 */

    timer    = getTimestamp();
    deltaT   = (double)(getTimestamp() - timer) / 1000000.0;
    read_all();

    last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

    gyro_offset_x = gyro_scaled_x;
    gyro_offset_y = gyro_scaled_y;
    gyro_total_x  = last_x - gyro_offset_x;
    gyro_total_y  = last_y - gyro_offset_y;

    /* ── Self-tests ─────────────────────────────────────────────────────── */
    printf("\n[BOOT] Running self-tests ...\n");
    testAxesData();
    testMotorControl();
    printf("[BOOT] Self-tests complete.\n\n");

    /* ── RT scheduling setup ────────────────────────────────────────────── */
    struct sched_param main_param;
    pthread_attr_t     main_attr;
    pid_t mainpid = getpid();

    int rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    int rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    main_param.sched_priority = rt_max_prio;
    if (sched_setscheduler(mainpid, SCHED_FIFO, &main_param) < 0)
        perror("sched_setscheduler");
    print_scheduler();

    /* ── Semaphores ─────────────────────────────────────────────────────── */
    if (sem_init(&semS1, 0, 0)) { printf("sem_init S1 failed\n"); exit(-1); }
    if (sem_init(&semS2, 0, 0)) { printf("sem_init S2 failed\n"); exit(-1); }
    if (sem_init(&semS3, 0, 0)) { printf("sem_init S3 failed\n"); exit(-1); }
    if (sem_init(&semS4, 0, 0)) { printf("sem_init S4 failed\n"); exit(-1); }

    /* ── Thread attributes ──────────────────────────────────────────────── */
    pthread_t       threads[NUM_THREADS];
    threadParams_t  threadParams[NUM_THREADS];
    pthread_attr_t  rt_sched_attr[NUM_THREADS];
    struct sched_param rt_param[NUM_THREADS];
    cpu_set_t threadcpu;
    int i, rc;

    for (i = 0; i < NUM_THREADS; i++)
    {
        CPU_ZERO(&threadcpu);
        CPU_SET(3, &threadcpu);
        rc = pthread_attr_init(&rt_sched_attr[i]);
        rc = pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
        rc = pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
        rt_param[i].sched_priority = rt_max_prio - i;
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
        threadParams[i].threadIdx = i;
    }

    threadParams[0].sequencePeriods = 8000000;

    /* ── Service 1: IMU sensor read (max-2) ─────────────────────────────── */
    rt_param[1].sched_priority = rt_max_prio - 2;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc = pthread_create(&threads[1], &rt_sched_attr[1], service_1, (void *)&threadParams[1]);
    if (rc < 0) perror("pthread_create S1");
    else printf("[BOOT] Service 1 (IMU)     started  prio=max-2\n");

    /* ── Service 2: PID compute (max-3) ─────────────────────────────────── */
    rt_param[2].sched_priority = rt_max_prio - 3;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc = pthread_create(&threads[2], &rt_sched_attr[2], service_2, (void *)&threadParams[2]);
    if (rc < 0) perror("pthread_create S2");
    else printf("[BOOT] Service 2 (PID)     started  prio=max-3\n");

    /* ── Service 3: Motor command (max-4) ───────────────────────────────── */
    rt_param[3].sched_priority = rt_max_prio - 4;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
    rc = pthread_create(&threads[3], &rt_sched_attr[3], service_3, (void *)&threadParams[3]);
    if (rc < 0) perror("pthread_create S3");
    else printf("[BOOT] Service 3 (Motor)   started  prio=max-4\n");

    /* ── Service 4: ADAS sensor + state machine (max-5) ─────────────────── */
    rt_param[4].sched_priority = rt_max_prio - 5;
    pthread_attr_setschedparam(&rt_sched_attr[4], &rt_param[4]);
    rc = pthread_create(&threads[4], &rt_sched_attr[4], service_4, (void *)&threadParams[4]);
    if (rc < 0) perror("pthread_create S4");
    else printf("[BOOT] Service 4 (ADAS)    started  prio=max-5\n");

    /* ── Sequencer: highest prio (max) ──────────────────────────────────── */
    rt_param[0].sched_priority = rt_max_prio;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc = pthread_create(&threads[0], &rt_sched_attr[0], Sequencer, (void *)&threadParams[0]);
    if (rc < 0) perror("pthread_create Sequencer");
    else printf("[BOOT] Sequencer           started  prio=max\n\n");

    /* ── Feasibility tests ──────────────────────────────────────────────── */
    U32_T numServices = 4;
    printf("[SCHED] Completion-time test  : %s\n",
        completion_time_feasibility(numServices, ex1_period, ex1_wcet, ex1_deadline) ? "FEASIBLE" : "INFEASIBLE");
    printf("[SCHED] Scheduling-point test : %s\n\n",
        scheduling_point_feasibility(numServices, ex1_period, ex1_wcet, ex1_deadline) ? "FEASIBLE" : "INFEASIBLE");

    /* Join all threads */
    for (i = 0; i < NUM_THREADS; i++)
        pthread_join(threads[i], NULL);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Sequencer – 800 Hz tick, releases services at sub-rates                   */
/* ─────────────────────────────────────────────────────────────────────────── */
void *Sequencer(void *threadp)
{
    struct timespec delay_time = {0, 1250000};  /* 1.25 ms → 800 Hz          */
    struct timespec remaining_time;
    double residual;
    int rc, delay_cnt;
    unsigned long long seqCnt = 0;
    threadParams_t *tp = (threadParams_t *)threadp;

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    printf("[SEQ] Sequencer started @ %ld.%06ld\n", tv_now.tv_sec, tv_now.tv_usec);

    do {
        delay_cnt = 0; residual = 0.0;
        do {
            rc = nanosleep(&delay_time, &remaining_time);
            if (rc == EINTR)
            {
                residual = remaining_time.tv_sec +
                           (double)remaining_time.tv_nsec / NANOSEC_PER_SEC;
                delay_cnt++;
            }
            else if (rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }
        } while (residual > 0.0 && delay_cnt < 100);

        seqCnt++;

        /* S1: 400 Hz (every 2 ticks) */
        if ((seqCnt % 2) == 0) sem_post(&semS1);

        /* S4 (ADAS): 200 Hz (every 4 ticks) */
        if ((seqCnt % 4) == 0) sem_post(&semS4);

    } while (!abortTest && seqCnt < tp->sequencePeriods);

    sem_post(&semS1); sem_post(&semS4);
    abortS1 = 1; abortS4 = 1;
    pthread_exit(NULL);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Service 1 – IMU read + complementary filter  (400 Hz)                    */
/* ─────────────────────────────────────────────────────────────────────────── */
void *service_1(void *threadp)
{
    timer  = getTimestamp();
    deltaT = (double)(getTimestamp() - timer) / 1000000.0;
    read_all();

    last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    gyro_offset_x = gyro_scaled_x;
    gyro_offset_y = gyro_scaled_y;
    gyro_total_x  = last_x - gyro_offset_x;
    gyro_total_y  = last_y - gyro_offset_y;

    double WCET = 0, max_time = 0;

    while (!abortS1)
    {
        sem_wait(&semS1);

        t      = getTimestamp();
        deltaT = (double)(t - timer) / 1000000.0;
        timer  = t;

        double start = getTimestamp();
        read_all();

        gyro_scaled_x -= gyro_offset_x;
        gyro_scaled_y -= gyro_offset_y;

        gyro_x_delta = gyro_scaled_x * deltaT;
        gyro_y_delta = gyro_scaled_y * deltaT;

        gyro_total_x += gyro_x_delta;
        gyro_total_y += gyro_y_delta;

        rotation_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
        rotation_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

        last_x = K0 * (last_x + gyro_x_delta) + K1 * rotation_x;
        last_y = K0 * (last_y + gyro_y_delta) + K1 * rotation_y;

        double finish = getTimestamp();
        WCET = finish - start;
        if (WCET > max_time) max_time = WCET;

        /* Fall-over guard – stop motors immediately if >60° */
        if (last_y < -TILT_FALLOVER_DEG || last_y > TILT_FALLOVER_DEG)
        {
            stop_motors();
            syslog(LOG_WARNING, "S1: Fall-over tilt detected (%.1f deg), motors stopped", last_y);
        }

        sem_post(&semS2);
    }
    pthread_exit(NULL);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Service 2 – PID compute  (400 Hz)                                        */
/* ─────────────────────────────────────────────────────────────────────────── */
void *service_2(void *threadp)
{
    double WCET = 0, max_time = 0;

    while (1)
    {
        sem_wait(&semS2);
        double start = getTimestamp();

        pid();          /* speed is now clamped to g_adas.speed_limit */

        double finish = getTimestamp();
        WCET = finish - start;
        if (WCET > max_time) max_time = WCET;

        printf("\rerror=%+7.3lf  speed=%+7.3lf  ADAS=%d",
               error, speed, g_adas.state);
        fflush(stdout);

        sem_post(&semS3);
    }
    pthread_exit(NULL);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Service 3 – Motor command  (400 Hz)                                       */
/*  Reads ADAS steer offsets and applies them to the motor call.             */
/* ─────────────────────────────────────────────────────────────────────────── */
void *service_3(void *threadp)
{
    double WCET = 0, max_time = 0;

    while (1)
    {
        sem_wait(&semS3);
        double start = getTimestamp();

        /* Read ADAS overrides under mutex */
        double off_L, off_R;
        int    aes;
        pthread_mutex_lock(&adas_mutex);
        off_L = g_adas.steer_offset_L;
        off_R = g_adas.steer_offset_R;
        aes   = g_adas.aes_active;
        pthread_mutex_unlock(&adas_mutex);

        if (aes)
        {
            /* Autonomous Emergency Stop – bypass PID entirely */
            stop_motors();
        }
        else
        {
            /* Normal PID drive with optional ADAS steering offsets */
            motors(speed, off_L, off_R);
        }

        double finish = getTimestamp();
        WCET = finish - start;
        if (WCET > max_time) max_time = WCET;
    }
    pthread_exit(NULL);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Service 4 – ADAS sensor scan + state machine  (200 Hz)                   */
/* ─────────────────────────────────────────────────────────────────────────── */
void *service_4(void *threadp)
{
    adas_data_t local;

    while (!abortS4)
    {
        sem_wait(&semS4);

        /* Run ADAS state machine (samples all 3 ultrasonic sensors) */
        adas_update(&local, last_y);   /* last_y = current tilt from S1      */

        /* Publish to shared ADAS structure */
        pthread_mutex_lock(&adas_mutex);
        g_adas = local;
        pthread_mutex_unlock(&adas_mutex);
    }
    pthread_exit(NULL);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Utility functions                                                          */
/* ─────────────────────────────────────────────────────────────────────────── */
double getTimeMsec(void)
{
    struct timespec ts = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}

void print_scheduler(void)
{
    int t = sched_getscheduler(getpid());
    switch (t)
    {
        case SCHED_FIFO:  printf("[SCHED] Policy = SCHED_FIFO\n");  break;
        case SCHED_OTHER: printf("[SCHED] Policy = SCHED_OTHER\n"); exit(-1);
        case SCHED_RR:    printf("[SCHED] Policy = SCHED_RR\n");    exit(-1);
        default:          printf("[SCHED] Policy = UNKNOWN\n");     exit(-1);
    }
}

int completion_time_feasibility(U32_T n, U32_T p[], U32_T w[], U32_T d[])
{
    int i, j, feasible = 1;
    U32_T an, anext;
    for (i = 0; i < (int)n; i++)
    {
        an = anext = 0;
        for (j = 0; j <= i; j++) an += w[j];
        while (1)
        {
            anext = w[i];
            for (j = 0; j < i; j++)
                anext += (U32_T)(ceil((double)an / p[j]) * w[j]);
            if (anext == an) break;
            an = anext;
        }
        if (an > d[i]) feasible = 0;
    }
    return feasible;
}

int scheduling_point_feasibility(U32_T n, U32_T p[], U32_T w[], U32_T d[])
{
    int i, j, k, l, rc = 1, status, temp;
    for (i = 0; i < (int)n; i++)
    {
        status = 0;
        for (k = 0; k <= i; k++)
        {
            for (l = 1; l <= (int)floor((double)p[i] / p[k]); l++)
            {
                temp = 0;
                for (j = 0; j <= i; j++)
                    temp += w[j] * (int)ceil((double)l * p[k] / p[j]);
                if (temp <= (int)(l * p[k])) { status = 1; break; }
            }
            if (status) break;
        }
        if (!status) rc = 0;
    }
    return rc;
}
