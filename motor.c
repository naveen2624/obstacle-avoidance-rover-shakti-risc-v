/***************************************************************************
 * Project                      : shakti devt board
 * Name of the file             : rover_nav_ultra_motor_buzzer.c
 * Brief Description of file    : 3x HC-SR04 + L298N + buzzer integrated logic.
 *                                Forward with front sensor; on obstacle (<=100 cm),
 *                                stop, scan left/right, turn toward max distance.
 *                                If both sides <=100 cm, stop and beep buzzer.
 * Author                       : (combined & uses your exact pins)
 *****************************************************************************/
#include "platform.h"
#include "gpio.h"
#include "utils.h"
#include <stdio.h>
#include <stdint.h>

/* ======================= YOUR EXACT PIN MAP ======================= */
/* Buzzer */
#define BUZZER_PIN     (GPIO0)

/* Ultrasonic sensors */
#define TRIG_F         (GPIO1)    /* Front  TRIG (output) */
#define ECHO_F         (GPIO2)    /* Front  ECHO (input)  */

#define TRIG_L         (GPIO3)    /* Left   TRIG (output) */
#define ECHO_L         (GPIO4)    /* Left   ECHO (input)  */
#define TRIG_R         (GPIO5)    /* Right  TRIG (output) */
#define ECHO_R         (GPIO6)    /* Right  ECHO (input)  */

/* L298N */
#define IN1            (GPIO7)    /* Motor A IN1 */
#define IN2            (GPIO8)    /* Motor A IN2 */
#define IN3            (GPIO9)    /* Motor B IN3 */
#define IN4            (GPIO10)   /* Motor B IN4 */
#define ENA_PIN        (GPIO12)   /* ENA (drive HIGH unless jumpered) */
#define ENB_PIN        (GPIO13)   /* ENB (drive HIGH unless jumpered) */
/* ================================================================ */

#define CPU_FREQ_HZ    (50000000ULL)
#define US_TO_CYC(us)  ((CPU_FREQ_HZ/1000000ULL) * (uint64_t)(us))

#define LIMIT_CM               (100U)
#define LIMIT_TENTHS_CM        (LIMIT_CM * 10U)

#define TURN_TIME_MS           (1000U)     /* ~1s turn */
#define BETWEEN_PINGS_DELAY    do { delay_loop(1000,1000); } while(0)

#define BUZZ_ON_COUNT          (50000UL)
#define BUZZ_OFF_COUNT         (50000UL)

/* -------------------- low-level helpers -------------------- */
static inline uint64_t rdcycle(void){ uint64_t x; __asm__ volatile("rdcycle %0":"=r"(x)); return x; }

static inline void gpio_set(uint32_t mask){
    uint32_t v = read_word(GPIO_DATA_REG);
    write_word(GPIO_DATA_REG, v | mask);
}
static inline void gpio_clr(uint32_t mask){
    uint32_t v = read_word(GPIO_DATA_REG);
    write_word(GPIO_DATA_REG, v & ~mask);
}

static inline void busy_ticks(volatile unsigned long t){
    while (t--) { __asm__ volatile("nop"); }
}

/* Trigger: ~10 µs pulse */
static inline void trigger_sensor(uint32_t trig_mask){
    gpio_clr(trig_mask);
    delay_loop(100,100);
    gpio_set(trig_mask);
    delay_loop(10,10);
    gpio_clr(trig_mask);
}

/* Measure HIGH pulse (us) on ECHO; 0 on timeout */
static uint32_t measure_echo_us(uint32_t echo_mask){
    const uint64_t wait_rise_deadline = rdcycle() + US_TO_CYC(30000);
    const uint64_t max_high_cycles    = US_TO_CYC(30000);

    while ((read_word(GPIO_DATA_REG) & echo_mask) != 0U){
        if (rdcycle() > wait_rise_deadline) return 0U;
    }
    while ((read_word(GPIO_DATA_REG) & echo_mask) == 0U){
        if (rdcycle() > wait_rise_deadline) return 0U;
    }
    const uint64_t start = rdcycle();

    const uint64_t high_deadline = start + max_high_cycles;
    while ((read_word(GPIO_DATA_REG) & echo_mask) != 0U){
        if (rdcycle() > high_deadline) return 30000U;
    }

    const uint64_t cyc = rdcycle() - start;
    return (uint32_t)((cyc * 1000000ULL) / CPU_FREQ_HZ);
}

/* Convert microseconds to tenths of cm (integer) */
static inline uint32_t us_to_tenths_cm(uint32_t us){ return (us * 343U) / 200U; }

/* -------------------- motor control (RMW) -------------------- */
static inline void en_high(void){
    if (ENA_PIN) gpio_set(ENA_PIN);
    if (ENB_PIN) gpio_set(ENB_PIN);
}
static inline void motors_stop(void){
    /* Clear only IN bits; keep EN and others intact */
    uint32_t v = read_word(GPIO_DATA_REG);
    v &= ~(IN1 | IN2 | IN3 | IN4);
    write_word(GPIO_DATA_REG, v);
}
static inline void motors_forward(void){
    en_high();
    uint32_t v = read_word(GPIO_DATA_REG);
    v &= ~(IN2 | IN4);         /* ensure opposite pins LOW */
    v |=  (IN1 | IN3);         /* A fwd, B fwd */
    write_word(GPIO_DATA_REG, v);
}
static inline void motors_turn_left(void){
    en_high();
    uint32_t v = read_word(GPIO_DATA_REG);
    v &= ~(IN1 | IN2 | IN4);   /* stop left motor, ensure IN4 low */
    v |=  IN3;                 /* right motor forward */
    write_word(GPIO_DATA_REG, v);
}
static inline void motors_turn_right(void){
    en_high();
    uint32_t v = read_word(GPIO_DATA_REG);
    v &= ~(IN3 | IN4 | IN2);   /* stop right motor, ensure IN2 low */
    v |=  IN1;                 /* left motor forward */
    write_word(GPIO_DATA_REG, v);
}

/* -------------------- buzzer -------------------- */
static inline void buzzer_beep_blocking(void){
    while (1){
        gpio_set(BUZZER_PIN); busy_ticks(BUZZ_ON_COUNT);
        gpio_clr(BUZZER_PIN); busy_ticks(BUZZ_OFF_COUNT);
    }
}

/* -------------------- main -------------------- */
void main(void)
{
    /* Outputs: buzzer, all TRIGs, motor INs, EN pins */
    uint32_t dir_out = BUZZER_PIN | TRIG_F | TRIG_L | TRIG_R | IN1 | IN2 | IN3 | IN4 | ENA_PIN | ENB_PIN;
    write_word(GPIO_DIRECTION_CNTRL_REG, dir_out);

    /* init states */
    motors_stop();
    gpio_clr(BUZZER_PIN);
    en_high();  /* keep ENA/ENB high if wired */

    while (1){
        /* Phase 1: forward, only front sensor active */
        motors_forward();

        trigger_sensor(TRIG_F);
        uint32_t f_us = measure_echo_us(ECHO_F);
        uint32_t f_tc = f_us ? us_to_tenths_cm(f_us) : 0;
        printf("\nFRONT: %u us | %u.%u cm", f_us, f_tc/10, f_tc%10);

        if (f_us && f_tc <= LIMIT_TENTHS_CM){
            motors_stop();
            printf("\nOBSTACLE <=100 cm. Scanning sides...");

            BETWEEN_PINGS_DELAY;

            /* Left */
            trigger_sensor(TRIG_L);
            uint32_t l_us = measure_echo_us(ECHO_L);
            uint32_t l_tc = l_us ? us_to_tenths_cm(l_us) : 0;
            printf("\nLEFT : %u us | %u.%u cm", l_us, l_tc/10, l_tc%10);

            BETWEEN_PINGS_DELAY;

            /* Right */
            trigger_sensor(TRIG_R);
            uint32_t r_us = measure_echo_us(ECHO_R);
            uint32_t r_tc = r_us ? us_to_tenths_cm(r_us) : 0;
            printf("\nRIGHT: %u us | %u.%u cm", r_us, r_tc/10, r_tc%10);

            uint8_t left_ok  = (l_us && l_tc > LIMIT_TENTHS_CM);
            uint8_t right_ok = (r_us && r_tc > LIMIT_TENTHS_CM);

            if (left_ok || right_ok){
                if (left_ok && (!right_ok || l_tc >= r_tc)){
                    printf("\nTURN LEFT...");
                    motors_turn_left();
                } else {
                    printf("\nTURN RIGHT...");
                    motors_turn_right();
                }
                delay_loop(TURN_TIME_MS, 1000);
                motors_stop();
                BETWEEN_PINGS_DELAY;
            } else {
                printf("\nBLOCKED BOTH SIDES. BUZZER ON.");
                motors_stop();
                buzzer_beep_blocking();
            }
        }

        BETWEEN_PINGS_DELAY;
    }
}

