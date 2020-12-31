

/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example uses GPIO18/19/4/5 as LEDC output,
 *    and it will change the duty repeatedly.
 *
 * 5. GPIO18/19 are from high speed channel group.
 *    GPIO4/5 are from low speed channel group.
 *
 */
#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER             LEDC_TIMER_0
#define LEDC_HS_MODE              LEDC_HIGH_SPEED_MODE

#define LEDC_HS_ONBOARD           (2) // onboard blue LED
#define LEDC_HS_ONBOARD_CHANNEL   LEDC_CHANNEL_0

#define LEDC_HS_BLUE              (19) // blue
#define LEDC_HS_BLUE_CHANNEL      LEDC_CHANNEL_1
#endif

#define LEDC_LS_TIMER             LEDC_TIMER_1
#define LEDC_LS_MODE              LEDC_LOW_SPEED_MODE

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define LEDC_LS_CH0_GPIO          (18)
#define LEDC_LS_CH0_CHANNEL       LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO          (19)
#define LEDC_LS_CH1_CHANNEL       LEDC_CHANNEL_1
#endif

#define LEDC_LS_RED               (4) // red
#define LEDC_LS_RED_CHANNEL       LEDC_CHANNEL_2

#define LEDC_LS_GREEN             (5) // green
#define LEDC_LS_GREEN_CHANNEL     LEDC_CHANNEL_3

#define LEDC_NUM_CHANNELS         (4)
#define LEDC_DUTY_CYCLE           (4000)
#define LEDC_FADE_TIME            (3000)