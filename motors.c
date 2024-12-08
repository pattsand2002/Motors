#include "motors.h"
#include <nrf_pwm.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"
#include <stdio.h>

#define PERIOD_MS 1000 
#define GEAR_RATIO 51.45
#define ENCODER_CPR 12
#define EFFECTIVE_CPR (GEAR_RATIO * ENCODER_CPR)

// Define PWM pins and motor control pins
#define PWM_MOTOR_AIN1_PIN 3 //5   
#define PWM_MOTOR_AIN2_PIN 2 //4   
#define PWM_MOTOR_BIN1_PIN 1 //3  
#define PWM_MOTOR_BIN2_PIN 0 //2   

// Define Encoder pins for both motors
#define ENCODER_A_CHANNEL_A_PIN 12 
#define ENCODER_A_CHANNEL_B_PIN 8 
#define ENCODER_B_CHANNEL_A_PIN 19 
#define ENCODER_B_CHANNEL_B_PIN 16 

// Define PWM frequency and duty cycle variables
#define PWM_FREQUENCY 20000  // 20 kHz
#define MAX_DUTY_CYCLE 100
#define MIN_DUTY_CYCLE 0

static nrf_pwm_values_individual_t pwm_duty_cycle = {0, 0, 0, 0};

// Encoder variables
volatile int motor_a_position = 0;
volatile int motor_b_position = 0;

void encoder_init(void) {
    // Configure encoder pins as inputs with pull-up resistors
    nrf_gpio_cfg_input(ENCODER_A_CHANNEL_A_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(ENCODER_A_CHANNEL_B_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(ENCODER_B_CHANNEL_A_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(ENCODER_B_CHANNEL_B_PIN, NRF_GPIO_PIN_PULLUP);
}

void pwm_init(void) {

    NRF_PWM0->PSEL.OUT[0] = PWM_MOTOR_AIN1_PIN;  // Motor AIN1
    NRF_PWM0->PSEL.OUT[1] = PWM_MOTOR_AIN2_PIN;  // Motor AIN2
    NRF_PWM0->PSEL.OUT[2] = PWM_MOTOR_BIN1_PIN;  // Motor BIN1
    NRF_PWM0->PSEL.OUT[3] = PWM_MOTOR_BIN2_PIN;  // Motor BIN2

    NRF_PWM0->ENABLE = PWM_ENABLE_ENABLE_Enabled;
    NRF_PWM0->MODE = PWM_MODE_UPDOWN_Up;
    NRF_PWM0->COUNTERTOP = 16e6 / PWM_FREQUENCY; // Set frequency
    NRF_PWM0->LOOP = PWM_LOOP_CNT_Disabled;
    NRF_PWM0->DECODER = PWM_DECODER_LOAD_Individual | PWM_DECODER_MODE_RefreshCount;
    NRF_PWM0->SEQ[0].PTR = (uint32_t)&pwm_duty_cycle;
    NRF_PWM0->SEQ[0].CNT = sizeof(pwm_duty_cycle) / sizeof(uint16_t);
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;

    NRF_PWM0->TASKS_SEQSTART[0] = 1; // Start PWM

    printf("Updated PWM frequency: %d Hz\n", PWM_FREQUENCY);
    printf("COUNTERTOP value: %lu\n", (unsigned long)NRF_PWM0->COUNTERTOP);   
    printf("PWM initialized on pins: AIN1(%d), AIN2(%d), BIN1(%d), BIN2(%d)\n",
       PWM_MOTOR_AIN1_PIN, PWM_MOTOR_AIN2_PIN, PWM_MOTOR_BIN1_PIN, PWM_MOTOR_BIN2_PIN);

}

void set_motor_speed(uint8_t motor, uint8_t duty_cycle) {
    if (duty_cycle > MAX_DUTY_CYCLE) duty_cycle = MAX_DUTY_CYCLE;

    // Set duty cycle for the specified motor
    if (motor == 0) {  // Motor A
        pwm_duty_cycle.channel_0 = duty_cycle;  // AIN1
        pwm_duty_cycle.channel_1 = 0;          // AIN2
    } else if (motor == 1) {  // Motor B
        pwm_duty_cycle.channel_2 = duty_cycle;  // BIN1
        pwm_duty_cycle.channel_3 = 0;          // BIN2
    }
    NRF_PWM0->TASKS_SEQSTART[0] = 1; // Update PWM
    
    printf("Motor %d Duty Cycle: Channel 0: %d, Channel 1: %d\n", motor, 
       pwm_duty_cycle.channel_0, pwm_duty_cycle.channel_1);
}

void change_motor_direction(uint8_t motor, bool forward) {
    if (motor == 0) {  // Motor A
        pwm_duty_cycle.channel_0 = forward ? pwm_duty_cycle.channel_0 : 0;
        pwm_duty_cycle.channel_1 = forward ? 0 : pwm_duty_cycle.channel_0;
    } else if (motor == 1) {  // Motor B
        pwm_duty_cycle.channel_2 = forward ? pwm_duty_cycle.channel_2 : 0;
        pwm_duty_cycle.channel_3 = forward ? 0 : pwm_duty_cycle.channel_2;
    }
    NRF_PWM0->TASKS_SEQSTART[0] = 1; // Update PWM
}

void accelerate_motors(uint8_t start_duty, uint8_t end_duty, uint8_t step, uint16_t delay_ms) {
    for (uint8_t duty = start_duty; duty <= end_duty; duty += step) {
        set_motor_speed(0, duty); // Motor A
        set_motor_speed(1, duty); // Motor B
        nrf_delay_ms(delay_ms);
    }
}

void decelerate_motors(uint8_t start_duty, uint8_t end_duty, uint8_t step, uint16_t delay_ms) {
    for (uint8_t duty = start_duty; duty >= end_duty; duty -= step) {
        set_motor_speed(0, duty); // Motor A
        set_motor_speed(1, duty); // Motor B
        nrf_delay_ms(delay_ms);
    }
}

// Encoder polling function
void read_encoder_values(void) {
    static bool last_a_a = false, last_a_b = false;
    static bool last_b_a = false, last_b_b = false;

    bool current_a_a = nrf_gpio_pin_read(ENCODER_A_CHANNEL_A_PIN);
    bool current_a_b = nrf_gpio_pin_read(ENCODER_A_CHANNEL_B_PIN);
    bool current_b_a = nrf_gpio_pin_read(ENCODER_B_CHANNEL_A_PIN);
    bool current_b_b = nrf_gpio_pin_read(ENCODER_B_CHANNEL_B_PIN);

    // Motor A
    if (current_a_a != last_a_a || current_a_b != last_a_b) {
        if (current_a_a == current_a_b) {
            motor_a_position++;
        } else {
            motor_a_position--;
        }
        last_a_a = current_a_a;
        last_a_b = current_a_b;
    }

    // Motor B
    if (current_b_a != last_b_a || current_b_b != last_b_b) {
        if (current_b_a == current_b_b) {
            motor_b_position++;
        } else {
            motor_b_position--;
        }
        last_b_a = current_b_a;
        last_b_b = current_b_b;
    }
    printf("Motor A Encoder A: %d, Encoder B: %d\n", current_a_a, current_a_b);
    printf("Motor A Position: %d\n", motor_a_position);
    printf("Motor B Encoder A: %d, Encoder B: %d\n", current_b_a, current_b_b);
    printf("Motor B Position: %d\n", motor_b_position);

}

void calculate_rpm(void) {
    static int last_motor_a_position = 0;
    static int last_motor_b_position = 0;

    int delta_motor_a = motor_a_position - last_motor_a_position;
    int delta_motor_b = motor_b_position - last_motor_b_position;

    last_motor_a_position = motor_a_position;
    last_motor_b_position = motor_b_position;

    float rpm_motor_a = (delta_motor_a / EFFECTIVE_CPR) * (60000.0 / PERIOD_MS);
    float rpm_motor_b = (delta_motor_b / EFFECTIVE_CPR) * (60000.0 / PERIOD_MS);

    printf("Motor A RPM: %.2f\n", rpm_motor_a);
    printf("Motor B RPM: %.2f\n", rpm_motor_b);
}


int main(void) {
    pwm_init();
    encoder_init();
    // Debug: Test GPIO motor control 
    nrf_gpio_cfg_output(PWM_MOTOR_AIN1_PIN);
    nrf_gpio_cfg_output(PWM_MOTOR_AIN2_PIN);
    nrf_gpio_pin_write(PWM_MOTOR_AIN1_PIN, 1); // Enable AIN1
    nrf_gpio_pin_write(PWM_MOTOR_AIN2_PIN, 0); // Disable AIN2

    // Debug:
    change_motor_direction(0, true); // Ensure Motor A moves forward
    set_motor_speed(0, 50);          
    nrf_delay_ms(2000); // Run for 2 seconds
    printf("Motor A test complete.\n");

    while (1) {

        calculate_rpm();
        nrf_delay_ms(PERIOD_MS);
        read_encoder_values();

        accelerate_motors(0, 100, 10, 10); 
        nrf_delay_ms(2000);               
        decelerate_motors(100, 0, 10, 10);

        // Independent motor actions
        change_motor_direction(0, false); 
        set_motor_speed(0, 50);           
        set_motor_speed(1, 50);      
    }
}
