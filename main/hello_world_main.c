/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
//Holllllllllllllla
//Adioosss


#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/ledc.h"

#define NUM_SERVOS 7
#define PWM_FREQ_HZ 50
#define PWM_RESOLUTION 10

// Configuración de pines y canales
const int servo_pins[NUM_SERVOS] = {10, 11, 12, 13, 14, 15, 16};
const ledc_channel_t servo_channels[NUM_SERVOS] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
    LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_CHANNEL_5,
    LEDC_CHANNEL_6
};

void init_servos() {
    // Configuración del timer PWM
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);

    // Configuración de canales para cada servo
    for (int i = 0; i < NUM_SERVOS; i++) {
        ledc_channel_config_t channel_config = {
            .gpio_num = servo_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = servo_channels[i],
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel_config);
    }
}

void move_servo(uint8_t servo_num, float angle) {
    // Validación de parámetros
    if (servo_num < 1 || servo_num > NUM_SERVOS) {
        printf("Error: Número de servo inválido (1-%d)\n", NUM_SERVOS);
        return;
    }
    
    // Ajustar índice (convertir de 1-7 a 0-6)
    uint8_t index = servo_num - 1;
    
    // Limitar ángulo a rango válido
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Conversión de ángulo a duty cycle (para 10 bits)
    const uint32_t min_duty = 25;    // 0.5ms (0°)
    const uint32_t max_duty = 128;   // 2.5ms (180°)
    uint32_t duty = (uint32_t)(min_duty + (angle * (max_duty - min_duty) / 180.0));
    
    // Aplicar el duty cycle
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servo_channels[index], duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servo_channels[index]);
    
    printf("Servo %d movido a %.1f° (Duty: %" PRIu32 ")\n", servo_num, angle, duty);
}


void app_main() {
    // Inicializar todos los servos
    init_servos();
    
    // Secuencia de movimiento como en tu ejemplo
    while(1) {
        move_servo(1, 30);  // Servo 1 a 30°
        vTaskDelay(1000/ portTICK_PERIOD_MS);
        
        move_servo(2, 45);  // Servo 2 a 45°
        vTaskDelay(2000/ portTICK_PERIOD_MS);
        
        move_servo(3, 60);  // Servo 3 a 60°
        vTaskDelay(2000/ portTICK_PERIOD_MS);
        
        move_servo(4, 90);  // Servo 4 a 90°
        vTaskDelay(2000/ portTICK_PERIOD_MS);
        
        move_servo(5, 0);   // Servo 5 a 0°
        vTaskDelay(2000/ portTICK_PERIOD_MS);
        
        move_servo(6, 180); // Servo 6 a 180°
        vTaskDelay(2000/ portTICK_PERIOD_MS);
        
        move_servo(7, 90);  // Servo 7 a 90°
        vTaskDelay(2000/ portTICK_PERIOD_MS);
        
        // Opcional: volver a posición inicial
        for(int i = 1; i <= NUM_SERVOS; i++) {
            move_servo(i, 90);
        }
        vTaskDelay(2000/ portTICK_PERIOD_MS);
    }
}