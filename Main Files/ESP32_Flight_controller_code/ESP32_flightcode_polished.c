/**
 * @file main.c
 * @brief Deterministic ESP32‑based Quadcopter Flight Controller using FreeRTOS.
 *
 * Fixes applied in this revision:
 * - [PID FIX] Inverted Roll and Pitch mixer equations to properly counteract frame tilt.
 * - [WIFI FIX] Disabled Wi-Fi Power Save (WIFI_PS_NONE) to guarantee consistent UDP streams.
 * - [WIFI FIX] Added an event-driven auto-reconnection routing to handle boot noise drops.
 * - [ESC FIX] Postponed Wi-Fi activation until after a 3-second stable PWM idle window.
 * - [FILTER FIX] Added First-Order Low-Pass Filter on raw gyro rates.
 * - [I2C BUG] Reduced I2C timeout to 1ms to prevent PID loop freezing.
 * - [IBUS BUG] Added uart_flush on bad packets to prevent permanent desync.
 * @author: TROY FRANCO G. CELDRAN / Head Project Engineer & Lead Firmware Engineer
 * @date 2026‑05-25
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

/*─────────────────────────────────────────────────────────────────────────────*
 * CONFIGURATION
 *─────────────────────────────────────────────────────────────────────────────*/

#define WIFI_SSID  "WIFI_NAME"
#define WIFI_PASS  "WIFI_PSWRD"
#define PC_IP      "IP_ADDRESS"
#define UDP_PORT   YOUR_DESIRED_UDP_PORT

#define ESC1_PIN 13  ///< CCW (Front-Right)
#define ESC2_PIN 25  ///< CW  (Front-Left)  
#define ESC3_PIN 26  ///< CCW (Back-Left)   
#define ESC4_PIN 27  ///< CW  (Back-Right)

#define I2C_SCL 22
#define I2C_SDA 21
#define I2C_PORT I2C_NUM_0
#define I2C_HZ   400000

#define IBUS_RX_PIN   4
#define IBUS_UART_NUM UART_NUM_1

#define COPRO_TX_PIN   16   
#define COPRO_UART_NUM UART_NUM_2

#define LOOP_TIME_S 0.004f         
#define RAD_TO_DEG  57.2958f
#define I_MAX       50.0f          

#define WIFI_GOT_IP_BIT BIT0

static const char *TAG = "FlightCtrl";

/*─────────────────────────────────────────────────────────────────────────────*
 * DATA STRUCTURES
 *─────────────────────────────────────────────────────────────────────────────*/

typedef struct {
    uint16_t roll, pitch, throttle, yaw;
    uint32_t last_packet_ticks;
} RC_Input_t;

typedef struct {
    float roll_angle, pitch_angle;
    float gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate;
    float roll_offset, pitch_offset;
    float gyro_roll_offset, gyro_pitch_offset, gyro_yaw_offset;
} IMU_Data_t;

typedef struct {
    float Kp, Ki, Kd;
    float prev_err;
    float integral;
} PID_t;

typedef enum {
    STATE_BOOTING = 0,
    STATE_CALIBRATING,
    STATE_DISARMED,
    STATE_ARMED,
    STATE_FAILSAFE
} FlightState_t;

/*─────────────────────────────────────────────────────────────────────────────*
 * GLOBALS
 *─────────────────────────────────────────────────────────────────────────────*/
RC_Input_t    rc    = {1500, 1500, 1000, 1500, 0};
IMU_Data_t    imu   = {0};
PID_t pid_r = {1.5, 0.0, 0.02, 0, 0};
PID_t pid_p = {1.5, 0.0, 0.02, 0, 0};
PID_t pid_y = {2.0, 0.0, 0.03, 0, 0};
FlightState_t state = STATE_BOOTING;

static portMUX_TYPE telemetry_mux = portMUX_INITIALIZER_UNLOCKED;
static EventGroupHandle_t wifi_events;

/*─────────────────────────────────────────────────────────────────────────────*
 * PID COMPUTATION
 *─────────────────────────────────────────────────────────────────────────────*/

static float pid_compute(PID_t *p, float error) {
    float P = p->Kp * error;
    p->integral += error * LOOP_TIME_S;
    if (p->integral >  I_MAX) p->integral =  I_MAX;
    if (p->integral < -I_MAX) p->integral = -I_MAX;
    float I = p->Ki * p->integral;
    float D = p->Kd * (error - p->prev_err) / LOOP_TIME_S;
    p->prev_err = error;
    return P + I + D;
}

static inline void pid_reset_all(void){
    pid_r.integral = pid_p.integral = pid_y.integral = 0;
}

/*─────────────────────────────────────────────────────────────────────────────*
 * HARDWARE INITIALIZATION
 *─────────────────────────────────────────────────────────────────────────────*/

static void pwm_init(void){
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz         = 50,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);
    const int pins[4] = {ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN};
    for (int i = 0; i < 4; i++){
        ledc_channel_config_t c = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = i,
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = pins[i],
            .duty       = 409,
            .hpoint     = 0
        };
        ledc_channel_config(&c);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
    }
}

static void set_throttle(ledc_channel_t ch, int us){
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    uint32_t duty = (us * 8192) / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void i2c_init(void){
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA,
        .scl_io_num       = I2C_SCL,
        .sda_pullup_en    = 1,
        .scl_pullup_en    = 1,
        .master.clk_speed = I2C_HZ
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    uint8_t wake[2]     = {0x6B, 0x00};
    uint8_t gyro_cfg[2] = {0x1B, 0x08};
    i2c_master_write_to_device(I2C_PORT, 0x68, wake,     2, pdMS_TO_TICKS(100));
    i2c_master_write_to_device(I2C_PORT, 0x68, gyro_cfg, 2, pdMS_TO_TICKS(100));
}

/* Updated Event Handler to support aggressive auto-reconnects */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data){
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START){
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED){
        ESP_LOGW(TAG, "Wi-Fi connection dropped. Retrying...");
        xEventGroupClearBits(wifi_events, WIFI_GOT_IP_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP){
        ESP_LOGI(TAG, "Wi-Fi IP acquired successfully");
        xEventGroupSetBits(wifi_events, WIFI_GOT_IP_BIT);
    }
}

static void wifi_init_sta(void){
    wifi_events = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t w = {0};
    strcpy((char*)w.sta.ssid,     WIFI_SSID);
    strcpy((char*)w.sta.password, WIFI_PASS);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &w);
    esp_wifi_start();

    /* [CRITICAL TELEMETRY FIX]: Turn off Wi-Fi power saving.
     * Without this, the ESP32 sleeps periodically, causing massive UDP packet loss 
     * and causing netcat/wireshark to see absolutely nothing. */
    esp_wifi_set_ps(WIFI_PS_NONE);
}

static void copro_uart_init(void){
    uart_config_t c = {9600, UART_DATA_8_BITS, UART_PARITY_DISABLE,
                    UART_STOP_BITS_1, UART_HW_FLOWCTRL_DISABLE, 0};
    uart_param_config(COPRO_UART_NUM, &c);
    uart_set_pin(COPRO_UART_NUM, COPRO_TX_PIN, 17, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(COPRO_UART_NUM, 256, 0, 0, NULL, 0);
}

/*─────────────────────────────────────────────────────────────────────────────*
 * TASKS
 *─────────────────────────────────────────────────────────────────────────────*/

static void task_udp(void *a){
    xEventGroupWaitBits(wifi_events, WIFI_GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "UDP socket thread active");

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(UDP_PORT)
    };
    dest.sin_addr.s_addr = inet_addr(PC_IP);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0){
        ESP_LOGE(TAG, "UDP Socket error");
        vTaskDelete(NULL);
        return;
    }

    char buf[128];
    for(;;){
        float roll_snap, pitch_snap;
        int   state_snap, throttle_snap;

        taskENTER_CRITICAL(&telemetry_mux);
        roll_snap     = imu.roll_angle;
        pitch_snap    = imu.pitch_angle;
        state_snap    = (int)state;
        throttle_snap = rc.throttle;
        taskEXIT_CRITICAL(&telemetry_mux);

        snprintf(buf, sizeof buf, "%lu,%d,%.2f,%.2f,%d\n",
                 (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                state_snap, roll_snap, pitch_snap, throttle_snap);
                
        sendto(sock, buf, strlen(buf), 0, (struct sockaddr*)&dest, sizeof dest);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void task_ibus(void *a){
    uart_config_t c = {115200, UART_DATA_8_BITS, UART_PARITY_DISABLE,
                    UART_STOP_BITS_1, UART_HW_FLOWCTRL_DISABLE, 0};
    uart_param_config(IBUS_UART_NUM, &c);
    uart_set_pin(IBUS_UART_NUM, UART_PIN_NO_CHANGE, IBUS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(IBUS_UART_NUM, 1024, 0, 0, NULL, 0);

    uint8_t pkt[32];
    for(;;){
        int len = uart_read_bytes(IBUS_UART_NUM, pkt, 32, pdMS_TO_TICKS(20));
        if (len == 32 && pkt[0] == 0x20 && pkt[1] == 0x40){
            uint16_t chk = 0xFFFF;
            for (int i = 0; i < 30; i++) chk -= pkt[i];
            if (chk == (pkt[30] | (pkt[31] << 8))){
                taskENTER_CRITICAL(&telemetry_mux);
                rc.roll              = pkt[2] | (pkt[3] << 8);
                rc.pitch             = pkt[4] | (pkt[5] << 8);
                rc.throttle          = pkt[6] | (pkt[7] << 8);
                rc.yaw               = pkt[8] | (pkt[9] << 8);
                rc.last_packet_ticks = xTaskGetTickCount();
                taskEXIT_CRITICAL(&telemetry_mux);
            } else {
                uart_flush(IBUS_UART_NUM); // [IBUS FIX] Flush on bad checksum to resync
            }
        } else if (len > 0) {
            uart_flush(IBUS_UART_NUM); // [IBUS FIX] Flush garbage data so it instantly finds the next header
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void task_flight(void *a){
    uint8_t raw[14];
    TickType_t last         = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(4);

    float sx=0, sy=0, sgx=0, sgy=0, sgz=0;
    uint16_t sample = 0;
    
    // --- LOW PASS FILTER VARIABLES ---
    static float lpf_grr = 0.0f;
    static float lpf_gpr = 0.0f;
    static float lpf_gyr = 0.0f;
    const float LPF_ALPHA = 0.15f; 

    for(;;){
        if ((state == STATE_ARMED || state == STATE_DISARMED) &&
            (xTaskGetTickCount() - rc.last_packet_ticks) * portTICK_PERIOD_MS > 500){
            ESP_LOGE(TAG, "Signal lost -> FAILSAFE");
            taskENTER_CRITICAL(&telemetry_mux);
            state = STATE_FAILSAFE;
            taskEXIT_CRITICAL(&telemetry_mux);
        }

        // [I2C BUG FIX]: Timeout reduced to 1ms to prevent PID loop freezing
        if (i2c_master_write_read_device(I2C_PORT, 0x68, (uint8_t[]){0x3B}, 1, raw, 14, pdMS_TO_TICKS(1)) == ESP_OK) {
            int16_t ax = (raw[0]<<8)|raw[1],  ay = (raw[2]<<8)|raw[3],  az = (raw[4]<<8)|raw[5];
            int16_t gx = (raw[8]<<8)|raw[9],  gy = (raw[10]<<8)|raw[11], gz = (raw[12]<<8)|raw[13];

            // Finite State Machine (FSM) Implementation 
            switch(state){
            case STATE_BOOTING:
                for (int i = 0; i < 4; i++) set_throttle(i, 1000);
                if (xTaskGetTickCount() > pdMS_TO_TICKS(3000)){
                    ESP_LOGI(TAG, "Beginning calibration...");
                    taskENTER_CRITICAL(&telemetry_mux);
                    state = STATE_CALIBRATING;
                    taskEXIT_CRITICAL(&telemetry_mux);
                }
                break;

            case STATE_CALIBRATING:
                sx  += atan2f(ay, az) * RAD_TO_DEG;
                sy  += atan2f(-ax, hypotf(ay, az)) * RAD_TO_DEG;
                sgx += gx / 65.5f;
                sgy += gy / 65.5f;
                sgz += gz / 65.5f;
                if (++sample >= 500){
                    taskENTER_CRITICAL(&telemetry_mux);
                    imu.roll_offset       = sx  / 500;
                    imu.pitch_offset      = sy  / 500;
                    imu.gyro_roll_offset  = sgx / 500;
                    imu.gyro_pitch_offset = sgy / 500;
                    imu.gyro_yaw_offset   = sgz / 500;
                    imu.roll_angle  = atan2f(ay, az) * RAD_TO_DEG - imu.roll_offset;
                    imu.pitch_angle = atan2f(-ax, hypotf(ay, az)) * RAD_TO_DEG - imu.pitch_offset;
                    state = STATE_DISARMED;
                    taskEXIT_CRITICAL(&telemetry_mux);
                    ESP_LOGI(TAG, "Calibration complete -> DISARMED");
                }
                break;

            case STATE_DISARMED:
            case STATE_ARMED:{
                taskENTER_CRITICAL(&telemetry_mux);
                float gr_off = imu.gyro_roll_offset;
                float gp_off = imu.gyro_pitch_offset;
                float gy_off = imu.gyro_yaw_offset;
                float r_off  = imu.roll_offset;
                float p_off  = imu.pitch_offset;
                float r_prev = imu.roll_angle;
                float p_prev = imu.pitch_angle;
                taskEXIT_CRITICAL(&telemetry_mux);

                // 1. Get raw rates
                float raw_grr = (gx / 65.5f) - gr_off;
                float raw_gpr = (gy / 65.5f) - gp_off;
                float raw_gyr = (gz / 65.5f) - gy_off;

                // 2. Apply Low-Pass Filter
                lpf_grr = (LPF_ALPHA * raw_grr) + ((1.0f - LPF_ALPHA) * lpf_grr);
                lpf_gpr = (LPF_ALPHA * raw_gpr) + ((1.0f - LPF_ALPHA) * lpf_gpr);
                lpf_gyr = (LPF_ALPHA * raw_gyr) + ((1.0f - LPF_ALPHA) * lpf_gyr);

                // 3. Compute angles using filtered rates
                float ar = atan2f(ay, az) * RAD_TO_DEG - r_off;
                float ap = atan2f(-ax, hypotf(ay, az)) * RAD_TO_DEG - p_off;
                float new_roll  = 0.98f*(r_prev + lpf_grr*LOOP_TIME_S) + 0.02f*ar;
                float new_pitch = 0.98f*(p_prev + lpf_gpr*LOOP_TIME_S) + 0.02f*ap;

                taskENTER_CRITICAL(&telemetry_mux);
                imu.gyro_roll_rate  = lpf_grr; // Update telemetry to show smooth data
                imu.gyro_pitch_rate = lpf_gpr;
                imu.gyro_yaw_rate   = lpf_gyr;
                imu.roll_angle      = new_roll;
                imu.pitch_angle     = new_pitch;
                taskEXIT_CRITICAL(&telemetry_mux);

                if (state == STATE_DISARMED){
                    for (int i = 0; i < 4; i++) set_throttle(i, 1000);
                    if (rc.throttle < 1050 && rc.yaw > 1900){
                        static uint16_t c = 0;
                        if (++c >= 125){
                            pid_reset_all();
                            taskENTER_CRITICAL(&telemetry_mux);
                            state = STATE_ARMED;
                            taskEXIT_CRITICAL(&telemetry_mux);
                            ESP_LOGW(TAG, "ARMED");
                            c = 0;
                        }
                    }
                } else {
                    float er = ((rc.roll  - 1500) * 0.06f) + new_roll;
                    float ep = ((rc.pitch - 1500) * 0.06f) + new_pitch; 
                    float ey = ((rc.yaw   - 1500) * 0.15f) - lpf_gyr; // Use filtered yaw rate
                    
                    float pr = pid_compute(&pid_r, er);
                    float pp = pid_compute(&pid_p, ep);
                    float py = pid_compute(&pid_y, ey);

                    if (rc.throttle > 1050){
                        /* [CRITICAL PID DIRECTION FIX]:
                         * Inverted the algebraic mixing operators for 'pp' and 'pr' 
                         * to guarantee the control loop drives motors to counteract 
                         * the detected physical pitch and roll offsets. */
                        set_throttle(0, rc.throttle + pp - pr + py); // ESC1: Front-Right (CCW)
                        set_throttle(1, rc.throttle + pp + pr - py); // ESC2: Front-Left  (CW)
                        set_throttle(2, rc.throttle - pp + pr + py); // ESC3: Back-Left   (CCW)
                        set_throttle(3, rc.throttle - pp - pr - py); // ESC4: Back-Right  (CW)
                    } else {
                        for (int i = 0; i < 4; i++) set_throttle(i, 1000);
                        pid_reset_all();
                    }
                    
                    if (rc.throttle < 1050 && rc.yaw < 1100){
                        static uint16_t c = 0;
                        if (++c >= 125){
                            taskENTER_CRITICAL(&telemetry_mux);
                            state = STATE_DISARMED;
                            taskEXIT_CRITICAL(&telemetry_mux);
                            ESP_LOGI(TAG, "DISARMED");
                            c = 0;
                        }
                    }
                }} break;

            case STATE_FAILSAFE:
                for (int i = 0; i < 4; i++) set_throttle(i, 1000);
                if ((xTaskGetTickCount() - rc.last_packet_ticks) * portTICK_PERIOD_MS < 300){
                    taskENTER_CRITICAL(&telemetry_mux);
                    state = STATE_DISARMED;
                    taskEXIT_CRITICAL(&telemetry_mux);
                    ESP_LOGI(TAG, "Signal recovered -> DISARMED");
                }
                break;
            }
        } // End of I2C check

        uint8_t s = (uint8_t)state;
        uart_write_bytes(COPRO_UART_NUM, (char*)&s, 1);
        vTaskDelayUntil(&last, period);
    }
}

/*─────────────────────────────────────────────────────────────────────────────*
 * MAIN
 *─────────────────────────────────────────────────────────────────────────────*/
void app_main(void){
    nvs_flash_init();
    ESP_LOGI(TAG, "Booting flight controller");
    
    // 1. Initialize PWM first so a stable idle train starts immediately
    pwm_init();
    
    // 2. [ESC FIX] Wait 3 seconds in absolute radio silence so ESC4 (GPIO27) boots flawlessly
    vTaskDelay(pdMS_TO_TICKS(3000));   
    
    // 3. Fire up the remaining subsystems safely
    wifi_init_sta();
    copro_uart_init();
    i2c_init();

    xTaskCreatePinnedToCore(task_ibus,   "RX",     4096, NULL,  5, NULL, 1);
    xTaskCreatePinnedToCore(task_flight, "Flight", 6144, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(task_udp,    "UDP",    4096, NULL,  2, NULL, 0);
}
