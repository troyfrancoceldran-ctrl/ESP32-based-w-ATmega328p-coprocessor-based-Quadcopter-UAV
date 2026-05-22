/**
 * @file main.c
 * @brief Deterministic ESP32‑based Quadcopter Flight Controller using FreeRTOS.
 *
 * This firmware executes a hard‑real‑time 250 Hz flight loop that reads IMU data,
 * runs sensor fusion, evaluates PID control laws, and drives four ESCs through
 * hardware PWM. A strict Finite‑State Machine (FSM) ensures safe booting,
 * arming/disarming, and automatic failsafe recovery.
 *
 * Architecture overview:
 * - Core 1: deterministic flight dynamics + radio parsing
 * - Core 0: UDP telemetry and Wi‑Fi stack
 *
 * Fixes applied:
 *  - [BUG FIX] ESC2 remapped from GPIO12 (strapping pin / MTDI) to GPIO25
 *    to prevent boot-mode selection conflicts on power-up.
 *  - [BUG FIX] Shared globals (imu, state, rc) protected with portMUX_TYPE
 *    spinlock to prevent cross-core race conditions between task_flight
 *    (Core 1) and task_udp (Core 0).
 *  - [BUG FIX] task_udp now waits for the IP_EVENT_STA_GOT_IP event via an
 *    EventGroup before opening the UDP socket, preventing silent sendto()
 *    failures on slow or congested Wi-Fi networks. Socket creation is also
 *    guarded against failure.
 *
 * @authors
 * Troy Franco G. Celdran (et al.)
 * @date 2026‑04‑29
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

/** Wi‑Fi credentials and telemetry target */
#define WIFI_SSID  "3rdF"
#define WIFI_PASS  "HabilisEE-3rd"
#define PC_IP      "192.168.68.106"
#define UDP_PORT   4444

/**
 * ESC signal pins (50 Hz servo‑style PWM).
 * GPIO12 (MTDI strapping pin) was replaced with GPIO25 for ESC2.
 * GPIO25 is a safe output-capable pin with no boot-mode side effects.
 */
#define ESC1_PIN 13  ///< CCW (Front-Right)
#define ESC2_PIN 25  ///< CW  (Front-Left)  — was GPIO12 (strapping pin, FIXED)
#define ESC3_PIN 26  ///< CCW (Back-Left)
#define ESC4_PIN 27  ///< CW  (Back-Right)

/** I²C connections to IMU (GY‑521 / MPU6050) */
#define I2C_SCL 22
#define I2C_SDA 21
#define I2C_PORT I2C_NUM_0
#define I2C_HZ   400000

/** FlySky receiver and coprocessor serial ports */
#define IBUS_RX_PIN 4
#define IBUS_UART_NUM UART_NUM_1
#define COPRO_TX_PIN 17
#define COPRO_UART_NUM UART_NUM_2

/** Timing constants */
#define LOOP_TIME_S 0.004f         ///< Loop period (4 ms = 250 Hz)
#define RAD_TO_DEG 57.2958f
#define I_MAX 50.0f                ///< Integral‑term saturation limit

/** EventGroup bit signalling that Wi-Fi has an IP address */
#define WIFI_GOT_IP_BIT BIT0

static const char *TAG = "FlightCtrl";

/*─────────────────────────────────────────────────────────────────────────────*
 * DATA STRUCTURES
 *─────────────────────────────────────────────────────────────────────────────*/

/** @brief Aggregated i‑BUS control channels */
typedef struct {
    uint16_t roll, pitch, throttle, yaw;
    uint32_t last_packet_ticks;
} RC_Input_t;

/** @brief Pre‑filtered IMU data (attitude + biases) */
typedef struct {
    float roll_angle, pitch_angle;
    float gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate;
    float roll_offset, pitch_offset;
    float gyro_roll_offset, gyro_pitch_offset, gyro_yaw_offset;
} IMU_Data_t;

/** @brief PID tuning constants and memory for one axis */
typedef struct {
    float Kp, Ki, Kd;
    float prev_err;
    float integral;
} PID_t;

/** @brief Flight controller state machine enumeration */
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
RC_Input_t    rc    = {1500,1500,1000,1500,0};
IMU_Data_t    imu   = {0};
PID_t pid_r = {0.5,0.0,0.02,0,0}, pid_p = {0.5,0.0,0.02,0,0}, pid_y = {1.0,0.0,0.03,0,0};
FlightState_t state = STATE_BOOTING;

/**
 * Spinlock protecting imu, state, and rc from cross-core data races.
 * task_flight (Core 1) writes; task_udp (Core 0) reads.
 */
static portMUX_TYPE telemetry_mux = portMUX_INITIALIZER_UNLOCKED;

/** EventGroup used to signal Wi-Fi IP acquisition to task_udp. */
static EventGroupHandle_t wifi_events;

/*─────────────────────────────────────────────────────────────────────────────*
 * PID COMPUTATION
 *─────────────────────────────────────────────────────────────────────────────*/

/**
 * @brief Computes PID output for one control axis.
 * @param p Pointer to PID struct.
 * @param error Current error signal.
 * @return Control effort.
 */
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

/** Resets integrators when disarming to avoid jump during next arming. */
static inline void pid_reset_all(void){
    pid_r.integral = pid_p.integral = pid_y.integral = 0;
}

/*─────────────────────────────────────────────────────────────────────────────*
 * HARDWARE INITIALIZATION
 *─────────────────────────────────────────────────────────────────────────────*/

/**
 * @brief Initializes four LEDC PWM outputs at 50 Hz for ESCs.
 */
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
    }
}

/**
 * @brief Writes microsecond‑scaled throttle to an ESC channel.
 */
static void set_throttle(ledc_channel_t ch, int us){
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    uint32_t duty = (us * 8192) / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/**
 * @brief Initializes I²C and wakes IMU (MPU6050 in ±500 °/s range).
 */
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
    i2c_master_write_to_device(I2C_PORT, 0x68, wake,     2, 100/portTICK_PERIOD_MS);
    i2c_master_write_to_device(I2C_PORT, 0x68, gyro_cfg, 2, 100/portTICK_PERIOD_MS);
}

/**
 * @brief Wi-Fi event handler — sets WIFI_GOT_IP_BIT when an IP is assigned.
 */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data){
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP){
        ESP_LOGI(TAG, "Wi-Fi IP acquired");
        xEventGroupSetBits(wifi_events, WIFI_GOT_IP_BIT);
    }
}

/** Initializes minimal Wi‑Fi STA for UDP telemetry. */
static void wifi_init_sta(void){
    wifi_events = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    /* Register handler BEFORE starting Wi-Fi so no event is missed */
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                            wifi_event_handler, NULL);

    wifi_config_t w = {0};
    strcpy((char*)w.sta.ssid,     WIFI_SSID);
    strcpy((char*)w.sta.password, WIFI_PASS);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &w);
    esp_wifi_start();
    esp_wifi_connect();
}

/** Initializes dedicated UART2 link to ATmega coprocessor. */
static void copro_uart_init(void){
    uart_config_t c = {9600, UART_DATA_8_BITS, UART_PARITY_DISABLE,
                    UART_STOP_BITS_1, UART_HW_FLOWCTRL_DISABLE, 0};
    uart_param_config(COPRO_UART_NUM, &c);
    uart_set_pin(COPRO_UART_NUM, COPRO_TX_PIN,
                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(COPRO_UART_NUM, 256, 0, 0, NULL, 0);
}

/*─────────────────────────────────────────────────────────────────────────────*
 * TASKS
 *─────────────────────────────────────────────────────────────────────────────*/

/**
 * @brief Periodic UDP telemetry (10 Hz) — runs on Core 0.
 *
 * Blocks on WIFI_GOT_IP_BIT before opening the socket, ensuring the network
 * stack is fully ready. Guards against socket() returning -1.
 * Reads shared globals under telemetry_mux to avoid data races with Core 1.
 */
static void task_udp(void *a){
    /* Wait indefinitely until Wi-Fi has an IP */
    xEventGroupWaitBits(wifi_events, WIFI_GOT_IP_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "UDP task starting");

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(UDP_PORT)
    };
    dest.sin_addr.s_addr = inet_addr(PC_IP);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0){
        ESP_LOGE(TAG, "UDP socket creation failed — telemetry disabled");
        vTaskDelete(NULL);
        return;
    }

    char buf[128];
    for(;;){
        float roll_snap, pitch_snap;
        int   state_snap, throttle_snap;

        /* Atomic snapshot of shared data written by Core 1 */
        taskENTER_CRITICAL(&telemetry_mux);
        roll_snap     = imu.roll_angle;
        pitch_snap    = imu.pitch_angle;
        state_snap    = (int)state;
        throttle_snap = rc.throttle;
        taskEXIT_CRITICAL(&telemetry_mux);

        snprintf(buf, sizeof buf, "%lu,%d,%.2f,%.2f,%d\n",
                 (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                state_snap, roll_snap, pitch_snap, throttle_snap);
        sendto(sock, buf, strlen(buf), 0,
            (struct sockaddr*)&dest, sizeof dest);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Parses i‑BUS packets (FlySky receiver) — runs on Core 1.
 */
static void task_ibus(void *a){
    uart_config_t c = {115200, UART_DATA_8_BITS, UART_PARITY_DISABLE,
                    UART_STOP_BITS_1, UART_HW_FLOWCTRL_DISABLE, 0};
    uart_param_config(IBUS_UART_NUM, &c);
    uart_set_pin(IBUS_UART_NUM, UART_PIN_NO_CHANGE, IBUS_RX_PIN,
                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(IBUS_UART_NUM, 1024, 0, 0, NULL, 0);

    uint8_t pkt[32];
    for(;;){
        int len = uart_read_bytes(IBUS_UART_NUM, pkt, 32, pdMS_TO_TICKS(20));
        if (len == 32 && pkt[0] == 0x20 && pkt[1] == 0x40){
            uint16_t chk = 0xFFFF;
            for (int i = 0; i < 30; i++) chk -= pkt[i];
            if (chk == (pkt[30] | (pkt[31] << 8))){
                taskENTER_CRITICAL(&telemetry_mux);
                rc.roll             = pkt[2] | (pkt[3] << 8);
                rc.pitch            = pkt[4] | (pkt[5] << 8);
                rc.throttle         = pkt[6] | (pkt[7] << 8);
                rc.yaw              = pkt[8] | (pkt[9] << 8);
                rc.last_packet_ticks = xTaskGetTickCount();
                taskEXIT_CRITICAL(&telemetry_mux);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Core flight‑control logic @ 250 Hz — runs on Core 1.
 *
 * Implements:
 * - IMU burst read
 * - Complementary filter
 * - FSM handling (Boot → Calibrate → Disarm/Arm/Failsafe)
 * - Motor mixing
 * - Safety constraints
 *
 * Shared globals written here are protected by telemetry_mux so task_udp
 * on Core 0 always reads a consistent snapshot.
 */
static void task_flight(void *a){
    uint8_t raw[14];
    TickType_t last        = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(4);

    float sx=0,sy=0,sgx=0,sgy=0,sgz=0;
    uint16_t sample=0;

    for(;;){
        /* FAILSAFE MONITOR --------------------------------------------------*/
        if ((state == STATE_ARMED || state == STATE_DISARMED) &&
            (xTaskGetTickCount() - rc.last_packet_ticks) * portTICK_PERIOD_MS > 500){
            ESP_LOGE(TAG, "Signal lost -> FAILSAFE");
            taskENTER_CRITICAL(&telemetry_mux);
            state = STATE_FAILSAFE;
            taskEXIT_CRITICAL(&telemetry_mux);
        }

        /* IMU SENSOR BURST --------------------------------------------------*/
        i2c_master_write_read_device(I2C_PORT, 0x68,
                                    (uint8_t[]){0x3B}, 1, raw, 14, 100);
        int16_t ax = (raw[0]<<8)|raw[1],  ay = (raw[2]<<8)|raw[3],
                az = (raw[4]<<8)|raw[5];
        int16_t gx = (raw[8]<<8)|raw[9],  gy = (raw[10]<<8)|raw[11],
                gz = (raw[12]<<8)|raw[13];

        switch(state){
        /*──────────────────────── BOOTING ───────────────────────*/
        case STATE_BOOTING:
            for (int i = 0; i < 4; i++) set_throttle(i, 1000);
            if (xTaskGetTickCount() > pdMS_TO_TICKS(3000)){
                ESP_LOGI(TAG, "Beginning calibration…");
                taskENTER_CRITICAL(&telemetry_mux);
                state = STATE_CALIBRATING;
                taskEXIT_CRITICAL(&telemetry_mux);
            }
            break;

        /*──────────────────────── CALIBRATION ───────────────────*/
        case STATE_CALIBRATING:
            sx  += atan2f(ay, az) * RAD_TO_DEG;
            sy  += atan2f(-ax, hypotf(ay, az)) * RAD_TO_DEG;
            sgx += gx / 65.5f;
            sgy += gy / 65.5f;
            sgz += gz / 65.5f;
            if (++sample >= 500){
                taskENTER_CRITICAL(&telemetry_mux);
                imu.roll_offset        = sx  / 500;
                imu.pitch_offset       = sy  / 500;
                imu.gyro_roll_offset   = sgx / 500;
                imu.gyro_pitch_offset  = sgy / 500;
                imu.gyro_yaw_offset    = sgz / 500;
                imu.roll_angle  = atan2f(ay,az)*RAD_TO_DEG - imu.roll_offset;
                imu.pitch_angle = atan2f(-ax,hypotf(ay,az))*RAD_TO_DEG - imu.pitch_offset;
                state = STATE_DISARMED;
                taskEXIT_CRITICAL(&telemetry_mux);
                ESP_LOGI(TAG, "Calibration complete → DISARMED");
            }
            break;

        /*──────────────────────── DISARM / ARM ───────────────────*/
        case STATE_DISARMED:
        case STATE_ARMED:{
            /* Sensor fusion — read offsets under lock for consistency */
            taskENTER_CRITICAL(&telemetry_mux);
            float gr_off = imu.gyro_roll_offset;
            float gp_off = imu.gyro_pitch_offset;
            float gy_off = imu.gyro_yaw_offset;
            float r_off  = imu.roll_offset;
            float p_off  = imu.pitch_offset;
            float r_prev = imu.roll_angle;
            float p_prev = imu.pitch_angle;
            taskEXIT_CRITICAL(&telemetry_mux);

            float grr = (gx / 65.5f) - gr_off;
            float gpr = (gy / 65.5f) - gp_off;
            float gyr = (gz / 65.5f) - gy_off;

            float ar = atan2f(ay, az) * RAD_TO_DEG - r_off;
            float ap = atan2f(-ax, hypotf(ay, az)) * RAD_TO_DEG - p_off;
            float new_roll  = 0.98f*(r_prev + grr*LOOP_TIME_S) + 0.02f*ar;
            float new_pitch = 0.98f*(p_prev + gpr*LOOP_TIME_S) + 0.02f*ap;

            taskENTER_CRITICAL(&telemetry_mux);
            imu.gyro_roll_rate  = grr;
            imu.gyro_pitch_rate = gpr;
            imu.gyro_yaw_rate   = gyr;
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
                float er = ((rc.roll  - 1500) * 0.06f) - new_roll;
                float ep = ((rc.pitch - 1500) * 0.06f) - new_pitch;
                float ey = ((rc.yaw   - 1500) * 0.15f) - gyr;
                float pr = pid_compute(&pid_r, er);
                float pp = pid_compute(&pid_p, ep);
                float py = pid_compute(&pid_y, ey);

                if (rc.throttle > 1050){
                    set_throttle(0, rc.throttle - pp + pr + py);
                    set_throttle(1, rc.throttle - pp - pr - py);
                    set_throttle(2, rc.throttle + pp - pr + py);
                    set_throttle(3, rc.throttle + pp + pr - py);
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

        /*──────────────────────── FAILSAFE ───────────────────────*/
        case STATE_FAILSAFE:
            for (int i = 0; i < 4; i++) set_throttle(i, 1000);
            if ((xTaskGetTickCount() - rc.last_packet_ticks) * portTICK_PERIOD_MS < 300){
                taskENTER_CRITICAL(&telemetry_mux);
                state = STATE_DISARMED;
                taskEXIT_CRITICAL(&telemetry_mux);
                ESP_LOGI(TAG, "Signal recovered → DISARMED");
            }
            break;
        }

        /* Notify coprocessor of state (1 byte) */
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
    pwm_init();
    wifi_init_sta();
    copro_uart_init();
    vTaskDelay(pdMS_TO_TICKS(3000));   // Allow ESC signal lines to settle
    i2c_init();

    xTaskCreatePinnedToCore(task_ibus,   "RX",     4096, NULL,  5, NULL, 1);
    xTaskCreatePinnedToCore(task_flight, "Flight", 6144, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(task_udp,    "UDP",    4096, NULL,  2, NULL, 0);
}
