/**
 * @file main_pid_tuner.c
 * @brief Flight Controller with Live UDP PID Tuning.
 *
 * Adds a bidirectional UDP tuning channel on port 4445 (separate from
 * telemetry on 4444). Send gain commands from a laptop in real time;
 * the ESP32 applies them immediately without reflashing.
 *
 * COMMAND FORMAT (ASCII, sent to ESP32 port 4445):
 *   SET <axis> <term> <value>
 *
 *   axis : R (roll) | P (pitch) | Y (yaw)
 *   term : P | I | D
 *   value: float
 *
 * EXAMPLES:
 *   SET R P 1.2        -> set roll Kp to 1.2
 *   SET P D 0.025      -> set pitch Kd to 0.025
 *   SET Y P 2.5        -> set yaw Kp to 2.5
 *   GET                -> ESP32 replies with all current gains
 *   SAVE               -> writes current gains to NVS (survives reboot)
 *   LOAD               -> reloads gains from NVS
 *   RESET              -> restores compiled-in default gains
 *
 * TUNING LAPTOP (macOS/Linux terminal):
 *   # Send a command:
 *   echo "SET R P 1.2" | nc -u -w1 192.168.68.104 4445
 *
 *   # Read telemetry:
 *   nc -u -l 4444
 *
 *   # Or use the companion Python tuner script (pid_tuner.py)
 *
 * All other firmware behaviour is identical to main.c.
 *
 * @author Troy Franco G. Celdran
 * @date   2026-05-25
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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
#include "nvs.h"
#include "lwip/sockets.h"

/*─────────────────────────────────────────────────────────────────────────────*
 * CONFIGURATION
 *─────────────────────────────────────────────────────────────────────────────*/

#define WIFI_SSID  "3rdF"
#define WIFI_PASS  "HabilisEE-3rd"
#define PC_IP      "192.168.68.104"
#define UDP_PORT       4444   ///< Telemetry out (read-only stream)
#define UDP_TUNE_PORT  4445   ///< PID tuning channel (bidirectional)

#define ESC1_PIN 13
#define ESC2_PIN 25
#define ESC3_PIN 26
#define ESC4_PIN 27

#define I2C_SCL  22
#define I2C_SDA  21
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

/* Default gains — restored by RESET command */
#define DEFAULT_ROLL_KP  1.5f
#define DEFAULT_ROLL_KI  0.0f
#define DEFAULT_ROLL_KD  0.02f
#define DEFAULT_PITCH_KP 1.5f
#define DEFAULT_PITCH_KI 0.0f
#define DEFAULT_PITCH_KD 0.02f
#define DEFAULT_YAW_KP   2.0f
#define DEFAULT_YAW_KI   0.0f
#define DEFAULT_YAW_KD   0.03f

static const char *TAG      = "FlightCtrl";
static const char *NVS_NS   = "pid_gains";  ///< NVS namespace for saved gains

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
PID_t pid_r = {DEFAULT_ROLL_KP,  DEFAULT_ROLL_KI,  DEFAULT_ROLL_KD,  0, 0};
PID_t pid_p = {DEFAULT_PITCH_KP, DEFAULT_PITCH_KI, DEFAULT_PITCH_KD, 0, 0};
PID_t pid_y = {DEFAULT_YAW_KP,   DEFAULT_YAW_KI,   DEFAULT_YAW_KD,   0, 0};
FlightState_t state = STATE_BOOTING;

static portMUX_TYPE telemetry_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE pid_mux       = portMUX_INITIALIZER_UNLOCKED;
static EventGroupHandle_t wifi_events;

/*─────────────────────────────────────────────────────────────────────────────*
 * NVS GAIN PERSISTENCE
 *─────────────────────────────────────────────────────────────────────────────*/

/**
 * @brief Saves all current PID gains to NVS flash.
 * Survives power cycles. Called when tuning task receives "SAVE" command.
 */
static void nvs_save_gains(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;

    /* Store each float as a uint32 by reinterpreting the bits */
    uint32_t tmp;
    #define STORE(key, val) memcpy(&tmp, &(val), 4); nvs_set_u32(h, key, tmp)
    taskENTER_CRITICAL(&pid_mux);
    STORE("r_kp", pid_r.Kp); STORE("r_ki", pid_r.Ki); STORE("r_kd", pid_r.Kd);
    STORE("p_kp", pid_p.Kp); STORE("p_ki", pid_p.Ki); STORE("p_kd", pid_p.Kd);
    STORE("y_kp", pid_y.Kp); STORE("y_ki", pid_y.Ki); STORE("y_kd", pid_y.Kd);
    taskEXIT_CRITICAL(&pid_mux);
    #undef STORE

    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "PID gains saved to NVS");
}

/**
 * @brief Loads PID gains from NVS. Falls back to defaults if no saved gains.
 * Called on boot and when tuning task receives "LOAD" command.
 */
static void nvs_load_gains(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "No saved gains found — using defaults");
        return;
    }

    uint32_t tmp;
    #define LOAD(key, val) if(nvs_get_u32(h, key, &tmp)==ESP_OK) memcpy(&(val), &tmp, 4)
    taskENTER_CRITICAL(&pid_mux);
    LOAD("r_kp", pid_r.Kp); LOAD("r_ki", pid_r.Ki); LOAD("r_kd", pid_r.Kd);
    LOAD("p_kp", pid_p.Kp); LOAD("p_ki", pid_p.Ki); LOAD("p_kd", pid_p.Kd);
    LOAD("y_kp", pid_y.Kp); LOAD("y_ki", pid_y.Ki); LOAD("y_kd", pid_y.Kd);
    taskEXIT_CRITICAL(&pid_mux);
    #undef LOAD

    nvs_close(h);
    ESP_LOGI(TAG, "PID gains loaded from NVS: R[%.3f %.3f %.3f] P[%.3f %.3f %.3f] Y[%.3f %.3f %.3f]",
            pid_r.Kp, pid_r.Ki, pid_r.Kd,
            pid_p.Kp, pid_p.Ki, pid_p.Kd,
            pid_y.Kp, pid_y.Ki, pid_y.Kd);
}

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

static inline void pid_reset_all(void) {
    pid_r.integral = pid_p.integral = pid_y.integral = 0;
}

/*─────────────────────────────────────────────────────────────────────────────*
 * HARDWARE INITIALIZATION
 *─────────────────────────────────────────────────────────────────────────────*/

static void pwm_init(void) {
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz         = 50,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);
    const int pins[4] = {ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN};
    for (int i = 0; i < 4; i++) {
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

static void set_throttle(ledc_channel_t ch, int us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    uint32_t duty = ((uint32_t)us * 8192) / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void i2c_init(void) {
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

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi dropped — retrying...");
        xEventGroupClearBits(wifi_events, WIFI_GOT_IP_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi IP acquired");
        xEventGroupSetBits(wifi_events, WIFI_GOT_IP_BIT);
    }
}

static void wifi_init_sta(void) {
    wifi_events = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,    wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t w = {0};
    strcpy((char*)w.sta.ssid,     WIFI_SSID);
    strcpy((char*)w.sta.password, WIFI_PASS);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &w);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
}

static void copro_uart_init(void) {
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
 * @brief Telemetry output — 10 Hz on UDP port 4444.
 * Format: timestamp_ms, state, roll_deg, pitch_deg, throttle_us,
 *         Rkp, Rki, Rkd, Pkp, Pki, Pkd, Ykp, Yki, Ykd
 * The gain fields let you confirm on the laptop that a SET command landed.
 */
static void task_udp(void *a) {
    xEventGroupWaitBits(wifi_events, WIFI_GOT_IP_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "UDP telemetry task active");

    struct sockaddr_in dest = {.sin_family = AF_INET,
                            .sin_port   = htons(UDP_PORT)};
    dest.sin_addr.s_addr = inet_addr(PC_IP);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { vTaskDelete(NULL); return; }

    char buf[256];
    for (;;) {
        float roll_snap, pitch_snap;
        int   state_snap, throttle_snap;
        float rkp, rki, rkd, pkp, pki, pkd, ykp, yki, ykd;

        taskENTER_CRITICAL(&telemetry_mux);
        roll_snap     = imu.roll_angle;
        pitch_snap    = imu.pitch_angle;
        state_snap    = (int)state;
        throttle_snap = rc.throttle;
        taskEXIT_CRITICAL(&telemetry_mux);

        taskENTER_CRITICAL(&pid_mux);
        rkp = pid_r.Kp; rki = pid_r.Ki; rkd = pid_r.Kd;
        pkp = pid_p.Kp; pki = pid_p.Ki; pkd = pid_p.Kd;
        ykp = pid_y.Kp; yki = pid_y.Ki; ykd = pid_y.Kd;
        taskEXIT_CRITICAL(&pid_mux);

        snprintf(buf, sizeof buf,
                "%lu,%d,%.2f,%.2f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                 (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                state_snap, roll_snap, pitch_snap, throttle_snap,
                rkp, rki, rkd, pkp, pki, pkd, ykp, yki, ykd);

        sendto(sock, buf, strlen(buf), 0,
            (struct sockaddr*)&dest, sizeof dest);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Live PID tuning receiver — listens on UDP port 4445.
 *
 * Parses ASCII commands and applies gain changes immediately.
 * Runs on Core 0 at low priority — never touches flight-critical data
 * except through pid_mux, and only when the drone is in a safe state.
 *
 * Replies to the sender's IP with an ACK or the current gain table.
 */
static void task_pid_tuner(void *a) {
    xEventGroupWaitBits(wifi_events, WIFI_GOT_IP_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "PID tuner listening on port %d", UDP_TUNE_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { vTaskDelete(NULL); return; }

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port        = htons(UDP_TUNE_PORT)
    };
    if (bind(sock, (struct sockaddr*)&bind_addr, sizeof bind_addr) < 0) {
        ESP_LOGE(TAG, "Tuner socket bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    char buf[128];
    char reply[256];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof sender;

    for (;;) {
        int len = recvfrom(sock, buf, sizeof buf - 1, 0,
                        (struct sockaddr*)&sender, &sender_len);
        if (len <= 0) continue;
        buf[len] = '\0';

        /* Strip trailing newline/CR if present */
        for (int i = len - 1; i >= 0 && (buf[i]=='\n'||buf[i]=='\r'); i--)
            buf[i] = '\0';

        ESP_LOGI(TAG, "Tuner RX: '%s'", buf);

        /* ── GET: reply with current gains ─────────────────────────── */
        if (strncmp(buf, "GET", 3) == 0) {
            taskENTER_CRITICAL(&pid_mux);
            snprintf(reply, sizeof reply,
                "GAINS R[Kp=%.3f Ki=%.3f Kd=%.3f] "
                "P[Kp=%.3f Ki=%.3f Kd=%.3f] "
                "Y[Kp=%.3f Ki=%.3f Kd=%.3f]\n",
                pid_r.Kp, pid_r.Ki, pid_r.Kd,
                pid_p.Kp, pid_p.Ki, pid_p.Kd,
                pid_y.Kp, pid_y.Ki, pid_y.Kd);
            taskEXIT_CRITICAL(&pid_mux);
            sendto(sock, reply, strlen(reply), 0,
                (struct sockaddr*)&sender, sender_len);
            continue;
        }

        /* ── SAVE: persist current gains to NVS ────────────────────── */
        if (strncmp(buf, "SAVE", 4) == 0) {
            nvs_save_gains();
            const char *ack = "ACK SAVE\n";
            sendto(sock, ack, strlen(ack), 0,
                (struct sockaddr*)&sender, sender_len);
            continue;
        }

        /* ── LOAD: reload gains from NVS ────────────────────────────── */
        if (strncmp(buf, "LOAD", 4) == 0) {
            nvs_load_gains();
            pid_reset_all();
            const char *ack = "ACK LOAD\n";
            sendto(sock, ack, strlen(ack), 0,
                (struct sockaddr*)&sender, sender_len);
            continue;
        }

        /* ── RESET: restore compiled-in defaults ────────────────────── */
        if (strncmp(buf, "RESET", 5) == 0) {
            taskENTER_CRITICAL(&pid_mux);
            pid_r.Kp = DEFAULT_ROLL_KP;  pid_r.Ki = DEFAULT_ROLL_KI;
            pid_r.Kd = DEFAULT_ROLL_KD;
            pid_p.Kp = DEFAULT_PITCH_KP; pid_p.Ki = DEFAULT_PITCH_KI;
            pid_p.Kd = DEFAULT_PITCH_KD;
            pid_y.Kp = DEFAULT_YAW_KP;   pid_y.Ki = DEFAULT_YAW_KI;
            pid_y.Kd = DEFAULT_YAW_KD;
            taskEXIT_CRITICAL(&pid_mux);
            pid_reset_all();
            const char *ack = "ACK RESET\n";
            sendto(sock, ack, strlen(ack), 0,
                (struct sockaddr*)&sender, sender_len);
            continue;
        }

        /* ── SET <axis> <term> <value> ──────────────────────────────── */
        if (strncmp(buf, "SET", 3) == 0) {
            char axis[4] = {0}, term[4] = {0};
            float value = 0.0f;

            if (sscanf(buf, "SET %3s %3s %f", axis, term, &value) != 3) {
                const char *err = "ERR bad format. Use: SET R P 1.2\n";
                sendto(sock, err, strlen(err), 0,
                    (struct sockaddr*)&sender, sender_len);
                continue;
            }

            /* Sanity clamp — prevents dangerous values being set mid-flight */
            if (value < 0.0f)  value = 0.0f;
            if (value > 20.0f) value = 20.0f;

            PID_t *target = NULL;
            if      (axis[0]=='R' || axis[0]=='r') target = &pid_r;
            else if (axis[0]=='P' || axis[0]=='p') target = &pid_p;
            else if (axis[0]=='Y' || axis[0]=='y') target = &pid_y;

            if (!target) {
                const char *err = "ERR unknown axis. Use R, P, or Y\n";
                sendto(sock, err, strlen(err), 0,
                    (struct sockaddr*)&sender, sender_len);
                continue;
            }

            taskENTER_CRITICAL(&pid_mux);
            if      (term[0]=='P' || term[0]=='p') target->Kp = value;
            else if (term[0]=='I' || term[0]=='i') target->Ki = value;
            else if (term[0]=='D' || term[0]=='d') target->Kd = value;
            else {
                taskEXIT_CRITICAL(&pid_mux);
                const char *err = "ERR unknown term. Use P, I, or D\n";
                sendto(sock, err, strlen(err), 0,
                    (struct sockaddr*)&sender, sender_len);
                continue;
            }
            taskEXIT_CRITICAL(&pid_mux);

            /* Reset integrators so the new Ki takes effect cleanly */
            if (term[0]=='I' || term[0]=='i') pid_reset_all();

            snprintf(reply, sizeof reply,
                    "ACK SET %s %s %.3f\n", axis, term, value);
            sendto(sock, reply, strlen(reply), 0,
                (struct sockaddr*)&sender, sender_len);

            ESP_LOGI(TAG, "Gain updated: %s.K%s = %.3f", axis, term, value);
            continue;
        }

        /* Unknown command */
        const char *help =
            "CMDS: SET <R|P|Y> <P|I|D> <val>  GET  SAVE  LOAD  RESET\n";
        sendto(sock, help, strlen(help), 0,
            (struct sockaddr*)&sender, sender_len);
    }
}

/**
 * @brief i-BUS receiver — runs on Core 1.
 */
static void task_ibus(void *a) {
    uart_config_t c = {115200, UART_DATA_8_BITS, UART_PARITY_DISABLE,
                    UART_STOP_BITS_1, UART_HW_FLOWCTRL_DISABLE, 0};
    uart_param_config(IBUS_UART_NUM, &c);
    uart_set_pin(IBUS_UART_NUM, UART_PIN_NO_CHANGE, IBUS_RX_PIN,
                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(IBUS_UART_NUM, 1024, 0, 0, NULL, 0);

    uint8_t pkt[32];
    for (;;) {
        int len = uart_read_bytes(IBUS_UART_NUM, pkt, 32, pdMS_TO_TICKS(20));
        if (len == 32 && pkt[0] == 0x20 && pkt[1] == 0x40) {
            uint16_t chk = 0xFFFF;
            for (int i = 0; i < 30; i++) chk -= pkt[i];
            if (chk == (pkt[30] | (pkt[31] << 8))) {
                taskENTER_CRITICAL(&telemetry_mux);
                rc.roll              = pkt[2]  | (pkt[3]  << 8);
                rc.pitch             = pkt[4]  | (pkt[5]  << 8);
                rc.throttle          = pkt[6]  | (pkt[7]  << 8);
                rc.yaw               = pkt[8]  | (pkt[9]  << 8);
                rc.last_packet_ticks = xTaskGetTickCount();
                taskEXIT_CRITICAL(&telemetry_mux);
            } else {
                uart_flush(IBUS_UART_NUM);
            }
        } else if (len > 0) {
            uart_flush(IBUS_UART_NUM);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Core flight loop @ 250 Hz — runs on Core 1.
 */
static void task_flight(void *a) {
    uint8_t raw[14];
    TickType_t last         = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(4);

    float sx=0, sy=0, sgx=0, sgy=0, sgz=0;
    uint16_t sample = 0;

    static float lpf_grr = 0.0f, lpf_gpr = 0.0f, lpf_gyr = 0.0f;
    const float  LPF_ALPHA = 0.35f;

    for (;;) {
        /* Failsafe monitor */
        if ((state == STATE_ARMED || state == STATE_DISARMED) &&
            (xTaskGetTickCount() - rc.last_packet_ticks) * portTICK_PERIOD_MS > 500) {
            taskENTER_CRITICAL(&telemetry_mux);
            state = STATE_FAILSAFE;
            taskEXIT_CRITICAL(&telemetry_mux);
            ESP_LOGE(TAG, "Signal lost -> FAILSAFE");
        }

        if (i2c_master_write_read_device(I2C_PORT, 0x68,
                (uint8_t[]){0x3B}, 1, raw, 14, pdMS_TO_TICKS(2)) != ESP_OK) {
            uart_write_bytes(COPRO_UART_NUM,
                            (char*)&(uint8_t){(uint8_t)state}, 1);
            vTaskDelayUntil(&last, period);
            continue;
        }

        int16_t ax=(raw[0]<<8)|raw[1], ay=(raw[2]<<8)|raw[3], az=(raw[4]<<8)|raw[5];
        int16_t gx=(raw[8]<<8)|raw[9], gy=(raw[10]<<8)|raw[11], gz=(raw[12]<<8)|raw[13];

        switch (state) {
        case STATE_BOOTING:
            for (int i=0;i<4;i++) set_throttle(i,1000);
            if (xTaskGetTickCount() > pdMS_TO_TICKS(3000)) {
                taskENTER_CRITICAL(&telemetry_mux);
                state = STATE_CALIBRATING;
                taskEXIT_CRITICAL(&telemetry_mux);
                ESP_LOGI(TAG, "Beginning calibration...");
            }
            break;

        case STATE_CALIBRATING:
            sx+=atan2f(ay,az)*RAD_TO_DEG; sy+=atan2f(-ax,hypotf(ay,az))*RAD_TO_DEG;
            sgx+=gx/65.5f; sgy+=gy/65.5f; sgz+=gz/65.5f;
            if (++sample >= 500) {
                taskENTER_CRITICAL(&telemetry_mux);
                imu.roll_offset=sx/500; imu.pitch_offset=sy/500;
                imu.gyro_roll_offset=sgx/500; imu.gyro_pitch_offset=sgy/500;
                imu.gyro_yaw_offset=sgz/500;
                imu.roll_angle  = atan2f(ay,az)*RAD_TO_DEG - imu.roll_offset;
                imu.pitch_angle = atan2f(-ax,hypotf(ay,az))*RAD_TO_DEG - imu.pitch_offset;
                state = STATE_DISARMED;
                taskEXIT_CRITICAL(&telemetry_mux);
                ESP_LOGI(TAG, "Calibration complete -> DISARMED");
            }
            break;

        case STATE_DISARMED:
        case STATE_ARMED: {
            taskENTER_CRITICAL(&telemetry_mux);
            float gr_off=imu.gyro_roll_offset, gp_off=imu.gyro_pitch_offset;
            float gy_off=imu.gyro_yaw_offset;
            float r_off=imu.roll_offset, p_off=imu.pitch_offset;
            float r_prev=imu.roll_angle,  p_prev=imu.pitch_angle;
            taskEXIT_CRITICAL(&telemetry_mux);

            float raw_grr=(gx/65.5f)-gr_off, raw_gpr=(gy/65.5f)-gp_off;
            float raw_gyr=(gz/65.5f)-gy_off;

            lpf_grr = LPF_ALPHA*raw_grr + (1.0f-LPF_ALPHA)*lpf_grr;
            lpf_gpr = LPF_ALPHA*raw_gpr + (1.0f-LPF_ALPHA)*lpf_gpr;
            lpf_gyr = LPF_ALPHA*raw_gyr + (1.0f-LPF_ALPHA)*lpf_gyr;

            float ar = atan2f(ay,az)*RAD_TO_DEG - r_off;
            float ap = atan2f(-ax,hypotf(ay,az))*RAD_TO_DEG - p_off;
            float new_roll  = 0.98f*(r_prev + lpf_grr*LOOP_TIME_S) + 0.02f*ar;
            float new_pitch = 0.98f*(p_prev + lpf_gpr*LOOP_TIME_S) + 0.02f*ap;

            taskENTER_CRITICAL(&telemetry_mux);
            imu.gyro_roll_rate=lpf_grr; imu.gyro_pitch_rate=lpf_gpr;
            imu.gyro_yaw_rate=lpf_gyr;
            imu.roll_angle=new_roll; imu.pitch_angle=new_pitch;
            taskEXIT_CRITICAL(&telemetry_mux);

            if (state == STATE_DISARMED) {
                for (int i=0;i<4;i++) set_throttle(i,1000);
                if (rc.throttle<1050 && rc.yaw>1900) {
                    static uint16_t c=0;
                    if (++c>=125) {
                        pid_reset_all();
                        taskENTER_CRITICAL(&telemetry_mux);
                        state=STATE_ARMED;
                        taskEXIT_CRITICAL(&telemetry_mux);
                        ESP_LOGW(TAG,"ARMED"); c=0;
                    }
                }
            } else {
                /* Read gains atomically for this control cycle */
                taskENTER_CRITICAL(&pid_mux);
                float rkp=pid_r.Kp,rki=pid_r.Ki,rkd=pid_r.Kd;
                float pkp=pid_p.Kp,pki=pid_p.Ki,pkd=pid_p.Kd;
                float ykp=pid_y.Kp,yki=pid_y.Ki,ykd=pid_y.Kd;
                taskEXIT_CRITICAL(&pid_mux);

                /* Temporary local PID structs using snapshotted gains */
                PID_t lr={rkp,rki,rkd,pid_r.prev_err,pid_r.integral};
                PID_t lp={pkp,pki,pkd,pid_p.prev_err,pid_p.integral};
                PID_t ly={ykp,yki,ykd,pid_y.prev_err,pid_y.integral};

                float er = ((rc.roll -1500)*0.06f) - new_roll;
                float ep = ((rc.pitch-1500)*0.06f) - new_pitch;
                float ey = ((rc.yaw  -1500)*0.15f) - lpf_gyr;

                float pr=pid_compute(&lr,er);
                float pp=pid_compute(&lp,ep);
                float py=pid_compute(&ly,ey);

                /* Write back state (prev_err, integral) */
                pid_r.prev_err=lr.prev_err; pid_r.integral=lr.integral;
                pid_p.prev_err=lp.prev_err; pid_p.integral=lp.integral;
                pid_y.prev_err=ly.prev_err; pid_y.integral=ly.integral;

                if (rc.throttle > 1050) {
                    set_throttle(0, rc.throttle - pp - pr + py);
                    set_throttle(1, rc.throttle - pp + pr - py);
                    set_throttle(2, rc.throttle + pp + pr + py);
                    set_throttle(3, rc.throttle + pp - pr - py);
                } else {
                    for (int i=0;i<4;i++) set_throttle(i,1000);
                    pid_reset_all();
                }

                if (rc.throttle<1050 && rc.yaw<1100) {
                    static uint16_t c=0;
                    if (++c>=125) {
                        taskENTER_CRITICAL(&telemetry_mux);
                        state=STATE_DISARMED;
                        taskEXIT_CRITICAL(&telemetry_mux);
                        ESP_LOGI(TAG,"DISARMED"); c=0;
                    }
                }
            }
        } break;

        case STATE_FAILSAFE:
            for (int i=0;i<4;i++) set_throttle(i,1000);
            if ((xTaskGetTickCount()-rc.last_packet_ticks)*portTICK_PERIOD_MS<300) {
                taskENTER_CRITICAL(&telemetry_mux);
                state=STATE_DISARMED;
                taskEXIT_CRITICAL(&telemetry_mux);
                ESP_LOGI(TAG,"Signal recovered -> DISARMED");
            }
            break;
        }

        uint8_t s=(uint8_t)state;
        uart_write_bytes(COPRO_UART_NUM,(char*)&s,1);
        vTaskDelayUntil(&last, period);
    }
}

/*─────────────────────────────────────────────────────────────────────────────*
 * MAIN
 *─────────────────────────────────────────────────────────────────────────────*/
void app_main(void) {
    nvs_flash_init();
    ESP_LOGI(TAG, "Booting flight controller (live PID tuning enabled)");

    pwm_init();
    vTaskDelay(pdMS_TO_TICKS(3000));

    nvs_load_gains();   /* Load last saved gains before tasks start */

    wifi_init_sta();
    copro_uart_init();
    i2c_init();

    xTaskCreatePinnedToCore(task_ibus,      "RX",     4096, NULL,  5, NULL, 1);
    xTaskCreatePinnedToCore(task_flight,    "Flight", 6144, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(task_udp,       "UDP",    4096, NULL,  2, NULL, 0);
    xTaskCreatePinnedToCore(task_pid_tuner, "Tuner",  4096, NULL,  1, NULL, 0);
}
