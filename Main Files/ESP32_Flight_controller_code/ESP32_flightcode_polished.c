/**
 * @file main.c
 * @brief Deterministic ESP32‑based Quadcopter Flight Controller using FreeRTOS.
 *
 * This firmware executes a hard‑real‑time 250 Hz flight loop that reads IMU data,
 * runs sensor fusion, evaluates PID control laws, and drives four ESCs through
 * hardware PWM. A strict Finite‑State Machine (FSM) ensures safe booting,
 * arming/disarming, and automatic failsafe recovery.
 *
 * Architecture overview:
 * - Core 1: deterministic flight dynamics + radio parsing
 * - Core 0: UDP telemetry and Wi‑Fi stack
 *
 * @authors
 * Troy Franco G. Celdran (et al.)
 * @date 2026‑04‑29
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
#define WIFI_SSID  "YOUR-WIFI-NAME"
#define WIFI_PASS  "YOUR-WIFI-PASSWORD"
#define PC_IP      "YOUR-COMPUTER'S-IP-ADDRESS"
#define UDP_PORT   YOU-UPD-PORT-VALUE

/** ESC signal pins (50 Hz servo‑style PWM) */
#define ESC1_PIN 13 // CCW
#define ESC2_PIN 12 // CW
#define ESC3_PIN 14 // CCW
#define ESC4_PIN 27 // CW

/** I²C connections to IMU (GY‑521 / MPU6050) */
#define I2C_SCL 22
#define I2C_SDA 21
#define I2C_PORT I2C_NUM_0
#define I2C_HZ   400000

/** FlySky receiver and coprocessor serial ports */
#define IBUS_RX_PIN 4 // <-- the latest update, moved RX from GPIO-16 to GPIO-4
#define IBUS_UART_NUM UART_NUM_1
#define COPRO_TX_PIN 17
#define COPRO_UART_NUM UART_NUM_2

/** Timing constants */
#define LOOP_TIME_S 0.004f         ///< Loop period (4 ms = 250 Hz)
#define RAD_TO_DEG 57.2958f
#define I_MAX 50.0f                ///< Integral‑term saturation limit

static const char *TAG = "FlightCtrl";

/*─────────────────────────────────────────────────────────────────────────────*
 * DATA STRUCTURES
 *─────────────────────────────────────────────────────────────────────────────*/

/** @brief Aggregated i‑BUS control channels */
typedef struct {
    uint16_t roll, pitch, throttle, yaw;
    uint32_t last_packet_ticks;
} RC_Input_t;

/** @brief Pre‑filtered IMU data (attitude + biases) */
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
RC_Input_t rc = {1500,1500,1000,1500,0};
IMU_Data_t imu = {0};
PID_t pid_r = {0.5,0.0,0.02,0,0}, pid_p = {0.5,0.0,0.02,0,0}, pid_y = {1.0,0.0,0.03,0,0};
FlightState_t state = STATE_BOOTING;

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
    if (p->integral > I_MAX) p->integral = I_MAX;
    if (p->integral < -I_MAX) p->integral = -I_MAX;
    float I = p->Ki * p->integral;
    float D = p->Kd * (error - p->prev_err) / LOOP_TIME_S;
    p->prev_err = error;
    return P + I + D;
}

/** Resets integrators when disarming to avoid jump during next arming. */
static inline void pid_reset_all(void){ pid_r.integral=pid_p.integral=pid_y.integral=0; }

/*─────────────────────────────────────────────────────────────────────────────*
 * HARDWARE INITIALIZATION
 *─────────────────────────────────────────────────────────────────────────────*/

/**
 * @brief Initializes four LEDC PWM outputs at 50 Hz for ESCs.
 */
static void pwm_init(void){
    ledc_timer_config_t t={ .speed_mode=LEDC_LOW_SPEED_MODE,.timer_num=LEDC_TIMER_0,
                            .duty_resolution=LEDC_TIMER_13_BIT,.freq_hz=50,.clk_cfg=LEDC_AUTO_CLK};
    ledc_timer_config(&t);
    const int pins[4]={ESC1_PIN,ESC2_PIN,ESC3_PIN,ESC4_PIN};
    for(int i=0;i<4;i++){
        ledc_channel_config_t c={.speed_mode=LEDC_LOW_SPEED_MODE,.channel=i,
            .timer_sel=LEDC_TIMER_0,.intr_type=LEDC_INTR_DISABLE,
            .gpio_num=pins[i],.duty=409,.hpoint=0};
        ledc_channel_config(&c);
    }
}

/**
 * @brief Writes microsecond‑scaled throttle to an ESC channel.
 */
static void set_throttle(ledc_channel_t ch,int us){
    if (us < 1000) {
        us = 1000;
    }
    if (us > 2000) {
        us = 2000;
    }
    uint32_t duty=(us*8192)/20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE,ch,duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,ch);
}

/**
 * @brief Initializes I²C and wakes IMU (MPU6050 in ±500 °/s range).
 */
static void i2c_init(void){
    i2c_config_t conf={.mode=I2C_MODE_MASTER,.sda_io_num=I2C_SDA,.scl_io_num=I2C_SCL,
                    .sda_pullup_en=1,.scl_pullup_en=1,.master.clk_speed=I2C_HZ};
    i2c_param_config(I2C_PORT,&conf); i2c_driver_install(I2C_PORT,conf.mode,0,0,0);
    uint8_t wake[2]={0x6B,0x00}; i2c_master_write_to_device(I2C_PORT,0x68,wake,2,100/portTICK_PERIOD_MS);
    uint8_t gyro_cfg[2]={0x1B,0x08}; i2c_master_write_to_device(I2C_PORT,0x68,gyro_cfg,2,100/portTICK_PERIOD_MS);
}

/** Initializes minimal Wi‑Fi STA for UDP telemetry. */
static void wifi_init_sta(void){
    esp_netif_init(); esp_event_loop_create_default(); esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT(); esp_wifi_init(&cfg);
    wifi_config_t w={0}; strcpy((char*)w.sta.ssid,WIFI_SSID); strcpy((char*)w.sta.password,WIFI_PASS);
    esp_wifi_set_mode(WIFI_MODE_STA); esp_wifi_set_config(WIFI_IF_STA,&w);
    esp_wifi_start(); esp_wifi_connect();
}

/** Initializes dedicated UART2 link to ATmega coprocessor. */
static void copro_uart_init(void){
    uart_config_t c={9600,UART_DATA_8_BITS,UART_PARITY_DISABLE,UART_STOP_BITS_1,UART_HW_FLOWCTRL_DISABLE, 0};
    uart_param_config(COPRO_UART_NUM,&c);
    uart_set_pin(COPRO_UART_NUM,COPRO_TX_PIN,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    uart_driver_install(COPRO_UART_NUM,256,0,0,NULL,0);
}

/*─────────────────────────────────────────────────────────────────────────────*
 * TASKS
 *─────────────────────────────────────────────────────────────────────────────*/

/**
 * @brief Periodic UDP telemetry (10 Hz) — runs on Core 0.
 */
static void task_udp(void *a){
    struct sockaddr_in dest={.sin_family=AF_INET,.sin_port=htons(UDP_PORT)};
    dest.sin_addr.s_addr=inet_addr(PC_IP);
    int sock=socket(AF_INET,SOCK_DGRAM,IPPROTO_IP);
    char buf[128];
    for(;;){
        snprintf(buf,sizeof buf,"%lu,%d,%.2f,%.2f,%d\n",
                (uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS),
                state,imu.roll_angle,imu.pitch_angle,rc.throttle);
        sendto(sock,buf,strlen(buf),0,(struct sockaddr*)&dest,sizeof dest);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Parses i‑BUS packets (FlySky receiver) — runs on Core 1.
 */
static void task_ibus(void *a){
    uart_config_t c={115200,UART_DATA_8_BITS,UART_PARITY_DISABLE,UART_STOP_BITS_1,UART_HW_FLOWCTRL_DISABLE, 0};
    uart_param_config(IBUS_UART_NUM,&c);
    uart_set_pin(IBUS_UART_NUM,UART_PIN_NO_CHANGE,IBUS_RX_PIN,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    uart_driver_install(IBUS_UART_NUM,1024,0,0,NULL,0);

    uint8_t pkt[32];
    for(;;){
        int len=uart_read_bytes(IBUS_UART_NUM,pkt,32,pdMS_TO_TICKS(20));
        if(len==32 && pkt[0]==0x20 && pkt[1]==0x40){
            uint16_t chk=0xFFFF; for(int i=0;i<30;i++) chk-=pkt[i];
            if(chk==(pkt[30]|(pkt[31]<<8))){
                rc.roll=pkt[2]|(pkt[3]<<8);
                rc.pitch=pkt[4]|(pkt[5]<<8);
                rc.throttle=pkt[6]|(pkt[7]<<8);
                rc.yaw=pkt[8]|(pkt[9]<<8);
                rc.last_packet_ticks=xTaskGetTickCount();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Core flight‑control logic @ 250 Hz.
 *
 * Implements:
 * - IMU burst read
 * - complementary filter
 * - FSM handling (Boot → Calibrate → Disarm/Arm/Failsafe)
 * - motor mixing
 * - safety constraints
 */
static void task_flight(void *a){
    uint8_t raw[14];
    TickType_t last=xTaskGetTickCount();
    const TickType_t period=pdMS_TO_TICKS(4);

    /* accumulators for offset estimation */
    float sx=0,sy=0,sgx=0,sgy=0,sgz=0; uint16_t sample=0;

    for(;;){
        /* FAILSAFE MONITOR --------------------------------------------------*/
        if((state==STATE_ARMED||state==STATE_DISARMED) &&
        (xTaskGetTickCount()-rc.last_packet_ticks)*portTICK_PERIOD_MS>500){
            ESP_LOGE(TAG,"Signal lost -> FAILSAFE");
            state=STATE_FAILSAFE;
        }

        /* IMU SENSOR BURST --------------------------------------------------*/
        i2c_master_write_read_device(I2C_PORT,0x68,(uint8_t[]){0x3B},1,raw,14,100);
        int16_t ax=(raw[0]<<8)|raw[1], ay=(raw[2]<<8)|raw[3], az=(raw[4]<<8)|raw[5];
        int16_t gx=(raw[8]<<8)|raw[9], gy=(raw[10]<<8)|raw[11], gz=(raw[12]<<8)|raw[13];

        switch(state){
        /*──────────────────────── BOOTING ───────────────────────*/
        case STATE_BOOTING:
            for(int i=0;i<4;i++) set_throttle(i,1000);
            if(xTaskGetTickCount()>pdMS_TO_TICKS(3000)){
                ESP_LOGI(TAG,"Beginning calibration…");
                state=STATE_CALIBRATING;
            }
            break;

        /*──────────────────────── CALIBRATION ───────────────────*/
        case STATE_CALIBRATING:
            sx+=atan2f(ay,az)*RAD_TO_DEG;
            sy+=atan2f(-ax,hypotf(ay,az))*RAD_TO_DEG;
            sgx+=gx/65.5f; sgy+=gy/65.5f; sgz+=gz/65.5f;
            if(++sample>=500){
                imu.roll_offset=sx/500; imu.pitch_offset=sy/500;
                imu.gyro_roll_offset=sgx/500; imu.gyro_pitch_offset=sgy/500; imu.gyro_yaw_offset=sgz/500;
                imu.roll_angle=atan2f(ay,az)*RAD_TO_DEG-imu.roll_offset;
                imu.pitch_angle=atan2f(-ax,hypotf(ay,az))*RAD_TO_DEG-imu.pitch_offset;
                ESP_LOGI(TAG,"Calibration complete → DISARMED");
                state=STATE_DISARMED;
            }
            break;

        /*──────────────────────── DISARM / ARM ───────────────────*/
        case STATE_DISARMED:
        case STATE_ARMED:{
            /* sensor fusion */
            imu.gyro_roll_rate =(gx/65.5f)-imu.gyro_roll_offset;
            imu.gyro_pitch_rate=(gy/65.5f)-imu.gyro_pitch_offset;
            imu.gyro_yaw_rate  =(gz/65.5f)-imu.gyro_yaw_offset;

            float ar=atan2f(ay,az)*RAD_TO_DEG-imu.roll_offset;
            float ap=atan2f(-ax,hypotf(ay,az))*RAD_TO_DEG-imu.pitch_offset;
            imu.roll_angle =0.98f*(imu.roll_angle + imu.gyro_roll_rate*LOOP_TIME_S)+0.02f*ar;
            imu.pitch_angle=0.98f*(imu.pitch_angle+ imu.gyro_pitch_rate*LOOP_TIME_S)+0.02f*ap;

            if(state==STATE_DISARMED){
                for(int i=0;i<4;i++) set_throttle(i,1000);
                if(rc.throttle<1050 && rc.yaw>1900){ static uint16_t c=0;
                    if(++c>=125){ pid_reset_all(); state=STATE_ARMED; ESP_LOGW(TAG,"ARMED"); c=0;}
                }
            }else{
                float er=((rc.roll-1500)*0.06f)-imu.roll_angle;
                float ep=((rc.pitch-1500)*0.06f)-imu.pitch_angle;
                float ey=((rc.yaw-1500)*0.15f)-imu.gyro_yaw_rate;
                float pr=pid_compute(&pid_r,er), pp=pid_compute(&pid_p,ep), py=pid_compute(&pid_y,ey);

                if(rc.throttle>1050){
                    set_throttle(0,rc.throttle-pp-pr+py);
                    set_throttle(1,rc.throttle-pp+pr-py);
                    set_throttle(2,rc.throttle+pp+pr+py);
                    set_throttle(3,rc.throttle+pp-pr-py);
                }else{
                    for(int i=0;i<4;i++) set_throttle(i,1000);
                    pid_reset_all();
                }
                if(rc.throttle<1050 && rc.yaw<1100){ static uint16_t c=0;
                    if(++c>=125){ state=STATE_DISARMED; ESP_LOGI(TAG,"DISARMED"); c=0;}
                }
            }} break;

        /*──────────────────────── FAILSAFE ───────────────────────*/
        case STATE_FAILSAFE:
            for(int i=0;i<4;i++) set_throttle(i,1000);
            if((xTaskGetTickCount()-rc.last_packet_ticks)*portTICK_PERIOD_MS<300){
                state=STATE_DISARMED;
                ESP_LOGI(TAG,"Signal recovered → DISARMED");
            }
            break;
        }

        /* Notify coprocessor of state (1 byte) */
        uint8_t s=(uint8_t)state;
        uart_write_bytes(COPRO_UART_NUM,(char*)&s,1);

        vTaskDelayUntil(&last,period);
    }
}

/*─────────────────────────────────────────────────────────────────────────────*
 * MAIN
 *─────────────────────────────────────────────────────────────────────────────*/
void app_main(void){
    nvs_flash_init();
    ESP_LOGI(TAG,"Booting flight controller");
    pwm_init(); wifi_init_sta(); copro_uart_init();
    vTaskDelay(pdMS_TO_TICKS(3000));   // Allow Wi‑Fi association
    i2c_init();

    xTaskCreatePinnedToCore(task_ibus ,"RX"     ,4096,NULL,5 ,NULL,1);
    xTaskCreatePinnedToCore(task_flight,"Flight",6144,NULL,10,NULL,1);
    xTaskCreatePinnedToCore(task_udp   ,"UDP"   ,4096,NULL,2 ,NULL,0);
}
