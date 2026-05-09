#include <stdio.h>
#include <math.h>
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"

#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_ADDR 0x68
#define LOOP_DT 0.004f          // 4 ms (250 Hz)
#define RAD2DEG 57.2958f

/* ESC pins */
#define ESC1 13
#define ESC2 12
#define ESC3 14
#define ESC4 27

/* Simple globals */
float roll = 0, pitch = 0;
float gyro_r = 0, gyro_p = 0;
float roll_off = 0, pitch_off = 0;
float gyro_r_bias = 0, gyro_p_bias = 0;
float e_prev_r = 0, i_r = 0;

/* PID constants */
float Kp = 0.6f, Ki = 0.0f, Kd = 0.2f;

/* --- hardware helpers --- */
void esc_write(int ch, int us) {
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  uint32_t duty = (us * 8192) / 20000;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

void init_pwm() {
  ledc_timer_config_t t = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .duty_resolution = LEDC_TIMER_13_BIT,
      .freq_hz = 50,
      .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&t);
  int pins[4] = {ESC1, ESC2, ESC3, ESC4};
  for (int i = 0; i < 4; i++) {
    ledc_channel_config_t c = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = i,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = pins[i],
        .duty = 409,
        .hpoint = 0};
    ledc_channel_config(&c);
  }
}

void i2c_init_mpu() {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA,
      .scl_io_num = I2C_SCL,
      .sda_pullup_en = 1,
      .scl_pullup_en = 1,
      .master.clk_speed = 400000};
  i2c_param_config(I2C_NUM_0, &conf);
  i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
  uint8_t wake[2] = {0x6B, 0};
  i2c_master_write_to_device(I2C_NUM_0, MPU_ADDR, wake, 2,
                             100 / portTICK_PERIOD_MS);
}

/* --- IMU read + complementary filter --- */
void read_mpu(float *ax, float *ay, float *az, float *gx, float *gy) {
  uint8_t data[14];
  i2c_master_write_read_device(I2C_NUM_0, MPU_ADDR, (uint8_t[]){0x3B}, 1,
                               data, 14, 100);
  *ax = (data[0] << 8 | data[1]);
  *ay = (data[2] << 8 | data[3]);
  *az = (data[4] << 8 | data[5]);
  *gx = (data[8] << 8 | data[9]) / 65.5f;
  *gy = (data[10] << 8 | data[11]) / 65.5f;
}

/* --- simple PID --- */
float pid_roll(float ref, float meas) {
  float e = ref - meas;
  i_r += e * LOOP_DT;
  float d = (e - e_prev_r) / LOOP_DT;
  e_prev_r = e;
  return Kp * e + Ki * i_r + Kd * d;
}

/* --- main ---- */
void app_main(void) {
  nvs_flash_init();
  init_pwm();
  i2c_init_mpu();

  printf("Boot + Calibrating IMU...\n");
  float ax, ay, az, gx, gy;
  for (int i = 0; i < 500; i++) {
    read_mpu(&ax, &ay, &az, &gx, &gy);
    roll_off += atan2(ay, az) * RAD2DEG;
    pitch_off += atan2(-ax, sqrt(ay * ay + az * az)) * RAD2DEG;
    gyro_r_bias += gx;
    gyro_p_bias += gy;
    vTaskDelay(pdMS_TO_TICKS(4));
  }
  roll_off /= 500; pitch_off /= 500;
  gyro_r_bias /= 500; gyro_p_bias /= 500;
  printf("Calib OK\n");

  uint32_t last = esp_log_timestamp();
  while (1) {
    while ((esp_log_timestamp() - last) < (LOOP_DT * 1000));
    last = esp_log_timestamp();

    read_mpu(&ax, &ay, &az, &gx, &gy);
    float acc_r = atan2(ay, az) * RAD2DEG - roll_off;
    roll = 0.98f * (roll + (gx - gyro_r_bias) * LOOP_DT) + 0.02f * acc_r;

    float u = pid_roll(0, roll);
    int base = 1200;
    esc_write(0, base + u);
    esc_write(1, base - u);
    esc_write(2, base + u);
    esc_write(3, base - u);
  }
}
