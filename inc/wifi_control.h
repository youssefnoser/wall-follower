#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H

#include "esp8266.h"
#include "hc_sr04.h"
#include "l298n.h"
#include "mpu6050.h"

void WiFiControl_Init(const char *ssid, const char *password);
void WiFiControl_Update(L298N_t *motors, HC_SR04_t *front, HC_SR04_t *left,
                        HC_SR04_t *right, MPU6050_t *imu);

#endif /* WIFI_CONTROL_H */
