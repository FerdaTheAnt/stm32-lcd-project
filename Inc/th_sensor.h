#ifndef INC_TH_SENSOR_H_
#define INC_TH_SENSOR_H_

#include "main.h"

#define TH_SENSOR_ADDR 0x44

extern const uint8_t TH_INIT;
extern const uint8_t TH_DATA_LEN;

int th_sensor_measure(int* t_degC, int* t_degF, int* rh_pRH);

#endif /* INC_TH_SENSOR_H_ */
