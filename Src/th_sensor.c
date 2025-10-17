#include "th_sensor.h"

#include "main.h"

const uint8_t TH_INIT = 0xFD;
const uint8_t TH_DATA_LEN = 6;

extern I2C_HandleTypeDef hi2c1;

int th_sensor_measure(int* t_degC, int* t_degF, int* rh_pRH)
{
	uint8_t rx_bytes[TH_DATA_LEN];
	if(HAL_I2C_Master_Transmit(&hi2c1, TH_SENSOR_ADDR << 1, &TH_INIT, 1, 1) == HAL_OK)
	{
		HAL_Delay(20);
		if(HAL_I2C_Master_Receive(&hi2c1, TH_SENSOR_ADDR << 1, rx_bytes, TH_DATA_LEN, 1) == HAL_OK)
		{
			/*
			 * process received data according to data sheet
			 */
			int t_ticks = rx_bytes[0] * 256 + rx_bytes[1];
			int checksum_t = rx_bytes[2];
			int rh_ticks = rx_bytes[3]*256 + rx_bytes[4];
			int checksum_rh = rx_bytes[5];

			*t_degC = -45 + 175 * t_ticks / 65535;
			*t_degF = -49 + 315 * t_ticks / 65535;
			*rh_pRH = -6 + 125 * rh_ticks / 65535;

			if(*rh_pRH > 100)
				*rh_pRH = 100;
			if(*rh_pRH < 0)
				*rh_pRH = 0;

			return 0;
		}
	}
	return -1;
}


