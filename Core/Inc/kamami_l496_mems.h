// Functions for easy use of LSM303C MEMS accelerometer on Kamami KAMeLeon STM32L496 board.
// jk 11'2017

#ifndef	_KAMAMI_L496_MEMS_H
#define	_KAMAMI_L496_MEMS_H
#include <stdint.h>

// Possible datarates of accelerometer i.e. possible values for acc_datarate parameter of mems_init function.
#define MEMS_ACC_DATARATE_10HZ		1
#define MEMS_ACC_DATARATE_50HZ		2
#define MEMS_ACC_DATARATE_100HZ		3
#define MEMS_ACC_DATARATE_200HZ		4
#define MEMS_ACC_DATARATE_400HZ		5
#define MEMS_ACC_DATARATE_800HZ		6

// Possible fullscale range of accelerometer i.e. possible values for acc_fullscale parameter of mems_init function.
// Fullscale range of x means the values range from -x to +x.
#define MEMS_ACC_FULLSCALE_2G		0
#define MEMS_ACC_FULLSCALE_4G		2
#define MEMS_ACC_FULLSCALE_8G		3

#define MEMS_ACC_MAXVAL				0x7fff

/* Stucture for results read from MEMs sensor.
 * The result is comprised of 3 signed 16-bit numbers - one for each axis. */
struct mems_xyz_res {
	int16_t x;
	int16_t y;
	int16_t z;
};

/* Initialize LSM303C MEMS accelerometer. 
 * The function should be called before using any other accelerometer function. 
 * It initializes GPIO lines and I2C interface used to communicate with the sensor. 
 * @return 0 on succes, 1 if the function was called with incorrect parameters or 
 * interfaceing with sensor failed. */
uint8_t mems_init(uint8_t acc_datarate, uint8_t acc_fullscale);

/* Read accelerometer measurement of all 3 axes. The results are put in a mems_xyz_res structure. 
 * @param res: pointer to mems_xyz_res structure to put results in. */
void mems_acc_read_xyz(struct mems_xyz_res* res);

/* Read x-axis acceleration.  
 * @return x-axis acceleration */
int16_t mems_acc_read_x(void);

/* Read y-axis acceleration.  
 * @return y-axis acceleration */
int16_t mems_acc_read_y(void);

/* Read z-axis acceleration.  
 * @return z-axis acceleration */
int16_t mems_acc_read_z(void);

#endif
