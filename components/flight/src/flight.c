/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <flight.h>
#include <stdio.h>
#include <utils.h>
#include <assert.h>
#include <common.h>
#include <platsupport/i2c.h>
#include <math.h>
#include <platsupport/timer.h>
#include <platsupport/plat/timer.h>
#include <flight_inf.h> // For types
#define ACC_WEIGHT 0.05 
#define MAG_WEIGHT 0.0 

//#define DEBUG_FLIGHT
#ifdef DEBUG_FLIGHT
#define DFLI(args...) \
    do { \
        printf("FLIGHT %s(%d): ", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DFLI(...) do{}while(0)
#endif

vector_6f_t state;
double dcm[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

// 3d Vector maths
double vector3d_mod(double *v) {
	return sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

void vector3d_normalize(double *v) {
	double size = vector3d_mod(v);
	v[0] /= size;
	v[1] /= size;
	v[2] /= size;
}

// c = a x b
void vector3d_cross(double *a, double *b, double *c) {
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}

double vector3d_dot(double a[3], double b[3]) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector3d_scale(double s, double a[3], double b[3]) {
	b[0] = s * a[0];
	b[1] = s * a[1];
	b[2] = s * a[2];
}

void vector3d_add(double a[3], double b[3], double c[3]) {
	c[0] = a[0] + b[0];
	c[1] = a[1] + b[1];
	c[2] = a[2] + b[2];
}

void dcm_orthonormalize(double dcm[3][3]) {
	double err = vector3d_dot(dcm[0], dcm[1]);
	double delta[2][3];
	vector3d_scale(-err / 2, dcm[1], delta[0]);
	vector3d_scale(-err / 2, dcm[0], delta[1]);
	vector3d_add(dcm[0], delta[0], dcm[0]);
	vector3d_add(dcm[1], delta[1], dcm[1]);

	vector3d_cross(dcm[0], dcm[1], dcm[2]);
	vector3d_normalize(dcm[0]);
	vector3d_normalize(dcm[1]);
	vector3d_normalize(dcm[2]);
}

void dcm_rotate(double dcm[3][3], double w[3]) {
	int i;
	double dR[3];
	for (i = 0; i < 3; ++i) {
		vector3d_cross(w, dcm[i], dR);
		vector3d_add(dcm[i], dR, dcm[i]);
	}
	dcm_orthonormalize(dcm);
}

double degreesToRads(double d) {
	return (d / 180) * 3.14159;
}

double radsToDegrees(double r) {
	return (r / 3.14159) * 180;
}

void flight_update() {
	flight_set_state(state);
}

vector_6f_t flight_get_state(void) {
	return state;
}

pstimer_t *timer;

vector_6f_t flight_set_state(vector_6f_t command) {
	DFLI("Setting state");
	uint64_t start = timer->get_time(timer);
	// Read from SPI sensors and compass
	sensors_t s = spi_sensors_read();
	uint64_t spi = timer->get_time(timer);

	// Fuse sensor data using DCM algorithm
	// Based on code from www.starlino.com/dcm_tutorial.html
	// Normalise acellometer
	// TODO: combine readings from both acc
	double a1_norm[3];
	a1_norm[0] = s.mpu.acc.x;
	a1_norm[1] = s.mpu.acc.y;
	a1_norm[2] = s.mpu.acc.z;
	vector3d_normalize(a1_norm);

	// calculate correction vector
	double wA[3];
	vector3d_cross(dcm[2], a1_norm, wA);

	// Normalise magnometer
	double m_norm[3];
	m_norm[0] = s.ecompass.mag.x;
	m_norm[1] = s.ecompass.mag.y;
	m_norm[2] = s.ecompass.mag.z;
	vector3d_normalize(m_norm);
	// calculate correction vector
	double wM[3];
	vector3d_cross(dcm[0], m_norm, wM);

	// Calculate dThetaG
	double wG[3];
	// Assume we're sampling at 100Hz...
	wG[0] = (degreesToRads(s.gyro.x) / 100);
	wG[1] = (degreesToRads(s.gyro.y) / 100);
	wG[2] = (degreesToRads(s.gyro.z) / 100);

	// Do a weighted average
	int j;
	double w[3];
	for (j = 0; j < 3; ++j) {
		w[j] = (wG[j] + ACC_WEIGHT * wA[j] + MAG_WEIGHT * wM[j])
				/ (1 + ACC_WEIGHT + MAG_WEIGHT);
	}

	// Perform the DCM update
	dcm_rotate(dcm, w);

	// Get the current state from the DCM matrix
	state.roll = radsToDegrees(-asin(dcm[2][0]));
	state.pitch = radsToDegrees(atan2(dcm[2][1], dcm[2][2]));
	state.yaw = radsToDegrees(atan2(dcm[1][0], dcm[0][0]));

	// Implement rate controller
	double kpP = 0.00008, kpR = 0.00008, kpY = 0.0004;
	static double kiP = 0;
	static double kiY = 0;
	if (command.y)
		kiP = command.x / 1000000;
	else
		kiY = command.x / 1000000;
	if (kiP < 0)
		kiP = 0;
	if (kiY < 0)
		kiY = 0;

	// Pitch
	double angle_error_y = -command.pitch + s.gyro.y;
	static double i_acc_p = 0;
	i_acc_p += angle_error_y;
	double pitch_output = angle_error_y * kpP + i_acc_p * kiP;

	// Roll
	double angle_error_x = -command.roll + s.gyro.x;
	static double i_acc_r = 0;
	i_acc_r += angle_error_x;
	double roll_output = angle_error_x * kpR + i_acc_r * kiP;

	// Yaw
	double angle_error_z = -command.yaw + s.gyro.z;
	static double i_acc_y = 0;
	i_acc_y += angle_error_z;
	double yaw_output = angle_error_z * kpY + i_acc_y * kiY;

	/*
	 if (command.z < 0.02) {
	 motors_set(0, 0, 0, 0);
	 i_acc_p = 0;
	 i_acc_r = 0;
	 i_acc_y = 0;
	 } else {
	 motors_set(command.z - roll_output + pitch_output + yaw_output, // FL
	 command.z + roll_output + pitch_output - yaw_output, // FR
	 command.z - roll_output - pitch_output - yaw_output, // BL
	 command.z + roll_output - pitch_output + yaw_output); // BR
	 }
	 */


	motors_set(command.z, command.z, command.z, command.z);
	DFLI("Done set state");
	return state;
}

void flight__init() {
	DFLI("Start init");
	timer = generic_timer_get_timer();
	state.x = 0;
	state.y = 0;
	state.z = 0;
	state.roll = 0;
	state.pitch = 0;
	state.yaw = 0;
	DFLI("Finished init");
}
