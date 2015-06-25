/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <planner.h>
#include <stdio.h>
#include <utils.h>
#include <assert.h>
#include <common.h>
#include <ppm_inf.h>
#include <flight_inf.h> // For types
#include <platsupport/timer.h>
#include <platsupport/plat/timer.h>

//#define DEBUG_PLANNER
#ifdef DEBUG_PLANNER
#define DPLAN(args...) \
    do { \
        printf("PLANNER %s(%d): ", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DPLAN(...) do{}while(0)
#endif

#define PLANNER_PERIOD 10000000 // 100Hz
pstimer_t *timer;

void init__planner() {
	DPLAN("Starting planner init");
	timer_register_wakeup(PLANNER_APP_ID, PLANNER_PERIOD, PLANNER_PERIOD);
	timer = generic_timer_get_timer();
}

//char teststr[300];

void collect_data() {
	DPLAN("Collecting data\n");
	/*
	 vector_6f_t state = flight_get_state();
	 x = state.roll;
	 y = state.yaw;
	 z = state.pitch;
	 sprintf(teststr, "POSE: %lf %lf %lf 0.0 0.0 0.0\n", x, y, z);
	 */
	sensors_t s = spi_sensors_get_last_read();
	sprintf(teststr, "BARO: %lf\nGYRO: %lf %lf %lf\n"
			"MPU: %lf %lf %lf %lf %lf %lf\nECOM: %lf %lf %lf %lf %lf %lf\n",
			s.baro.pressure, s.gyro.x, s.gyro.y, s.gyro.z, s.mpu.acc.x,
			s.mpu.acc.y, s.mpu.acc.z, s.mpu.gyro.x, s.mpu.gyro.y, s.mpu.gyro.z,
			s.ecompass.acc.x, s.ecompass.acc.y, s.ecompass.acc.z,
			s.ecompass.mag.x, s.ecompass.mag.y, s.ecompass.mag.z);
	DPLAN("Finised collecting");
}

int run() {
	DPLAN("Run");
	char buffer[100];
	int stamp = 0;
	init__planner();
	uint64_t prev_time = 0;
	int i = 0;
	uint64_t av = 0;

	int lowPass = 10;

	while (true) {
		wakeup_signal_wait(); // Wait until the timer signals to run
		uint64_t cur_time = timer->get_time(timer);
		// Read the ppm values from the shared buffer
		ppm_channels_t *channels = ppm;

		// Using a rate controller so pass in the current desired rate into flight

		static vector_6f_t state;

		state.roll += ((channels->c[0] - 1500) / 500) * 100;
		state.pitch += ((channels->c[1] - 1500) / 500) * 100;
		state.yaw += ((channels->c[3] - 1500) / 500) * 100;
		state.z += (channels->c[2] - 1000) / 1000;
		state.x += (channels->c[5] - 1000) / 1000;
		state.y += channels->c[4] < 1500 ? 0 : 1;

		if (i % lowPass == 0) {
			state.roll /= lowPass;
			state.pitch /= lowPass;
			state.yaw /= lowPass;
			state.z /= lowPass;
			state.x /= lowPass;
			state.y /= lowPass;

			flight_set_state(state);

			state.roll = 0;
			state.pitch = 0;
			state.yaw = 0;
			state.z = 0;
			state.x = 0;
			state.y = 0;
		}

		//collect_data();

		uint64_t grab_time = timer->get_time(timer);
		av += grab_time - cur_time;
		if (i % 50 == 0) {
			av = 0;
			// Read the ppm values from the shared buffer
			//ppm_channels_t *channels = ppm;
			//sprintf(buffer, "PPM Channels: %lf %lf %lf %lf %lf %lf\n",
			//       channels->c[0], channels->c[1], channels->c[2],
			//        channels->c[3], channels->c[4], channels->c[5]);
			//printf("%s\n", buffer);
			//printf("%s\n", teststr);
		}

		++i;
		++stamp;
		prev_time = cur_time;

	}
	return 0;
}

