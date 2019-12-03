void Feature_Extraction_State_0(void *handle, int * ttt_1, int * ttt_2,
		int * ttt_3, int * ttt_mag_scale) {

	int ttt[3];
	int ttt_initial[3];
	int axis_index;
	float accel_mag;
	char  msg[128];

	/*
	 * Acquire acceleration values prior to motion
	 */

	getAccel(handle, ttt_initial);

	sprintf(msg, "\r\nStart First State Motion to New Orientation when LED On");
	CDC_Fill_Buffer((uint8_t *) msg, strlen(msg));
	BSP_LED_On(LED1);


	HAL_Delay(2000);

	sprintf(msg, "\r\nStop Motion");
	CDC_Fill_Buffer((uint8_t *) msg, strlen(msg));
	BSP_LED_Off(LED1);
	HAL_Delay(1000);


	getAccel(handle, ttt);


	accel_mag = 0;
	for (axis_index = 0; axis_index < 3; axis_index++) {
		ttt[axis_index] = ttt[axis_index] - ttt_initial[axis_index];
		accel_mag = accel_mag + pow((ttt[axis_index]), 2);
	}

	sprintf(msg, "\r\nvalues: %d %d %d", ttt[0], ttt[2], ttt[2]);
	CDC_Fill_Buffer((uint8_t *) msg, strlen(msg));

	accel_mag = sqrt(accel_mag);
	*ttt_1 = ttt[0];
	*ttt_2 = ttt[1];
	*ttt_3 = ttt[2];
	*ttt_mag_scale = (int)(accel_mag);

	return;
}


void Feature_Extraction_State_1(void *handle_g, int * ttt_1, int * ttt_2,
			int * ttt_3, int * ttt_mag_scale) {

		int ttt[3], ttt_state_0[3], ttt_initial[3], ttt_offset[3];
		char msg1[128];
		int axis_index, sample_index;
		float rotate_angle[3];
		float angle_mag, angle_mag_max_threshold;
		float Tsample;


		/*
		 * Assign the initial values provided by execution of State 0 in ttt_1 and ttt_2
		 */
		ttt_state_0[0] = *ttt_1;
		ttt_state_0[1] = *ttt_2;

	angle_mag_max_threshold = ANGLE_MAG_MAX_THRESHOLD;

	/*
	 * Compute sample period with scaling from milliseconds
	 * to seconds
	 */

	Tsample = (float)(DATA_PERIOD_MS)/1000;

	/*
	 * Initialize rotation angle values
	 */

	for (axis_index = 0; axis_index < 3; axis_index++) {
		ttt[axis_index] = 0;
		rotate_angle[axis_index] = 0;
	}

	/*
	 * Rotation Rate Signal integration loop
	 *
	 * Note that loop cycle time is DATA_PERIOD_MS matching the SensorTile
	 * sensor sampling period
	 *
	 * Permit integration loop to operate no longer than a maximum
	 * number of cycles, MAX_ROTATION_ACQUIRE_CYCLES. Note: This sets
	 * maximum acquisition time to be MAX_ROTATION_ACQUIRE_CYCLES*Tsample
	 *
	 * For 4 second delay, apply MAX_ROTATION_ACQUIRE_CYCLES 400
	 *
	 */

	/*
	 * Acquire Rotation Rate values prior to motion
	 *
	 * This includes the initial sensor offset value to be subtracted
	 * from all subsequent samples
	 */

	getAngularVelocity(handle_g, ttt_offset);

	/*
	 * Notify user to initiate motion
	 */

	sprintf(msg1, "\r\nStart Second State Motion to New Orientation when LED On");
	CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
	BSP_LED_On(LED1);



	for (sample_index = 0; sample_index < MAX_ROTATION_ACQUIRE_CYCLES; sample_index++) {

		/*
		 * Acquire initial sample value of rotation rate and remove offset value
		 */

		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt_initial[axis_index] = ttt[axis_index] - ttt_offset[axis_index];
		}

		/*
		 * Introduce integration time period delay
		 */

		HAL_Delay(DATA_PERIOD_MS);

		/*
		 * Acquire current sample value of rotation rate and remove
		 * offset value
		 */

		getAngularVelocity(handle_g, ttt);
		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt[axis_index] = ttt[axis_index] - ttt_offset[axis_index];
		}


		/*
		 * Suppress value of Z-Axis rotation signals
		 */

		ttt_initial[2] = 0;
		ttt[2] = 0;

		/*
		 * Compute rotation angles by integration
		 */

		for (axis_index = 0; axis_index < 3; axis_index++) {
			rotate_angle[axis_index] = rotate_angle[axis_index]
					+ (float)((ttt_initial[axis_index] + ttt[axis_index]) * Tsample / 2);
		}

		/*
		 *
		 *
		 * Compute magnitude of rotational angle summing over X and Y
		 * axis Rotation Rates.
		 *
		 * Convert from milli-degrees to degrees (Note that Rotation
		 * Rate is sampled in milli-degrees per second).
		 *
		 */

		angle_mag = 0;
		for (axis_index = 0; axis_index < 3; axis_index++) {
			angle_mag = angle_mag + pow((rotate_angle[axis_index]), 2);
		}

		/*
		 * Compute angle magnitude and convert from milli-degrees to degrees
		 */

		angle_mag = sqrt(angle_mag)/1000;

		/*
		 * Compute rotation angle magnitude
		 */


		if (angle_mag >= angle_mag_max_threshold) {
			sprintf(msg1, "\r\nvalues: %f %f %f", rotate_angle[0], rotate_angle[2], rotate_angle[2]);
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			*ttt_1 /= 20;
			*ttt_2 = rotate_angle[0] / 1000;
			*ttt_3 = rotate_angle[1] / 1000;
			*ttt_mag_scale = (int) (angle_mag * 100);

			sprintf(msg1, " \r\n");
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

			sprintf(msg1, "\r\nMotion with Angle Mag of %i degrees complete, Now Return to Next Start Position, ", (int)(angle_mag));
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			BSP_LED_Off(LED1);
			HAL_Delay(3000);
			return;

//			break;
		} else {
			*ttt_2 = 0;
			*ttt_3 = 0;
		}

	}

	/*
	 * The loop above will exit under two conditions:
	 *
	 * 1) The SensorTile has not been moved over a period determined by
	 * MAX_ROTATION_ACQUIRE_CYCLES. Each loop cycle requires 10 milliseconds
	 * set by the integration interval of DATA_PERIOD_MS.  Thus, this is a
	 * period of 4 seconds
	 * 2) The SensorTile has been moved to introduce an orientation angle
	 * change of greater than or equal to angle_mag_max_threshold.
	 *
	 * Thus, this system detects whether in State 1, there is a change
	 * in orientation after State 0 or there is no change in orientation.
	 *
     * Assignment of features including for the third feature, *ttt_3.
	 *
	 * Consider a method similar to that of the Project of Tutorial 12
	 *
	 * Specifically, if angle_mag is greater than or equal to a threshold value, then
	 *  *ttt_3 is sent to the average of ttt_state_0[0] and ttt_state_0[1]
	 *
	 *  And if angle_mag is less than a threshold value, then *ttt_3 is set
	 *  to zero
	 *
	 *  Please also note that the loop above exits when angle_mag is equal to
	 *  angle_mag_max_threshold.  Therefore, angle_mag will not exceed this
	 *  value.  So, the threshold value for setting *ttt_3 should be chosen
	 *  to be just less than angle_mag_max_threshold.  Consider why this is
	 *  required.
	 */

	/*
	 * **************************************************
	 * In the section below, replace the statement, *ttt_3 = 0; with the code
	 * including the conditional described in the comment above
	 */



	/*
	 * ***************************************************
	 */

	*ttt_1 = ttt_state_0[0];  	// The first two features are set to their initial values from State 0
	*ttt_2 = ttt_state_0[1];	// The first two features are set to their initial values from State 0
	*ttt_mag_scale = (int) (angle_mag * 100);

	sprintf(msg1, " \r\n");
	CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

	sprintf(msg1, "\r\nMotion with Angle Mag of %i degrees complete, Now Return to Next Start Position, ", (int)(angle_mag));
	CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
	BSP_LED_Off(LED1);
	HAL_Delay(3000);
	return;
}