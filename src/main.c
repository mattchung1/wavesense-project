// Firmware code for Wavesense device that uses VL53L0X and AMG88xx sensors

/* Portions of this file derived from VL53L0X driver
 *	
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/dt-bindings/pinctrl/nrf-pinctrl.h>

#include <zephyr/drivers/pwm.h>

LOG_MODULE_REGISTER(ECE8873, LOG_LEVEL_INF);

#define LED0_NODE  DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Get the first VL53L0X device from devicetree */
static const struct device *const tof = DEVICE_DT_GET_ANY(st_vl53l0x);

/* Choose your threshold (millimeters) */
#define THRESH_MM 200  /* LED on when distance < 200 mm */

// Define the servo motor parameters
#define SERVO_MOTOR1     DT_NODELABEL(servo1)
static const struct pwm_dt_spec pwm_servo1 = PWM_DT_SPEC_GET(SERVO_MOTOR1);

// Define another PWM for temperature alert
#define SERVO_MOTOR2     DT_NODELABEL(servo2)
static const struct pwm_dt_spec pwm_servo2 = PWM_DT_SPEC_GET(SERVO_MOTOR2);


/* STEP 5.5 - Use DT_PROP() to obtain the minimum and maximum duty cycle */
#define PWM_SERVO_MIN_PULSE_WIDTH    DT_PROP(SERVO_MOTOR1, min_pulse)
#define PWM_SERVO_MAX_PULSE_WIDTH    DT_PROP(SERVO_MOTOR1, max_pulse)
#define PWM_PERIOD   PWM_MSEC(20)
#define PWM_FREQUENCY PWM_MSEC(10)
/* STEP  2.2 - Define minimum and maximum duty cycle */
/* STEP 4.2 - Change the duty cycles for the LED */
#define PWM_MIN_PULSE_WIDTH 0
#define PWM_MAX_PULSE_WIDTH 50000000

/* STEP 2.1 - Create a function to set the angle of the motor */
/* STEP 5.8 - Change set_motor_angle() to use the pwm_servo device */
int set_motor_angle(uint32_t pulse_width_ns)
{
    int err;
    err = pwm_set_dt(&pwm_servo1, PWM_PERIOD, pulse_width_ns);
    if (err) {
        LOG_ERR("pwm_set_dt_returned %d", err);
    }
    return err;
}


// Define for IR Temperature Sensor
#define VL_NODE  DT_COMPAT_GET_ANY_STATUS_OKAY(st_vl53l0x)
#define AMG_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(panasonic_amg88xx)

// Detects if nodes are ok
#if !DT_NODE_HAS_STATUS(VL_NODE, okay)
#error "No enabled VL53L0X node found in devicetree"
#endif

#if !DT_NODE_HAS_STATUS(AMG_NODE, okay)
#error "No enabled AMG88xx node found in devicetree (check overlay and int-gpios)"
#endif

// Structure to hold temp values from AMG
static struct sensor_value temp_value[64];

// Trigger handler for AMG88xx
#ifdef CONFIG_AMG88XX_TRIGGER
K_SEM_DEFINE(sem, 0, 1);

static void trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trigger)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trigger);

	k_sem_give(&sem);
}
#endif

bool isAboveThreshold = false;

/*
************
USER SET VARIABLES
************
*/
int thresholdTemperature = 30; // Temperature threshold in Celsius




/*
************
FUNCTIONS
************
*/
// Function for AMG printing buffer
void print_buffer(void *ptr, size_t l)
{
	struct sensor_value *tv = ptr;
	int ln = 0;

	//printk("---|");
	for (int i = 0; i < 8; i++) {
		//printk("  %02d  ", i);
	}
	//printk("\n");

	// This is value of integer part
	int tempVal1 = tv[0].val1;
	// This is value of fractional / decimal part
	int tempVal2 = tv[0].val2;

	// for loop to print the values in a grid
	//printk("%03d|", ln);
	for (int i = 0; i < l; i++) {

		// Added these for formatting and calculations
		tempVal1 = tv[i].val1;
		tempVal2 = tv[i].val2;

		// Reformatted print to show decimal values
		// First is multiplying integer part by 100 to shift decimal
		// Second is dividing fractional part by 10000 to get two decimal places

		//printk("%05d ", (tempVal1 * 100 + tempVal2 / 10000));
		if (!((i + 1) % 8)) {
			//printk("\n");
			ln++;
			//printk("%03d|", ln);
		}

		// Calculate if any temperature in the grid is over threshold temperature
		if (tempVal1 >= thresholdTemperature)
		{
			isAboveThreshold = true;
		}
	}
	printk("\n");
}



// Function to map distance in mm to frequency for the motor
int mmToFrequency(int input)
{
	// Defining variables. Modify the input and output ranges as needed.
	int output;
	int input_start = 14;
	int input_end = 1200;
	int output_start = PWM_MIN_PULSE_WIDTH;
	int output_end = PWM_MAX_PULSE_WIDTH;

	// Calculating the slope and output value
	double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
	output = output_start + slope * (input - input_start);
    //output = output_start + (input_end - input) * slope;
	return output;			
}


/*
************
MAIN CODE
************
*/
int main(void)
{
	// Initializing devices and variables
	static const struct device *const VL53  = DEVICE_DT_GET(VL_NODE);
	static const struct device *const AMG = DEVICE_DT_GET(AMG_NODE);
	struct sensor_value value;
	int ret;
	int err = 0;
	

	if (!device_is_ready(VL53)) {
		printk("sensor: vl53l0x device not ready.\n");
		//return 0;
	}
	if (!device_is_ready(AMG)) {
		printk("sensor: amg8833 device not ready.\n");
		//return 0;
	}
	if (!device_is_ready(led0.port)) {
        printk("LED0 GPIO not ready\n");
        return 0;
    }

	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

	// ADDED FOR AMG8833
	#ifdef CONFIG_AMG88XX_TRIGGER
	struct sensor_value attr = {
		.val1 = 27,
		.val2 = 0,
	};

	if (sensor_attr_set(AMG, SENSOR_CHAN_AMBIENT_TEMP,
			    SENSOR_ATTR_UPPER_THRESH, &attr)) {
		printk("Could not set threshold\n");
		return 0;
	}

	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_THRESHOLD,
		.chan = SENSOR_CHAN_AMBIENT_TEMP,
	};

	if (sensor_trigger_set(AMG, &trig, trigger_handler)) {
		printk("Could not set trigger\n");
		return 0;
	}
	#endif

	// ADDED FOR PWM
	/* STEP 5.7 - Check if the motor device is ready and set its initial value */
    if (!pwm_is_ready_dt(&pwm_servo1)) {
        LOG_ERR("Error: PWM device %s is not ready", pwm_servo1.dev->name);
        return 0;
    }

	err = pwm_set_dt(&pwm_servo1, PWM_PERIOD, PWM_SERVO_MIN_PULSE_WIDTH);
    if (err) {
        LOG_ERR("pwm_set_dt returned %d", err);
        return 0;
    }




	/*
	************
	MAIN LOOP
	************
	*/
	while (1) {

		// FOR AMG8833
		#ifdef CONFIG_AMG88XX_TRIGGER
		printk("Waiting for a threshold event\n");
		k_sem_take(&sem, K_FOREVER);
		#endif

		// Resetting isAboveThreshold flag
		isAboveThreshold = false;
		

		// FOR VL53
		ret = sensor_sample_fetch(VL53);
		if (ret) {
			printk("sensor_sample_fetch failed ret %d\n", ret);
			return 0;
		}
		
		ret = sensor_channel_get(VL53, SENSOR_CHAN_PROX, &value);
		//printk("prox is %d\n", value.val1);


		ret = sensor_channel_get(VL53,
					 SENSOR_CHAN_DISTANCE,
					 &value);
		//printf("distance is %.3fm\n", sensor_value_to_double(&value));

		// *** Adding code for eceClass ***
		// Turning on LED if object is close
		if (sensor_sample_fetch(tof) == 0) {
            struct sensor_value dist;

            if (sensor_channel_get(tof, SENSOR_CHAN_DISTANCE, &dist) == 0) {
                /* Convert Zephyr sensor_value to millimeters */
                /* dist is in meters; val1=int part, val2=micro part */
                double meters = sensor_value_to_double(&dist);
                int mm = (int)(meters * 1000.0 + 0.5);
				printk("Distance: %d mm\n", mm);

				int convertedF = mmToFrequency(mm);
				//int calculatedPulseWidthNS = mmToFrequency(mm);
				int calculatedPulseWidthNS = (20000000 - convertedF);


				if(calculatedPulseWidthNS < 0)
				{
					calculatedPulseWidthNS = 0;
				}
				pwm_set_dt(&pwm_servo1, PWM_PERIOD, calculatedPulseWidthNS);
				printk("Object output high time of %dns \nDuty Cycle: %d\n", calculatedPulseWidthNS, (calculatedPulseWidthNS*100)/20000000);

                /* If/else threshold */
				// if (mm > 0 && mm < THRESH_MM) {
				// 	printk("Object detected within %d mm\n", THRESH_MM);
				// 	pwm_set_dt(&pwm_servo, PWM_PERIOD, calculatedFrequency);
				// }
				// else{
				// 	pwm_set_dt(&pwm_servo, PWM_PERIOD, 0);
				// }

                //gpio_pin_set_dt(&led0, (mm > 0 && mm < THRESH_MM) ? 1 : 0);

				//set_motor_angle(PWM_SERVO_MAX_PULSE_WIDTH);
				
            }
        }

		// FOR AMG8833

		#ifdef CONFIG_AMG88XX_TRIGGER
		printk("Waiting for a threshold event\n");
		k_sem_take(&sem, K_FOREVER);
		#endif

		// continuing to use same ret int variable that we used for VL53
		ret = sensor_sample_fetch(AMG);

		if (ret) {
			printk("Failed to fetch a sample, %d\n", ret);
			//return 0;
		}

		ret = sensor_channel_get(AMG, SENSOR_CHAN_AMBIENT_TEMP,
					 (struct sensor_value *)temp_value);
		if (ret) {
			printk("Failed to get sensor values, %d\n", ret);
			//return 0;
		}

		// Calling print buffer to print the temperature values in an array
		//printk("new sample:\n");
		print_buffer(temp_value, ARRAY_SIZE(temp_value));

		//Turning on PWM if temperature is above threshold
		if(isAboveThreshold)
		{
			pwm_set_dt(&pwm_servo2, PWM_PERIOD, PWM_SERVO_MAX_PULSE_WIDTH);
			printk("Temperature above threshold! Activating alert servo.\n");
		}
		else
		{
			pwm_set_dt(&pwm_servo2, PWM_PERIOD, 0);
		}



		k_sleep(K_MSEC(100));

	}
	return 0;
}


