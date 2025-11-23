Overview
********

This sample periodically measures distance between vl53l0x sensor
and target. The sample also periodically measures surface temperature 
of the object using the amg88xx sensor. The user is meant to wave 
their hand in order to detect objects and subsequently receive a 
haptic feedback proportional to the distance from the user to the 
target. They also receive a separate haptic feedback signal for when 
detecting an object that would be too hot to touch. This is a 
reconfigurable portion of the device

Both the hardware and firmware were made in collaboration with 
Andrew Liu and Matt Chung.


Requirements
************

This firmware sample requires the use of the nordic library. These can be
installed from a VS Code extension:
https://docs.nordicsemi.com/bundle/nrf-connect-vscode/page/get_started/quick_setup.html

The repository that the above extension can be found here:
https://github.com/zephyrproject-rtos/zephyr/tree/main/samples

In order to use the sample, an board with a VL53L0X and AMG88xx connected by
I2C to an ISP is required. Two outputs from the ISP should be PWM signals running
to 2 different H-bridges connected to their own motors.

The firmware code uses sample header files from Zephyr and this file should be 
placed in the directory:
~/ncs/v2.9.0/zephyr/wavesense-firmware-amg88xx-vl53

References
**********

 - VL53L0X: https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html
 - AMG88xx: https://learn.adafruit.com/adafruit-amg8833-8x8-thermal-camera-sensor/overview


Sample Output
=============

 .. code-block:: console


  00> 
  00> Distance: 2137 mm
  00> Object output high time of 0ns 
  00> Duty Cycle: 0
  00> new sample:
  00> 
  00> Distance: 2093 mm
  00> Object output high time of 0ns 
  00> Duty Cycle: 0
  00> new sample:
  00> 
  00> Temperature above threshold! Activating alert servo.
  00> Distance: 2107 mm
  00> Object output high time of 0ns 
  00> Duty Cycle: 0
  00> new sample:
  00> 
  00> Temperature above threshold! Activating alert servo.
  00> Distance: 116 mm
  00> Object output high time of 15699832ns 
  00> Duty Cycle: 78
  00> new sample:
  00> 
  00> Temperature above threshold! Activating alert servo.
  00> Distance: 60 mm
  00> Object output high time of 18060709ns 
  00> Duty Cycle: 90
  00> new sample:
  00> 


