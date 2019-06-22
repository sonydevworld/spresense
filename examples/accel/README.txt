examples/accel
^^^^^^^^^^^^^^

  This is simple example of take accelerometer sensor device using Sensor
  Control Unit. Available as nsh built-in command.
  This example using a timer and take sensing data periodically (500ms) and
  shows number of samples in every 500ms. And show the latest read data
  when program finished.

  Supported sensors are:

  - Bosch BMI160 sensor
  - Kionix KX022/KX122 acceleration sensor

  This application depends on these devices and SCU configuration, but each
  sensors can't be used on the same time.

  Configuration Pre-requisites:

    CONFIG_BMI160    - BMI160
    CONFIG_KX022     - KX022
    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

  Example Configuration:

    CONFIG_EXAMPLES_ACCEL - Enable accelerometer sensor example
    CONFIG_EXAMPLES_ACCEL_PROGNAME - Program name.
    CONFIG_EXAMPLES_ACCEL_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_ACCEL_STACKSIZE - Example stack size. Default: 2048

  Operation:

    Run this application from nsh. This captures the acceleration sensor data
    between a second period 10 times and displays the last caputured data.
