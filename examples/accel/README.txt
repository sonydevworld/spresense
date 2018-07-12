examples/accel
^^^^^^^^^^^^^^

  This is simple example of take accelerometer sensor device using Sensor
  Control Unit. Available as nsh built-in command.
  This example using a timer and take sensing data periodically (500ms) and
  shows number of samples in every 500ms. And show the latest read data
  when program finished.

  Supported sensors are:

  - Bosch BMI160 sensor
  - Kionix KX022 acceleration sensor

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

    Program will be stop when hit space to console.
    If you can't stop this example, keep pressing the space until finished.
    Or enable CONFIG_SCHED_WAITPID to run as foreground task.
