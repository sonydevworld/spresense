examples/gyro
^^^^^^^^^^^^^^

  This is a simple example of take gyroscope sensor using Sensor Control Unit.
  Available as nsh built-in command.
  This example obtains and shows gyroscope sensor data in every 1 second. It
  will never finished.

  Supported sensor is Bosch BMI160 sensor.

  This application depends on its device and SCU configuration.

  Configuration Pre-requisites:

    CONFIG_BMI160    - BMI160
    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

  Example Configuration:

    CONFIG_EXAMPLES_GYRO - Enable this example
    CONFIG_EXAMPLES_GYRO_PROGNAME - Program name
    CONFIG_EXAMPLES_GYRO_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_GYRO_STACKSIZE - Example stack size. Default: 2048

  Operation:

    No console operation is needed. Take your board and tilt it, then
    you can see the value of synchronized with your move.
