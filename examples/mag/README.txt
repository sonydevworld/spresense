examples/mag
^^^^^^^^^^^^

  This is simple example of take magnetometer sensor device using Sensor
  Control Unit. Available as nsh built-in command.
  This example obtains and shows magnetic sensor data in every 1 second. It
  will never finished.

  Supported sensors are:

  - AsahiKASEI AK09912 sensor
  - Rohm BM1422GMV sensor

  This application depends on these devices and SCU configuration, but each
  sensors can't be used on the same time.

  Configuration Pre-requisites:

    CONFIG_AK09912   - AK09912
    CONFIG_BM1422GMV - BM1422GMV
    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

  Example Configuration:

    CONFIG_EXAMPLES_MAG - Enable magnetometer sensor example
    CONFIG_EXAMPLES_MAG_PROGNAME - Program name.
    CONFIG_EXAMPLES_MAG_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_MAG_STACKSIZE - Example stack size. Default: 2048

  Operation:

    No console operation is needed. Take your board and move it, then
    you can see the value of synchronized with your move.
