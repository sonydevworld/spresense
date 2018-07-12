examples/colorsensor
^^^^^^^^^^^^^^^^^^^^

  This is simple example of take color sensor device using Sensor Control
  Unit. Available as nsh built-in command.
  This example obtains and shows color sensor data in every 1 second. It
  will never finished.

  Supported sensor is Rohm BH1745NUC sensor.

  This application depends on its device and SCU configuration.

  Configuration Pre-requisites:

    CONFIG_BH1745NUC - BH1745NUC
    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

  Example Configuration:

    CONFIG_EXAMPLES_COLORSENSOR - Enable this example
    CONFIG_EXAMPLES_COLORSENSOR_PROGNAME - Program name.
    CONFIG_EXAMPLES_COLORSENSOR_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_COLORSENSOR_STACKSIZE - Example stack size. Default: 2048

  Operation:

    No console operation is needed. Bring an object close to a color
    sensor, then you can see the sampling data.
