examples/press
^^^^^^^^^^^^^^

  This is a simple example of take pressure sensor using Sensor Control Unit.
  Available as nsh built-in command.
  This example obtains and shows pressure sensor data in every 1 second. It
  will never finished.

  Supported sensors are:

  - Bosch BMP280 Barometic Pressure Sensor
  - Rohm BM1383GLV Pressure Sensor

  This application depends on its device and SCU configuration, but each
  sensors can't be used on the same time.

  Configuration Pre-requisites:

    CONFIG_BMP280    - BMI280
    CONFIG_BM1383GLV - BM1383GLV
    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

  Configuration in this example:

    CONFIG_EXAMPLES_PRESS - Enable this example
    CONFIG_EXAMPLES_PRESS_PROGNAME - Program name
    CONFIG_EXAMPLES_PRESS_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_PRESS_STACKSIZE - Example stack size. Default: 2048
    CONFIG_EXAMPLES_PRESS_DEVNAME - Pressure sensor device path.
      Default: "/dev/press0"
    CONFIG_EXAMPLES_PRESS_SIGNO - Signal number to get watermark signal
      from driver. Default: 14

  Configuration in this example when using BMP280:

    CONFIG_EXAMPLES_TEMP_DEVNAME - Temperature sensor device path.
      Default: "/dev/temp0"
    CONFIG_EXAMPLES_TEMP_SIGNO - Signal number ot get watermark signal
      from temperature driver. Default: 15

  Operation:

    No console operation is needed.

  Bosch BMP280 Specific information:

    When using BMP280 device, this example take pressure and temperature data,
    and conpensate their values. See BMP280 device specification for detail.
