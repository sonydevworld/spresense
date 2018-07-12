examples/light
^^^^^^^^^^^^^^

  This is simple example of take light sensor device using Sensor Control
  Unit. Available as nsh built-in command.
  This example obtains and shows light sensor data in every 1 second. It
  will never finished.

  Supported sensors are:

  - Avago APDS-9930 sensor
  - muRata LT-1PA01 sensor
  - Rohm BH1721FVC sensor
  - Rohm PRP-0521RS sensor

  This application depends on these devices and SCU configuration, but each
  sensors can't be used on the same time.

  Configuration Pre-requisites:

    CONFIG_APDS9930  - APDS-9930
    CONFIG_LT1PA01   - LT-1PA01
    CONFIG_BH1721FVC - BH1721FVC
    CONFIG_RPR0521RS - PRP-0521RS
    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

  Example Configuration:

    CONFIG_EXAMPLES_LIGHT - Enable light sensor example
    CONFIG_EXAMPLES_LIGHT_PROGNAME - Program name.
    CONFIG_EXAMPLES_LIGHT_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_LIGHT_STACKSIZE - Example stack size. Default: 2048

  Operation:

    No console operation is needed. Bring an object close to a sensor,
    then you can see the sampling data.
