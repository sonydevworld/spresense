examples/proximity
^^^^^^^^^^^^^^^^^^

  This is simple example of take proximity sensor device using Sensor
  Control Unit. Available as nsh built-in command.
  In this example, there are two cases depending on sensor interrupt
  configuration.

  - Obtains and shows proximity sensor raw data in every 1 second. It
    will never finished.
    When sensor interrupt configuration is invalid.

  - Using a timer and refer to interrupt status periodically. When
    the status has changed, shows current status.
    When sensor interrupt configuration is valid.

  Supported sensors are:

  - Avago APDS-9930 sensor
  - muRata LT-1PA01 sensor
  - Rohm PRP-0521RS sensor

  This application depends on these devices and SCU configuration, but each
  sensors can't be used on the same time.

  Configuration Pre-requisites:

    CONFIG_APDS9930  - APDS-9930
    CONFIG_APDS9930_PROXIMITY_INTERRUPT
    CONFIG_LT1PA01   - LT-1PA01
    CONFIG_LT1PA01_PROXIMITY_INTERRUPT
    CONFIG_RPR0521RS - PRP-0521RS
    CONFIG_RPR0521RS_PROXIMITY_INTERRUPT
    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

  Example Configuration:

    CONFIG_EXAMPLES_PROXIMITY - Enable proximity sensor example
    CONFIG_EXAMPLES_PROXIMITY_PROGNAME - Program name.
    CONFIG_EXAMPLES_PROXIMITY_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_PROXIMITY_STACKSIZE - Example stack size. Default: 2048

  Operation:

    No console operation is needed. Bring an object close to a sensor,
    then you can see the sampling data or current status.
