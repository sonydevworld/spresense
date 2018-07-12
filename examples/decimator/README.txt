examples/decimator
^^^^^^^^^^^^^^^^^^

  This is simple example of decimator in Sensor Control Unit. Available as
  nsh built-in command.

  Supported sensors are decimator supported devices. See below option menu.

  [CXD56xx Configuration Options]
    [Sensor Control Unit Support]
      [SCU Decimator assignments]
        [BMI160 Gyroscope to decimator]
        [BMI160 Accelerometer to decimator]
        [AK09912 Magnetometer to decimator]
        [KX022 Accelerometer to decimator]
        [BM1422GMV Magnetometer to decimator]

  This application depends on these devices and SCU configuration.

  User can be choose the using sensor device by sensor driver, decimator
  assignments and CONFIG_EXAMPLES_DECIMATOR[01]_DEVNAME.
  For example, to use BMI160 accelerometer to assign decimator, DEVNAME will
  be "/dev/accel0" and "/dev/accel1", for decimator 0 and 1 respectively.

  Configuration Pre-requisites:

    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

    And assign sensor to decimator listed in above.

  Example Configuration:

    CONFIG_EXAMPLES_DECIMATOR - Enable decimator sensor example
    CONFIG_EXAMPLES_DECIMATOR_PROGNAME - Program name.
    CONFIG_EXAMPLES_DECIMATOR_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_DECIMATOR_STACKSIZE - Example stack size. Default: 2048
    CONFIG_EXAMPLES_DECIMATOR0_DEVNAME  - Device name of decimator 0
    CONFIG_EXAMPLES_DECIMATOR0_SIGNO    - Signal number of decimator 0
    CONFIG_EXAMPLES_DECIMATOR1_DEVNAME  - Device name of decimator 0
    CONFIG_EXAMPLES_DECIMATOR1_SIGNO    - Signal number of decimator 0

  Operation:

    Run 'decimator' program from nsh. This example shows sampling data from
    decimator 0/1 in every seconds.
    Each decimators are set in different decimation ratio, As a result,
    decimator 1 will be half number of samples from decimator 0.
