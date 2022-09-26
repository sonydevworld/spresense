examplex/i2c_direct
^^^^^^^^^^^^^^^^^^^

  Example for using i2c_direct with BMI270 

  Sample application to control I2C-connected sensors
  by directly manipulating /dev/i2cx.
  This sample controls a BMI270 Bosch 6-axis IMU, taking
  512 measurements and displaying acceleration and angular
  rate of rotation. It also displays the tilt using LEDs
  on the SPRESENSE Main Board.
 
  The following configuration options can be selected:

  CONFIG_EXAMPLES_I2C_DIRECT -- i2c_direct example. 
       Default: n
  CONFIG_EXAMPLES_I2C_DIRECT_PROGNAME -- You can choice other name for
    this program if change this field. Default: "i2c_direct"
  CONFIG_EXAMPLES_I2C_DIRECT_PRIORITY -- Specified this example's task priority.
	  Default: 100
  CONFIG_EXAMPLES_I2C_DIRECT_STACKSIZE -- Specified this example's stack size.
	  Default: 2048
  CONFIG_EXAMPLES_I2C_DIRECT_DEBUG_ON -- Toggle with or without debug display.
    Default: n
  CONFIG_EXAMPLES_I2C_DIRECT_DPRINT_DEBUG -- Toggle with or without DEBUG display.
    Default: n
  CONFIG_EXAMPLES_I2C_DIRECT_DPRINT_TRACE -- Toggle with or without TRACE display.
    Default: n
  CONFIG_EXAMPLES_I2C_DIRECT_DPRINT_INFO -- Toggle with or without INFO display.
    Default: n
  CONFIG_EXAMPLES_I2C_DIRECT_DPRINT_WARN -- Toggle with or without WARN display.
    Default: n
  CONFIG_EXAMPLES_I2C_DIRECT_DPRINT_ERROR -- Toggle with or without ERROR display.
    Default: n

  Build kernel and SDK:

  This application can be used by i2c_direct default config.

  $ ./tools/config.py examples/i2c_direct
  $ make

  Execute under nsh:

  Type 'i2c_direct' on nsh like this.
  nsh> i2c_direct
