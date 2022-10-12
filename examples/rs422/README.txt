examplex/rs422
^^^^^^^^^^^^^^

  Example for communicate nodes via RS422. 

  Sample application to link two SPRESENSEs using RS422. 
  There is a loopback mode to verify operation with one
  SPRESENSE and a master/slave mode to link two SPRESENSEs.
  Communication results are displayed in stdout.
 
  The following configuration options can be selected:

  CONFIG_EXAMPLES_RS422 -- rs422 example. 
       Default: n
  CONFIG_EXAMPLES_RS422_RE_PIN_EMMC_DATA3 
  or CONFIG_EXAMPLES_RS422_RE_PIN_I2S_DIN
    -- Read Enable Pin Assign
      Default: "CONFIG_EXAMPLES_RS422_RE_PIN_I2S_DIN"
  CONFIG_EXAMPLES_RS422_RE_PIN_EMMC_DATA3 
  or CONFIG_EXAMPLES_RS422_RE_PIN_I2S_DIN
    -- Data Enable Pin Assign
      Default: "CONFIG_EXAMPLES_RS422_DE_PIN_I2S_DOUT"
  CONFIG_EXAMPLES_RS422_PROGNAME -- this program if change this field.
      Default: "rs422"
  CONFIG_EXAMPLES_RS422_PRIORITY -- Specified this example's task priority.
	  Default: 100
  CONFIG_EXAMPLES_RS422_STACKSIZE -- Specified this example's stack size.
	  Default: 2048

  Build kernel and SDK:

  This application can be used by rs422 default config.

  $ ./tools/config.py examples/rs422
  $ make

  Execute under nsh:

  Type 'rs422' and option on nsh like this.

  If you want to run in loopback mode,
  nsh> rs422 -l

  If you want to run in master/slave mode,
  First execute the following in the slave mode side of the presense
  nsh> rs422 -s

  Next, on the master mode side, in the spresense, do the following
  nsh> rs422 -m

