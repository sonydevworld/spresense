SPRESENSE is the code name of the product CXD5602 produced by Sony Semiconductor
Solutions Corporation.  
The CXD5602 is ARM powered SoC, including many peripherals for IoT and
wearable solutions.
  
SPRESENSE SDK is based on NuttX. So please refer to [original NuttX site](http://www.nuttx.org/) to see basic
kernel information.

This SDK constructed by series of libraies, drivers and architecture specific
code, these helps to you getting started and create faster for your products.


# Directory structure

```
sdk
|-- bsp            - Board support package
|   |-- board      - Board specific code
|   |-- include    - SDK headers
|   |-- scripts    - Linker scripts
|   `-- src        - Common source
|-- configs        - Default configuration files
|   `-- kernel
|-- drivers        - External device drivers
|   |-- lcd
|   `-- sensors
|-- lib            - Kernel and SDK library output directory
|-- modules
|   |-- asmp       - ASMP framework
|   |-- audio      - Audio library
|   |-- fwuputils  - Firmware update utility
|   |-- include    - Library headers
|   |-- sensing    - Smart sensing library
|   |   |-- arm_gesture   - Arm gesture detection
|   |   |-- barometer     - Barometer
|   |   |-- compass       - Compass
|   |   |-- gnss          - GNSS
|   |   `-- tap           - Tap detection
|   `-- skeleton
|-- system         - System commands
`-- tools          - Build utilities
```

# Build instruction

Getting started

```
$ make buildkernel KERNCONF=release
$ tools/config.py default
$ make
```

SDK and NuttX kernel's configuration and build sequence are separated.

# Configuration

SDK provides `tools/config.py` as configuration frontend for SDK and NuttX
kernel configuration. `tools/config.py` invokes kconfig tool so this must
be installed prior it is used.
`tools/config.py` must be called from top of SDK source tree.

`tools/config.py` can be switched with `-k` or `--kernel` option to configure
the kernel.
If no `-k` or `--kernel` option has been specified, then the configuration
affects SDK.

e.g.
```
$ tools/config.py --kernel --list (list kernel predefined configs)
$ tools/config.py --list  (list SDK predefined configs)
```

The first timeyou run SDK setup, or if you want to change to predefined config,
specify predefined `<config name>` as parameter:

```
$ tools/config.py <config name> [<config name>...]
```

You can specify multiple predefines, it allows combination of predefines.

e.g.
```
$ tools/config.py audio gnss
```

`tools/config.py` can be invoked menu style configuration UI by following
options.

```
$ tools/config.py --menuconfig (same as 'make menuconfig')
$ tools/config.py --qconfig    (same as 'make qconfig')
$ tools/config.py --gconfig    (same as 'make gconfig')
```

Additionally, you can use menu invokation with predefined configs like this:

```
$ tools/config.py --menuconfig default
```

## Save your configuration

If you want to save your edited options, then try `tools/mkdefconfig.py`.
It is for saving user configuration as predefined config.

You can save current SDK configuration like this:

```
$ tools/mkdefconfig.py <config name>
```

And also kernel configuration too.

```
$ tools/mkdefconfig.py -k <config name>
```

After that, you can choose their saved default configs to specify `tools/config.py`'s
config name.

If your specified config name is already exists, `tools/mkdefconfig.py` shows
comfirm message to overwrite.

By default, `tools/config.py` and `tools/mkdefconfig.py` find out specified
configurations under `configs` directory.
You can change the default configuration file directory with `-d` or `--dir` option,
both of `tools/config.py` and `tools/mkdefconfig.py` are supported.

If you want to save your customized configuration to where you want, type like this:

```
$ tools/mkdefconfig.py -d ../path/to/configs <config name>
```

When you want to use configurations in alternative directory:

```
$ tools/config.py -d ../path/to/configs <config name>
```

Also they can be use `-k` option with `-d` to saving/restoring kernel
configuration.


# Build

SDK build system has been separated from NuttX, it to be able to build NuttX
kernel indipendently, and build SDK faster.
User must be build kernel before SDK build by following command.

```
$ make buildkernel
```

If you don't interest about kernel sources, then never been built again.
And you can specify kernel default config from `KERNCONF`, configure specified
config and build at the same time.

```
$ make buildkernel KERNCONF=<config name>
```

Finally, build SDK drivers and libraries.

```
$ make
```

After built successfully, you can see `nuttx` and `nuttx.spk` files in top of
SDK source tree.
