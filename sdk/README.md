SPRESENSE is the code name of the product CXD5602 produced by Sony Semiconductor
Solutions Corporation.  
The CXD5602 is ARM powered SoC, including many peripherals for IoT and
wearable solutions.
  
SPRESENSE SDK is based on NuttX. So please refer to [original NuttX site](https://nuttx.apache.org/) to see basic
kernel information.

This SDK is constructed by series of libraries, drivers and architecture specific
code, and it will help you get started and create faster for your products.


# Directory structure

```
sdk
|-- configs        - Pre-defined configuration files
|   |-- default    - Default configuration file
|   |-- device     - Device configuration files
|   |-- feature    - Feature configuration files
|   `-- examples   - Examples configuration files
|-- modules
|   |-- asmp           - ASMP framework
|   |-- audio          - Audio library
|   |-- bluetooth      - Bluetooth library
|   |-- digital_filter - Digital filter library
|   |-- dnnrt          - DNN Runtime library
|   |-- fwuputils      - Firmware update utility
|   |-- include        - Library headers
|   |-- mpcomm         - Multi processor communication framework
|   |-- sensing        - Smart sensing library
|   `-- tflmrt         - TensorFlow Lite for Microcontrollers Runtime library
|-- system         - System commands
`-- tools          - Build utilities
```

# Build instruction

Please see [Spresense SDK Getting Started Guide](https://developer.sony.com/develop/spresense/docs/sdk_set_up_en.html) for more details.

```
$ tools/config.py default
$ make
```


# Configuration

SDK provides `tools/config.py` as configuration frontend for SDK and NuttX
kernel configuration. `tools/config.py` invokes kconfig tool so this must
be installed prior it is used.
`tools/config.py` must be called from top of SDK source tree.

e.g.
```
$ tools/config.py --list  (list SDK predefined configs)
```

The first time you run SDK setup, or if you want to change to predefined config,
specify predefined `<config name>` as parameter:

```
$ tools/config.py <config name> [<config name>...]
```

You can specify multiple predefines, it allows combination of predefines.

e.g.
```
$ tools/config.py examples/audio_player examples/gnss
```

`tools/config.py` can be invoked menu style configuration UI by following
options.

```
$ tools/config.py --menuconfig (same as 'make menuconfig')
$ tools/config.py --qconfig    (same as 'make qconfig')
$ tools/config.py --gconfig    (same as 'make gconfig')
```

Additionally, you can use menu invocation with predefined configs like this:

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

After that, you can choose their saved default configs to specify `tools/config.py`'s
config name.

If your specified config name already exists, `tools/mkdefconfig.py` shows
a confirm message to overwrite.

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


# Build

Build the NuttX and SDK source code by following the above configuration.

```
$ make
```

To increase build speed, you can pass the `-jN` flag to `make`,
where `N` is the number of parallel jobs to run simultaneously.

After built successfully, you can see `nuttx` and `nuttx.spk` files in top of
SDK source tree.
