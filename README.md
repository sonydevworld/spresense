[![twitter spresense handle][]][twitter spresense badge]
[![twitter devworld handle][]][twitter devworld badge]

# Welcome to SPRESENSE project

Clone this repository and update submodules.

```
$ git clone --recursive https://github.com/sonydevworld/spresense.git
```

# Submodules

```
spresense                  - This repository
|-- nuttx                  - NuttX original kernel + SPRESENSE port
|-- sdk
|   `-- apps               - NuttX original application + SPRESENSE port
`-- externals
    `-- nnablart
      `-- nnabla-c-runtime - Neural Network Runtime library
```

# Spresense SDK build instructions

Build instructions are documented at [Spresense SDK Getting Started Guide](https://developer.sony.com/develop/spresense/docs/sdk_set_up_en.html).

## Prerequisites

Install the necessary packages and GCC ARM toolchain for cross-compilation.
```
$ wget https://raw.githubusercontent.com/sonydevworld/spresense/master/install-tools.sh
$ bash install-tools.sh
```

## Build

Go to the folder where you cloned the {SDK_FULL}, and enter the `sdk` folder name:
``` bash
$ cd spresense/sdk
```
Set up the SDK configuration
``` bash
$ tools/config.py examples/hello
```
Build the example image:
``` bash
$ make
```

A `nuttx.spk` file appears in the `sdk` folder when this step has successfully finished.
This file is the final result and can be flashed into the your board.

# Using docker

A pre-compiled docker container is available with all the pre-requisite that is needed in order to build the Spresense SDK.

In order to start using it simply type:

```
$ source spresense_env.sh
```

This script will create an alias `spresense` which should proceed the regular SDK build scripts and Make commands.

Examples:
```
SpresenseSDK: $ spresense tools/config.py examples/hello
SpresenseSDK: $ spresense make
```

[twitter spresense handle]: https://img.shields.io/twitter/follow/SpresensebySony?style=social&logo=twitter
[twitter spresense badge]: https://twitter.com/intent/follow?screen_name=SpresensebySony
[twitter devworld handle]: https://img.shields.io/twitter/follow/SonyDevWorld?style=social&logo=twitter
[twitter devworld badge]: https://twitter.com/intent/follow?screen_name=SonyDevWorld
