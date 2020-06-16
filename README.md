# Spresense SDK repository

Clone this repository and update submodules.

```
$ git clone --recursive https://github.com/sonydevworld/spresense.git
```

After repositories have been cloned, each submodule is in 'Detached HEAD' state.
Master branch have to be checked out before you can start developing.

```
$ git submodule foreach git checkout master
```

# Submodules

```
.
├── examples  - Spresense SDK examples
├── nuttx     - NuttX original kernel source + CXD5602 port.
└── sdk       - Spresense SDK source and PC tools.
 └─ apps      - NuttX original tools and sample applications.
```

# Using docker

A pre-compiled docker container is available with all the pre-requisite that is needed in order to build the Spresense SDK.

In order to start using it simply type:

```
$ source spresense_env.sh
```

This script will create an alias `spresense` which should preceed the regular SDK build scripts and Make commands.
examples
```
SpresenseSDK: $ spresense make
```

# Spresense SDK build instructions

## Prerequisites

Install the necessary packages and GCC ARM toolchain for cross-compilation.
```
$ sudo apt-get install git gperf libncurses5-dev flex bison gcc-arm-none-eabi
```
Install the *kconfig-frontends* package from [nuttx-tools](https://bitbucket.org/nuttx/tools.git).
``` bash
$ git clone -n https://patacongo@bitbucket.org/nuttx/tools.git tools
$ cd tools
$ git config core.autocrlf false
$ git checkout 891510d2534287afef6b5c90a375f94442a7e94d
$ cd kconfig-frontends/
$ ./configure
$ make
$ sudo make install
$ sudo ldconfig
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
