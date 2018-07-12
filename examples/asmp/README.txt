
Usage of asmp example
===========================

Usage
---------------------------

Select options in below.

[ASMP] <= Y
  [ASMP shared memory size] = 0x100000

Or use asmp default configuration

$ ./tools/config.py asmp

Build and install
--------------------------

Type 'make' to build SDK.
After that, you can see worker binary 'hello' in directory worker/hello.
If you not set ROMFS file system, then you need to copy it to target board via
USB MSC.
If you set ROMFS file system, then it already contained nuttx binary image
as ROMFS file image.

CAUTION
apps build system cannot build automatically by configuration or/and example
source modification.
Please 'make clean' first.
