
Usage of prime example
===========================

Usage
---------------------------

Configure the sdk to enable the 'prime' application through menuconfig:

make menuconfig
[*] ASMP  --->  Enable ASMP

Examples --->
 [*] Multicore prime number calculator
            (1024) Example worker stack size


Or use the prime default config file:

$ ./tools/config.py examples/prime

Build and install
--------------------------

$ cd sdk
$ tools/config.py --kernel release
$ tools/config.py examples/prime
$ make buildkernel
$ make
$ tools/flash.sh nuttx.spk

Once this is done, this can be simplified into a oneliner:

$ make && tools/flash.sh -b 1152000 nuttx.spk && minicom

This will build, flash and start minicom.
Once in minicom, type 'help' to list all available commands. 'prime' should be listed as a built in app.
Type 'prime' and hit the enter key to start the application.

The default settings for the prime are:

Default number of CPUs to run the task on is: 5
The default range per CPU is set to:           15000

It is possible to try different amount of CPUs to run the task on and also to try custom lenght range specifying:

'prime [N_CPU] [RANGE_LENGTH]'

where N_CPU is number of CPU (Max is 5) and RANGE_LENGTH is the range per CPU to calculate.

Example:
    'prime 3 5000' will run the task on 3 CPUs with where each CPU will calculate range with 5000 numbers in it.
