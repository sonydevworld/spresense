# Examples / `ambient_cli` Ambient CLI application

## Description

The `ambient_cli` is an application to demonstrate uploading any data to the
Ambient service. Ambient is an IoT data visualization service. It receives,
stores, and visualizes sensor data sent from microcontrollers and other devices.

https://ambidata.io/

To use Ambient service, you need to register as a user.

Please see below for more details.

https://ambidata.io/docs/gettingstarted/

The procedure to use the Ambient service is as below:

1. User registration (free of charge)
2. Channel generation
3. Microcontroller-side programming
4. Data transmission
5. Visualization (graphing)

## How to configuration and build

This application uses an Ambient library and any network feature.
The explanation shows an example using LTE network.

```bash
$ cd spresense/sdk
$ ./tools/config.py feature/lte examples/ambient_cli
$ make
```

## Running this application


The first step is to connect to the LTE network.

```bash
*** Usage ***
nsh> lte_sysctl
lte_sysctl: missing required argument(s)

USAGE: lte_sysctl command
 [-a <apn_name>] [-i <ip_type>] [-v <auth_type>] [-u <user_name>] [-p <password>] [-r <rat_type>] start
  -a: APN name
  -i: IP type 0=IPv4, 1=IPv6, 2=IPv4 and IPv6, 3=Non-IP
  -v: Authenticaion type 0=NONE, 1=PAP, 2=CHAP
  -u: User name for authenticaion
  -p: Password for authenticaion
  -r: Radio Access Technology type M1=CAT-M1, NB=NB-IoT
 stop
 stat
 [-h]: Show this message

*** Specify the appropriate parameters based on your SIM and start ***
nsh> lte_sysctl -a <apn_name> -u <user_name> -p <password> start

*** Enable the network interface ***
nsh> ifup eth0
ifup eth0...OK

*** You can confirm if your board is connected to the network ***
nsh> ifconfig
```

The next step uses the channel ID and write key you generated on the Ambient server.

```bash
*** Usage ***
nsh> ambient_cli -h
Usage: ambient_cli [-c <channel>] [-w <write_key>] field=data ...
Options:
  -c: channel ID
  -w: write key string
  field=data: field is 1~10. data is string to be sent.
```

For example, it shows a command in the following cases.
- channelID : 123456
- write_key : 1234567890123456
- send data (field, value) : (1, 0.55), (2, 0.67)


```bash
nsh> ambient_cli -c 123456 -w 1234567890123456 1=0.55 2=0.67
```

You can edit an `ambient_cli_main.c` and pre-set your own channel and write key.
Then you do not need to specify `-c` and `-w` options of the `ambient_cli` arguments.

```c
#define MY_CHANNEL  123456
#define MY_WRITEKEY "1234567890123456"
```

```bash
nsh> ambient_cli 1=0.55 2=0.67
```

Finally, you will view the sent data on the Ambient server.

