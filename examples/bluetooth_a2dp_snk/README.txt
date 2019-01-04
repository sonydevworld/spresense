bt_a2dp_snk
===========

How to build
-----------------------------
$ cd spresense/sdk
$ ./tools/config.py examples/bt_a2dp_snk
$ make buildkernel
$ make
$ ./tools/flash.sh nuttx.spk

How to run
-----------------------------
1. rename A2DP supported smart phone device name as "SONY_BT_A2DP_SNK_SAMPLE"

2. execute under nsh:
nsh> bt_a2dp_snk

3. Please connect the device named "SONY_BT_A2DP_SNK_SAMPLE" from the peer device, and then you can call avrcSendCommand to do some operation, such as "Play", "Pause", "Stop".
