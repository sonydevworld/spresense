bt_hfp_hf
===========

How to build
-----------------------------
$ cd spresense/sdk
$ ./tools/config.py examples/bt_hfp_hf
$ make buildkernel
$ make
$ ./tools/flash.sh nuttx.spk

How to run
-----------------------------
1. rename HFP supported smart phone device name as "SONY_BT_HFP_HF_SAMPLE"

2. execute under nsh:
nsh> bt_hfp_hf
the example will search for the device "SONY_BT_HFP_HF_SAMPLE" and try to connect to it, make sure only one device named like this in your environment.

3. nsh console output could see HFP HF status changes

4. input enter key into hfp_hf shell after initialising BT device
hfp_hf>

5. input 'connectHf' command to connect the peer deivce with HFP profile after binding BT device
hfp_hf>connectHf
