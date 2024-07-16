# Create sysutil

## How to create sysutil

1. Configure SDK with `feature/sysutil`

```bash
$ cd sdk
$ ./tools/config.py feature/sysutil
```

2. Build SDK
. Create sysutil firmware binary by following command

```bash
$ make
$ ../nuttx/tools/cxd56/mkspk -c2 nuttx sysutil sysutil.spk
```

## Update startup script (Optional)

NOTE: This sequence expects already configured in `feature/sysutil`.

1. Edit `rcS.template`
2. During building, generate `etc_romfs.c` and compile it.
