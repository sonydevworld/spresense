
TARGET = cxd5602

ifeq ($(shell uname -s),Linux)
SVDCONV = ../../../externals/cmsis/CMSIS_5/CMSIS/Utilities/Linux-gcc-4.8.3/SVDConv
else
SVDCONV = ../../../externals/cmsis/CMSIS_5/CMSIS/Utilities/Win32/SVDConv.exe
endif

all: $(TARGET).svd

%.svd: %.xml
	xmllint --xinclude --format -o $@ $<
	@ -$(SVDCONV) $@

$(TARGET).svd: $(wildcard *.xml)
