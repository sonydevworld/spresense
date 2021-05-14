# Makefile for compiling flash_writer and xmodem_writer

UNAME := $(shell uname -o 2>/dev/null)
PYINSTALLER := pyinstaller

SRCDIR := scripts

# Detect host platform
ifeq ($(UNAME),GNU/Linux)
  OUT := ../linux
  EXT := .exe
else ifeq ($(UNAME),)
  OUT := ../macos
  EXT :=
else
  OUT := ../windows
  EXT := .exe
endif

hide := @

OPT := --onefile

all: $(OUT)/flash_writer$(EXT) $(OUT)/xmodem_writer$(EXT)

# compile flash_writer source code
$(OUT)/%: $(OUT)
	$(hide) $(PYINSTALLER) $(OPT) $(SRCDIR)/$(basename $(notdir $@)).py
	$(hide) mv dist/* $(OUT)/
	$(hide) rm -rf build dist *.spec

# Create output directory
$(OUT):
	$(hide) mkdir -p $(OUT)
