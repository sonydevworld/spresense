# flash_writer/Makefile

UNAME := $(shell uname -o 2>/dev/null)
PYINSTALLER := pyinstaller
SRC := scripts/flash_writer.py

# Detect host platform
ifeq ($(UNAME),GNU/Linux)
  OUT := ../linux
else ifeq ($(UNAME),)
  OUT := ../macos
else
  OUT := ../windows
endif

hide := @

OPT := --onefile

# compile target source code
all: $(OUT)
	$(hide) $(PYINSTALLER) $(OPT) $(SRC)
	$(hide) mv dist/* $(OUT)/
	$(hide) rm -rf build dist *.spec

# Create output directory
$(OUT):
	$(hide) mkdir -p $(OUT)
