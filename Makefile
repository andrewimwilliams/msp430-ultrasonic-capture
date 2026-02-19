MCU = msp430fr5994
CC = msp430-elf-gcc

# Force temp dir for Windows-native gcc
TMPDIR_WIN = C:\Users\Andrew Williams\AppData\Local\Temp

# Support files (headers + linker scripts)
SUPPORT_INC    = /c/toolchains/msp430-gcc-support-files/include
SUPPORT_LD_DIR = /c/toolchains/msp430-gcc-support-files/include

CFLAGS  = -mmcu=$(MCU) -Wall -O2 -I"$(SUPPORT_INC)"
LDFLAGS = -Wl,-L"$(SUPPORT_LD_DIR)"

DSLite_EXE = /c/ti/uniflash_9.4.1/deskdb/content/TICloudAgent/win/ccs_base/DebugServer/bin/DSLite.exe
CCXML      = tools/MSP430FR5994.ccxml
ELF        = firmware.elf

TARGET = firmware
OBJ = build/main.o

all: $(TARGET).elf

build:
	mkdir -p build

$(OBJ): src/main.c | build
	TEMP="$(TMPDIR_WIN)" TMP="$(TMPDIR_WIN)" $(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJ)
	TEMP="$(TMPDIR_WIN)" TMP="$(TMPDIR_WIN)" $(CC) -mmcu=$(MCU) $^ $(LDFLAGS) -o $@ -lgcc

flash: $(TARGET).elf
	"$(DSLite_EXE)" flash --config="$(CCXML)" --flash --verify "$(ELF)" --reset=0 --run

clean:
	rm -rf build *.elf
