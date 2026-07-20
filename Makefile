##############################################################################
#  Makefile – Vehicle Safety System Firmware
#  Target:   ATmega328P @ 16 MHz (Arduino UNO / standalone)
#  Toolchain: AVR-GCC + avrdude
#  Usage:
#    make           – Build firmware (ELF + HEX)
#    make flash     – Flash via USBasp programmer
#    make flash_uno – Flash via Arduino bootloader (USB, adjust PORT)
#    make clean     – Remove build artifacts
#    make size      – Show memory usage
##############################################################################

# ── Project settings ─────────────────────────────────────────────────────────
PROJECT   = vehicle_safety_system
MCU       = atmega328p
F_CPU     = 16000000UL
BAUD      = 9600

# ── Toolchain ─────────────────────────────────────────────────────────────────
CC        = avr-gcc
OBJCOPY   = avr-objcopy
SIZE      = avr-size
AVRDUDE   = avrdude

# ── Programmer (USBasp) ──────────────────────────────────────────────────────
PROGRAMMER    = usbasp
PROGRAMMER_PORT =          # Leave blank for USBasp (USB direct)

# ── Arduino bootloader programmer (uncomment if flashing via Arduino UNO) ────
# PROGRAMMER     = arduino
# PROGRAMMER_PORT = COM3     # Change to your COM port (Windows) or /dev/ttyUSBx

# ── Compiler flags ────────────────────────────────────────────────────────────
CFLAGS  = -mmcu=$(MCU)
CFLAGS += -DF_CPU=$(F_CPU)
CFLAGS += -Os                    # Optimize for size
CFLAGS += -std=c99               # C99 standard
CFLAGS += -Wall                  # All warnings
CFLAGS += -Wextra                # Extra warnings
CFLAGS += -Wshadow               # Warn on shadowed variables
CFLAGS += -Wstrict-prototypes    # Strict function prototypes
CFLAGS += -ffunction-sections    # Place each function in own section
CFLAGS += -fdata-sections        # Place each data in own section
CFLAGS += -I.                    # Headers in project root

# ── Linker flags ──────────────────────────────────────────────────────────────
LDFLAGS  = -mmcu=$(MCU)
LDFLAGS += -Wl,--gc-sections     # Remove unused sections
LDFLAGS += -Wl,-Map=$(BUILD)/$(PROJECT).map
LDFLAGS += -lm                   # Link math library (for sqrtf, fabsf in mpu6050.c)

# ── Source files ──────────────────────────────────────────────────────────────
# NOTE: All source files are in the project root (flat layout)
SRCS  = main.c
SRCS += uart.c
SRCS += lcd_i2c.c
SRCS += gsm_sim800l.c
SRCS += gps_neo6m.c
SRCS += ultrasonic.c
SRCS += mpu6050.c
SRCS += mq3.c
SRCS += dht11.c
SRCS += ir_sensor.c
SRCS += ldr.c
SRCS += collision.c
SRCS += alerts.c

# ── Build directory ───────────────────────────────────────────────────────────
BUILD = build

# ── Derived file lists ────────────────────────────────────────────────────────
OBJS = $(addprefix $(BUILD)/,$(SRCS:.c=.o))
ELF  = $(BUILD)/$(PROJECT).elf
HEX  = $(BUILD)/$(PROJECT).hex

# ── Default target ────────────────────────────────────────────────────────────
.PHONY: all
all: $(HEX) size

# ── Create build directory ────────────────────────────────────────────────────
$(BUILD):
	mkdir -p $@

# ── Compile each .c to .o ─────────────────────────────────────────────────────
$(BUILD)/%.o: %.c | $(BUILD)
	@echo "  CC  $<"
	$(CC) $(CFLAGS) -c $< -o $@

# ── Link ELF ──────────────────────────────────────────────────────────────────
$(ELF): $(OBJS)
	@echo "  LD  $@"
	$(CC) $(LDFLAGS) $^ -o $@

# ── Convert ELF to Intel HEX ──────────────────────────────────────────────────
$(HEX): $(ELF)
	@echo "  HEX $@"
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# ── Flash via USBasp ──────────────────────────────────────────────────────────
.PHONY: flash
flash: $(HEX)
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U flash:w:$(HEX):i

# ── Flash via Arduino bootloader ──────────────────────────────────────────────
.PHONY: flash_uno
flash_uno: $(HEX)
	$(AVRDUDE) -c arduino -p $(MCU) -P $(PROGRAMMER_PORT) -b 115200 \
	           -U flash:w:$(HEX):i

# ── Show memory usage ─────────────────────────────────────────────────────────
.PHONY: size
size: $(ELF)
	@echo ""
	@echo "Memory usage:"
	$(SIZE) --format=avr --mcu=$(MCU) $(ELF)

# ── Clean build artifacts ─────────────────────────────────────────────────────
.PHONY: clean
clean:
	@echo "  CLEAN"
	rm -rf $(BUILD)

# ── Dependency generation (optional, for incremental builds) ──────────────────
DEPS = $(OBJS:.o=.d)
-include $(DEPS)

$(BUILD)/%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
