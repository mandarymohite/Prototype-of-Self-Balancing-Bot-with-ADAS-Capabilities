# =============================================================================
#  Makefile  –  Self-Balancing Robot with ADAS Extension
#  Target: Raspberry Pi (ARM), cross-compilable with arm-linux-gnueabihf-gcc
# =============================================================================

CC      = gcc
CFLAGS  = -O0 -g -Wall -Wextra
LDFLAGS = -lpthread -lrt -lwiringPi -lm

TARGET  = segway_adas
SRCS    = mySegway.c motors.c adas.c
OBJS    = $(SRCS:.c=.o)
HDRS    = adas.h

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "Build complete → $(TARGET)"

%.o: %.c $(HDRS)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	-rm -f $(OBJS) $(TARGET)
	@echo "Clean complete"
