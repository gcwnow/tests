TARGET := ipu
CFLAGS := -Wall -O2

CROSS_COMPILE ?= mipsel-gcw0-linux-uclibc-
CC := $(CROSS_COMPILE)gcc
OBJS := ipu.o

.PHONY: all
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

.PHONY: clean
clean:
	rm -f $(TARGET) $(OBJS)
