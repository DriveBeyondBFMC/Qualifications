# Compiler
CC = gcc

# Compiler Flags
CFLAGS = -Wall -Wextra -I. 

# Linker Flags
LDFLAGS = -lwiringPi -lpthread -lm

# Source Files
SRCS = main.c deca_device.c deca_spi.c deca_sleep.c deca_mutex.c deca_params_init.c deca_range_tables.c port.c trilateration.c

# Object Files
OBJS = $(SRCS:.c=.o)

# Executable Name
TARGET = dwm1000_Tag

# Default Rule (Compile Everything)
all: $(TARGET)

# Link the Executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile Each C File into an Object File
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean (Remove Compiled Files)
clean:
	rm -f $(TARGET) $(OBJS)

# Run the Program (Requires sudo)
run: $(TARGET)
	sudo ./$(TARGET)
