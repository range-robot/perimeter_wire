


# Makefile for AtmelStudio.
# When building from linux use catkin

# Solution path may not contiain spaces

# Path to bash (from WSL) called by x86 executable!
BASH=C:\Windows\sysnative\bash.exe
TARGET=build\devel\lib\container_motor_control_firmware\firmware-atmega328p
ELFFILE=$(TARGET).elf 
NAME=firmware

build:
	mkdir $@

build/Makefile:
	$(BASH) -c 'source /opt/ros/kinetic/setup.bash; cd build; cmake .. -DCMAKE_BUILD_TYPE=DEBUG'

$(ELFFILE): build build/Makefile
	$(BASH) -c 'LANG=en_US; cd build; make'

$(NAME).%: $(TARGET).%
	copy $< $@

all: $(NAME).elf $(NAME).map $(NAME).hex

clean:
	rm -r build

.PHONY: all clean $(ELFFILE)