build: main.c
	gcc calculate_pattern_table.c -o calc
	./calc > pattern_table.h
	rm calc
	avr8-gnu-toolchain-darwin_x86_64/bin/avr-gcc -w -fmax-errors=3 -O2 -DF_CPU=24000000UL -mmcu=avr64dd28 -I. main.c -o goop -static
flash:
	avrdude -p avr64dd28 -c serialupdi -P /dev/cu.usbserial-* -U flash:w:goop
setfuse:
	echo todo
