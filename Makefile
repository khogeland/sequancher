build: main.c
	avr8-gnu-toolchain-darwin_x86_64/bin/avr-gcc -w -fmax-errors=3 -Os -DF_CPU=24000000UL -mmcu=avr64dd28 main.c -o goop -static
flash:
	avrdude -p avr64dd28 -c serialupdi -P /dev/cu.usbserial-* -U flash:w:goop
