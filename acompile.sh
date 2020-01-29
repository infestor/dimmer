IFLAGS="-I/local/benejan/tmp/avr/avr/include -L/local/benejan/tmp/avr/avr/lib -I//local/benejan/tmp/avr-libc-2.0.0"

MCU=atmega328p

CFLAGS="-Wall -W"
CLAGS+=" -pedantic"
CFLAGS+=" -gdwarf-2 -gstrict-dwarf"
CFLAGS+=" -DF_CPU=16000000UL -Os"
CFLAGS+=" -ffreestanding"
#CFLAGS+=" -mshort-calls"
#CFLAGS+=" -msize"
CFLAGS+=" -fno-tree-scev-cprop"
#CFLAGS+=" -mcall-prologues"
CFLAGS+=" -fno-jump-tables"
CFLAGS+=" -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums"
CFLAGS+=" -fno-split-wide-types"
CFLAGS+=" -Wa,-a,-ad"
CFLAGS+=" -std=c++98"

#CFLAGS+=" -nostartfiles"

#/local/benejan/tmp/avr/bin/avr-g++ $IFLAGS -mmcu=$MCU $CFLAGS -c arduino_simple.c -o arduino_simple.o > arduino_simple.lst
#/local/benejan/tmp/avr/bin/avr-g++ $IFLAGS -mmcu=$MCU $CFLAGS -c spilib.c -o spilib.o > spilib.lst
#/local/benejan/tmp/avr/bin/avr-g++ $IFLAGS -mmcu=$MCU $CFLAGS -c Mirf.cpp -o Mirf.o > Mirf.lst
#/local/benejan/tmp/avr/bin/avr-g++ $IFLAGS -mmcu=$MCU $CFLAGS -c dimmer.cpp -o dimmer.o > dimmer.lst

CFLAGS+=" -Wl,--relax"
CFLAGS+=" -Wl,--gc-sections"
CFLAGS+=" -ffunction-sections"
CFLAGS+=" -fdata-sections"

/local/benejan/tmp/avr/bin/avr-g++ $IFLAGS -g3 -mmcu=$MCU $CFLAGS spilib.c Mirf.cpp dimmer.cpp -o dimmer.elf  > dimmer.llt

CFLAGS+=" -fwhole-program -flto"
/local/benejan/tmp/avr/bin/avr-g++ $IFLAGS -mmcu=$MCU $CFLAGS spilib.c Mirf.cpp dimmer.cpp -o dimmer-o.elf  > dimmer-o.llt

#/local/benejan/tmp/avr/bin/avr-g++ -mmcu=$MCU $CFLAGS spilib.o Mirf.o dimmer.o -o dimmer-o.elf

/local/benejan/tmp/avr/bin/avr-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature dimmer.elf dimmer.hex
/local/benejan/tmp/avr/bin/avr-size --mcu=$MCU --format=avr dimmer.elf
/local/benejan/tmp/avr/bin/avr-size --mcu=$MCU --format=avr dimmer-o.elf
/local/benejan/tmp/avr/bin/avr-objdump -h -S dimmer.elf > dimmer.lss
/local/benejan/tmp/avr/bin/avr-objdump -h -S dimmer-o.elf > dimmer-o.lss