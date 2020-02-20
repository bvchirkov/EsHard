APP_NAME	:= rfa

#Device type
ST	:= LHT
#Slave address
SA	:= 0x01

AVRGCC		:= /usr/bin/avr-gcc
AVROBJCOPY	:= /usr/bin/avr-objcopy
AVRDUDE		:= /usr/bin/avrdude

MMCU		:= atmega128rfa1

#JFLAGS		:= -ggdb
FLAGS		:= -Os -Wall -std=gnu11 -mmcu=$(MMCU) $(JFLAGS)
FLAGS 		+= -DSLAVE_ADDR=$(SA)
FLAGS		+= -DSLAVE_TYPE=$(ST)

MMCU_PROG	:= m128rfa1
PROGRAMMER	:= usbasp
#PROGRAMMER	:= atmelice
DBGPORT		:= 1234

source_dir		:=	.
source_files	:=	$(wildcard $(addsuffix /*.c, $(source_dir) ) )
object_files	:=	$(notdir $(source_files) )
object_files	:=	$(object_files:.c=.o)

.PHONY: all clean

all: clean $(APP_NAME).o
	$(AVROBJCOPY) -j .text -j .data -O ihex $(APP_NAME).o $(APP_NAME).hex

$(APP_NAME).o: $(source_files)
	$(AVRGCC) $(FLAGS) $^ -o $(APP_NAME).o

flush:
	$(AVRDUDE) -c $(PROGRAMMER) -P usb -p $(MMCU_PROG) -U flash:w:$(APP_NAME).hex

#General fuses
fuse:
	$(AVRDUDE) -c $(PROGRAMMER) -P usb -p $(MMCU_PROG) -F -U lfuse:w:0xbf:m
	$(AVRDUDE) -c $(PROGRAMMER) -P usb -p $(MMCU_PROG) -F -U hfuse:w:0x55:m
#	$(AVRDUDE) -c $(PROGRAMMER) -P usb -p $(MMCU_PROG) -U efuse:w:0xfe:m
	
#JTAG fuses
fusej:
	$(AVRDUDE) -c $(PROGRAMMER) -P usb -p $(MMCU_PROG) -F -U lfuse:w:0xbf:m
	$(AVRDUDE) -c $(PROGRAMMER) -P usb -p $(MMCU_PROG) -F -U hfuse:w:0x15:m
	$(AVRDUDE) -c $(PROGRAMMER) -P usb -p $(MMCU_PROG) -F -U efuse:w:0xfe:m
	
#DBG mode
dbg: fusej all
	screen -dm avarice --edbg :$(DBGPORT)
	avr-gdb $(APP_NAME).o -ex 'target remote :$(DBGPORT)'

clean:
	rm -f ./*.o ./*.hex
	
	
#For use dbg
#1) set fuses JTAG (use make fusej)
#2) set programmer atmelice (use PROGRAMMER := atmelice)
#3) set compile flag -ggdb (use JFLAGS := -ggdb)
#4) connect ITMELICE to PC
#5) start avarice in detached screen (use screen -dm avarice --edbg :1234) or
#	background (use avarice --edbg :$(DBGPORT) &)
#	!WARNING: avarice -v >= 2.13svn20160229
#6) start avr-gdb [elf file] -ex 'target remote :1234'
#6.1) list [src file]
#6.2) set breakpoint (use break file:line or break file:function)
#6.3) continue and next
#6.4) show varialable (use print/[var type] [var name] or display/[var type] [var name])
#	[var type] u - unsigned int, x - hex
