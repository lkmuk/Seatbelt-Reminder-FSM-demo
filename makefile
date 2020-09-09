default:	app.c
	arm-none-eabi-gcc -c -mcpu=cortex-m3 -Wall -O3 -g app.c -o app.o	
build: app.o
	make default
	arm-none-eabi-as -g STM32F1_vecTable.S -o STM32F1_vecTable.o
	arm-none-eabi-as -g startup.S -o startup.o
	arm-none-eabi-ld STM32F1_vecTable.o startup.o app.o -o build.elf -T STM32F103RB.ld
load: build.elf
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f1x.cfg -c "program build.elf verify reset exit"
ocd:
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f1x.cfg
	
all:
	make build
	make load
	make ocd
clean:
	rm *.o $(objects)
	rm *.elf
