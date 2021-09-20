CC= arm-none-eabi-gcc
FLAGS= mcpu
TYPE= cortex-m4
ARC= mthumb
all: main.o stm32f4xx_gpio_driver.o stm32_startup.o finalled.elf
main.o:main.c 
	$(CC) -c -$(FLAGS)=$(TYPE) -$(ARC) -std=gnu11 $^ -o $@

stm32f4xx_gpio_driver.o:stm32f4xx_gpio_driver.c
	$(CC) -c -$(FLAGS)=$(TYPE) -$(ARC) -std=gnu11 $^ -o $@
stm32_startup.o:stm32_startup.c
	$(CC) -c -$(FLAGS)=$(TYPE) -$(ARC) -std=gnu11 $^ -o $@

finalled.elf: main.o stm32f4xx_gpio_driver.o stm32_startup.o
	$(CC) -nostdlib -T stm32_ls.ld -Wl,-Map=finalled.map *.o  -o finalled.elf
clean:
	rm -rf *.o *.elf
