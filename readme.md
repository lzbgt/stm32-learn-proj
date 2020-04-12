
## how to flash and monitor uart
```
STM32_Programmer_CLI -c port=SWD -w build/uart-dma.elf
picocom /dev/tty.wchusbserial1420 -b 115200 -l
```


## f1/uart-dma

instead of using SWD for debugging, implements two functions for outputing string to serial port.
```
static void Println(char *fmt, ...); // auto adds tailing \r\n
static void Print(char *fmt, ...); // no tailing \r\n
```
