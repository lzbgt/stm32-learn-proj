```
STM32_Programmer_CLI -c port=SWD -w build/uart-dma.elf
picocom /dev/tty.wchusbserial1420 -b 115200 -l
```