# STM32 Learn Project

STM32 learning examples for UART/DMA, PWM/servo control, TFT display work, and radar/motion experiments.

## Paid embedded bring-up review

If you are adapting these examples or debugging an embedded prototype, I offer an Embedded IoT Bring-Up Review:

- Review page: https://x2.brucelu.top/iot/?source=github-stm32-learn-proj-top
- Sample review: https://x2.brucelu.top/iot/sample/?source=github-stm32-learn-proj-top
- Ask first: https://x2.brucelu.top/products/contact/?offer=iot&source=github-stm32-learn-proj-top
- Checkout: https://x2.brucelu.top/iot/checkout/?source=github-stm32-learn-proj-top

Best fit: STM32 UART/DMA, PWM, display bring-up, sensor/radar experiments, flashing/debug setup, serial logs, board wiring, and prototype-to-field-test risk.

Boundary: this is remote engineering review and setup guidance. It does not include hardware repair, safety/regulatory approval, credential handling, guaranteed RF/sensor performance, or custom firmware delivery.

---

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

## f1/servo-pwm
PA0 - pwm


## f1/tftm096
doppler radar motion detection
