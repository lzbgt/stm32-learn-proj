# config
```
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
```

# command line compile
1. vscode settings.json

```
"terminal.integrated.profiles.windows": {
    "stm32": {
      "path": [
        "cmd.exe"
      ],
      "args": [
        "/k",
        "C:/vs_terminals/stm32.bat"
      ]
    },
  }
```

2. C:/vs_terminals/stm32.bat

```
@set PATH=C:/ST/STM32CubeIDE_1.11.2/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.0.100.202202231230/tools/bin/;C:/ST/STM32CubeIDE_1.8.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.10.3-2021.10.win32_1.0.200.202301161003/tools/bin/;%PATH%
@set GCC_PATH=C:/ST/STM32CubeIDE_1.8.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.10.3-2021.10.win32_1.0.200.202301161003/tools/bin@set PREFIX=arm-none-eabi-

@echo stm32 compilation environment ready for go!
```



PC13 is for rcwl-0516
