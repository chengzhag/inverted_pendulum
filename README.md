# ebox_stm32f103RCT6_VS
为stm32f103RCT6配置好的ebox，visual studio项目，方便以后使用

vs工程和移植注意事项在ebox_stm32f103RCT6_VS/ebox_stm32f103RCT6_VS/ebox_stm32f103RCT6_VS文件夹下

## 包含驱动
### 逻辑器件
- 74HC595：移位寄存器
### 储存
- E2PROM
    - AT24C02
- Flash
    - AT45DB
    - w25x16
- RAM
    - FM25V
- SD/mmc
    - mmc_sd
### 输入设备
- 按键
    - Button
    - key_dm
- 矩阵键盘
    - TM1638：带键盘扫描接口的LED驱动控制专用电路
### 输出设备
- 显示屏
    - FT800
    - lcd_1.8
    - LCD1602
    - Nokia5110
    - OLED
    - OLED_ssd1306
- LED
    - colorled
    - LED
    - led_segment：数码管，使用74hc595
    - TM1638：带键盘扫描接口的LED驱动控制专用电路
    - ws2812：内置IC串行RGBLED
- OSD
    - Max7456：单通道、单色随屏显示（OSD）发生器，内置EEPROM
- GPIO
    - parallel_gpio：8位并行输出，可以组合任意端口
### 传感器
- 湿度传感器
    - dht11
- 温度传感器
    - ds18b20
- 尘埃传感器
    - GP2Y1051
- 红外遥控
    - ir_decoder
    - ir_encoder
- 陀螺仪
    - MPU6050
    - L3G4200D
- 电子罗盘
    - lsm303dhlc
- ADC
    - LTC1446：12位逐次逼近式工作的A/D转换器
- 距离传感器
    - ultrasonic_wave：超声波
### 传输
- 2.4G无线
    - NRF24L01
- 数传
    - si4432：低于1GHz频段的无线数传芯片
- 以太网
    - w5500：全硬件TCP/IP嵌入式以太网控制器
### RTC（实时时钟）
- ds3231：i2c
### 正交编码器
- encoder
- encoder_exti：旋转正交编码器。基于外部中断

## 如何将工程搬移到visualGDB上？
> https://visualgdb.com/tutorials/arm/keil/

### 导入keil工程目录
1. 文件扩展名必须小写
1. 导入工程文件夹时去掉原工程没有导入的c/cpp文件，避免include未设置在包含路径内的h文件

### 设置编译器、链接器flag
1. COMMONFLAGS去掉```--split_sections```，否则会报错
1. 改为```--list "$(BINARYDIR)\ebox.map"```，将输出内容保存到Debug文件夹
1. 去掉```--scatter ".\Objects\ebox.sct"```，否则会报错
1. LDFLAGS需要添加```--ro-base```、```--rw-base```等flag，这些设置在keil的项目设置的linker选项卡下

### 设置包含目录
1. 项目的visualGDB propoties的makefile settings中也需要设置include，否则intellisense无法正常
1. 在需要链接dll文件的情况下，需要再makefile settings中设置additional linker inputs

### 设置debug settings
1. 配置j-link时注意手动选择设备类型
1. 下载报错：```Cannot resolve the address of _estack. Skipping stack pointer validity check.```，可以在debug settings中取消勾选validate stack pointer when starting debugging选项解决。

### 其他
1. 全局变量初始化为-1导致程序运行失败，要按照教程后修改Makefile，将```ifeq ($(TARGETTYPE),APP)...endif```替换为
```
ifeq ($(TARGETTYPE),APP)
ROM_SECTION_NAME := ER_RO
$(BINARYDIR)/$(TARGETNAME): $(all_objs) $(EXTERNAL_LIBS)
    $(LD) -o $(@:.elf=.axf) $(LDFLAGS) $(START_GROUP) $(all_objs) \
      $(LIBRARY_LDFLAGS) $(END_GROUP)
    $(FROMELF) --bin --output $(@:.elf=.bin) $(@:.elf=.axf)
    $(OBJCOPY) --remove-section $(ROM_SECTION_NAME) \
        --add-section $(ROM_SECTION_NAME)=$(@:.elf=.bin) \
        --set-section-flags \
        $(ROM_SECTION_NAME)=CONTENTS,ALLOC,LOAD,READONLY,CODE \
        --change-section-address $(ROM_SECTION_NAME)=0x08000000 \
        $(@:.elf=.axf) $@
endif
```
1. makefile,.mak文件中使用```\```进行换行时，后面不能有包括空格在内的任何字符
