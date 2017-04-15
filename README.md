# inverted_pendulum

电子科技大学2017电子设计竞赛校队控制组第一次训练，KZ-3组

小组成员：章程，韦仕才，徐瑶

采用ebox库，vs2015开发环境。[模板和使用说明](https://github.com/pidan1231239/ebox_stm32f103RCT6_VS#ebox_stm32f103rct6_vs)参考队长[章程](https://github.com/pidan1231239)的ebox_stm32f103RCT6_VS

## 重新编写编码器驱动```encoder_timer```

### 可用引脚

> 高级定时器和通用定时器可以将ch1和ch2配置成正交编码器AB相输入

TIM1~TIM4可用io口如下：
- TIM1:PA8 PA9
- TIM2:PA0 PA1
- TIM3:PA6 PA7
- TIM4:PB6 PB7

本次倒立摆设计采用TIM1和TIM3配置正交编码器。用TIM2配置PWM控制电机
