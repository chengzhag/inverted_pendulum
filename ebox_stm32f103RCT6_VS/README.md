# 第41组软件方向说明

## 1 工程模板

> stm32 + ebox的工程模板已发布到[github](https://github.com/pidan1231239/ebox_stm32f103RCT6_VS)上
在压缩包中对应文件：**ebox_stm32f103RCT6_VS.zip**

- 开发环境采用Visual Studio 2015 + visualGDB，编译器采用keil5的arm编译器，配置过程参考文件：**Visual Studio搭建STM32开发环境（自动补全+跳转+DEBUG**
- 了解ebox的功能和已包含驱动，访问[ebox官网](http://www.eboxmaker.com/)和参考上方github链接中[包含驱动](https://github.com/pidan1231239/ebox_stm32f103RCT6_VS/blob/master/README.md#包含驱动)
- 深入了解[stm32 ebox工程从keil到visual studio的迁移过程](https://github.com/pidan1231239/ebox_stm32f103RCT6_VS/blob/master/README.md#如何将工程搬移到visualgdb上)，和[visualGDB官方教程](https://visualgdb.com/tutorials/arm/keil/)

## 2 开发环境

> 以下必备开发软件务必都装在默认位置！尤其是keil 5，因为visualGDB的工程使用了keil的编译器，引用的是绝对地址

### 2.1 Visual Studio 2015 Community

> 在Microsoft官网下载免费版，并更新至update3

### 2.2 keil 5

> 安装keil for arm，即mdk

### 2.3 VisualGDB

> 安装visualgdb 5.16，文件：**visualgdb516rpj(www.greenxf.com).zip**

### 2.4 VassistX

> 安装VisualAssistX 10.9.2108，文件：**[WWW.WuleBa.COM]Vis.ual.Ass.ist.X 10.9.2108 官.方.原.版+破@jie补丁（Visual Studio 超强插件，吾乐吧软件站分享）.rar**

> 以上环境的配置参考文件:**Visual Studio搭建STM32开发环境（自动补全+跳转+DEBUG）.pdf**

### 2.5 Visual Studio Code

编辑MarkDown，浏览代码等

### 2.6 gihub gui客户端

版本控制，代码共享、合作

## 3. 开发硬件

### 3.1 STM32 F103

> 受限于ebox库，目前任得到维护的只有针对STM32 F103的ebox

### 3.2 RL78/G13 开发套件 R5F100LEA 单片机

> 一款瑞萨的单片机开发板，校队可能会发，为前几届比赛个别控制体的指定用MCU

### 3.3 可能需要的基于linux的开发板，如树莓派

> 图像处理等大计算量运算

## 4 必备技能

### 4.1 C++

> - 要求掌握对象、继承、重载、模板、命名空间等常用知识
> - 开发单片机用

### 4.2 MarkDown

> - 学习此文档中使用的一些基本语法
> - 用于代码文档的编写

### 4.3 github gui客户端

> - 注册github账号，完成官方基础教程
> - 下载安装GitKraken和SourceTree

### 4.4 matlab

> - 用于算法的实验、学习控制理论
> - 其他未知用途

### 4.5 印象笔记

> - 下载手机、电脑客户端
> - 注册账号
> - 记录重要事件
> - 记录分享赛题资料、思考，协同编辑

### 4.6 Processon

> - 一款在线作图应用
> - 注册账号
> - 实时协作
> - 常用系统框图和流程图、思维导图等

### 4.7 SolidWorks 2017

> - 三维制图，零件、装配体
> - 工程图生成：装配图、零件图、剖面、三视图

## 5 统一编程命名规范

- 类名采用首字母大写的驼峰命名法：```class EncoderMotor```
- 函数、变量采用首字母小写的驼峰命名法：```int motorPosition```
- 宏定义，全大写，单词之间用"_"隔开：```#define ENCODER_MOTOR```
- 枚举变量，enum用首字母大写的单词，以"_"隔开：
```cpp
typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,  
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;
```


