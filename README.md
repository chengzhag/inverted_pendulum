# inverted_pendulum

电子科技大学2017电子设计竞赛校队控制组第一次训练，KZ-3组

小组成员：章程，韦仕才，徐瑶

采用ebox库，vs2015开发环境。[模板和使用说明](https://github.com/pidan1231239/ebox_stm32f103RCT6_VS#ebox_stm32f103rct6_vs)参考队长[章程](https://github.com/pidan1231239)的ebox_stm32f103RCT6_VS

## 机械部分说明

### 材料选取及用途：
- 3mm厚碳板和铜柱作为快装板和电机支撑
- 碳方管作为横梁
- 碳圆管作为摆杆
- 铝棒作为转轴
- 底部用三脚架支撑
- 连接处用法兰联轴器以及五转六的电机联轴以及绑扎带固定
- 采用导电滑环防止绕线

## 软件部分说明

### 重新编写编码器驱动```encoder_timer```
- 可用引脚：高级定时器和通用定时器可以将ch1和ch2配置成正交编码器AB相输入
- TIM1~TIM4可用io口如下：
  - TIM1:PA8 PA9
  - TIM2:PA0 PA1
  - TIM3:PA6 PA7
  - TIM4:PB6 PB7
- 本次倒立摆设计采用TIM1和TIM3配置正交编码器。用TIM2配置PWM控制电机

### 编写电机驱动```encoder_motor.h```
- 设置速度和位置模式以及PID初始化


### 将光编码器作为绝对编码器使用
- 计算摆杆的角加速度和角速度，以弧度制为单位


### 采用四级PID控制
- 对摆杆角度、摆杆角速度、横梁角度、横梁角速度进行PID算法控制


### 采用状态机和四个独立键盘对倒立摆进行模式切换
- 选择io如下：PB12 PB13 PB14 PB15
- 对应模式：```Swing SwingInvert Invert Round```

  
### 用freertos设置进程
- 实现多任务并行处理


## 电路部分说明
- 电路组成：
  - LM2596S-5.0作为降压模块
  - tb6612作为电机驱动
  - stm32f103c8t6最小系统作为主控
  - 四个独立按键
- 电路设计：
  - 电机驱动模块插座
  - c8t6最小系统板插座
  - 降压模块插座
  - 4pin xh2.56光编码器接口
  - 6pin xh2.56电机接口
  - 4pin xh2.56延长线
  - 12v电池插头

## 有限状态机
### 设计思路
- 状态以类的方式存储
- 状态的entry、work、exit函数以指针方式存储
- 状态的向外转移列表以类链表存储

### class FiniteStateMachine
> 有限状态机类
- 切换状态
  - 执行上一个State的exit函数
  - （执行状态转移函数，需要状态过渡时使用）
  - 设置当前状态指针
  - 执行下一个State的entry函数
- refresh函数中
  - 调用状态的work函数
  - 判断是否满足下列条件，如果满足，进行转移
    - 满足状态的向外转移列表中事件对应的bool判别式（函数，传入event参数）

### class FiniteStateMachineState
> 状态类
- 包含entry、work、exit函数指针
  - 提供接口对函数指针进行初始化
  - 提供接口对函数指针进行调用
- 向外转移列表（链表）
  - 在FiniteStateMachine调用refresh进行转移判断时对当前State的列表进行遍历，满足条件则进行转移
  - 提供接口addTransItem用以增加表项
  - 在初始化时可直接将该表传入

### class FiniteStateMachineTrans
> 向外转移列表类，单向链表，自身构成链表头
- 通过next指针形成链表
- 在初始化时可通过before衔接已有的链表
- 包含一个bool代数式（函数）condition
- 包含一个目标State*（状态指针）

### 改进方向
- FiniteStateMachineState添加向外转移表项的函数改为可变参数
- 改进FiniteStateMachineTrans类为FiniteStateMachineTransList和FiniteStateMachineTransItem，成为一个封装完善的单向链表，且具有添加操作
- 考虑以unsigned long型保存event，用多选一的方式表示单个event，这样可以通过或操作完成多个event的表示，将event和condition分离，以event作为触发，condition作为条件
