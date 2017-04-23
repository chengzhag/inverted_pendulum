#ifndef __FSM
#define __FSM

//以函数作为状态的有限状态机
class FSM
{
	//当前状态
	void (*activeState)();

public:
	//设置初始状态
	FSM(void(*startState)());

	//设置当前状态，用于状态跳转
	void setActiveState(void (*state)());

	//刷新，执行当前状态函数
	void refresh();
};


#endif