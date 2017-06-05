#ifndef __FSM_H
#define __FSM_H

#include "FunctionPointer.h"

//状态转出列表项
class FiniteStateMachineState;
class FiniteStateMachineTrans
{
	FunctionPointerArg1<bool, int> _condition;
	FiniteStateMachineState* targetState;
	FiniteStateMachineTrans* next;
public:
	friend class FiniteStateMachineState;
	friend class FiniteStateMachine;

	//实例化一个TransItem类
	//event：转移条件-事件
	//condition：转移条件-布尔判别式
	//targetState：目标状态
	//before：链表的上一个TransItem
	FiniteStateMachineTrans(bool(*condition)(int), FiniteStateMachineState* targetState, FiniteStateMachineTrans* before = NULL) :
		targetState(targetState),
		next(NULL)
	{
		if (before != NULL)
		{
			before->next = this;
		}
		_condition.attach(condition);
	}

	//实例化一个TransItem类
	//event：转移条件-事件
	//classPointer：如果condition函数是成员函数，需要类的指针
	//condition：转移条件-布尔判别式
	//targetState：目标状态
	//before：链表的上一个TransItem
	template<typename T>
	FiniteStateMachineTrans(T* classPointer, bool(T::*condition)(int), FiniteStateMachineState* targetState, FiniteStateMachineTrans* before = NULL) :
		targetState(targetState),
		next(NULL)
	{
		if (before != NULL)
		{
			before->next = this;
		}
		_condition.attach(classPointer, condition);
	}
	bool condition(int event)
	{
		return _condition.call(event);
	}
};

class FiniteStateMachineState
{
	FunctionPointer _entry;
	FunctionPointer _work;
	FunctionPointer _exit;
	//向外转移列表项，以链表方式存储
	FiniteStateMachineTrans *transList, *transEnd;//维护一个transEnd指针，方便在末尾添加项
	friend class FiniteStateMachine;

	void refreshTransEnd()
	{
		if (transList != NULL)
		{
			FiniteStateMachineTrans *nowTransItem = transList;
			while (1)
			{
				if (nowTransItem->next != NULL)
				{
					nowTransItem = nowTransItem->next;
				}
				else
				{
					break;
				}
			}
			transEnd = nowTransItem;
		}
		else
		{
			transEnd = NULL;
		}
	}
public:

	//创建一个以普通函数作为成员的状态
	FiniteStateMachineState(void(*work)(void), void(*entry)(void), void(*exit)(void), FiniteStateMachineTrans* transList = NULL) :
		_work(work),
		_entry(entry),
		_exit(exit),
		transList(transList),
		transEnd(transList)
	{
		refreshTransEnd();
	}

	//创建一个以成员函数作为成员的状态
	template<typename T>
	FiniteStateMachineState(T* classPointer, void(T::*work)(void), void(T::*entry)(void), void(T::*exit)(void), FiniteStateMachineTrans* transList = NULL) :
		_work(classPointer, work),
		_entry(classPointer, entry),
		_exit(classPointer, exit),
		transList(transList),
		transEnd(transList)
	{
		refreshTransEnd();
	}

	//创建一个以成员函数作为成员的状态
	template<typename T>
	FiniteStateMachineState(T* classPointer, void(T::*work)(void), int empty1, int empty2, FiniteStateMachineTrans* transList = NULL) :
		_work(classPointer, work),
		transList(transList),
		transEnd(transList)
	{

	}

	//创建一个以成员函数作为成员的状态
	template<typename T>
	FiniteStateMachineState(T* classPointer, void(T::*work)(void), int empty1, void(T::*exit)(void), FiniteStateMachineTrans* transList = NULL) :
		_work(classPointer, work),
		_exit(classPointer, exit),
		transList(transList),
		transEnd(transList)
	{
		refreshTransEnd();
	}

	//创建一个以成员函数作为成员的状态
	template<typename T>
	FiniteStateMachineState(T* classPointer, void(T::*work)(void), void(T::*entry)(void), int empty2, FiniteStateMachineTrans* transList = NULL) :
		_work(classPointer, work),
		_entry(classPointer, entry),
		transList(transList),
		transEnd(transList)
	{
		refreshTransEnd();
	}


	//为状态添加向外转移表
	void setTransList(FiniteStateMachineTrans* transList)
	{
		this->transList = transList;
	}

	//为向外转移表添加表项
	void addTransItem(FiniteStateMachineTrans* transItem)
	{
		if (transEnd == NULL)
		{
			transList = transItem;
			transEnd = transItem;
		}
		else
		{
			transEnd->next = transItem;
			transEnd = transItem;
		}
	}

	void entry()
	{
		_entry.call();
	}

	void work()
	{
		_work.call();
	}

	void exit()
	{
		_exit.call();
	}

};

//有限状态机
//1. 创建创建状态机
//2. 创建各状态
//3. 创建各状态的状态转移表
//4. 设置当前状态
class FiniteStateMachine
{
	FiniteStateMachineState* activeState;
public:

	//建立有限状态机
	FiniteStateMachine(FiniteStateMachineState* activeState = NULL) :
		activeState(activeState)
	{
		if (activeState != NULL)
		{
			activeState->entry();
		}
	}

	//设置当前状态
	//进入状态之前执行上一个State的exit函数
	//再执行下一个State的entry函数
	//再在每次refresh中执行work
	void setActivaState(FiniteStateMachineState* activeState)
	{
		//进入状态之前执行上一个State的exit函数
		this->activeState->exit();

		this->activeState = activeState;

		//再执行下一个State的entry函数
		this->activeState->entry();
	}

	void refresh(int event)
	{
		if (activeState != NULL)
		{
			this->activeState->work();

			//遍历转移列表
			if (activeState->transList != NULL)//转移列表非空
			{
				FiniteStateMachineTrans* nowItem = activeState->transList;
				while (1)
				{
					//满足当前事件（Event）与当前状态的向外转移列表中事件相同
					//满足状态的向外转移列表中事件对应的bool判别式（函数）
					if (nowItem->condition(event))
					{
						setActivaState(nowItem->targetState);
						break;
					}
					nowItem = nowItem->next;
					if (nowItem == NULL)
					{
						break;
					}
				}
			}
		}
	}
};


#endif

