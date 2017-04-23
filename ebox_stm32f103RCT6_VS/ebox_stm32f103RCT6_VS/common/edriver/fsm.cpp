#include "fsm.h"

FSM::FSM(void(*startState)()) :
	activeState(startState)
{

}

void FSM::setActiveState(void(*state)())
{
	activeState = state;
}

void FSM::refresh()
{
	if (!activeState)
	{
		(*activeState)();
	}
}
