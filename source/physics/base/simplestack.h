/*
 * simplestack.h
 *
 *  Created on: Jun 1, 2013
 *      Author: mike
 */

#ifndef SIMPLESTACK_H_
#define SIMPLESTACK_H_

#include "common.h"

#define SIMPLE_STACK_COUNT 		20

template< class StackData >
class SimpleStack
{
private:
	u32 cur;
	StackData stacks[SIMPLE_STACK_COUNT];

public:
	SimpleStack()
	{
		cur = 0;
	}

	void push(const StackData &stack)
	{
		stacks[cur++] = stack;
		ASSERT(cur < SIMPLE_STACK_COUNT);
	}

	StackData pop()
	{
		return stacks[--cur];
	}

	bool isEmpty()
	{
		return cur == 0;
	}

	s32 getStackCount()
	{
		return cur;
	}
};

#endif /* SIMPLESTACK_H_ */
