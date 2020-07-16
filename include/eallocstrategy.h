/*
 * eallocstrategy.h
 *
 *  Created on: Mar 18, 2013
 *      Author: mike
 */

#ifndef EALLOCSTRATEGY_H_
#define EALLOCSTRATEGY_H_

namespace irr
{
	namespace core
	{
		enum eAllocStrategy
		{
			ALLOC_STRATEGY_SAFE = 0,
			ALLOC_STRATEGY_DOUBLE = 1,
			ALLOC_STRATEGY_SQRT = 2
		};
	}
}


#endif /* EALLOCSTRATEGY_H_ */
