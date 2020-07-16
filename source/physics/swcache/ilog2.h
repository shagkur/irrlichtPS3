/*
 * ilog2.h
 *
 *  Created on: Jun 13, 2013
 *      Author: mike
 */

#ifndef ILOG2_H_
#define ILOG2_H_

static __inline__ signed int _ilog2(signed int x)
{
#ifdef __SPU__
	return (32 - spu_extract(spu_cntlz(spu_promote(x - 1, 0)), 0));
#else
	signed int result;

	for(result=0, x--;x > 0;result++, x>>=1);
	return result;
#endif
}


#endif /* ILOG2_H_ */
