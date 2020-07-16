/*
 * parallelsort.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef PARALLELSORT_H_
#define PARALLELSORT_H_

#include "base/common.h"

void printBuff(SortData *inBuff, u32 n);
void parallelsort(SortData *data, SortData *buff, u32 numData, u32 taskId, u32 numTasks);

#endif /* PARALLELSORT_H_ */
