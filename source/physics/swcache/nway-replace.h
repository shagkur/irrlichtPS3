/*
 * nway-replace.h
 *
 *  Created on: Jun 13, 2013
 *      Author: mike
 */

#ifndef NWAY_REPLACE_H_
#define NWAY_REPLACE_H_

static vec_uint4 spe_cache_replace_cntr[SPE_CACHE_NSETS+1];

static inline vec_uint4 _spe_cache_replace_(int set, int avail)
{
    unsigned int mask = ((1 << SPE_CACHE_NWAY) - 1) & avail;
    unsigned int curr, currbit, next;

    curr = spu_extract(spe_cache_replace_cntr[set], 0) & SPE_CACHE_NWAY_MASK;
    currbit = (1 << curr);
    next = (curr + 1) & SPE_CACHE_NWAY_MASK;
    spe_cache_replace_cntr[set] = (vec_uint4) spu_promote(next, 0);
    mask = (mask & currbit) ? currbit : mask;

    return (vec_uint4) spu_promote(mask, 0);
}

#endif /* NWAY_REPLACE_H_ */
