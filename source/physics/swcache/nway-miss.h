/*
 * nway-miss.h
 *
 *  Created on: Jun 13, 2013
 *      Author: mike
 */

#ifndef NWAY_MISS_H_
#define NWAY_MISS_H_

static int _spe_cache_miss_(unsigned int ea, int set, int avail)
{
    unsigned int ea_aligned = ea & ~SPE_CACHELINE_MASK;
    vec_uint4 slot;
    vec_uint4 exists = _spe_cache_set_lookup_(set, ea_aligned);
    int idx, line;

    /* Double check to make sure that the entry has not
     * already been allocated in this set.  This condition
     * might occur if multiple lookups are being perfomed
     * simultaneously.
     */
    if (unlikely(spu_extract(exists, 0) != 0)) {
        return _spe_cache_idx_num_(exists);
    }

    /* Now check to see if there are empty slots
     * that are available in the set.
     */
    slot = _spe_cache_replace_(set, avail);
    idx = _spe_cache_idx_num_(slot);
    line = _spe_cacheline_num_(set, idx);

    spu_mfcdma32(&spe_cache_mem[line], ea_aligned, SPE_CACHELINE_SIZE,
        SPE_CACHE_SET_TAGID(set), SPE_CACHE_GET);

    spe_cache_dir[set][SPE_CACHE_NWAY_MASK - idx] = ea_aligned;

    return idx;
}

#endif /* NWAY_MISS_H_ */
