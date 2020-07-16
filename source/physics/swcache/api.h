/*
 * api.h
 *
 *  Created on: Jun 13, 2013
 *      Author: mike
 */

#ifndef API_H_
#define API_H_

typedef void* spe_cache_entry_t;

#define spe_cache_rd(ea)                _spe_cache_lookup_xfer_wait_(ea, 0, 1)
#define spe_cache_tr(ea)                _spe_cache_lookup_xfer_(ea, 0, 1)
#define spe_cache_lr(ea)                _spe_cache_lookup_(ea, 0)

#define spe_cache_wait(entry)           _spe_cache_wait_(entry)

#endif /* API_H_ */
