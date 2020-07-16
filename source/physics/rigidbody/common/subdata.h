/*
 * subdata.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef SUBDATA_H_
#define SUBDATA_H_

#include "base/common.h"

struct SubData {
	enum {
		SubDataNone = 0,
		SubDataFacetLocal,
	};

	u8  type;

	struct {
		u8 islandIdx;
		u8 facetIdx;
		u16 s;
		u16 t;
	} facetLocal;

	SubData()
	{
		type = 0;
		facetLocal.islandIdx = 0;
		facetLocal.facetIdx = 0;
		facetLocal.s = 0;
		facetLocal.t = 0;
	}

	void  setIslandIndex(u8 i) {facetLocal.islandIdx = i;}
	void  setFacetIndex(u8 i) {facetLocal.facetIdx = i;}
	void  setFacetLocalS(f32 s) {facetLocal.s = (u16)(s*65535.0f);}
	void  setFacetLocalT(f32 t) {facetLocal.t = (u16)(t*65535.0f);}

	u8 getIslandIndex() {return facetLocal.islandIdx;}
	u8 getFacetIndex() {return facetLocal.facetIdx;}
	f32 getFacetLocalS() {return facetLocal.s/65535.0f;}
	f32 getFacetLocalT() {return facetLocal.t/65535.0f;}
};

#endif /* SUBDATA_H_ */
