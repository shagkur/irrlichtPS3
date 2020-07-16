/*
 * contactphase.h
 *
 *  Created on: Jan 28, 2014
 *      Author: mike
 */

#ifndef CONTACTPHASE_H_
#define CONTACTPHASE_H_

void applyContactImpulseExPcl(SortData *sorts, u32 numSorts, const PclContactSolverConfig& config)
{
	s32 tag1 = 10;
	PclState *pclStates = (PclState*)ALLOCATE(128, sizeof(PclState)*numSorts*2);
	u32 *pclStatesEA = (u32*)ALLOCATE(16, sizeof(u32)*numSorts*2);
	u32 numBatch = 5;
	u8 numStates = 0;
	s32 numBatchPclStates[2] = {0};
	spu_dma_list_element *pclStateDmaList[2];
	u8 *pclStateRefTable = (u8*)ALLOCATE(16, sizeof(u8)*solverIOPcl.numPclStates);

	memset(pclStateRefTable, 0xff, sizeof(u8)*solverIOPcl.numPclStates);

	pclStateDmaList[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	pclStateDmaList[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);

	for(u32 j=0;j < numBatch*2;j++) {
		pclStateDmaList[0][j].notify = pclStateDmaList[1][j].notify = 0;
		pclStateDmaList[0][j].size = pclStateDmaList[1][j].size = sizeof(PclState);
	}

	u32 pairAddr = solverIOPcl.contactsPclAddr;
	PclContactPair *pairBuffer[2];
	spu_dma_list_element *pairDmaListBuf[3];

	pairBuffer[0] = (PclContactPair*)ALLOCATE(128, sizeof(PclContactPair)*numBatch);
	pairBuffer[1] = (PclContactPair*)ALLOCATE(128, sizeof(PclContactPair)*numBatch);
	pairDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);// PUT

	s32 batchFlag = 0;
	s32 currData = 0;
	s32 numPairs[2] = {0};
	PclState *nextPclStatesPtr = pclStates;

	for(u32 j=0;j < numBatch;j++) {
		pairDmaListBuf[0][j].notify = 0;
		pairDmaListBuf[0][j].size = sizeof(PclContactPair);
		pairDmaListBuf[1][j].notify = 0;
		pairDmaListBuf[1][j].size = sizeof(PclContactPair);
	}

	for(u32 j=0;j < numBatch && j < numSorts;j++) {
		pairDmaListBuf[0][numPairs[0]].eal = pairAddr + sizeof(PclContactPair)*getPair(sorts[j]);
		numPairs[0]++;

		u16 stateA = getStateA(sorts[j]);
		u16 stateB = getStateB(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			pclStatesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateDmaList[0][numBatchPclStates[0]].eal = pclStatesEA[numStates];
			pclStateRefTable[stateA] = numStates;
			numBatchPclStates[0]++;
			numStates++;
		}

		if(pclStateRefTable[stateB] == 0xff) {
			pclStatesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
			pclStateDmaList[0][numBatchPclStates[0]].eal = pclStatesEA[numStates];
			pclStateRefTable[stateB] = numStates;
			numBatchPclStates[0]++;
			numStates++;
		}
	}

	spu_dma_list_get(pairBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
	spu_dma_list_get(pclStates, 0, pclStateDmaList[0], sizeof(spu_dma_list_element)*numBatchPclStates[0], tag1, 0, 0);

	nextPclStatesPtr += numBatchPclStates[0];

	spu_dma_wait_tag_status_all(1<<tag1);

	for(u32 j=numBatch;j < numBatch + numBatch && j < numSorts;j++) {
		pairDmaListBuf[1][numPairs[1]].eal = pairAddr + sizeof(PclContactPair)*getPair(sorts[j]);
		numPairs[1]++;

		u16 stateA = getStateA(sorts[j]);
		u16 stateB = getStateB(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			pclStatesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateDmaList[1][numBatchPclStates[1]].eal = pclStatesEA[numStates];
			pclStateRefTable[stateA] = numStates;
			numBatchPclStates[1]++;
			numStates++;
		}

		if(pclStateRefTable[stateB] == 0xff) {
			pclStatesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
			pclStateDmaList[1][numBatchPclStates[1]].eal = pclStatesEA[numStates];
			pclStateRefTable[stateB] = numStates;
			numBatchPclStates[1]++;
			numStates++;
		}
	}

	spu_dma_list_get(pairBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
	spu_dma_list_get(nextPclStatesPtr, 0, pclStateDmaList[1], sizeof(spu_dma_list_element)*numBatchPclStates[1], tag1, 0, 0);

	nextPclStatesPtr += numBatchPclStates[1];

	batchFlag = 1;
	currData = 0;

	for(u32 i=0;i < numSorts;) {
		// UPDATE
		for(s32 j=0;j < numPairs[currData];j++) {
			PclContactPair& pair = pairBuffer[currData][j];
			PclState& particleA = pclStates[pclStateRefTable[pair.stateIndex[0]]];
			PclState& particleB = pclStates[pclStateRefTable[pair.stateIndex[1]]];

			applyImpulsePcl(pair, particleA, particleB, config);
		}

		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		memcpy(pairDmaListBuf[2], pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]);
		spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[2], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

		numPairs[currData] = 0;
		numBatchPclStates[currData] = 0;

		batchFlag++;
		s32 start = batchFlag*numBatch;
		for(u32 j=start;j < start + numBatch && j < numSorts;j++) {
			pairDmaListBuf[currData][numPairs[currData]].eal = pairAddr + sizeof(PclContactPair)*getPair(sorts[j]);
			numPairs[currData]++;

			u16 stateA = getStateA(sorts[j]);
			u16 stateB = getStateB(sorts[j]);

			// State
			if(pclStateRefTable[stateA] == 0xff) {
				pclStatesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
				pclStateDmaList[currData][numBatchPclStates[currData]].eal = pclStatesEA[numStates];
				pclStateRefTable[stateA] = numStates;
				numBatchPclStates[currData]++;
				numStates++;
			}

			if(pclStateRefTable[stateB] == 0xff) {
				pclStatesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
				pclStateDmaList[currData][numBatchPclStates[currData]].eal = pclStatesEA[numStates];
				pclStateRefTable[stateB] = numStates;
				numBatchPclStates[currData]++;
				numStates++;
			}
		}

		spu_dma_list_getf(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		spu_dma_list_get(nextPclStatesPtr, 0, pclStateDmaList[currData], sizeof(spu_dma_list_element)*numBatchPclStates[currData], tag1, 0, 0);

		nextPclStatesPtr += numBatchPclStates[currData];

		currData = 1 - currData;
	}

	spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

	spu_dma_list_element *dmaList = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numStates);
	if(numStates > 0) {
		for(s32 i=0;i < numStates;i++) {
			dmaList[i].notify = 0;
			dmaList[i].eal = pclStatesEA[i];
			dmaList[i].size = sizeof(PclState);
		}
		spu_dma_list_put(pclStates, 0, dmaList,sizeof(spu_dma_list_element)*numStates, tag1, 0, 0);
	}
	spu_dma_wait_tag_status_all(1<<tag1);

	DEALLOCATE(dmaList);

	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);

	DEALLOCATE(pairBuffer[1]);
	DEALLOCATE(pairBuffer[0]);

	DEALLOCATE(pclStateDmaList[1]);
	DEALLOCATE(pclStateDmaList[0]);

	DEALLOCATE(pclStateRefTable);

	DEALLOCATE(pclStatesEA);
	DEALLOCATE(pclStates);
}

void applyContactImpulseExRig(SortData *sorts, u32 numSorts, const PclContactSolverConfig& config, u32 commonBuffAddr, f32 timeStep, bool twoWayInteraction)
{
	(void) commonBuffAddr;

	//PRINTF("applyContactImpulseExRig ----------------------------------------\n");

	s32 tag1 = 10;
	s32 numBatch = 5;
	u32 pairAddr = solverIOPcl.contactsRigAddr;
	u32 bodyAddr = solverIOPcl.bodiesAddr;
	PclContactPair *pairBuffer[2];
	spu_dma_list_element *pairDmaListBuf[3];
	TrbDynBody *bodyBuffer[2];
	spu_dma_list_element *bodyDmaListBuf[3];

	pairBuffer[0] = (PclContactPair*)ALLOCATE(128, sizeof(PclContactPair)*numBatch);
	pairBuffer[1] = (PclContactPair*)ALLOCATE(128, sizeof(PclContactPair)*numBatch);
	pairDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch);// PUT

	bodyBuffer[0] = (TrbDynBody*)ALLOCATE(128, sizeof(TrbDynBody)*numBatch);
	bodyBuffer[1] = (TrbDynBody*)ALLOCATE(128, sizeof(TrbDynBody)*numBatch);
	bodyDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch);
	bodyDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch);

	s32 currData = 0;
	s32 numPairs[2] = {0};
	for(s32 j=0;j < numBatch;j++) {
		pairDmaListBuf[0][j].notify = 0;
		pairDmaListBuf[0][j].size = sizeof(PclContactPair);
		pairDmaListBuf[1][j].notify = 0;
		pairDmaListBuf[1][j].size = sizeof(PclContactPair);
		bodyDmaListBuf[0][j].notify = 0;
		bodyDmaListBuf[0][j].size = sizeof(TrbDynBody);
		bodyDmaListBuf[1][j].notify = 0;
		bodyDmaListBuf[1][j].size = sizeof(TrbDynBody);
	}

	{
		Cache<PclState> pclStates(&gPool, solverIOPcl.pclStatesAddr, numSorts, solverIOPcl.numPclStates, 5, tag1);
		Cache<TrbState> rigStates(&gPool, solverIOPcl.rigStatesAddr, numSorts, solverIOPcl.numRigStates, 5, tag1);

		for(s32 j=0;j < numBatch && j < (s32)numSorts;j++) {
			pairDmaListBuf[0][numPairs[0]].eal = pairAddr + sizeof(PclContactPair)*getPair(sorts[j]);
			bodyDmaListBuf[0][numPairs[0]].eal = bodyAddr + sizeof(TrbDynBody)*getBodyB(sorts[j]);
			numPairs[0]++;

			pclStates.fetch(getStateA(sorts[j]));
			rigStates.fetch(getStateB(sorts[j]));
		}

		//PRINTF("%d ls:%x num:%d\n",0,nextRigStatesPtr,numBatchRigStates[0]);
		spu_dma_list_get(pairBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
		spu_dma_list_get(bodyBuffer[0], 0, bodyDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);

		pclStates.swap();
		rigStates.swap();

		spu_dma_wait_tag_status_all(1<<tag1);

		for(s32 j=numBatch;j < numBatch + numBatch && j < (s32)numSorts;j++) {
			pairDmaListBuf[1][numPairs[1]].eal = pairAddr + sizeof(PclContactPair)*getPair(sorts[j]);
			bodyDmaListBuf[1][numPairs[1]].eal = bodyAddr + sizeof(TrbDynBody)*getBodyB(sorts[j]);
			numPairs[1]++;

			pclStates.fetch(getStateA(sorts[j]));
			rigStates.fetch(getStateB(sorts[j]));
		}

		//PRINTF("%d ls:%x num:%d\n",1,nextRigStatesPtr,numBatchRigStates[1]);
		spu_dma_list_get(pairBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
		spu_dma_list_get(bodyBuffer[1], 0, bodyDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);

		pclStates.swap();
		rigStates.swap();

		currData = 0;

		for(u32 i=0;i < numSorts;i+=numBatch) {
			spu_dma_wait_tag_status_all(1<<tag1);

			// UPDATE
			for(s32 j=0;j < numBatch && (i + j) < numSorts;j++) {
				PclContactPair& pair = pairBuffer[currData][j];
				TrbDynBody& bodyB = bodyBuffer[currData][j];
				PclState& particleA = pclStates.get(getStateA(sorts[i+j]));
				TrbState& rigidbodyB = rigStates.get(getStateB(sorts[i+j]));

				Vector3 linVel(0.0f);
				Vector3 angVel(0.0f);
				applyImpulseRig(pair, particleA, rigidbodyB, config, bodyB, timeStep, twoWayInteraction, linVel, angVel);
				rigidbodyB.setLinearVelocity(rigidbodyB.getLinearVelocity() + linVel);
				rigidbodyB.setAngularVelocity(rigidbodyB.getAngularVelocity() + angVel);
			}

			// PUT & GET
			memcpy(pairDmaListBuf[2], pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]);
			spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[2], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

			numPairs[currData] = 0;

			for(u32 j=i + 2*numBatch;j < i + 3*numBatch && j < numSorts;j++) {
				pairDmaListBuf[currData][numPairs[currData]].eal = pairAddr + sizeof(PclContactPair)*getPair(sorts[j]);
				bodyDmaListBuf[currData][numPairs[currData]].eal = bodyAddr + sizeof(TrbDynBody)*getBodyB(sorts[j]);

				numPairs[currData]++;

				pclStates.fetch(getStateA(sorts[j]));
				rigStates.fetch(getStateB(sorts[j]));
			}

			//PRINTF("%d ls:%x num:%d\n",currData,nextRigStatesPtr,numBatchRigStates[currData]);
			spu_dma_list_getf(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
			spu_dma_list_get(bodyBuffer[currData], 0, bodyDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

			pclStates.swap();
			rigStates.swap();

			currData = 1 - currData;
		}

		spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

		pclStates.putAll();
		rigStates.putAll();

		spu_dma_wait_tag_status_all(1<<tag1);
	}

	DEALLOCATE(bodyDmaListBuf[1]);
	DEALLOCATE(bodyDmaListBuf[0]);
	DEALLOCATE(bodyBuffer[1]);
	DEALLOCATE(bodyBuffer[0]);

	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);
	DEALLOCATE(pairBuffer[1]);
	DEALLOCATE(pairBuffer[0]);
}

void applyContactImpulsePhaseEx(uint32_t commonBuffAddr)
{
	PclContactSolverConfig config;
	config.timeStep = solverIOPcl.timeStep;
	config.separateBias = solverIOPcl.separateBias;

	spu_dma_list_element *dmaListPairs = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*MAX_SOLVER_PAIRS);
	for(s32 i=0;i < MAX_SOLVER_PAIRS;i++) {
		dmaListPairs[i].notify = 0;
		dmaListPairs[i].size = sizeof(SortData);
	}

	for(u32 phaseId=0;phaseId < contactSolverInfoPcl->numPhases;phaseId++) {
		for(u32 groupId=0;groupId < contactSolverInfoPcl->numGroups[phaseId];groupId++) {
			u32 numPairs = contactSolverInfoPcl->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId];
			if(groupId%solverIOPcl.numSPU == taskId && numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				spu_dma_get(group, solverIOPcl.contactGroupsPclAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId), sizeof(SolverGroup), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128, sizeof(SortData)*numPairs);
				for(u32 i=0;i < numPairs;i++)
					dmaListPairs[i].eal = solverIOPcl.contactSortsPclAddr + sizeof(SortData) * group->pairIndices[i];

				spu_dma_list_get(sortBuff, 0, dmaListPairs,sizeof(spu_dma_list_element)*numPairs, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				applyContactImpulseExPcl(sortBuff, numPairs, config);

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	for(u32 phaseId=0;phaseId < contactSolverInfoRig->numPhases;phaseId++) {
		for(u32 groupId=0;groupId < contactSolverInfoRig->numGroups[phaseId];groupId++) {
			u32 numPairs = contactSolverInfoRig->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId];
			if(groupId%solverIOPcl.numSPU == taskId && numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				spu_dma_get(group, solverIOPcl.contactGroupsRigAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId), sizeof(SolverGroup), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128, sizeof(SortData)*numPairs);
				for(u32 i=0;i < numPairs;i++)
					dmaListPairs[i].eal = solverIOPcl.contactSortsRigAddr + sizeof(SortData) * group->pairIndices[i];

				spu_dma_list_get(sortBuff, 0, dmaListPairs, sizeof(spu_dma_list_element)*numPairs, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				applyContactImpulseExRig(sortBuff, numPairs, config, commonBuffAddr, solverIOPcl.timeStep, solverIOPcl.twoWayInteraction);

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	DEALLOCATE(dmaListPairs);
}

#ifdef PARTICLE_VELOCITY_BASE

//  state,contactPairをプリフェッチ
void preContactImpulseExPcl(SortData *sorts,uint32_t numSorts,const PclContactSolverConfig &config)
{
	int tag1 __attribute__((aligned(16))) = 10;

	// ---------------------------------------------------------------------------
	// メモリ確保

	PclState *pclStates = (PclState*)ALLOCATE(128,sizeof(PclState)*numSorts*2);

	// ---------------------------------------------------------------------------
	// パーティクルステート

	uint32_t numBatch __attribute__((aligned(16))) = 5;
	uint8_t numStates  __attribute__((aligned(16))) = 0;

	uint8_t *pclStateRefTable  __attribute__((aligned(16))) = (uint8_t*)ALLOCATE(16,sizeof(uint8_t)*solverIOPcl.numPclStates);

	memset(pclStateRefTable,0xff,sizeof(uint8_t)*solverIOPcl.numPclStates);

	CellDmaListElement *pclStateDmaList[2] __attribute__((aligned(16)));

	int numBatchPclStates[2] = {0};

	pclStateDmaList[0] = (CellDmaListElement*)ALLOCATE(16,sizeof(CellDmaListElement)*numBatch*2);
	pclStateDmaList[1] = (CellDmaListElement*)ALLOCATE(16,sizeof(CellDmaListElement)*numBatch*2);

	for(uint32_t j=0;j<numBatch*2;j++) {
		pclStateDmaList[0][j].notify = pclStateDmaList[1][j].notify = 0;
		pclStateDmaList[0][j].size = pclStateDmaList[1][j].size = sizeof(PclState);
	}

	// ---------------------------------------------------------------------------
	// データ処理

	uint32_t pairAddr __attribute__((aligned(16))) = solverIOPcl.contactsPclAddr;

	PclContactPair *pairBuffer[2]		__attribute__((aligned(16)));
	CellDmaListElement *pairDmaListBuf[3]	__attribute__((aligned(16)));

	pairBuffer[0] = (PclContactPair*)ALLOCATE(128,sizeof(PclContactPair)*numBatch);
	pairBuffer[1] = (PclContactPair*)ALLOCATE(128,sizeof(PclContactPair)*numBatch);
	pairDmaListBuf[0] = (CellDmaListElement*)ALLOCATE(128,sizeof(CellDmaListElement)*numBatch);
	pairDmaListBuf[1] = (CellDmaListElement*)ALLOCATE(128,sizeof(CellDmaListElement)*numBatch);
	pairDmaListBuf[2] = (CellDmaListElement*)ALLOCATE(128,sizeof(CellDmaListElement)*numBatch);// PUT用

	int batchFlag = 0;
	int currData = 0;
	int numPairs[2] = {0};

	PclState *nextPclStatesPtr = pclStates;

	// あらかじめサイズをセット
	for(uint32_t j=0;j<numBatch;j++) {
		pairDmaListBuf[0][j].notify = 0;
		pairDmaListBuf[0][j].size = sizeof(PclContactPair);
		pairDmaListBuf[1][j].notify = 0;
		pairDmaListBuf[1][j].size = sizeof(PclContactPair);
	}

	// 初期状態
	for(uint32_t j=0;j<numBatch&&j<numSorts;j++) {
		pairDmaListBuf[0][numPairs[0]].eal = pairAddr + sizeof(PclContactPair)*Pair(sorts[j]);
		numPairs[0]++;

		uint16_t stateA = StateA(sorts[j]);
		uint16_t stateB = StateB(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			pclStateDmaList[0][numBatchPclStates[0]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateRefTable[stateA] = numStates;
			numBatchPclStates[0]++;
			numStates++;
		}

		if(pclStateRefTable[stateB] == 0xff) {
			pclStateDmaList[0][numBatchPclStates[0]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
			pclStateRefTable[stateB] = numStates;
			numBatchPclStates[0]++;
			numStates++;
		}
	}

	if(numPairs[0] > 0) {
		cellDmaListGet(pairBuffer[0],0,pairDmaListBuf[0],sizeof(CellDmaListElement)*numPairs[0],tag1,0,0);
	}
	if(numBatchPclStates[0] > 0) {
		cellDmaListGet(pclStates,0,pclStateDmaList[0],sizeof(CellDmaListElement)*numBatchPclStates[0],tag1,0,0);
	}

	nextPclStatesPtr += numBatchPclStates[0];

	cellDmaWaitTagStatusAll(1<<tag1);

	for(uint32_t j=numBatch;j<numBatch+numBatch&&j<numSorts;j++) {
		pairDmaListBuf[1][numPairs[1]].eal = pairAddr + sizeof(PclContactPair)*Pair(sorts[j]);
		numPairs[1]++;

		uint16_t stateA = StateA(sorts[j]);
		uint16_t stateB = StateB(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			pclStateDmaList[1][numBatchPclStates[1]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateRefTable[stateA] = numStates;
			numBatchPclStates[1]++;
			numStates++;
		}

		if(pclStateRefTable[stateB] == 0xff) {
			pclStateDmaList[1][numBatchPclStates[1]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
			pclStateRefTable[stateB] = numStates;
			numBatchPclStates[1]++;
			numStates++;
		}
	}

	if(numPairs[1] > 0) {
		cellDmaListGet(pairBuffer[1],0,pairDmaListBuf[1],sizeof(CellDmaListElement)*numPairs[1],tag1,0,0);
	}
	if(numBatchPclStates[1] > 0) {
		cellDmaListGet(nextPclStatesPtr,0,pclStateDmaList[1],sizeof(CellDmaListElement)*numBatchPclStates[1],tag1,0,0);
	}

	nextPclStatesPtr += numBatchPclStates[1];

	currData = 0;
	batchFlag = 1;

	// プリフェッチ
	for(uint32_t i=0;i<numSorts;) {
		// UPDATE
		for(int j=0;j<numPairs[currData];j++) {
			PclContactPair &pair = pairBuffer[currData][j];
			PclState &particleA = pclStates[pclStateRefTable[pair.stateIndex[0]]];
			PclState &particleB = pclStates[pclStateRefTable[pair.stateIndex[1]]];

			pair.compositeElasticity = 0.5f * (particleA.getElasticity() + particleB.getElasticity());
			pair.compositeFriction = sqrtf(particleA.getFriction() * particleB.getFriction());
			pair.contactPoint.impulseDen = 1.0f / (1.0f/particleA.getMass() + 1.0f/particleB.getMass());
		}

		i+=numBatch;

		// PUT & GET

		// 前回のDMA転送待ち
		cellDmaWaitTagStatusAll(1<<tag1);

		// --------------------------------------------------------------
		// ペアの更新

		if(numPairs[currData] > 0) {
			memcpy(pairDmaListBuf[2],pairDmaListBuf[currData],sizeof(CellDmaListElement)*numPairs[currData]);
			cellDmaListPut(pairBuffer[currData],0,pairDmaListBuf[2],sizeof(CellDmaListElement)*numPairs[currData],tag1,0,0);
		}

		numPairs[currData] = 0;
		numBatchPclStates[currData] = 0;

		batchFlag++;
		int start = batchFlag * numBatch;

		for(uint32_t j=start;j<start+numBatch&&j<numSorts;j++) {
			pairDmaListBuf[currData][numPairs[currData]].eal = pairAddr + sizeof(PclContactPair)*Pair(sorts[j]);
			numPairs[currData]++;

			uint16_t stateA = StateA(sorts[j]);
			uint16_t stateB = StateB(sorts[j]);

			// State
			if(pclStateRefTable[stateA] == 0xff) {
				pclStateDmaList[currData][numBatchPclStates[currData]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
				pclStateRefTable[stateA] = numStates;
				numBatchPclStates[currData]++;
				numStates++;
			}

			if(pclStateRefTable[stateB] == 0xff) {
				pclStateDmaList[currData][numBatchPclStates[currData]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
				pclStateRefTable[stateB] = numStates;
				numBatchPclStates[currData]++;
				numStates++;
			}
		}

		if(numPairs[currData] > 0) {
			cellDmaListGetf(pairBuffer[currData],0,pairDmaListBuf[currData],sizeof(CellDmaListElement)*numPairs[currData],tag1,0,0);
		}
		if(numBatchPclStates[currData] > 0) {
			cellDmaListGet(nextPclStatesPtr,0,pclStateDmaList[currData],sizeof(CellDmaListElement)*numBatchPclStates[currData],tag1,0,0);
		}

		nextPclStatesPtr += numBatchPclStates[currData];

		currData = 1-currData;
	}

	// 残ったデータを反映する
	if(numPairs[currData] > 0) {
		cellDmaListPut(pairBuffer[currData],0,pairDmaListBuf[currData],sizeof(CellDmaListElement)*numPairs[currData],tag1,0,0);
	}

	DEALLOCATE(pclStateDmaList[1]);
	DEALLOCATE(pclStateDmaList[0]);

	DEALLOCATE(pclStateRefTable);

	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);
	DEALLOCATE(pairBuffer[1]);
	DEALLOCATE(pairBuffer[0]);

	DEALLOCATE(pclStates);
}

//  body,contactPairをプリフェッチ
void preContactImpulseExRig(SortData *sorts,uint32_t numSorts,const PclContactSolverConfig &config)
{
	int tag1 __attribute__((aligned(16))) = 10;

	// ---------------------------------------------------------------------------
	// メモリ確保

	PclState *pclStates = (PclState*)ALLOCATE(128,sizeof(PclState)*numSorts);

	// ---------------------------------------------------------------------------
	// 剛体ステート、剛体

	uint32_t numBatch __attribute__((aligned(16))) = 5;

	uint8_t numPclStates  __attribute__((aligned(16))) = 0;

	uint8_t *pclStateRefTable  __attribute__((aligned(16))) = (uint8_t*)ALLOCATE(16,sizeof(uint8_t)*solverIOPcl.numPclStates);

	memset(pclStateRefTable,0xff,sizeof(uint8_t)*solverIOPcl.numPclStates);

	CellDmaListElement *pclStateDmaList[2] __attribute__((aligned(16)));

	int numBatchPclStates[2] = {0};

	pclStateDmaList[0] = (CellDmaListElement*)ALLOCATE(16,sizeof(CellDmaListElement)*numBatch);
	pclStateDmaList[1] = (CellDmaListElement*)ALLOCATE(16,sizeof(CellDmaListElement)*numBatch);

	for(uint32_t j=0;j<numBatch;j++) {
		pclStateDmaList[0][j].notify = pclStateDmaList[1][j].notify = 0;
		pclStateDmaList[0][j].size = pclStateDmaList[1][j].size = sizeof(PclState);
	}

	// ---------------------------------------------------------------------------
	// データ処理

	uint32_t pairAddr __attribute__((aligned(16))) = solverIOPcl.contactsRigAddr;
	uint32_t bodyAddr __attribute__((aligned(16))) = solverIOPcl.bodiesAddr;

	PclContactPair *pairBuffer[2]		__attribute__((aligned(16)));
	CellDmaListElement *pairDmaListBuf[3]	__attribute__((aligned(16)));

	TrbDynBody *bodyBuffer[2] __attribute__((aligned(16)));
	CellDmaListElement *bodyDmaListBuf[2]	__attribute__((aligned(16)));

	pairBuffer[0] = (PclContactPair*)ALLOCATE(128,sizeof(PclContactPair)*numBatch);
	pairBuffer[1] = (PclContactPair*)ALLOCATE(128,sizeof(PclContactPair)*numBatch);
	pairDmaListBuf[0] = (CellDmaListElement*)ALLOCATE(128,sizeof(CellDmaListElement)*numBatch);
	pairDmaListBuf[1] = (CellDmaListElement*)ALLOCATE(128,sizeof(CellDmaListElement)*numBatch);
	pairDmaListBuf[2] = (CellDmaListElement*)ALLOCATE(128,sizeof(CellDmaListElement)*numBatch);// PUT用

	bodyBuffer[0] = (TrbDynBody*)ALLOCATE(128,sizeof(TrbDynBody)*numBatch);
	bodyBuffer[1] = (TrbDynBody*)ALLOCATE(128,sizeof(TrbDynBody)*numBatch);
	bodyDmaListBuf[0] = (CellDmaListElement*)ALLOCATE(128,sizeof(CellDmaListElement)*numBatch);
	bodyDmaListBuf[1] = (CellDmaListElement*)ALLOCATE(128,sizeof(CellDmaListElement)*numBatch);

	int batchFlag = 0;
	int currData = 0;
	int numPairs[2] = {0};

	PclState *nextPclStatesPtr = pclStates;

	// あらかじめサイズをセット
	for(uint32_t j=0;j<numBatch;j++) {
		pairDmaListBuf[0][j].notify = 0;
		pairDmaListBuf[0][j].size = sizeof(PclContactPair);
		pairDmaListBuf[1][j].notify = 0;
		pairDmaListBuf[1][j].size = sizeof(PclContactPair);
		bodyDmaListBuf[0][j].notify = 0;
		bodyDmaListBuf[0][j].size = sizeof(TrbDynBody);
		bodyDmaListBuf[1][j].notify = 0;
		bodyDmaListBuf[1][j].size = sizeof(TrbDynBody);
	}

	// 初期状態
	for(uint32_t j=0;j<numBatch&&j<numSorts;j++) {
		pairDmaListBuf[0][numPairs[0]].eal = pairAddr + sizeof(PclContactPair)*Pair(sorts[j]);
		bodyDmaListBuf[0][numPairs[0]].eal = bodyAddr + sizeof(TrbDynBody)*BodyB(sorts[j]);
		numPairs[0]++;

		uint16_t stateA = StateA(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			pclStateDmaList[0][numBatchPclStates[0]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateRefTable[stateA] = numPclStates;
			numBatchPclStates[0]++;
			numPclStates++;
		}
	}

	if(numPairs[0] > 0) {
		cellDmaListGet(pairBuffer[0],0,pairDmaListBuf[0],sizeof(CellDmaListElement)*numPairs[0],tag1,0,0);
		cellDmaListGet(bodyBuffer[0],0,bodyDmaListBuf[0],sizeof(CellDmaListElement)*numPairs[0],tag1,0,0);
	}
	if(numBatchPclStates[0] > 0) {
		cellDmaListGet(pclStates,0,pclStateDmaList[0],sizeof(CellDmaListElement)*numBatchPclStates[0],tag1,0,0);
	}

	nextPclStatesPtr += numBatchPclStates[0];

	cellDmaWaitTagStatusAll(1<<tag1);

	for(uint32_t j=numBatch;j<numBatch+numBatch&&j<numSorts;j++) {
		pairDmaListBuf[1][numPairs[1]].eal = pairAddr + sizeof(PclContactPair)*Pair(sorts[j]);
		bodyDmaListBuf[1][numPairs[1]].eal = bodyAddr + sizeof(TrbDynBody)*BodyB(sorts[j]);
		numPairs[1]++;

		uint16_t stateA = StateA(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			pclStateDmaList[1][numBatchPclStates[1]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateRefTable[stateA] = numPclStates;
			numBatchPclStates[1]++;
			numPclStates++;
		}
	}

	if(numPairs[1] > 0) {
		cellDmaListGet(pairBuffer[1],0,pairDmaListBuf[1],sizeof(CellDmaListElement)*numPairs[1],tag1,0,0);
		cellDmaListGet(bodyBuffer[1],0,bodyDmaListBuf[1],sizeof(CellDmaListElement)*numPairs[1],tag1,0,0);
	}
	if(numBatchPclStates[1] > 0) {
		cellDmaListGet(nextPclStatesPtr,0,pclStateDmaList[1],sizeof(CellDmaListElement)*numBatchPclStates[1],tag1,0,0);
	}

	nextPclStatesPtr += numBatchPclStates[1];

	currData = 0;
	batchFlag = 1;

	// プリフェッチ
	for(uint32_t i=0;i<numSorts;) {
		// UPDATE
		for(int j=0;j<numPairs[currData];j++) {
			PclContactPair &pair = pairBuffer[currData][j];
			PclState &particleA = pclStates[pclStateRefTable[pair.stateIndex[0]]];
			TrbDynBody &bodyB = bodyBuffer[currData][j];

			pair.compositeElasticity = 0.5f * (particleA.getElasticity() + bodyB.getElasticity());
			pair.compositeFriction = sqrtf(particleA.getFriction() * bodyB.getFriction());
			pair.contactPoint.impulseDen = particleA.getMass();
		}

		i+=numBatch;

		// PUT & GET

		// 前回のDMA転送待ち
		cellDmaWaitTagStatusAll(1<<tag1);

		// --------------------------------------------------------------
		// ペアの更新

		if(numPairs[currData] > 0) {
			memcpy(pairDmaListBuf[2],pairDmaListBuf[currData],sizeof(CellDmaListElement)*numPairs[currData]);
			cellDmaListPut(pairBuffer[currData],0,pairDmaListBuf[2],sizeof(CellDmaListElement)*numPairs[currData],tag1,0,0);
		}

		numPairs[currData] = 0;
		numBatchPclStates[currData] = 0;

		batchFlag++;
		int start = batchFlag * numBatch;

		for(uint32_t j=start;j<start+numBatch&&j<numSorts;j++) {
			pairDmaListBuf[currData][numPairs[currData]].eal = pairAddr + sizeof(PclContactPair)*Pair(sorts[j]);
			bodyDmaListBuf[currData][numPairs[currData]].eal = bodyAddr + sizeof(TrbDynBody)*BodyB(sorts[j]);
			numPairs[currData]++;

			uint16_t stateA = StateA(sorts[j]);

			// State
			if(pclStateRefTable[stateA] == 0xff) {
				pclStateDmaList[currData][numBatchPclStates[currData]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
				pclStateRefTable[stateA] = numPclStates;
				numBatchPclStates[currData]++;
				numPclStates++;
			}
		}

		if(numPairs[currData] > 0) {
			cellDmaListGetf(pairBuffer[currData],0,pairDmaListBuf[currData],sizeof(CellDmaListElement)*numPairs[currData],tag1,0,0);
			cellDmaListGet(bodyBuffer[currData],0,bodyDmaListBuf[currData],sizeof(CellDmaListElement)*numPairs[currData],tag1,0,0);
		}
		if(numBatchPclStates[currData] > 0) {
			cellDmaListGet(nextPclStatesPtr,0,pclStateDmaList[currData],sizeof(CellDmaListElement)*numBatchPclStates[currData],tag1,0,0);
		}

		nextPclStatesPtr += numBatchPclStates[currData];

		currData = 1-currData;
	}

	// 残ったデータを反映する
	if(numPairs[currData] > 0) {
		cellDmaListPut(pairBuffer[currData],0,pairDmaListBuf[currData],sizeof(CellDmaListElement)*numPairs[currData],tag1,0,0);
	}

	DEALLOCATE(bodyDmaListBuf[1]);
	DEALLOCATE(bodyDmaListBuf[0]);
	DEALLOCATE(bodyBuffer[1]);
	DEALLOCATE(bodyBuffer[0]);

	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);
	DEALLOCATE(pairBuffer[1]);
	DEALLOCATE(pairBuffer[0]);

	DEALLOCATE(pclStateDmaList[1]);
	DEALLOCATE(pclStateDmaList[0]);
	DEALLOCATE(pclStateRefTable);

	DEALLOCATE(pclStates);
}

void preContactImpulsePhaseEx()
{
	PclContactSolverConfig config;
	config.timeStep = solverIOPcl.timeStep;
	config.separateBias = solverIOPcl.separateBias;

	CellDmaListElement *dmaListPairs = (CellDmaListElement*)ALLOCATE(16,sizeof(CellDmaListElement)*MAX_SOLVER_PAIRS);
	for(int i=0;i<MAX_SOLVER_PAIRS;i++) {
		dmaListPairs[i].notify = 0;
		dmaListPairs[i].size = sizeof(SortData);
	}

	for(uint32_t phaseId=0;phaseId<contactSolverInfoPcl->numPhases;phaseId++) {
		for(uint32_t groupId=0;groupId<contactSolverInfoPcl->numGroups[phaseId];groupId++) {
			uint32_t numPairs = contactSolverInfoPcl->numPairs[phaseId*MAX_SOLVER_GROUPS+groupId];
			if(groupId%solverIOPcl.numSPU == taskId && numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128,sizeof(SolverGroup));
				cellDmaGet((uint32_t)group,
					solverIOPcl.contactGroupsPclAddr + sizeof(SolverGroup) * (phaseId*MAX_SOLVER_GROUPS+groupId),
					sizeof(SolverGroup),0,0,0);
				cellDmaWaitTagStatusAll(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128,sizeof(SortData)*numPairs);

				for(uint32_t i=0;i<numPairs;i++) {
					dmaListPairs[i].eal = solverIOPcl.contactSortsPclAddr + sizeof(SortData) * group->pairIndices[i];
				}

				cellDmaListGet(sortBuff,0,dmaListPairs,sizeof(CellDmaListElement)*numPairs,0,0,0);
				cellDmaWaitTagStatusAll(1);

				preContactImpulseExPcl(sortBuff,numPairs,config);

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	for(uint32_t phaseId=0;phaseId<contactSolverInfoRig->numPhases;phaseId++) {
		for(uint32_t groupId=0;groupId<contactSolverInfoRig->numGroups[phaseId];groupId++) {
			uint32_t numPairs = contactSolverInfoRig->numPairs[phaseId*MAX_SOLVER_GROUPS+groupId];
			if(groupId%solverIOPcl.numSPU == taskId && numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128,sizeof(SolverGroup));
				cellDmaGet((uint32_t)group,
					solverIOPcl.contactGroupsRigAddr + sizeof(SolverGroup) * (phaseId*MAX_SOLVER_GROUPS+groupId),
					sizeof(SolverGroup),0,0,0);
				cellDmaWaitTagStatusAll(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128,sizeof(SortData)*numPairs);

				for(uint32_t i=0;i<numPairs;i++) {
					dmaListPairs[i].eal = solverIOPcl.contactSortsRigAddr + sizeof(SortData) * group->pairIndices[i];
				}

				cellDmaListGet(sortBuff,0,dmaListPairs,sizeof(CellDmaListElement)*numPairs,0,0,0);
				cellDmaWaitTagStatusAll(1);

				preContactImpulseExRig(sortBuff,numPairs,config);

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	DEALLOCATE(dmaListPairs);
}

#endif // PARTICLE_VELOCITY_BASE

#endif /* CONTACTPHASE_H_ */
