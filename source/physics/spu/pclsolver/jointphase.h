/*
 * jointphase.h
 *
 *  Created on: Jan 28, 2014
 *      Author: mike
 */

#ifndef JOINTPHASE_H_
#define JOINTPHASE_H_

void applyJointImpulseExPcl(SortData *sorts, u32 numSorts, const PclJointSolverConfig& config)
{
	s32 tag1 = 10;
	PclState *pclStates = (PclState*)ALLOCATE(128, sizeof(PclState)*numSorts*2);
	u32 *statesEA = (u32*)ALLOCATE(16, sizeof(uint32_t)*numSorts*2);
	u32 numBatch = 5;
	u8 numStates = 0;
	s32 numBatchStates[2] = {0};
	u8 *pclStateRefTable = (uint8_t*)ALLOCATE(16, sizeof(u8)*solverIOPcl.numPclStates);
	spu_dma_list_element *pclStateDmaList[2];

	memset(pclStateRefTable, 0xff, sizeof(u8)*solverIOPcl.numPclStates);

	pclStateDmaList[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	pclStateDmaList[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);

	for(u32 j=0;j < numBatch*2;j++) {
		pclStateDmaList[0][j].notify = pclStateDmaList[1][j].notify = 0;
		pclStateDmaList[0][j].size = pclStateDmaList[1][j].size = sizeof(PclState);
	}

	u32 pairAddr = solverIOPcl.jointsAddr;

	PclJoint *jointBuffer[2];
	spu_dma_list_element *pairDmaListBuf[3];

	jointBuffer[0] = (PclJoint*)ALLOCATE(128, sizeof(PclJoint)*numBatch);
	jointBuffer[1] = (PclJoint*)ALLOCATE(128, sizeof(PclJoint)*numBatch);
	pairDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);// PUT

	s32 batchFlag = 0;
	s32 currData = 0;
	s32 numPairs[2] = {0};

	PclState *oldStatesPtr = pclStates;

	for(u32 j=0;j < numBatch;j++) {
		pairDmaListBuf[0][j].notify = 0;
		pairDmaListBuf[0][j].size = sizeof(PclJoint);
		pairDmaListBuf[1][j].notify = 0;
		pairDmaListBuf[1][j].size = sizeof(PclJoint);
	}

	for(u32 j=0;j < numBatch && j < numSorts;j++) {
		pairDmaListBuf[0][numPairs[0]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
		numPairs[0]++;

		u16 stateA = getStateA(sorts[j]);
		u16 stateB = getStateB(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			statesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateDmaList[0][numBatchStates[0]].eal = statesEA[numStates];
			pclStateRefTable[stateA] = numStates;
			numBatchStates[0]++;
			numStates++;
		}

		if(pclStateRefTable[stateB] == 0xff) {
			statesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
			pclStateDmaList[0][numBatchStates[0]].eal = statesEA[numStates];
			pclStateRefTable[stateB] = numStates;
			numBatchStates[0]++;
			numStates++;
		}
	}

	if(numPairs[0] > 0)
		spu_dma_list_get(jointBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);

	if(numBatchStates[0] > 0)
		spu_dma_list_get(pclStates, 0, pclStateDmaList[0], sizeof(spu_dma_list_element)*numBatchStates[0], tag1, 0, 0);

	oldStatesPtr += numBatchStates[0];

	spu_dma_wait_tag_status_all(1<<tag1);

	for(u32 j=numBatch;j < numBatch+numBatch && j < numSorts;j++) {
		pairDmaListBuf[1][numPairs[1]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
		numPairs[1]++;

		u16 stateA = getStateA(sorts[j]);
		u16 stateB = getStateB(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			statesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateDmaList[1][numBatchStates[1]].eal = statesEA[numStates];
			pclStateRefTable[stateA] = numStates;
			numBatchStates[1]++;
			numStates++;
		}

		if(pclStateRefTable[stateB] == 0xff) {
			statesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
			pclStateDmaList[1][numBatchStates[1]].eal = statesEA[numStates];
			pclStateRefTable[stateB] = numStates;
			numBatchStates[1]++;
			numStates++;
		}
	}

	if(numPairs[1] > 0)
		spu_dma_list_get(jointBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);

	if(numBatchStates[1] > 0)
		spu_dma_list_get(oldStatesPtr, 0, pclStateDmaList[1], sizeof(spu_dma_list_element)*numBatchStates[1], tag1, 0, 0);

	oldStatesPtr += numBatchStates[1];

	batchFlag = 1;
	currData = 0;

	for(u32 i=0;i < numSorts;) {
		// UPDATE
		for(s32 j=0;j < numPairs[currData];j++) {
			PclJoint& joint = jointBuffer[currData][j];
			PclState& particleA = pclStates[pclStateRefTable[joint.stateIndexA]];
			PclState& particleB = pclStates[pclStateRefTable[joint.stateIndexB]];

			applyJointPcl(joint, particleA, particleB, config);
		}

		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		if(numPairs[currData] > 0) {
			memcpy(pairDmaListBuf[2], pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]);
			spu_dma_list_put(jointBuffer[currData], 0, pairDmaListBuf[2], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		}

		numPairs[currData] = 0;
		numBatchStates[currData] = 0;

		batchFlag++;
		s32 start = batchFlag*numBatch;
		for(u32 j=start;j < start + numBatch && j < numSorts;j++) {
			pairDmaListBuf[currData][numPairs[currData]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
			numPairs[currData]++;

			u16 stateA = getStateA(sorts[j]);
			u16 stateB = getStateB(sorts[j]);

			// State
			if(pclStateRefTable[stateA] == 0xff) {
				statesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
				pclStateDmaList[currData][numBatchStates[currData]].eal = statesEA[numStates];
				pclStateRefTable[stateA] = numStates;
				numBatchStates[currData]++;
				numStates++;
			}

			if(pclStateRefTable[stateB] == 0xff) {
				statesEA[numStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
				pclStateDmaList[currData][numBatchStates[currData]].eal = statesEA[numStates];
				pclStateRefTable[stateB] = numStates;
				numBatchStates[currData]++;
				numStates++;
			}
		}

		if(numPairs[currData] > 0)
			spu_dma_list_getf(jointBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

		if(numBatchStates[currData] > 0)
			spu_dma_list_get(oldStatesPtr, 0, pclStateDmaList[currData], sizeof(spu_dma_list_element)*numBatchStates[currData], tag1, 0, 0);

		oldStatesPtr += numBatchStates[currData];

		currData = 1 - currData;
	}

	if(numPairs[currData] > 0)
		spu_dma_list_put(jointBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

	spu_dma_list_element *dmaList = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numStates);
	if(numStates > 0) {
		for(s32 i=0;i < numStates;i++) {
			dmaList[i].notify = 0;
			dmaList[i].eal = statesEA[i];
			dmaList[i].size = sizeof(PclState);
		}
		spu_dma_list_put(pclStates, 0, dmaList, sizeof(spu_dma_list_element)*numStates, tag1, 0, 0);
	}
	spu_dma_wait_tag_status_all(1<<tag1);

	DEALLOCATE(dmaList);

	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);
	DEALLOCATE(jointBuffer[1]);
	DEALLOCATE(jointBuffer[0]);

	DEALLOCATE(pclStateDmaList[1]);
	DEALLOCATE(pclStateDmaList[0]);

	DEALLOCATE(pclStateRefTable);
	DEALLOCATE(statesEA);
	DEALLOCATE(pclStates);
}

void applyJointImpulseExRig(SortData *sorts, u32 numSorts, const PclJointSolverConfig& config)
{
	s32 tag1 = 10;
	PclState *pclStates = (PclState*)ALLOCATE(128, sizeof(PclState)*numSorts);
	u32 *statesEA = (uint32_t*)ALLOCATE(16, sizeof(uint32_t)*numSorts);
	u32 numBatch = 5;
	u32 numPclStates = 0;
	s32 numBatchStates[2] = {0};
	spu_dma_list_element *pclStateDmaList[2];
	u8 *pclStateRefTable = (u8*)ALLOCATE(16, sizeof(u8)*solverIOPcl.numPclStates);

	memset(pclStateRefTable, 0xff, sizeof(u8)*solverIOPcl.numPclStates);

	pclStateDmaList[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	pclStateDmaList[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);

	for(u32 j=0;j < numBatch*2;j++) {
		pclStateDmaList[0][j].notify = pclStateDmaList[1][j].notify = 0;
		pclStateDmaList[0][j].size = pclStateDmaList[1][j].size = sizeof(PclState);
	}

	u32 pairAddr = solverIOPcl.jointsAddr;
	PclJoint *jointBuffer[2];
	spu_dma_list_element *pairDmaListBuf[3];
	TrbState *rigStateBuffer[2];
	spu_dma_list_element *rigStateDmaListBuf[2];

	jointBuffer[0] = (PclJoint*)ALLOCATE(128, sizeof(PclJoint)*numBatch);
	jointBuffer[1] = (PclJoint*)ALLOCATE(128, sizeof(PclJoint)*numBatch);
	pairDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);// PUT

	rigStateBuffer[0] = (TrbState*)ALLOCATE(128, sizeof(TrbState)*numBatch);
	rigStateBuffer[1] = (TrbState*)ALLOCATE(128, sizeof(TrbState)*numBatch);
	rigStateDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	rigStateDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);

	s32 batchFlag = 0;
	s32 currData = 0;
	s32 numPairs[2] = {0};
	PclState *oldStatesPtr = pclStates;

	for(u32 j=0;j < numBatch;j++) {
		pairDmaListBuf[0][j].notify = 0;
		pairDmaListBuf[0][j].size = sizeof(PclJoint);
		pairDmaListBuf[1][j].notify = 0;
		pairDmaListBuf[1][j].size = sizeof(PclJoint);
		rigStateDmaListBuf[0][j].notify = 0;
		rigStateDmaListBuf[0][j].size = sizeof(TrbState);
		rigStateDmaListBuf[1][j].notify = 0;
		rigStateDmaListBuf[1][j].size = sizeof(TrbState);
	}

	for(u32 j=0;j < numBatch && j < numSorts;j++) {
		pairDmaListBuf[0][numPairs[0]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
		rigStateDmaListBuf[0][numPairs[0]].eal = solverIOPcl.rigStatesAddr + sizeof(TrbState)*getStateB(sorts[j]);
		numPairs[0]++;

		u16 stateA = getStateA(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			statesEA[numPclStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateDmaList[0][numBatchStates[0]].eal = statesEA[numPclStates];
			pclStateRefTable[stateA] = numPclStates;
			numBatchStates[0]++;
			numPclStates++;
		}
	}

	if(numPairs[0] > 0) {
		spu_dma_list_get(jointBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
		spu_dma_list_get(rigStateBuffer[0], 0, rigStateDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
	}

	if(numBatchStates[0] > 0)
		spu_dma_list_get(pclStates, 0, pclStateDmaList[0], sizeof(spu_dma_list_element)*numBatchStates[0], tag1, 0, 0);

	oldStatesPtr += numBatchStates[0];

	spu_dma_wait_tag_status_all(1<<tag1);

	for(u32 j=numBatch;j < numBatch+numBatch && j < numSorts;j++) {
		pairDmaListBuf[1][numPairs[1]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
		rigStateDmaListBuf[1][numPairs[1]].eal = solverIOPcl.rigStatesAddr + sizeof(TrbState)*getStateB(sorts[j]);
		numPairs[1]++;

		u16 stateA = getStateA(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			statesEA[numPclStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateDmaList[1][numBatchStates[1]].eal = statesEA[numPclStates];
			pclStateRefTable[stateA] = numPclStates;
			numBatchStates[1]++;
			numPclStates++;
		}
	}

	if(numPairs[1] > 0) {
		spu_dma_list_get(jointBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
		spu_dma_list_get(rigStateBuffer[1], 0, rigStateDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
	}

	if(numBatchStates[1] > 0)
		spu_dma_list_get(oldStatesPtr, 0, pclStateDmaList[1], sizeof(spu_dma_list_element)*numBatchStates[1], tag1, 0, 0);

	oldStatesPtr += numBatchStates[1];

	batchFlag = 1;
	currData = 0;

	for(u32 i=0;i < numSorts;) {
		// UPDATE
		for(s32 j=0;j < numPairs[currData];j++) {
			PclJoint& joint = jointBuffer[currData][j];
			PclState& particleA = pclStates[pclStateRefTable[joint.stateIndexA]];
			TrbState& rigidbodyB = rigStateBuffer[currData][j];

			applyJointRig(joint, particleA, rigidbodyB, config);
		}

		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		if(numPairs[currData] > 0) {
			memcpy(pairDmaListBuf[2], pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]);
			spu_dma_list_put(jointBuffer[currData], 0, pairDmaListBuf[2],sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		}

		numPairs[currData] = 0;
		numBatchStates[currData] = 0;

		batchFlag++;
		s32 start = batchFlag*numBatch;
		for(u32 j=start;j < start + numBatch && j < numSorts;j++) {
			pairDmaListBuf[currData][numPairs[currData]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
			rigStateDmaListBuf[currData][numPairs[currData]].eal = solverIOPcl.rigStatesAddr + sizeof(TrbState)*getStateB(sorts[j]);
			numPairs[currData]++;

			u16 stateA = getStateA(sorts[j]);

			// State
			if(pclStateRefTable[stateA] == 0xff) {
				statesEA[numPclStates] = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
				pclStateDmaList[currData][numBatchStates[currData]].eal = statesEA[numPclStates];
				pclStateRefTable[stateA] = numPclStates;
				numBatchStates[currData]++;
				numPclStates++;
			}
		}

		if(numPairs[currData] > 0) {
			spu_dma_list_getf(jointBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
			spu_dma_list_get(rigStateBuffer[currData], 0, rigStateDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		}

		if(numBatchStates[currData] > 0)
			spu_dma_list_get(oldStatesPtr, 0, pclStateDmaList[currData], sizeof(spu_dma_list_element)*numBatchStates[currData], tag1, 0, 0);

		oldStatesPtr += numBatchStates[currData];

		currData = 1 - currData;
	}

	if(numPairs[currData] > 0)
		spu_dma_list_put(jointBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

	spu_dma_list_element *dmaList = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numPclStates);
	if(numPclStates > 0) {
		for(u32 i=0;i < numPclStates;i++) {
			dmaList[i].notify = 0;
			dmaList[i].eal = statesEA[i];
			dmaList[i].size = sizeof(PclState);
		}
		spu_dma_list_put(pclStates, 0, dmaList, sizeof(spu_dma_list_element)*numPclStates, tag1, 0, 0);
	}
	spu_dma_wait_tag_status_all(1<<tag1);


	DEALLOCATE(dmaList);

	DEALLOCATE(rigStateDmaListBuf[1]);
	DEALLOCATE(rigStateDmaListBuf[0]);
	DEALLOCATE(rigStateBuffer[1]);
	DEALLOCATE(rigStateBuffer[0]);

	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);
	DEALLOCATE(jointBuffer[1]);
	DEALLOCATE(jointBuffer[0]);

	DEALLOCATE(pclStateDmaList[1]);
	DEALLOCATE(pclStateDmaList[0]);

	DEALLOCATE(pclStateRefTable);
	DEALLOCATE(statesEA);
	DEALLOCATE(pclStates);
}

void applyJointImpulsePhaseEx()
{
	PclJointSolverConfig config;
	config.timeStep = solverIOPcl.timeStep;

	spu_dma_list_element *dmaListPairs = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*MAX_SOLVER_PAIRS);
	for(s32 i=0;i < MAX_SOLVER_PAIRS;i++) {
		dmaListPairs[i].notify = 0;
		dmaListPairs[i].size = sizeof(SortData);
	}

	for(u32 phaseId=0;phaseId < jointSolverInfoPcl->numPhases;phaseId++) {
		for(u32 groupId=0;groupId < jointSolverInfoPcl->numGroups[phaseId];groupId++) {
			u32 numPairs = jointSolverInfoPcl->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId];
			if(groupId%solverIOPcl.numSPU == taskId && numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				spu_dma_get(group, solverIOPcl.jointGroupsPclAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId), sizeof(SolverGroup), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128, sizeof(SortData)*numPairs);
				for(u32 i=0;i<numPairs;i++)
					dmaListPairs[i].eal = solverIOPcl.jointSortsPclAddr + sizeof(SortData)*group->pairIndices[i];

				spu_dma_list_get(sortBuff, 0, dmaListPairs, sizeof(spu_dma_list_element)*numPairs, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				applyJointImpulseExPcl(sortBuff, numPairs, config);

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	for(u32 phaseId=0;phaseId < jointSolverInfoRig->numPhases;phaseId++) {
		for(u32 groupId=0;groupId < jointSolverInfoRig->numGroups[phaseId];groupId++) {
			u32 numPairs = jointSolverInfoRig->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId];
			if(groupId%solverIOPcl.numSPU == taskId && numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				spu_dma_get(group, solverIOPcl.jointGroupsRigAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId), sizeof(SolverGroup), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128, sizeof(SortData)*numPairs);
				for(u32 i=0;i<numPairs;i++)
					dmaListPairs[i].eal = solverIOPcl.jointSortsRigAddr + sizeof(SortData)*group->pairIndices[i];

				spu_dma_list_get(sortBuff, 0, dmaListPairs,sizeof(spu_dma_list_element)*numPairs, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				applyJointImpulseExRig(sortBuff, numPairs, config);

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	DEALLOCATE(dmaListPairs);
}

#ifdef PARTICLE_VELOCITY_BASE

void preJointImpulseExPcl(SortData *sorts, u32 numSorts, const PclJointSolverConfig& config)
{
	s32 tag1 = 10;
	PclState *pclStates = (PclState*)ALLOCATE(128, sizeof(PclState)*numSorts*2);
	u32 numBatch = 5;
	u8 numStates = 0;
	int numBatchStates[2] = {0};
	spu_dma_list_element *pclStateDmaList[2];
	u8 *pclStateRefTable = (u8*)ALLOCATE(16, sizeof(u8)*solverIOPcl.numPclStates);

	memset(pclStateRefTable, 0xff, sizeof(u8)*solverIOPcl.numPclStates);

	pclStateDmaList[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	pclStateDmaList[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);

	for(u32 j=0;j < numBatch*2;j++) {
		pclStateDmaList[0][j].notify = pclStateDmaList[1][j].notify = 0;
		pclStateDmaList[0][j].size = pclStateDmaList[1][j].size = sizeof(PclState);
	}

	u32 pairAddr = solverIOPcl.jointsAddr;
	PclJoint *jointBuffer[2];
	spu_dma_list_element *pairDmaListBuf[3];

	jointBuffer[0] = (PclJoint*)ALLOCATE(128, sizeof(PclJoint)*numBatch);
	jointBuffer[1] = (PclJoint*)ALLOCATE(128, sizeof(PclJoint)*numBatch);
	pairDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);// PUT

	s32 batchFlag = 0;
	s32 currData = 0;
	s32 numPairs[2] = {0};
	PclState *oldStatesPtr = pclStates;

	for(u32 j=0;j < numBatch;j++) {
		pairDmaListBuf[0][j].notify = 0;
		pairDmaListBuf[0][j].size = sizeof(PclJoint);
		pairDmaListBuf[1][j].notify = 0;
		pairDmaListBuf[1][j].size = sizeof(PclJoint);
	}

	for(u32 j=0;j < numBatch && j < numSorts;j++) {
		pairDmaListBuf[0][numPairs[0]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
		numPairs[0]++;

		u16 stateA = getStateA(sorts[j]);
		u16 stateB = getStateB(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			pclStateDmaList[0][numBatchStates[0]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateRefTable[stateA] = numStates;
			numBatchStates[0]++;
			numStates++;
		}

		if(pclStateRefTable[stateB] == 0xff) {
			pclStateDmaList[0][numBatchStates[0]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
			pclStateRefTable[stateB] = numStates;
			numBatchStates[0]++;
			numStates++;
		}
	}

	if(numPairs[0] > 0)
		spu_dma_list_get(jointBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);

	if(numBatchStates[0] > 0)
		spu_dma_list_get(pclStates, 0, pclStateDmaList[0], sizeof(spu_dma_list_element)*numBatchStates[0], tag1, 0, 0);

	oldStatesPtr += numBatchStates[0];

	spu_dma_wait_tag_status_all(1<<tag1);

	for(u32 j=numBatch;j < numBatch + numBatch && j < numSorts;j++) {
		pairDmaListBuf[1][numPairs[1]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
		numPairs[1]++;

		u16 stateA = getStateA(sorts[j]);
		u16 stateB = getStateB(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			pclStateDmaList[1][numBatchStates[1]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateRefTable[stateA] = numStates;
			numBatchStates[1]++;
			numStates++;
		}

		if(pclStateRefTable[stateB] == 0xff) {
			pclStateDmaList[1][numBatchStates[1]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
			pclStateRefTable[stateB] = numStates;
			numBatchStates[1]++;
			numStates++;
		}
	}

	if(numPairs[1] > 0)
		spu_dma_list_get(jointBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);

	if(numBatchStates[1] > 0)
		spu_dma_list_get(oldStatesPtr, 0, pclStateDmaList[1], sizeof(spu_dma_list_element)*numBatchStates[1], tag1, 0, 0);

	oldStatesPtr += numBatchStates[1];

	currData = 0;
	batchFlag = 1;

	for(u32 i=0;i < numSorts;) {
		// UPDATE
		for(int j=0;j<numPairs[currData];j++) {
			PclJoint& joint = jointBuffer[currData][j];
			PclState& particleA = pclStates[pclStateRefTable[joint.stateIndexA]];
			PclState& particleB = pclStates[pclStateRefTable[joint.stateIndexB]];

			joint.impulseDen = 1.0f/(1.0f/particleA.getMass() + 1.0f/particleB.getMass());
		}

		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		if(numPairs[currData] > 0) {
			memcpy(pairDmaListBuf[2], pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]);
			spu_dma_list_put(jointBuffer[currData], 0, pairDmaListBuf[2], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		}

		numPairs[currData] = 0;
		numBatchStates[currData] = 0;

		batchFlag++;
		s32 start = batchFlag*numBatch;
		for(u32 j=start;j < start + numBatch && j < numSorts;j++) {
			pairDmaListBuf[currData][numPairs[currData]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
			numPairs[currData]++;

			u16 stateA = getStateA(sorts[j]);
			u16 stateB = getStateB(sorts[j]);

			// State
			if(pclStateRefTable[stateA] == 0xff) {
				pclStateDmaList[currData][numBatchStates[currData]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
				pclStateRefTable[stateA] = numStates;
				numBatchStates[currData]++;
				numStates++;
			}

			if(pclStateRefTable[stateB] == 0xff) {
				pclStateDmaList[currData][numBatchStates[currData]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateB;
				pclStateRefTable[stateB] = numStates;
				numBatchStates[currData]++;
				numStates++;
			}
		}

		if(numPairs[currData] > 0)
			spu_dma_list_getf(jointBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

		if(numBatchStates[currData] > 0)
			spu_dma_list_get(oldStatesPtr, 0, pclStateDmaList[currData], sizeof(spu_dma_list_element)*numBatchStates[currData], tag1, 0, 0);

		oldStatesPtr += numBatchStates[currData];

		currData = 1 - currData;
	}

	if(numPairs[currData] > 0)
		spu_dma_list_put(jointBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

	DEALLOCATE(pclStateDmaList[1]);
	DEALLOCATE(pclStateDmaList[0]);

	DEALLOCATE(pclStateRefTable);

	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);
	DEALLOCATE(jointBuffer[1]);
	DEALLOCATE(jointBuffer[0]);

	DEALLOCATE(pclStates);
}

void preJointImpulseExRig(SortData *sorts, u32 numSorts, const PclJointSolverConfig& config)
{
	s32 tag1 = 10;
	PclState *pclStates = (PclState*)ALLOCATE(128, sizeof(PclState)*numSorts);
	u32 numBatch = 5;
	u8 numPclStates = 0;
	s32 numBatchPclStates[2] = {0};
	spu_dma_list_element *pclStateDmaList[2];
	u8 *pclStateRefTable = (u8*)ALLOCATE(16, sizeof(u8)*solverIOPcl.numPclStates);

	memset(pclStateRefTable, 0xff, sizeof(u8)*solverIOPcl.numPclStates);

	pclStateDmaList[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch);
	pclStateDmaList[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch);

	for(u32 j=0;j < numBatch;j++) {
		pclStateDmaList[0][j].notify = pclStateDmaList[1][j].notify = 0;
		pclStateDmaList[0][j].size = pclStateDmaList[1][j].size = sizeof(PclState);
	}

	u32 pairAddr = solverIOPcl.jointsAddr;
	u32 bodyAddr = solverIOPcl.bodiesAddr;
	PclJoint *jointBuffer[2];
	spu_dma_list_element *pairDmaListBuf[3];
	TrbDynBody *bodyBuffer[2];
	spu_dma_list_element *bodyDmaListBuf[2];

	jointBuffer[0] = (PclJoint*)ALLOCATE(128, sizeof(PclJoint)*numBatch);
	jointBuffer[1] = (PclJoint*)ALLOCATE(128, sizeof(PclJoint)*numBatch);
	pairDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);// PUT用

	bodyBuffer[0] = (TrbDynBody*)ALLOCATE(128, sizeof(TrbDynBody)*numBatch);
	bodyBuffer[1] = (TrbDynBody*)ALLOCATE(128, sizeof(TrbDynBody)*numBatch);
	bodyDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	bodyDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);

	s32 batchFlag = 0;
	s32 currData = 0;
	s32 numPairs[2] = {0};
	PclState *oldStatesPtr = pclStates;

	for(u32 j=0;j < numBatch;j++) {
		pairDmaListBuf[0][j].notify = 0;
		pairDmaListBuf[0][j].size = sizeof(PclJoint);
		pairDmaListBuf[1][j].notify = 0;
		pairDmaListBuf[1][j].size = sizeof(PclJoint);
		bodyDmaListBuf[0][j].notify = 0;
		bodyDmaListBuf[0][j].size = sizeof(TrbDynBody);
		bodyDmaListBuf[1][j].notify = 0;
		bodyDmaListBuf[1][j].size = sizeof(TrbDynBody);
	}

	for(u32 j=0;j < numBatch && j < numSorts;j++) {
		pairDmaListBuf[0][numPairs[0]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
		bodyDmaListBuf[0][numPairs[0]].eal = bodyAddr + sizeof(TrbDynBody)*getBodyB(sorts[j]);
		numPairs[0]++;

		u16 stateA = getStateA(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			pclStateDmaList[0][numBatchPclStates[0]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateRefTable[stateA] = numPclStates;
			numBatchPclStates[0]++;
			numPclStates++;
		}
	}

	if(numPairs[0] > 0) {
		spu_dma_list_get(jointBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
		spu_dma_list_get(bodyBuffer[0], 0, bodyDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
	}

	if(numBatchPclStates[0] > 0)
		spu_dma_list_get(pclStates, 0, pclStateDmaList[0], sizeof(spu_dma_list_element)*numBatchPclStates[0], tag1, 0, 0);

	oldStatesPtr += numBatchPclStates[0];

	spu_dma_wait_tag_status_all(1<<tag1);

	for(u32 j=numBatch;j < numBatch + numBatch && j < numSorts;j++) {
		pairDmaListBuf[1][numPairs[1]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
		bodyDmaListBuf[1][numPairs[1]].eal = bodyAddr + sizeof(TrbDynBody)*getBodyB(sorts[j]);
		numPairs[1]++;

		u16 stateA = getStateA(sorts[j]);

		// State
		if(pclStateRefTable[stateA] == 0xff) {
			pclStateDmaList[1][numBatchPclStates[1]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
			pclStateRefTable[stateA] = numPclStates;
			numBatchPclStates[1]++;
			numPclStates++;
		}
	}

	if(numPairs[1] > 0) {
		spu_dma_list_get(jointBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
		spu_dma_list_get(bodyBuffer[1], 0, bodyDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
	}

	if(numBatchPclStates[1] > 0)
		spu_dma_list_get(oldStatesPtr, 0, pclStateDmaList[1], sizeof(spu_dma_list_element)*numBatchPclStates[1], tag1, 0, 0);

	oldStatesPtr += numBatchPclStates[1];

	currData = 0;
	batchFlag = 1;

	for(u32 i=0;i < numSorts;) {
		// UPDATE
		for(s32 j=0;j < numPairs[currData];j++) {
			PclJoint& joint = jointBuffer[currData][j];
			PclState& particleA = pclStates[pclStateRefTable[joint.stateIndexA]];
			//TrbDynBody &bodyB = bodyBuffer[currData][j];

			joint.impulseDen = particleA.getMass();
		}

		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		if(numPairs[currData] > 0) {
			memcpy(pairDmaListBuf[2], pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]);
			spu_dma_list_put(jointBuffer[currData], 0, pairDmaListBuf[2], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		}

		numPairs[currData] = 0;
		numBatchPclStates[currData] = 0;

		batchFlag++;
		s32 start = batchFlag*numBatch;
		for(u32 j=start;j < start + numBatch && j < numSorts;j++) {
			pairDmaListBuf[currData][numPairs[currData]].eal = pairAddr + sizeof(PclJoint)*getPair(sorts[j]);
			bodyDmaListBuf[currData][numPairs[currData]].eal = bodyAddr + sizeof(TrbDynBody)*getBodyB(sorts[j]);
			numPairs[currData]++;

			u16 stateA = getStateA(sorts[j]);

			// State
			if(pclStateRefTable[stateA] == 0xff) {
				pclStateDmaList[currData][numBatchPclStates[currData]].eal = solverIOPcl.pclStatesAddr + sizeof(PclState)*stateA;
				pclStateRefTable[stateA] = numPclStates;
				numBatchPclStates[currData]++;
				numPclStates++;
			}
		}

		if(numPairs[currData] > 0) {
			spu_dma_list_getf(jointBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
			spu_dma_list_get(bodyBuffer[currData], 0, bodyDmaListBuf[currData],sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		}

		if(numBatchPclStates[currData] > 0)
			spu_dma_list_get(oldStatesPtr, 0, pclStateDmaList[currData], sizeof(spu_dma_list_element)*numBatchPclStates[currData], tag1, 0, 0);

		oldStatesPtr += numBatchPclStates[currData];

		currData = 1 - currData;
	}

	if(numPairs[currData] > 0)
		spu_dma_list_put(jointBuffer[currData], 0, pairDmaListBuf[currData],sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

	DEALLOCATE(bodyDmaListBuf[1]);
	DEALLOCATE(bodyDmaListBuf[0]);
	DEALLOCATE(bodyBuffer[1]);
	DEALLOCATE(bodyBuffer[0]);

	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);
	DEALLOCATE(jointBuffer[1]);
	DEALLOCATE(jointBuffer[0]);

	DEALLOCATE(pclStateDmaList[1]);
	DEALLOCATE(pclStateDmaList[0]);
	DEALLOCATE(pclStateRefTable);

	DEALLOCATE(pclStates);
}

void preJointImpulsePhaseEx()
{
	PclJointSolverConfig config;
	config.timeStep = solverIOPcl.timeStep;
	config.jointDamping = solverIOPcl.jointDamping;
	config.jointBias = solverIOPcl.jointBias;

	spu_dma_list_element *dmaListPairs = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*MAX_SOLVER_PAIRS);
	for(s32 i=0;i < MAX_SOLVER_PAIRS;i++) {
		dmaListPairs[i].notify = 0;
		dmaListPairs[i].size = sizeof(SortData);
	}

	for(u32 phaseId=0;phaseId < jointSolverInfoPcl->numPhases;phaseId++) {
		for(u32 groupId=0;groupId < jointSolverInfoPcl->numGroups[phaseId];groupId++) {
			u32 numPairs = jointSolverInfoPcl->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId];
			if(groupId%solverIOPcl.numSPU == taskId && numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				spu_dma_get(group, solverIOPcl.jointGroupsPclAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId), sizeof(SolverGroup), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128, sizeof(SortData)*numPairs);
				for(u32 i=0;i < numPairs;i++)
					dmaListPairs[i].eal = solverIOPcl.jointSortsPclAddr + sizeof(SortData)*group->pairIndices[i];

				spu_dma_list_get(sortBuff, 0, dmaListPairs, sizeof(spu_dma_list_element)*numPairs, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				preJointImpulseExPcl(sortBuff, numPairs, config);

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	for(u32 phaseId=0;phaseId < jointSolverInfoRig->numPhases;phaseId++) {
		for(u32 groupId=0;groupId < jointSolverInfoRig->numGroups[phaseId];groupId++) {
			u32 numPairs = jointSolverInfoRig->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId];
			if(groupId%solverIOPcl.numSPU == taskId && numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				spu_dma_get(group, solverIOPcl.jointGroupsRigAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId), sizeof(SolverGroup), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128, sizeof(SortData)*numPairs);
				for(u32 i=0;i < numPairs;i++)
					dmaListPairs[i].eal = solverIOPcl.jointSortsRigAddr + sizeof(SortData)*group->pairIndices[i];

				spu_dma_list_get(sortBuff, 0, dmaListPairs, sizeof(spu_dma_list_element)*numPairs, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				preJointImpulseExRig(sortBuff, numPairs, config);

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	DEALLOCATE(dmaListPairs);
}

#endif // PARTICLE_VELOCITY_BASE

#endif /* JOINTPHASE_H_ */
