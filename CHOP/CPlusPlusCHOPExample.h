/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

#include <iostream>

#include "CHOP_CPlusPlusBase.h"

#include "NvFlex.h"
#include "NvFlexDevice.h"
#include "NvFlexExt.h"
#include "vector_types.h"
#include "vector_functions.h"

// To get more help about these functions, look at CHOP_CPlusPlusBase.h
class CPlusPlusCHOPExample : public CHOP_CPlusPlusBase
{
public:
	CPlusPlusCHOPExample(const OP_NodeInfo* info);
	virtual ~CPlusPlusCHOPExample();

	virtual void		getGeneralInfo(CHOP_GeneralInfo* ) override;
	virtual bool		getOutputInfo(CHOP_OutputInfo*) override;
	virtual const char*	getChannelName(int32_t index, void* reserved) override;

	virtual void		execute(const CHOP_Output*,
								OP_Inputs*,
								void* reserved) override;


	virtual int32_t		getNumInfoCHOPChans() override;
	virtual void		getInfoCHOPChan(int index,
										OP_InfoCHOPChan* chan) override;

	virtual bool		getInfoDATSize(OP_InfoDATSize* infoSize) override;
	virtual void		getInfoDATEntries(int32_t index,
											int32_t nEntries,
											OP_InfoDATEntries* entries) override;

	virtual void		setupParameters(OP_ParameterManager* manager) override;
	virtual void		pulsePressed(const char* name) override;

private:

	void setupCollisionPlanes(float w, float h);

	const OP_NodeInfo* myNodeInfo;

	int32_t myExecuteCount;
	double myOffset;

	bool m_dirty = true;
	bool m_buffers_ready = false;
	const size_t m_max_number_of_particles = 100000;
	size_t m_number_of_particles = 1;
	NvFlexLibrary* m_library;
	NvFlexSolver* m_solver;
	NvFlexParams m_params;

	NvFlexBuffer* m_particle_buffer;
	NvFlexBuffer* m_velocity_buffer;
	NvFlexBuffer* m_phases_buffer;
	NvFlexBuffer* m_active_indices_buffer;
};
