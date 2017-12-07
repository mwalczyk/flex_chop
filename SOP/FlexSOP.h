#pragma once
/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
* can only be used, and/or modified for use, in conjunction with
* Derivative's TouchDesigner software, and only if you are a licensee who has
* accepted Derivative's TouchDesigner license or assignment agreement (which
* also govern the use of this file).  You may share a modified version of this
* file with another authorized licensee of Derivative's TouchDesigner software.
* Otherwise, no redistribution or sharing of this file, with or without
* modification, is permitted.
*/

#include "SOP_CPlusPlusBase.h"

#include <string>
#include <iostream>
#include <memory>

#include "NvFlex.h"
#include "NvFlexDevice.h"
#include "NvFlexExt.h"
#include "vector_types.h"
#include "vector_functions.h"

struct SimBuffers
{
	// General
	NvFlexVector<float4> positions;
	NvFlexVector<float3> velocities;
	NvFlexVector<int> phases;
	NvFlexVector<int> activeIndices;

	// Springs
	NvFlexVector<int> springIndices;
	NvFlexVector<float> springLengths;
	NvFlexVector<float> springStiffness;
	NvFlexVector<int> triangles;

	SimBuffers(NvFlexLibrary* l) :
		positions(l), 
		velocities(l), 
		phases(l), 
		activeIndices(l),
		springIndices(l), 
		springLengths(l), 
		springStiffness(l), 
		triangles(l)
	{
	}
};

class FlexSOP : public SOP_CPlusPlusBase
{
public:

	FlexSOP(const OP_NodeInfo* info);
	virtual ~FlexSOP();

	virtual void getGeneralInfo(SOP_GeneralInfo*) override;
	virtual void execute(SOP_Output*, OP_Inputs*, void* reserved) override;
	virtual void executeVBO(SOP_VBOOutput* output, OP_Inputs* inputs, void* reserved) override;
	virtual int32_t getNumInfoCHOPChans() override;
	virtual void getInfoCHOPChan(int index, OP_InfoCHOPChan* chan) override;
	virtual bool getInfoDATSize(OP_InfoDATSize* infoSize) override;
	virtual void getInfoDATEntries(int32_t index, int32_t nEntries, OP_InfoDATEntries* entries) override;
	virtual void setupParameters(OP_ParameterManager* manager) override;
	virtual void pulsePressed(const char* name) override;

private:

	void setupSimulationParams();
	void setupCollisionPlanes(float w, float h);
	void MapBuffers();
	void UnmapBuffers();
	void CreateSpringGrid(float3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, float3 velocity, float invMass);
	void CreateSpring(int i, int j, float stiffness, float give = 0.0f);
	void buildOutputGeometry(SOP_Output* output);

	// Provided by TouchDesigner
	const OP_NodeInfo* m_node_info;
	int32_t	m_execute_count;

	// Flex-related member variables
	bool m_dirty = true;
	bool m_buffers_ready = false;
	const size_t m_max_number_of_particles = 100000;
	size_t m_number_of_particles = 1;
	size_t m_number_of_indices = 1;
	size_t m_number_of_triangles = 1;
	size_t m_number_of_springs = 1;
	NvFlexLibrary* m_library;
	NvFlexSolver* m_solver;
	NvFlexParams m_params;
	NvFlexExtForceFieldCallback* m_force_field_callback;
	NvFlexExtForceField m_force_field;
	std::shared_ptr<SimBuffers> m_sim_buffers;
};
