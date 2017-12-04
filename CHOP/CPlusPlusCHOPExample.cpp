/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

#include "CPlusPlusCHOPExample.h"

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <assert.h>

extern "C"
{

DLLEXPORT
int32_t
GetCHOPAPIVersion(void)
{
	return CHOP_CPLUSPLUS_API_VERSION;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo* info)
{
	return new CPlusPlusCHOPExample(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
{
	delete (CPlusPlusCHOPExample*)instance;
}

};

void CPlusPlusCHOPExample::setupCollisionPlanes(float w, float h)
{

	// BOTTOM
	m_params.planes[0][0] = 0.0f; // normal x
	m_params.planes[0][1] = 1.0f; // normal y
	m_params.planes[0][2] = 0.0f; // normal z
	m_params.planes[0][3] = h; // y-position

	// TOP
	m_params.planes[1][0] = 0.0f; // normal x
	m_params.planes[1][1] = -1.0f; // normal y
	m_params.planes[1][2] = 0.0f; // normal z
	m_params.planes[1][3] = h; // y-position

	// LEFT
	m_params.planes[2][0] = 1.0f; // normal x
	m_params.planes[2][1] = 0.0f; // normal y
	m_params.planes[2][2] = 0.0f; // normal z
	m_params.planes[2][3] = w; // y-position

	// RIGHT
	m_params.planes[3][0] = -1.0f; // normal x
	m_params.planes[3][1] = 0.0f; // normal y
	m_params.planes[3][2] = 0.0f; // normal z
	m_params.planes[3][3] = w; // y-position

	// FRONT
	m_params.planes[4][0] = 0.0f; // normal x
	m_params.planes[4][1] = 0.0f; // normal y
	m_params.planes[4][2] = 1.0f; // normal z
	m_params.planes[4][3] = w; // y-position

	// BACK
	m_params.planes[5][0] = 0.0f; // normal x
	m_params.planes[5][1] = 0.0f; // normal y
	m_params.planes[5][2] = -1.0f; // normal z
	m_params.planes[5][3] = w; // y-position

	m_params.numPlanes = 6;
}

CPlusPlusCHOPExample::CPlusPlusCHOPExample(const OP_NodeInfo* info) : myNodeInfo(info)
{
	myExecuteCount = 0;
	myOffset = 0.0;
	
	// Initialize Flex objects
	m_library = NvFlexInit();

	NvFlexSolverDesc solver_description;
	NvFlexSetSolverDescDefaults(&solver_description);
	solver_description.maxParticles = m_max_number_of_particles;
	solver_description.maxDiffuseParticles = 0;
	//solver_description.featureMode = eNvFlexFeatureModeSimpleSolids;

	m_solver = NvFlexCreateSolver(m_library, &solver_description);
	
	setupCollisionPlanes(0.5f, 0.5f);

	const float particle_radius = 0.01f;

	m_params.gravity[0] = 0.0f;
	m_params.gravity[1] = -9.8f;
	m_params.gravity[2] = 0.0f;
	m_params.wind[0] = 0.0f;
	m_params.wind[1] = 0.0f;
	m_params.wind[2] = 0.0f;
	m_params.viscosity = 0.0f;
	//m_params.dynamicFriction = 0.0f;
	//m_params.staticFriction = 0.0f;
	//m_params.particleFriction = 0.0f; 

	m_params.dynamicFriction = 0.4f;
	m_params.staticFriction = 0.4f;
	m_params.particleFriction = 0.25f;

	m_params.freeSurfaceDrag = 0.0f;
	m_params.drag = 0.0f;
	m_params.lift = 0.0f;
	m_params.numIterations = 3;
	m_params.anisotropyScale = 1.0f;
	m_params.anisotropyMin = 0.1f;
	m_params.anisotropyMax = 2.0f;
	m_params.smoothing = 1.0f;
	m_params.dissipation = 0.0f;
	m_params.damping = 0.0f;
	m_params.particleCollisionMargin = 0.0f;
	m_params.shapeCollisionMargin = 0.0f;
	m_params.collisionDistance = 0.005f;// 0.0f;
	m_params.sleepThreshold = 0.0f;
	m_params.shockPropagation = 0.0f;
	m_params.restitution = 0.0f;
	m_params.maxSpeed = 2.0f;
	m_params.maxAcceleration = 2.0f;
	m_params.relaxationMode = eNvFlexRelaxationLocal;
	m_params.relaxationFactor = 1.0f;
	m_params.solidPressure = 1.0f;
	m_params.adhesion = 0.0f;
	m_params.cohesion = 0.025f;
	m_params.surfaceTension = 0.0f;
	m_params.vorticityConfinement = 0.0f;
	m_params.buoyancy = 1.0f;
	m_params.diffuseThreshold = 100.0f;
	m_params.diffuseBuoyancy = 1.0f;
	m_params.diffuseDrag = 0.8f;
	m_params.diffuseBallistic = 16;
	m_params.radius = particle_radius * 2.0f;

	// If `radius` and `fluidRestDistance` are similar, the simulation will be less accurate but faster
	m_params.fluidRestDistance = m_params.radius * 0.5f; // must be less than or equal to the radius parameter
	m_params.solidRestDistance = m_params.radius * 0.5f; // must be less than or equal to the radius parameter

	m_force_field_callback = NvFlexExtCreateForceFieldCallback(m_solver);

	m_force_field.mLinearFalloff = true;
	m_force_field.mMode = eNvFlexExtModeForce;
	m_force_field.mPosition[0] = 0.0f;
	m_force_field.mPosition[1] = 0.0f;
	m_force_field.mPosition[2] = 0.0f;
	m_force_field.mRadius = 1.0f;
	m_force_field.mStrength = -30.0f;

	std::cout << "CHOP constructor called\n";
}

CPlusPlusCHOPExample::~CPlusPlusCHOPExample()
{
	// Only free buffers if they were allocated in the first place
	if (m_buffers_ready)
	{
		NvFlexFreeBuffer(m_particle_buffer);
		NvFlexFreeBuffer(m_velocity_buffer);
		NvFlexFreeBuffer(m_phases_buffer);
		NvFlexFreeBuffer(m_active_indices_buffer);
	}

	NvFlexExtDestroyForceFieldCallback(m_force_field_callback);

	NvFlexDestroySolver(m_solver);
	NvFlexShutdown(m_library);
}

void
CPlusPlusCHOPExample::getGeneralInfo(CHOP_GeneralInfo* ginfo)
{
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->inputMatchIndex = 0;
}

bool
CPlusPlusCHOPExample::getOutputInfo(CHOP_OutputInfo* info)
{
	// If there is an input connected, we are going to match it's channel names etc
	// otherwise we'll specify our own.
	if (info->opInputs->getNumInputs() > 0)
	{
		return false;
	}
	else
	{
		// For now, just output the position and velocity of each particle - note that
		// we need three channels for each (X Y Z)
		info->numChannels = 6;
		info->numSamples = m_number_of_particles;
		info->startIndex = 0;

		return true;
	}
}

const char*
CPlusPlusCHOPExample::getChannelName(int32_t index, void* reserved)
{
	switch (index)
	{
	case 0: return "tx";
	case 1: return "ty";
	case 2: return "tz";
	case 3: return "vx";
	case 4: return "vy";
	case 5: return "vz";
	}
}

void
CPlusPlusCHOPExample::execute(const CHOP_Output* output,
							  OP_Inputs* inputs,
							  void* reserved)
{
	if (m_dirty)
	{
		// Is there a CHOP connected to this one?
		if (inputs->getNumInputs() > 0)
		{
			const OP_CHOPInput	*cinput = inputs->getInputCHOP(0);

			// Does the CHOP have (at least) 6 channels (tx, ty, tz, vx, vy, vz)?
			if (cinput->numChannels >= 6)
			{
				std::cout << "Resetting Flex system...\n";

				m_number_of_particles = cinput->numSamples;

				// Re-allocate buffers
				m_particle_buffer = NvFlexAllocBuffer(m_library, m_number_of_particles, sizeof(float4), eNvFlexBufferHost);
				m_velocity_buffer = NvFlexAllocBuffer(m_library, m_number_of_particles, sizeof(float3), eNvFlexBufferHost);
				m_phases_buffer = NvFlexAllocBuffer(m_library, m_number_of_particles, sizeof(int), eNvFlexBufferHost);
				m_active_indices_buffer = NvFlexAllocBuffer(m_library, m_number_of_particles, sizeof(int), eNvFlexBufferHost);

				// Map buffers for writing
				float4* particles = (float4*)NvFlexMap(m_particle_buffer, eNvFlexMapWait);
				float3* velocities = (float3*)NvFlexMap(m_velocity_buffer, eNvFlexMapWait);
				int* phases = (int*)NvFlexMap(m_phases_buffer, eNvFlexMapWait);
				int* active = (int*)NvFlexMap(m_active_indices_buffer, eNvFlexMapWait);

				for (int i = 0; i < m_number_of_particles; ++i)
				{
					float tx = cinput->getChannelData(0)[i];
					float ty = cinput->getChannelData(1)[i];
					float tz = cinput->getChannelData(2)[i];

					float vx = cinput->getChannelData(3)[i];
					float vy = cinput->getChannelData(4)[i];
					float vz = cinput->getChannelData(5)[i];

					particles[i] = make_float4(tx, ty, tz, 1.0f);
					velocities[i] = make_float3(vx, vy, vz);
					phases[i] = NvFlexMakePhase(0, eNvFlexPhaseSelfCollide);// | eNvFlexPhaseFluid);
					active[i] = i;
				}

				NvFlexUnmap(m_particle_buffer);
				NvFlexUnmap(m_velocity_buffer);
				NvFlexUnmap(m_phases_buffer);
				NvFlexUnmap(m_active_indices_buffer);

				m_buffers_ready = true;
				m_dirty = false;
			}
		}

	}

	// Update Flex
	if (m_buffers_ready)
	{
		// Update the simulation based on the UI params
		{
			double bw, bh;
			inputs->getParDouble2("Boundingbox", bw, bh);
			setupCollisionPlanes(bw, bh);

			double gx, gy, gz;
			inputs->getParDouble3("Gravity", gx, gy, gz);
			m_params.gravity[0] = gx;
			m_params.gravity[1] = gy;
			m_params.gravity[2] = gz;
			m_params.restitution = inputs->getParDouble("Restitution");
			m_params.viscosity = inputs->getParDouble("Viscosity");
			m_params.vorticityConfinement = inputs->getParDouble("Vorticityconfinement");
			m_params.adhesion = inputs->getParDouble("Adhesion");
			m_params.cohesion = inputs->getParDouble("Cohesion");
			m_params.surfaceTension = inputs->getParDouble("Surfacetension");
		}
		NvFlexSetParams(m_solver, &m_params);

		NvFlexSetParticles(m_solver, m_particle_buffer, NULL);
		NvFlexSetVelocities(m_solver, m_velocity_buffer, NULL);
		NvFlexSetPhases(m_solver, m_phases_buffer, NULL);
		NvFlexSetActive(m_solver, m_active_indices_buffer, NULL);

		NvFlexSetActiveCount(m_solver, m_number_of_particles);

		// Update the force field based on UI params
		{
			double px, py, pz;
			inputs->getParDouble3("Forceposition", px, py, pz);
			m_force_field.mPosition[0] = px;
			m_force_field.mPosition[1] = py;
			m_force_field.mPosition[2] = pz;
			m_force_field.mRadius = inputs->getParDouble("Forceradius");
			m_force_field.mStrength = inputs->getParDouble("Forcestrength");
			m_force_field.mMode = eNvFlexExtModeForce;
			m_force_field.mLinearFalloff = true;
		}
		
		NvFlexExtSetForceFields(m_force_field_callback, &m_force_field, 1);

		// Update the solver
		const float dt = 1.0f / 60.0f;
		const int sub_steps = 1;
		NvFlexUpdateSolver(m_solver, dt, sub_steps, false);




		// Get particle positions and velocities from the Flex simulation
		NvFlexGetParticles(m_solver, m_particle_buffer, NULL);
		NvFlexGetVelocities(m_solver, m_velocity_buffer, NULL);

		float4* particles = (float4*)NvFlexMap(m_particle_buffer, eNvFlexMapWait);
		float3* velocities = (float3*)NvFlexMap(m_velocity_buffer, eNvFlexMapWait);

		// Copy to CHOP channels
		for (size_t i = 0; i < m_number_of_particles; ++i)
		{
			// Positions X Y Z are the first three CHOP channels
			output->channels[0][i] = particles[i].x;
			output->channels[1][i] = particles[i].y;
			output->channels[2][i] = particles[i].z;

			// Velocities X Y Z are the next three CHOP channels
			output->channels[3][i] = velocities[i].x;
			output->channels[4][i] = velocities[i].y;
			output->channels[5][i] = velocities[i].z;
		}

		NvFlexUnmap(m_particle_buffer);
		NvFlexUnmap(m_velocity_buffer);
	}

	myExecuteCount++;
}

int32_t
CPlusPlusCHOPExample::getNumInfoCHOPChans()
{
	return 2;
}

void
CPlusPlusCHOPExample::getInfoCHOPChan(int32_t index,
										OP_InfoCHOPChan* chan)
{
	if (index == 0)
	{
		chan->name = "executeCount";
		chan->value = (float)myExecuteCount;
	}

	if (index == 1)
	{
		chan->name = "offset";
		chan->value = (float)myOffset;
	}
}

bool		
CPlusPlusCHOPExample::getInfoDATSize(OP_InfoDATSize* infoSize)
{
	infoSize->rows = 2;
	infoSize->cols = 2;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void
CPlusPlusCHOPExample::getInfoDATEntries(int32_t index,
										int32_t nEntries,
										OP_InfoDATEntries* entries)
{
	// It's safe to use static buffers here because Touch will make it's own
	// copies of the strings immediately after this call returns
	// (so the buffers can be reuse for each column/row)
	static char tempBuffer1[4096];
	static char tempBuffer2[4096];

	if (index == 0)
	{
		// Set the value for the first column
#ifdef WIN32
		strcpy_s(tempBuffer1, "executeCount");
#else // macOS
        strlcpy(tempBuffer1, "executeCount", sizeof(tempBuffer1));
#endif
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
#ifdef WIN32
		sprintf_s(tempBuffer2, "%d", myExecuteCount);
#else // macOS
        snprintf(tempBuffer2, sizeof(tempBuffer2), "%d", myExecuteCount);
#endif
		entries->values[1] = tempBuffer2;
	}

	if (index == 1)
	{
		// Set the value for the first column
#ifdef WIN32
        strcpy_s(tempBuffer1, "offset");
#else // macOS
        strlcpy(tempBuffer1, "offset", sizeof(tempBuffer1));
#endif
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
#ifdef WIN32
        sprintf_s(tempBuffer2, "%g", myOffset);
#else // macOS
        snprintf(tempBuffer2, sizeof(tempBuffer2), "%g", myOffset);
#endif
		entries->values[1] = tempBuffer2;
	}
}

void
CPlusPlusCHOPExample::setupParameters(OP_ParameterManager* manager)
{
	{
		OP_NumericParameter	param;

		param.name = "Reset";
		param.label = "Reset";
		
		OP_ParAppendResult res = manager->appendPulse(param);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter param;
		param.name = "Boundingbox";
		param.label = "Bounding Box";
		param.defaultValues[0] = 0.5f;
		param.minSliders[0] = 0.5f;
		param.maxSliders[0] =  10.0;

		param.defaultValues[1] = 0.5f;
		param.minSliders[1] = 0.5f;
		param.maxSliders[1] =  10.0;

		OP_ParAppendResult res = manager->appendXYZ(param);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter param;
		param.name = "Forceposition";
		param.label = "Force Position";
		param.defaultValues[0] = 0.0f;
		param.minSliders[0] = -10.0f;
		param.maxSliders[0] =  10.0;

		param.defaultValues[1] = 0.0f;
		param.minSliders[1] = -10.0f;
		param.maxSliders[1] =  10.0;

		param.defaultValues[2] = 0.0f;
		param.minSliders[2] = -10.0f;
		param.maxSliders[2] =  10.0;

		OP_ParAppendResult res = manager->appendXYZ(param);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter param;
		param.name = "Forceradius";
		param.label = "Force Radius";
		param.defaultValues[0] = 1.0f;
		param.minSliders[0] = 0.0f;
		param.maxSliders[0] = 10.0f;

		OP_ParAppendResult res = manager->appendFloat(param);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter param;
		param.name = "Forcestrength";
		param.label = "Force Strength";
		param.defaultValues[0] = -30.0f;
		param.minSliders[0] = -100.0f;
		param.maxSliders[0] = 100.0f;

		OP_ParAppendResult res = manager->appendFloat(param);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter param;
		param.name = "Gravity";
		param.label = "Gravity";
		param.defaultValues[0] = 0.0;
		param.minSliders[0] = -10.0;
		param.maxSliders[0] =  10.0;

		param.defaultValues[1] = -9.8;
		param.minSliders[1] = -10.0;
		param.maxSliders[1] =  10.0;

		param.defaultValues[2] = 0.0;
		param.minSliders[2] = -10.0;
		param.maxSliders[2] =  10.0;

		OP_ParAppendResult res = manager->appendXYZ(param);
		assert(res == OP_ParAppendResult::Success);
	}
	

	{
		OP_NumericParameter param;
		param.name = "Restitution";
		param.label = "Restitution";
		param.defaultValues[0] = 0.0;
		param.minSliders[0] = 0.0;
		param.maxSliders[0] =  10.0;

		OP_ParAppendResult res = manager->appendFloat(param);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter param;
		param.name = "Viscosity";
		param.label = "Viscosity";
		param.defaultValues[0] = 0.0;
		param.minSliders[0] = 0.0;
		param.maxSliders[0] =  10.0;

		OP_ParAppendResult res = manager->appendFloat(param);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter param;
		param.name = "Vorticityconfinement";
		param.label = "Vorticity Confinement";
		param.defaultValues[0] = 0.0;
		param.minSliders[0] = 0.0;
		param.maxSliders[0] =  10.0;

		OP_ParAppendResult res = manager->appendFloat(param);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter param;
		param.name = "Adhesion";
		param.label = "Adhesion";
		param.defaultValues[0] = 0.0;
		param.minSliders[0] = 0.0;
		param.maxSliders[0] =  1.0;

		OP_ParAppendResult res = manager->appendFloat(param);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter param;
		param.name = "Cohesion";
		param.label = "Cohesion";
		param.defaultValues[0] = 0.025f;
		param.minSliders[0] = 0.0;
		param.maxSliders[0] =  1.0;

		OP_ParAppendResult res = manager->appendFloat(param);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter param;
		param.name = "Surfacetension";
		param.label = "Surface Tension";
		param.defaultValues[0] = 0.0f;
		param.minSliders[0] = 0.0;
		param.maxSliders[0] =  10.0;

		OP_ParAppendResult res = manager->appendFloat(param);
		assert(res == OP_ParAppendResult::Success);
	}
}

void 
CPlusPlusCHOPExample::pulsePressed(const char* name)
{
	if (!strcmp(name, "Reset"))
	{
		m_dirty = true;
	}
}

