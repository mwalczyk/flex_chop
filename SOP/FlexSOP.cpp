/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
* can only be used, and/or modified for use, in conjunction with
* Derivative's TouchDesigner software, and only if you are a licensee who has
* accepted Derivative's TouchDesigner license or assignment agreement (which
* also govern the use of this file).  You may share a modified version of this
* file with another authorized licensee of Derivative's TouchDesigner software.
* Otherwise, no redistribution or sharing of this file, with or without
* modification, is permitted.
*/

#include "FlexSOP.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

extern "C"
{

	DLLEXPORT
	int32_t
	GetSOPAPIVersion(void)
	{
		return SOP_CPLUSPLUS_API_VERSION;
	}

	DLLEXPORT
	SOP_CPlusPlusBase*
	CreateSOPInstance(const OP_NodeInfo* info)
	{
		return new FlexSOP(info);
	}

	DLLEXPORT
	void
	DestroySOPInstance(SOP_CPlusPlusBase* instance)
	{
		delete (FlexSOP*)instance;
	}

};

void FlexSOP::setupCollisionPlanes(float w, float h)
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

void FlexSOP::setupSimulationParams()
{
	// AABB that will enclose the scene
	setupCollisionPlanes(0.5f, 0.5f);

	const float particle_radius = 0.05f;
	m_params.gravity[0] = 0.0f;
	m_params.gravity[1] = -9.8f;
	m_params.gravity[2] = 0.0f;
	m_params.wind[0] = 0.0f;
	m_params.wind[1] = 0.0f;
	m_params.wind[2] = 0.0f;
	m_params.viscosity = 0.0f;
	m_params.dynamicFriction = 0.45f;
	m_params.staticFriction = 0.45f;
	m_params.particleFriction = 0.45f;
	m_params.freeSurfaceDrag = 0.0f;
	m_params.drag = 0.0f;
	m_params.lift = 3.0f;
	m_params.numIterations = 3;
	m_params.anisotropyScale = 1.0f;
	m_params.anisotropyMin = 0.1f;
	m_params.anisotropyMax = 2.0f;
	m_params.smoothing = 4.0f;
	m_params.dissipation = 0.0f;
	m_params.damping = 0.0f;
	m_params.particleCollisionMargin = 0.0f;
	m_params.shapeCollisionMargin = 0.0f;
	m_params.collisionDistance = particle_radius * 0.95f;
	m_params.sleepThreshold = 0.0f;
	m_params.shockPropagation = 0.0f;
	m_params.restitution = 0.0f;
	m_params.maxSpeed = 100.0f;
	m_params.maxAcceleration = 100.0f;
	m_params.relaxationMode = eNvFlexRelaxationGlobal;
	m_params.relaxationFactor = 0.25f;
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
}

void FlexSOP::MapBuffers()
{
	m_sim_buffers->positions.map();
	m_sim_buffers->velocities.map();
	m_sim_buffers->phases.map();
	m_sim_buffers->activeIndices.map();
	m_sim_buffers->springIndices.map();
	m_sim_buffers->springLengths.map();
	m_sim_buffers->springStiffness.map();
	m_sim_buffers->triangles.map();
}

void FlexSOP::UnmapBuffers()
{
	m_sim_buffers->positions.unmap();
	m_sim_buffers->velocities.unmap();
	m_sim_buffers->phases.unmap();
	m_sim_buffers->activeIndices.unmap();
	m_sim_buffers->springIndices.unmap();
	m_sim_buffers->springLengths.unmap();
	m_sim_buffers->springStiffness.unmap();
	m_sim_buffers->triangles.unmap();
}

inline int GridIndex(int x, int y, int dx) { return y * dx + x; }

void FlexSOP::CreateSpringGrid(float3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, float3 velocity, float invMass)
{
	int baseIndex = int(m_sim_buffers->positions.size());

	for (int z=0; z < dz; ++z)
	{
		for (int y=0; y < dy; ++y)
		{
			for (int x=0; x < dx; ++x)
			{
				float3 position;
				position.x = lower.x + radius * float(x);
				position.y = lower.y + radius * float(y);
				position.z = lower.z + radius * float(z);

				m_sim_buffers->positions.push_back(make_float4(position.x, position.y, position.z, invMass));
				m_sim_buffers->velocities.push_back(velocity);
				m_sim_buffers->phases.push_back(phase);

				if (x > 0 && y > 0)
				{
					m_sim_buffers->triangles.push_back(baseIndex + GridIndex(x-1, y-1, dx));
					m_sim_buffers->triangles.push_back(baseIndex + GridIndex(x, y-1, dx));
					m_sim_buffers->triangles.push_back(baseIndex + GridIndex(x, y, dx));

					m_sim_buffers->triangles.push_back(baseIndex + GridIndex(x-1, y-1, dx));
					m_sim_buffers->triangles.push_back(baseIndex + GridIndex(x, y, dx));
					m_sim_buffers->triangles.push_back(baseIndex + GridIndex(x-1, y, dx));
				}
			}
		}
	}	

	// horizontal
	for (int y=0; y < dy; ++y)
	{
		for (int x=0; x < dx; ++x)
		{
			int index0 = y*dx + x;

			if (x > 0)
			{
				int index1 = y*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (x > 1)
			{
				int index2 = y*dx + x - 2;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}

			if (y > 0 && x < dx-1)
			{
				int indexDiag = (y-1)*dx + x + 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}

			if (y > 0 && x > 0)
			{
				int indexDiag = (y-1)*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}
		}
	}

	// vertical
	for (int x=0; x < dx; ++x)
	{
		for (int y=0; y < dy; ++y)
		{
			int index0 = y*dx + x;

			if (y > 0)
			{
				int index1 = (y-1)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (y > 1)
			{
				int index2 = (y-2)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}
		}
	}	
}

void FlexSOP::CreateSpring(int i, int j, float stiffness, float give)
{
	m_sim_buffers->springIndices.push_back(i);
	m_sim_buffers->springIndices.push_back(j);

	float dx = m_sim_buffers->positions[i].x - m_sim_buffers->positions[j].x;
	float dy = m_sim_buffers->positions[i].y - m_sim_buffers->positions[j].y;
	float dz = m_sim_buffers->positions[i].z - m_sim_buffers->positions[j].z;
	float length = sqrtf(dx * dx + dy * dy + dz * dz);

	m_sim_buffers->springLengths.push_back((1.0f+give) * length);
	m_sim_buffers->springStiffness.push_back(stiffness);	
}

void FlexSOP::buildOutputGeometry(SOP_Output* output)
{
	// Read-back simulation data from flex buffers
	NvFlexGetParticles(m_solver, m_sim_buffers->positions.buffer, NULL);
	NvFlexGetVelocities(m_solver, m_sim_buffers->velocities.buffer, NULL);
	NvFlexGetDynamicTriangles(m_solver, m_sim_buffers->triangles.buffer, NULL, m_number_of_triangles);

	float4* particles = (float4*)NvFlexMap(m_sim_buffers->positions.buffer, eNvFlexMapWait);
	float3* velocities = (float3*)NvFlexMap(m_sim_buffers->velocities.buffer, eNvFlexMapWait);
	int* indices = (int*)NvFlexMap(m_sim_buffers->triangles.buffer, eNvFlexMapWait);

	// Copy to SOP output - could be GPU-bound if VBOs were registered with CUDA...
	for (size_t i = 0; i < m_sim_buffers->positions.size(); ++i)
	{
		output->addPoint(particles[i].x, 
						 particles[i].y, 
						 particles[i].z);
	}

	for (int i = 0; i < m_sim_buffers->triangles.size() / 3; ++i)
	{
		output->addTriangle(indices[i * 3 + 0],
							indices[i * 3 + 1],
							indices[i * 3 + 2]);
	}

	NvFlexUnmap(m_sim_buffers->positions.buffer);
	NvFlexUnmap(m_sim_buffers->velocities.buffer);
	NvFlexUnmap(m_sim_buffers->triangles.buffer);
}

FlexSOP::FlexSOP(const OP_NodeInfo* info) : m_node_info(info)
{
	m_execute_count = 0;

	// Initialize flex library
	m_library = NvFlexInit();

	// Initialize flex buffers
	m_sim_buffers = std::make_shared<SimBuffers>(m_library);

	// Create solver from solver description struct
	NvFlexSolverDesc solver_description;
	NvFlexSetSolverDescDefaults(&solver_description);
	solver_description.maxParticles = m_max_number_of_particles;
	solver_description.maxDiffuseParticles = 0;
	m_solver = NvFlexCreateSolver(m_library, &solver_description);

	// Setup params (gravity, friction, etc.)
	setupSimulationParams();

	// Create a force field
	m_force_field_callback = NvFlexExtCreateForceFieldCallback(m_solver);
	m_force_field.mLinearFalloff = true;
	m_force_field.mMode = eNvFlexExtModeForce;
	m_force_field.mPosition[0] = 0.0f;
	m_force_field.mPosition[1] = 0.0f;
	m_force_field.mPosition[2] = 0.0f;
	m_force_field.mRadius = 1.0f;
	m_force_field.mStrength = 0.0f;

	std::cout << "SOP constructor called\n";
}

FlexSOP::~FlexSOP()
{
	// SimBuffers will be destroyed automatically (RAII)

	NvFlexExtDestroyForceFieldCallback(m_force_field_callback);
	NvFlexDestroySolver(m_solver);
	NvFlexShutdown(m_library);
}

void FlexSOP::getGeneralInfo(SOP_GeneralInfo* ginfo)
{
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->directToGPU = false;
}

//-----------------------------------------------------------------------------------------------------
//										Generate a geometry on CPU
//-----------------------------------------------------------------------------------------------------

void FlexSOP::execute(SOP_Output* output, OP_Inputs* inputs, void* reserved)
{
	m_execute_count++;

	inputs->enablePar("Reset", true);

	//-----------------------------------------------------------------------------------------------------
	// Create spring grid
	//-----------------------------------------------------------------------------------------------------
	if (m_dirty)
	{
		std::cout << "Resetting Flex system...\n";

		MapBuffers();

		{
			const float stretchStiffness = 1.0f;
			const float bendStiffness = 0.95f;
			const float shearStiffness = 0.95f;
			const float radius = 0.09f;
			const int dimx = 30;
			const int dimz = 30;
			const int phase = NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);
			float spacing = radius * 0.8f;

			// Create springs
			auto lower = make_float3(-dimx * spacing * 0.5f, 1.5f, -dimz * spacing * 0.5f);
			auto velocity = make_float3(0.0f, 0.0f, 0.0f);
			CreateSpringGrid(lower, dimx, dimz, 1, spacing, phase, stretchStiffness, bendStiffness, shearStiffness, velocity, 0.5f);

			// Enable all particles
			for (size_t i = 0; i < m_sim_buffers->positions.size(); ++i)
			{
				m_sim_buffers->activeIndices.push_back(i);
			}

			m_buffers_ready = true;
			m_dirty = false;
		}

		UnmapBuffers();
	}


	//-----------------------------------------------------------------------------------------------------
	// Update Flex
	//-----------------------------------------------------------------------------------------------------
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

		// Set particles, velocities, etc. for flex simulation
		NvFlexSetParticles(m_solver, m_sim_buffers->positions.buffer, NULL);
		NvFlexSetVelocities(m_solver, m_sim_buffers->velocities.buffer, NULL);
		NvFlexSetPhases(m_solver, m_sim_buffers->phases.buffer, NULL);
		NvFlexSetActive(m_solver, m_sim_buffers->activeIndices.buffer, NULL);
		NvFlexSetDynamicTriangles(m_solver, m_sim_buffers->triangles.buffer, NULL, m_sim_buffers->triangles.size() / 3); 
		NvFlexSetSprings(m_solver, m_sim_buffers->springIndices.buffer, m_sim_buffers->springLengths.buffer, m_sim_buffers->springStiffness.buffer, m_sim_buffers->springLengths.size());
		
		NvFlexSetActiveCount(m_solver, m_sim_buffers->activeIndices.size());

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


		//-----------------------------------------------------------------------------------------------------
		// Update SOP output geometry
		//-----------------------------------------------------------------------------------------------------
		buildOutputGeometry(output);
	}
}

void FlexSOP::executeVBO(SOP_VBOOutput* output, OP_Inputs* inputs, void* reserved) 
{
}

//-----------------------------------------------------------------------------------------------------
//								CHOP, DAT, and custom parameters
//-----------------------------------------------------------------------------------------------------

int32_t FlexSOP::getNumInfoCHOPChans()
{
	return 1;
}

void FlexSOP::getInfoCHOPChan(int32_t index, OP_InfoCHOPChan* chan)
{
	if (index == 0)
	{
		chan->name = "executeCount";
		chan->value = (float)m_execute_count;
	}
}

bool FlexSOP::getInfoDATSize(OP_InfoDATSize* infoSize)
{
	infoSize->rows = 1;
	infoSize->cols = 1;
	infoSize->byColumn = false;
	return true;
}

void FlexSOP::getInfoDATEntries(int32_t index, int32_t nEntries, OP_InfoDATEntries* entries)
{
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
		sprintf_s(tempBuffer2, "%d", m_execute_count);
#else // macOS
		snprintf(tempBuffer2, sizeof(tempBuffer2), "%d", myExecuteCount);
#endif
		entries->values[1] = tempBuffer2;
	}
}

void FlexSOP::setupParameters(OP_ParameterManager* manager)
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
		param.defaultValues[0] = 0.0f;
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

void FlexSOP::pulsePressed(const char* name)
{
	if (!strcmp(name, "Reset"))
	{
		m_dirty = true;
	}
}

