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

	const float particle_radius = 0.1f;
	m_params.gravity[0] = 0.0f;
	m_params.gravity[1] = 0.0f;
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
	m_params.lift = 0.0f;
	m_params.numIterations = 3;
	m_params.anisotropyScale = 1.0f;
	m_params.anisotropyMin = 0.1f;
	m_params.anisotropyMax = 2.0f;
	m_params.smoothing = 4.0f;
	m_params.dissipation = 0.0f;
	m_params.damping = 0.0f;
	m_params.particleCollisionMargin = 0.0f;
	m_params.shapeCollisionMargin = 0.0f;
	m_params.collisionDistance = particle_radius * 0.75f;
	m_params.sleepThreshold = 0.0f;
	m_params.shockPropagation = 0.0f;
	m_params.restitution = 0.0f;
	m_params.maxSpeed = 100.0f;
	m_params.maxAcceleration = 100.0f;
	m_params.relaxationMode = eNvFlexRelaxationLocal;
	m_params.relaxationFactor = 0.6f;
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

	m_sim_buffers->rigidOffsets.map();
	m_sim_buffers->rigidIndices.map();
	//m_sim_buffers->rigidMeshSize.map();
	m_sim_buffers->rigidCoefficients.map();
	m_sim_buffers->rigidPlasticThresholds.map();
	m_sim_buffers->rigidPlasticCreeps.map();
	m_sim_buffers->rigidRotations.map();
	m_sim_buffers->rigidTranslations.map();
	m_sim_buffers->rigidLocalPositions.map();
	m_sim_buffers->rigidLocalNormals.map();
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

	m_sim_buffers->rigidOffsets.unmap();
	m_sim_buffers->rigidIndices.unmap();
	//m_sim_buffers->rigidMeshSize.unmap();
	m_sim_buffers->rigidCoefficients.unmap();
	m_sim_buffers->rigidPlasticThresholds.unmap();
	m_sim_buffers->rigidPlasticCreeps.unmap();
	m_sim_buffers->rigidRotations.unmap();
	m_sim_buffers->rigidTranslations.unmap();
	m_sim_buffers->rigidLocalPositions.unmap();
	m_sim_buffers->rigidLocalNormals.unmap();
}

// Calculates the center of mass of every rigid given a set of particle positions and rigid indices
void CalculateRigidCentersOfMass(const float4* restPositions, int numRestPositions, const int* offsets, float3* translations, const int* indices, int numRigids)
{
	// To improve the accuracy of the result, first transform the restPositions to relative coordinates (by finding the mean and subtracting that from all positions)
	// Note: If this is not done, one might see ghost forces if the mean of the restPositions is far from the origin.
	float3 shapeOffset = make_float3(0.0f, 0.0f, 0.0f);

	for (int i = 0; i < numRestPositions; i++)
	{
		float4 rp = restPositions[i];
		shapeOffset.x += rp.x;
		shapeOffset.y += rp.y;
		shapeOffset.z += rp.z;
	}

	shapeOffset.x /= float(numRestPositions);
	shapeOffset.y /= float(numRestPositions);
	shapeOffset.z /= float(numRestPositions);

	for (int i = 0; i < numRigids; ++i)
	{
		const int startIndex = offsets[i];
		const int endIndex = offsets[i + 1];

		const int n = endIndex-startIndex;

		assert(n);

		float3 com = make_float3(0.0f, 0.0f, 0.0f);

		for (int j=startIndex; j < endIndex; ++j)
		{
			const int r = indices[j];

			float4 rp = restPositions[r];
			rp.x -= shapeOffset.x;
			rp.y -= shapeOffset.y;
			rp.z -= shapeOffset.z;

			// By subtracting shapeOffset the calculation is done in relative coordinates
			com.x += rp.x;
			com.y += rp.y;
			com.z += rp.z;
		}

		com.x /= float(n);
		com.y /= float(n);
		com.z /= float(n);

		// Add the shapeOffset to switch back to absolute coordinates
		com.x += shapeOffset.x;
		com.y += shapeOffset.y;
		com.z += shapeOffset.z;

		translations[i] = com;
	}
}

// Calculates local space positions given a set of particle positions, rigid indices and centers of mass of the rigids
void CalculateRigidLocalPositions(const float4* restPositions, const int* offsets, const float3* translations, const int* indices, int numRigids, float3* localPositions)
{
	int count = 0;

	for (int i=0; i < numRigids; ++i)
	{
		const int startIndex = offsets[i];
		const int endIndex = offsets[i + 1];

		assert(endIndex - startIndex);

		for (int j=startIndex; j < endIndex; ++j)
		{
			const int r = indices[j];

			float4 rp = restPositions[r];
			float3 tr = translations[i];

			localPositions[count++] = make_float3(rp.x - tr.x, rp.y - tr.y, rp.z - tr.z);
		}
	}
}

void FlexSOP::createCubeSoftBody()
{
	float vertices[] = {
		// front
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,

		// back
		-1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f
	};
	
	int32_t indices[] = {
		// front
		0, 1, 2,
		2, 3, 0,
		// top
		1, 5, 6,
		6, 2, 1,
		// back
		7, 6, 5,
		5, 4, 7,
		// bottom
		4, 0, 3,
		3, 7, 4,
		// left
		4, 5, 1,
		1, 0, 4,
		// right
		3, 2, 6,
		6, 7, 3,
	};

	const size_t number_of_vertices = 8;
	const size_t number_of_indices = 36;

	const float radius = 0.1f;
	const float cluster_spacing = 1.0f;
	const float cluster_radius = 0.0f;
	const float cluster_stiffness = 0.25f;
	const float link_radius = 0.0f;
	const float link_stiffness = 1.0f;
	const float global_stiffness = 1.0f;
	const float surface_sampling = 0.0f;
	const float volume_sampling = 0.1f;
	const float cluster_plastic_threshold = 0.0f;// 0.0015f;
	const float cluster_plastic_creep = 0.125f;

	// Create soft body definition
	NvFlexExtAsset* asset = NvFlexExtCreateSoftFromMesh(
		vertices,
		number_of_vertices,
		indices,
		number_of_indices,
		radius,
		volume_sampling,
		surface_sampling,
		cluster_spacing * radius,
		cluster_radius * radius,
		cluster_stiffness,
		link_radius * radius,
		link_stiffness,
		global_stiffness, 
		cluster_plastic_threshold, 
		cluster_plastic_creep);
	
	// Add particle data to buffers
	for (size_t i = 0; i < asset->numParticles; ++i)
	{
		float px = asset->particles[i * 4 + 0];
		float py = asset->particles[i * 4 + 1];
		float pz = asset->particles[i * 4 + 2];
		float pw = asset->particles[i * 4 + 3];

		m_sim_buffers->positions.push_back(make_float4(px, py, pz, pw));
		m_sim_buffers->velocities.push_back(make_float3(0.0f, 0.0f, 0.0f));

		const int phase = NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);
		m_sim_buffers->phases.push_back(phase);
	}

	const size_t particleOffset = 0;
	const size_t indexOffset = 0;

	// Add shape data to solver
	for (int i = 0; i < asset->numShapeIndices; ++i)
	{
		m_sim_buffers->rigidIndices.push_back(asset->shapeIndices[i] + particleOffset);
	}

	for (int i = 0; i < asset->numShapes; ++i)
	{
		m_sim_buffers->rigidOffsets.push_back(asset->shapeOffsets[i] + indexOffset);

		float scx = asset->shapeCenters[i * 3 + 0];
		float scy = asset->shapeCenters[i * 3 + 1];
		float scz = asset->shapeCenters[i * 3 + 2];
		m_sim_buffers->rigidTranslations.push_back(make_float3(scx, scy, scz));
		m_sim_buffers->rigidRotations.push_back(make_float4(0.0f, 0.0f, 0.0f, 1.0f));
		m_sim_buffers->rigidCoefficients.push_back(asset->shapeCoefficients[i]);
		m_sim_buffers->rigidPlasticCreeps.push_back(asset->shapePlasticCreeps[i]);
		m_sim_buffers->rigidPlasticThresholds.push_back(asset->shapePlasticThresholds[i]);
	}

	// Add link data to the solver 
	for (size_t i = 0; i < asset->numSprings; ++i)
	{
		m_sim_buffers->springIndices.push_back(asset->springIndices[i * 2 + 0]);
		m_sim_buffers->springIndices.push_back(asset->springIndices[i * 2 + 1]);

		m_sim_buffers->springStiffness.push_back(asset->springCoefficients[i]);
		m_sim_buffers->springLengths.push_back(asset->springRestLengths[i]);
	}





	// Builds rigids constraints
	if (m_sim_buffers->rigidOffsets.size())
	{
		assert(m_sim_buffers->rigidOffsets.size() > 1);

		const int numRigids = m_sim_buffers->rigidOffsets.size() - 1;

		// If the centers of mass for the rigids are not yet computed, this is done here
		// (If the CreateParticleShape method is used instead of the NvFlexExt methods, the centers of mass will be calculated here)
		if (m_sim_buffers->rigidTranslations.size() == 0)
		{
			m_sim_buffers->rigidTranslations.resize(m_sim_buffers->rigidOffsets.size() - 1, make_float3(0.0f, 0.0f, 0.0f));
			CalculateRigidCentersOfMass(&m_sim_buffers->positions[0], m_sim_buffers->positions.size(), &m_sim_buffers->rigidOffsets[0], &m_sim_buffers->rigidTranslations[0], &m_sim_buffers->rigidIndices[0], numRigids);
		}

		// calculate local rest space positions
		m_sim_buffers->rigidLocalPositions.resize(m_sim_buffers->rigidOffsets.back());
		CalculateRigidLocalPositions(&m_sim_buffers->positions[0], &m_sim_buffers->rigidOffsets[0], &m_sim_buffers->rigidTranslations[0], &m_sim_buffers->rigidIndices[0], numRigids, &m_sim_buffers->rigidLocalPositions[0]);

		// set rigidRotations to correct length, probably NULL up until here
		m_sim_buffers->rigidRotations.resize(m_sim_buffers->rigidOffsets.size() - 1, make_float4(0.0f, 0.0f, 0.0f, 1.0f));

		// set rigidLocalNormals to correct length ... is this correct?
		m_sim_buffers->rigidLocalNormals.resize(m_sim_buffers->rigidOffsets.size() - 1, make_float4(0.0f, 0.0f, 0.0f, 1.0f));
	}

	std::cout << "Finished creating softbody cube...\n";

	NvFlexExtDestroyAsset(asset);

	std::cout << "Successfully destroyed asset...\n";
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
	//NvFlexGetDynamicTriangles(m_solver, m_sim_buffers->triangles.buffer, NULL, m_number_of_triangles);

	float4* particles = (float4*)NvFlexMap(m_sim_buffers->positions.buffer, eNvFlexMapWait);
	float3* velocities = (float3*)NvFlexMap(m_sim_buffers->velocities.buffer, eNvFlexMapWait);
	//int* indices = (int*)NvFlexMap(m_sim_buffers->triangles.buffer, eNvFlexMapWait);

	// Copy to SOP output - could be GPU-bound if VBOs were registered with CUDA...
	for (size_t i = 0; i < m_sim_buffers->positions.size(); ++i)
	{
		output->addPoint(particles[i].x, 
						 particles[i].y, 
						 particles[i].z);
	}

	//for (int i = 0; i < m_sim_buffers->triangles.size() / 3; ++i)
	//{
	//	output->addTriangle(indices[i * 3 + 0],
	//						indices[i * 3 + 1],
	//						indices[i * 3 + 2]);
	//}

	NvFlexUnmap(m_sim_buffers->positions.buffer);
	NvFlexUnmap(m_sim_buffers->velocities.buffer);
	//NvFlexUnmap(m_sim_buffers->triangles.buffer);
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
			//const float stretchStiffness = 1.0f;
			//const float bendStiffness = 0.95f;
			//const float shearStiffness = 0.95f;
			//const float radius = 0.09f;
			//const int dimx = 70;
			//const int dimz = 70;
			//const int phase = NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);
			//float spacing = radius * 0.8f;

			//// Create springs
			//auto lower = make_float3(-dimx * spacing * 0.5f, 1.5f, -dimz * spacing * 0.5f);
			//auto velocity = make_float3(0.0f, 0.0f, 0.0f);
			//CreateSpringGrid(lower, dimx, dimz, 1, spacing, phase, stretchStiffness, bendStiffness, shearStiffness, velocity, 0.5f);
			createCubeSoftBody();

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
		//NvFlexSetDynamicTriangles(m_solver, m_sim_buffers->triangles.buffer, NULL, m_sim_buffers->triangles.size() / 3); 
		NvFlexSetSprings(m_solver, m_sim_buffers->springIndices.buffer, m_sim_buffers->springLengths.buffer, m_sim_buffers->springStiffness.buffer, m_sim_buffers->springLengths.size());
		
		NvFlexSetRigids(m_solver, 
			m_sim_buffers->rigidOffsets.buffer, 
			m_sim_buffers->rigidIndices.buffer, 
			m_sim_buffers->rigidLocalPositions.buffer, 
			m_sim_buffers->rigidLocalNormals.buffer, 
			m_sim_buffers->rigidCoefficients.buffer, 
			m_sim_buffers->rigidPlasticThresholds.buffer, 
			m_sim_buffers->rigidPlasticCreeps.buffer, 
			m_sim_buffers->rigidRotations.buffer, 
			m_sim_buffers->rigidTranslations.buffer, 
			m_sim_buffers->rigidOffsets.size() - 1, 
			m_sim_buffers->rigidIndices.size());

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

