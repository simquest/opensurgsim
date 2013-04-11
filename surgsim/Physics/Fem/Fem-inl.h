#include "FEM.h"

#include <iomanip>
#include <mkl.h>
#include <mkl_spblas.h>
#include <mkl_dss.h>

template <int nbNodePerElement, int nbDOFPerNode> 
int Fem<nbNodePerElement, nbDOFPerNode>::m_convergenceVerbosity = -1;  // default: no spew!

template <int nbNodePerElement, int nbDOFPerNode> 
Fem<nbNodePerElement, nbDOFPerNode>::Fem(double dt, bool usesPrecomputedDecomposition, bool isDynamic, bool usesMLCP) :
	m_nbNodes(0),
	m_undeformedMesh(0),
	m_deformedMesh(0),
	m_lastDeformedMesh(0),
	m_isUndeformedMeshOwned(false),
	m_nbBCAllocated(16), 
	m_nbBC(0),
	m_BC((int*)malloc(sizeof(int) * 16)), 
	m_nbElementsAllocated(16),
	m_nbElements(0), 
	m_elements((FEMElement**)malloc(sizeof(FEMElement*) * 16)), 
	m_areElementsOwned(false), 
	m_RayleightDamping_Mass_coef(0.0), 
	m_RayleightDamping_Stif_coef(0.0), 
	m_dt(dt),
	m_CG_maxIteration(20),
	m_CG_epsilon(1.0e-5), 
	m_usesPrecomputedDecomposition(usesPrecomputedDecomposition), 
	m_isDynamic(isDynamic), 
	m_usesMLCP(usesMLCP),
	m_rotationPerNode(0), 
	m_rotationPerNodeQ(0),
	m_corotational(false),m_updatedModel(false),
	m_BCmass(1.0), m_BCinvMass(0.0),
	m_staticCorot_DispScaleFactor(0.1)
{
}

template <int nbNodePerElement, int nbDOFPerNode> 
Fem<nbNodePerElement, nbDOFPerNode>::Fem(int nbPts , Pt3D* pts, double dt, bool usesPrecomputedDecomposition, bool isDynamic, bool usesMLCP) :
	m_name("FEM"),
	m_nbNodes(nbPts),
	m_undeformedMesh((Pt3D*)malloc(sizeof(Pt3D) * nbPts)),
	m_deformedMesh((Pt3D*)malloc(sizeof(Pt3D) * nbPts)),
	m_isUndeformedMeshOwned(true),
	m_lastDeformedMesh((Pt3D*)malloc(sizeof(Pt3D) * nbPts)),
	m_nbBCAllocated(16),
	m_nbBC(0),
	m_BC((int*)malloc(sizeof(int) * 16)),
	m_nbElementsAllocated(16),
	m_nbElements(0),
	m_elements((FEMElement**)malloc(sizeof(FEMElement*) * 16)),
	m_areElementsOwned(false),
	m_RayleightDamping_Mass_coef(0.0),
	m_RayleightDamping_Stif_coef(0.0),
	m_dt(dt),
	m_CG_maxIteration(20),
	m_CG_epsilon(1.0e-5),
	m_usesPrecomputedDecomposition(usesPrecomputedDecomposition),
	m_isDynamic(isDynamic),
	m_usesMLCP(usesMLCP),
	m_rotationPerNode(0), 
	m_rotationPerNodeQ(0),
	m_corotational(false),m_updatedModel(false),
	m_BCmass(1.0), m_BCinvMass(0.0),
	m_staticCorot_DispScaleFactor(0.1)
{
	vec_generic_copy<double,double>((double*)m_undeformedMesh, (double*)pts, 3 * m_nbNodes );
	vec_generic_copy<double,double>((double*)m_deformedMesh, (double*)m_undeformedMesh, 3 * m_nbNodes );
	vec_generic_copy<double,double>((double*)m_lastDeformedMesh, (double*)m_deformedMesh, 3 * m_nbNodes );
}

template <int nbNodePerElement, int nbDOFPerNode> 
Fem<nbNodePerElement, nbDOFPerNode>::~Fem()
{
	if (m_rotationPerNode)
	{
		delete [] m_rotationPerNode;
		m_rotationPerNode = 0;
	}
	if(m_rotationPerNodeQ) 
	{
		delete [] m_rotationPerNodeQ; 
		m_rotationPerNodeQ = 0;
	}

	if (m_deformedMesh)
	{
		free(m_deformedMesh);
		m_deformedMesh = 0;
	}
	if (m_isUndeformedMeshOwned && m_undeformedMesh)
	{
		free(m_undeformedMesh); 
		m_undeformedMesh = 0;
	}
	if (m_lastDeformedMesh)
	{
		free(m_lastDeformedMesh);
		m_lastDeformedMesh = 0;
	}

	if (m_elements)
	{
		if (m_areElementsOwned)
		{
			for (int elementID = 0; elementID < m_nbElements; ++elementID) 
			{
				delete m_elements[elementID];
			}
		}
		free(m_elements);
		m_elements = 0;
	}

	if (m_BC) 
	{
		free(m_BC); 
		m_BC = 0;
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::setName(const std::string& name) 
{ 
	m_name = name; 
}
template <int nbNodePerElement, int nbDOFPerNode> 
const std::string& Fem<nbNodePerElement, nbDOFPerNode>::getName() const 
{ 
	return m_name; 
}

template <int nbNodePerElement, int nbDOFPerNode> 
bool Fem<nbNodePerElement, nbDOFPerNode>::simulationIsDynamic() const
{
	return m_isDynamic;
}
template <int nbNodePerElement, int nbDOFPerNode> 
bool Fem<nbNodePerElement, nbDOFPerNode>::simulationIsStatic() const  
{
	return !m_isDynamic; 
}

template <int nbNodePerElement, int nbDOFPerNode> 
bool Fem<nbNodePerElement, nbDOFPerNode>::simulationUsesPrecomputedDecomposition() const
{ 
	return m_usesPrecomputedDecomposition;
}
template <int nbNodePerElement, int nbDOFPerNode> 
bool Fem<nbNodePerElement, nbDOFPerNode>::simulationUsesConjugateGradient() const 
{
	return !m_usesPrecomputedDecomposition;
}

template <int nbNodePerElement, int nbDOFPerNode> 
bool Fem<nbNodePerElement, nbDOFPerNode>::simulationUsesMLCP() const 
{ 
	return m_usesMLCP;
}

template <int nbNodePerElement, int nbDOFPerNode> 
bool Fem<nbNodePerElement, nbDOFPerNode>::simulationComputesComplianceMatrix() const 
{ 
	return simulationUsesPrecomputedDecomposition() || simulationUsesMLCP(); 
}
template <int nbNodePerElement, int nbDOFPerNode> 
bool Fem<nbNodePerElement, nbDOFPerNode>::simulationNeedsFullMDx() const
{ 
	return simulationComputesComplianceMatrix() && simulationIsDynamic();
}
template <int nbNodePerElement, int nbDOFPerNode> 
bool Fem<nbNodePerElement, nbDOFPerNode>::simulationNeedsFullK() const
{ 
	return simulationComputesComplianceMatrix() || simulationUsesPrecomputedDecomposition() || simulationNeedsFullMDx();
}
template <int nbNodePerElement, int nbDOFPerNode> 
bool Fem<nbNodePerElement, nbDOFPerNode>::simulationNeedsFullM() const
{
	return simulationUsesPrecomputedDecomposition() || simulationNeedsFullMDx();
}

template <int nbNodePerElement, int nbDOFPerNode> 
int Fem<nbNodePerElement, nbDOFPerNode>::getNbNodes() const
{ 
	return m_nbNodes;
}
template <int nbNodePerElement, int nbDOFPerNode> 
const Pt3D* Fem<nbNodePerElement, nbDOFPerNode>::getUndeformedMesh() const
{
	return m_undeformedMesh; 
}
template <int nbNodePerElement, int nbDOFPerNode> 
const Pt3D* Fem<nbNodePerElement, nbDOFPerNode>::getDeformedMesh() const 
{ 
	return m_deformedMesh; 
}

template <int nbNodePerElement, int nbDOFPerNode> 
int Fem<nbNodePerElement, nbDOFPerNode>::getNbElements() const
{
	return m_nbElements;
}
template <int nbNodePerElement, int nbDOFPerNode> 
const Element<nbNodePerElement, nbDOFPerNode>* Fem<nbNodePerElement, nbDOFPerNode>::getElement(int i) const
{
	return m_elements[i];
}

template <int nbNodePerElement, int nbDOFPerNode> 
int Fem<nbNodePerElement, nbDOFPerNode>::getNbBC() const 
{ 
	return m_nbBC;
}
template <int nbNodePerElement, int nbDOFPerNode> 
int Fem<nbNodePerElement, nbDOFPerNode>::getBC(int i) const 
{ 
	return m_BC[i];
}

template <int nbNodePerElement, int nbDOFPerNode> 
int Fem<nbNodePerElement, nbDOFPerNode>::getNbDOF() const
{ 
	return m_nbNodes * nbDOFPerNode; 
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::setBC_VirtualMass(const double BC_virtualMass, const double BC_virtualInvMass)
{
	m_BCmass = BC_virtualMass;
	m_BCinvMass = BC_virtualInvMass;
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::setTimeStep(double dt)
{
	m_dt = dt;
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::setRayleightDamping(double massCoefficient, double stiffnessCoefficient)
{
	m_RayleightDamping_Mass_coef = massCoefficient;
	m_RayleightDamping_Stif_coef = stiffnessCoefficient;
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::setCGParameters(int maxIteration, double epsilon)
{ 
	m_CG_maxIteration = maxIteration; 
	m_CG_epsilon = epsilon;
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::setPts(int nbPts, Pt3D* pts)
{
	if (m_nbNodes != 0 && m_nbNodes != nbPts)
	{
		SQ_WARNING("Geometry of this FEM mesh has already been set!");
	}
	else
	{
		m_nbNodes = nbPts; 
		m_undeformedMesh = pts;
		m_isUndeformedMeshOwned = false;
		m_deformedMesh = (Pt3D*)realloc((void*)m_deformedMesh , sizeof(Pt3D) * m_nbNodes);
		vec_generic_copy<double,double>((double*)m_deformedMesh , (double*)m_undeformedMesh , 3 * m_nbNodes);
		m_lastDeformedMesh = (Pt3D*)realloc((void*)m_lastDeformedMesh , sizeof(Pt3D) * m_nbNodes);
		vec_generic_copy<double,double>((double*)m_lastDeformedMesh , (double*)m_deformedMesh , 3 * m_nbNodes);
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::addBoundaryCondition(int nodeID)
{
	if(m_nbBCAllocated == m_nbBC)
	{
		m_nbBCAllocated *= 2;
		m_BC = (int*)realloc((void*)m_BC , sizeof(int) * m_nbBCAllocated);
	}
	m_BC[m_nbBC] = nodeID;
	m_nbBC++;
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::addElement(FEMElement* element)
{
	if (m_nbElementsAllocated == m_nbElements)
	{
		m_nbElementsAllocated *= 2;
		m_elements = (FEMElement**)realloc((void*)m_elements , sizeof(FEMElement*) * m_nbElementsAllocated);
	}
	m_elements[m_nbElements] = element;
	element->setID(m_nbElements);
	m_nbElements++;
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::resetExternalForces()
{
	vec_generic_null<double>(m_F.getPointer(), nbDOFPerNode * m_nbNodes);
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::addGravity(const double gravity[3])
{
	for (int nodeIndex = 0 ; nodeIndex < m_nbNodes ; nodeIndex++)
	{
		double mass = getMassOnNode(nodeIndex);
		for (int i = 0; i < 3; ++i)
		{
			m_gravityForce[nbDOFPerNode*nodeIndex + i] = mass * gravity[i];
			m_F[nbDOFPerNode*nodeIndex + i] += m_gravityForce[nbDOFPerNode*nodeIndex + i];
		}
		for (int i = 3; i < 6; ++i)
		{
			m_gravityForce[nbDOFPerNode*nodeIndex + i] = 0.0;
		}
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::addGravity_nD(const double gravity[nbDOFPerNode])
{
	for (int nodeIndex = 0 ; nodeIndex < m_nbNodes ; nodeIndex++)
	{
		double mass = getMassOnNode(nodeIndex);
		for (int i = 0; i < nbDOFPerNode; ++i)
		{
			m_gravityForce[nbDOFPerNode*nodeIndex + i] = mass * gravity[i];
			m_F[nbDOFPerNode*nodeIndex + i] += m_gravityForce[nbDOFPerNode*nodeIndex + i];
		}
	}
}

/*!
If the gravity hasn't changed, use the precomputed gravity force, otherwise recompute the gravity force.
*/
template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::resetExternalForces_WithGravity(const double gravity[3])
{
	bool isGravitySame = true;
	for (int i = 0; i < 3; ++i)
	{
		if (abs(gravity[i] - m_gravity[i]) > std::numeric_limits<double>::epsilon())
		{
			isGravitySame = false;
		}
	}
	for (int i = 3; i < 6; ++i)
	{
		if ( fabs(m_gravity[i]) > std::numeric_limits<double>::epsilon())
		{
			isGravitySame = false;
		}
	}

	if (isGravitySame)
	{
		vec_generic_copy(m_F, m_gravityForce, nbDOFPerNode * m_nbNodes);
	}
	else
	{
		for (int nodeIndex = 0 ; nodeIndex < m_nbNodes ; nodeIndex++)
		{
			double mass = getMassOnNode(nodeIndex);
			for (int i = 0; i < 3; ++i)
			{
				m_gravityForce[nbDOFPerNode*nodeIndex + i] = mass * gravity[i];
				m_F[nbDOFPerNode*nodeIndex + i] = m_gravityForce[nbDOFPerNode*nodeIndex + i];
			}
			for (int i = 3; i < 6; ++i)
			{
				m_gravityForce[nbDOFPerNode*nodeIndex + i] = 0.0;
				m_F[nbDOFPerNode*nodeIndex + i] = m_gravityForce[nbDOFPerNode*nodeIndex + i];
			}
		}
	}
}

/*!
If the gravity hasn't changed, use the precomputed gravity force, otherwise recompute the gravity force.
*/
template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::resetExternalForces_WithGravity_nD(const double gravity[nbDOFPerNode])
{
	bool isGravitySame = true;
	for (int i = 0; i < nbDOFPerNode; ++i)
	{
		if (abs(gravity[i] - m_gravity[i]) > std::numeric_limits<double>::epsilon())
		{
			isGravitySame = false;
		}
	}

	if (isGravitySame)
	{
		vec_generic_copy(m_F, m_gravityForce, nbDOFPerNode * m_nbNodes);
	}
	else
	{
		for (int nodeIndex = 0 ; nodeIndex < m_nbNodes ; nodeIndex++)
		{
			double mass = getMassOnNode(nodeIndex);
			for (int i = 0; i < nbDOFPerNode; ++i)
			{
				m_gravityForce[nbDOFPerNode*nodeIndex + i] = mass * gravity[i];
				m_F[nbDOFPerNode*nodeIndex + i] = m_gravityForce[nbDOFPerNode*nodeIndex + i];
			}
		}
	}
}
template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::addExternalForceOnNode(const double force[nbDOFPerNode], int nodeID)
{
	SQ_ASSERT(nodeID >= 0 && nodeID < m_nbNodes, "Invalid FEM node index!");
	for (int i = 0; i < nbDOFPerNode; ++i)
	{
		m_F[nbDOFPerNode*nodeID + i] += force[i];
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
const Dynamic_Matrix<double>& Fem<nbNodePerElement, nbDOFPerNode>::getComplianceMatrix()
{
	if (m_C.getNbElem() > 0)
	{
		return m_C;
	}
	else
	{
		SQ_FAILURE("The compliance matrix is not available!");
	}
}


template <int nbNodePerElement, int nbDOFPerNode> 
double Fem<nbNodePerElement, nbDOFPerNode>::getMassOnNode(int nodeID) const
{
	return m_M_node[nodeID];  // Diagonal element of the mass matrix m_M[nbDOFPerNode*nodeID][nbDOFPerNode*nodeID]
}

template <int nbNodePerElement, int nbDOFPerNode> 
const double* Fem<nbNodePerElement, nbDOFPerNode>::getDisplacement(int nodeID) const 
{ 
	return m_Ut.getPointer() + nbDOFPerNode*nodeID; 
}
template <int nbNodePerElement, int nbDOFPerNode> 
const double* Fem<nbNodePerElement, nbDOFPerNode>::getDisplacements() const 
{ 
	return m_Ut.getPointer(); 
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::applyRigidTransformToInitialMesh(SqVec3d translation, SqQuaterniond quaternion)
{
	for (int nodeIndex = 0; nodeIndex < m_nbNodes; ++nodeIndex)
	{
		SqVec3d transformed = SqMatrix33d(quaternion) * SqVec3d(m_undeformedMesh[nodeIndex][0], m_undeformedMesh[nodeIndex][1], m_undeformedMesh[nodeIndex][2]) +
			translation;

		for (int i = 0; i < 3; ++i)
		{
			m_undeformedMesh[nodeIndex][i]   = transformed[i];
			m_deformedMesh[nodeIndex][i]     = transformed[i];
			m_lastDeformedMesh[nodeIndex][i] = transformed[i];
		}
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
int Fem<nbNodePerElement, nbDOFPerNode>::removeUnusedMeshNodes()
{
	bool* isNodeUsed = new bool[m_nbNodes];
	for(int nodeID = 0; nodeID < m_nbNodes; ++nodeID) 
	{
		isNodeUsed[nodeID] = false;
	}

	// Check whether each node is used
	for(int elementID = 0; elementID < m_nbElements; ++elementID)
	{
		for (int i = 0; i < nbNodePerElement; ++i)
		{
			isNodeUsed[m_elements[elementID]->getPointID(i)] = true;
		}
	}

	// Count the number of unused nodes
	int nbUnusedNodes = 0;

	// Remove any unused nodes
	for (int nodeID = m_nbNodes - 1; nodeID >= 0; --nodeID)
	{
		if (!isNodeUsed[nodeID])
		{
			// Update the points by shifting the next points downward
			for(int nextNodeID = nodeID + 1; nextNodeID < m_nbNodes; ++nextNodeID)
			{
				for (int i = 0; i < 3; ++i)
				{
					m_deformedMesh[nextNodeID-1][i] = m_deformedMesh[nextNodeID][i];
					m_undeformedMesh[nextNodeID-1][i] = m_undeformedMesh[nextNodeID][i];
					m_lastDeformedMesh[nextNodeID-1][i] = m_lastDeformedMesh[nextNodeID][i];
				}
			}

			m_nbNodes--;

			// Update the elements accordingly, shifting the indexes greater than node downward
			for (int elementID = 0; elementID < m_nbElements; ++elementID)
			{
				FEMElement* element = m_elements[elementID];

				for (int i = 0; i < nbNodePerElement; ++i)
				{
					if (element->getPointID(i) >= nodeID)
					{ 
						element->getPointID(i)--; 
					}
				}
			}

			// Update the boundary constraints accordingly, shifting the indices greater than node downward
			for (int boundaryConstraintID = 0; boundaryConstraintID < m_nbBC; ++boundaryConstraintID)
			{
				if (m_BC[boundaryConstraintID] >= nodeID)
				{
					m_BC[boundaryConstraintID]--;
				}
			}

			nbUnusedNodes++;
		}
	}

	// If there were unused nodes, reallocate the points to the exact number needed
	if (nbUnusedNodes > 0)
	{
		m_deformedMesh = (Pt3D*)realloc((void*)m_deformedMesh, sizeof(Pt3D) * m_nbNodes);
		m_undeformedMesh = (Pt3D*)realloc((void*)m_undeformedMesh, sizeof(Pt3D) * m_nbNodes);
		m_lastDeformedMesh = (Pt3D*)realloc((void*)m_lastDeformedMesh, sizeof(Pt3D) * m_nbNodes);

		// The realloc might have change the value of the pointers!
		// If any other location were pointing to this pointer, it should be updated here
		// Example: Element::m_undeformedMesh
		for (int elementIndex = 0; elementIndex < m_nbElements; ++elementIndex)
		{
			m_elements[elementIndex]->setUndeformedMesh(m_undeformedMesh);
		}
	}

	return nbUnusedNodes;
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::initializeAfterLoad()
{
	allocate();
	initialize( m_nbNodes , m_undeformedMesh );
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::allocate(void)
{
	SQ_ASSERT_WARNING(m_nbNodes,"FEM allocation with 0 node !");
	const int nbDOF = nbDOFPerNode * m_nbNodes;

	m_x0.resize(nbDOF);
	m_xt.resize(nbDOF);

	m_M_node.resize(m_nbNodes);
	if (simulationNeedsFullM())
	{
		m_M.resize(nbDOF, nbDOF);
	}
	if (simulationNeedsFullK())
	{
		m_K.resize(nbDOF, nbDOF);
		m_RK.resize(nbDOF, nbDOF); // Not sure yet if corotational will be used...so we have to allocate the matrix
	}

	if (simulationComputesComplianceMatrix())
	{
		m_K_LU.resize(nbDOF, nbDOF);
		m_LUpermutation.resize(nbDOF);

		m_C.resize(nbDOF, nbDOF); // Used for contact solving (Compliance matrix)
	}

	if (simulationNeedsFullMDx())
	{
		m_MDK.resize(nbDOF, nbDOF); // Used for backward Euler integration
		m_MD.resize(nbDOF, nbDOF); // Used for backward Euler integration
	}

	m_r.resize(nbDOF);
	m_d.resize(nbDOF);
	m_q.resize(nbDOF);
	m_FwithBC.resize(nbDOF);

	m_Ut.resize(nbDOF);
	vec_generic_null<double>(m_Ut.getPointer(), nbDOF);

	m_Uc_scaled.resize(nbDOF);
	vec_generic_null<double>(m_Uc_scaled.getPointer(), nbDOF);

	m_Ut_minus_dt.resize(nbDOF);
	vec_generic_null<double>(m_Ut_minus_dt.getPointer(), nbDOF);

	m_Vt.resize(nbDOF);
	vec_generic_null<double>(m_Vt.getPointer(), nbDOF);

	m_F.resize(nbDOF);
	vec_generic_null<double>(m_F.getPointer(), nbDOF);

	m_gravityForce.resize(nbDOF);
	vec_generic_null<double>(m_gravityForce.getPointer(), nbDOF);

	vec_generic_null<double>(m_gravity, nbDOFPerNode);

	// For corotational approach...
	m_rotationPerNode = new Static_Matrix<double,3,3>[m_nbNodes];
	m_rotationPerNodeQ = new SqQuaternion<double>[m_nbNodes];
};

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::initialize(int nbPts, double *pts, int byteOffset)
{
	// Prepare node poses
	for (int nodeIndex = 0; nodeIndex < m_nbNodes; ++nodeIndex)
	{
		for (int i = 0; i < 3; ++i)
		{
			m_x0[nbDOFPerNode*nodeIndex + i] = m_undeformedMesh[nodeIndex][i];
			m_xt[nbDOFPerNode*nodeIndex + i] = m_deformedMesh[nodeIndex][i];
		}
		for (int i = 3; i < nbDOFPerNode; ++i)
		{
			m_x0[nbDOFPerNode*nodeIndex + i] = 0.0;
			m_xt[nbDOFPerNode*nodeIndex + i] = 0.0;
		}
	}
};

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::Assemble_M()
{
	SQ_ASSERT(m_nbNodes > 0, "Can't assemble M matrix before defining the number of nodes in the mesh!");

	// Erase the mass matrix and node-mass vector
	mat_null<double>(m_M);
	vec_null<double>(m_M_node);

	// Fill up the stiffness matrix by assembling all the local element by element stiffness matrices
	for (int elementIndex = 0; elementIndex < m_nbElements; ++elementIndex)
	{
		FEMElement* t = m_elements[elementIndex];
		t->ComputeMassMatrix();
		const Static_Matrix<double,nbNodePerElement*nbDOFPerNode,nbNodePerElement*nbDOFPerNode> &Me = t->GetGlobalMassMatrix();

		double elementMass = t->getInitialVolume() * t->getVolumetricMass();

		for (int ptI = 0; ptI < nbNodePerElement; ++ptI)
		{
			int nodeI = t->getPointID(ptI);

			// M_node[I] == M[nbDOFPerNode*I][nbDOFPerNode*I], except we might not assemble the full M
			//m_M_node[nodeI] += Me[nbDOFPerNode*ptI][nbDOFPerNode*ptI];
			m_M_node[nodeI] += elementMass/(double)nbNodePerElement;

			if (simulationNeedsFullM())
			{
				for (int ptJ = 0; ptJ < nbNodePerElement; ++ptJ)
				{
					int nodeJ = t->getPointID(ptJ);

					for (int i = 0; i < nbDOFPerNode; i++)
					{
						for (int j = 0; j < nbDOFPerNode; j++)
						{
							m_M[nbDOFPerNode*nodeI + i][nbDOFPerNode*nodeJ + j] += Me[nbDOFPerNode*ptI + i][nbDOFPerNode*ptJ + j];
						}
					}
				}

				// Optional sanity check that M matches M_node
				//SQ_ASSERT(m_M_node[nodeI] == m_M[nbDOFPerNode*nodeI][nbDOFPerNode*nodeI], "M does not match M_node!");
			}
		}
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::Assemble_K()
{
	const int nbDOF = nbDOFPerNode * m_nbNodes;

	SQ_ASSERT(m_nbNodes > 0, "Can't assemble K matrix before defining the number of nodes in the mesh!");

	if (!simulationNeedsFullK())
	{
		return;
	}

	// Erase the stiffness matrix
	mat_null<double>(m_K);

	// Fill up the stiffness matrix by assembling all the local element by element stiffness matrices
	for (int elementIndex = 0; elementIndex < m_nbElements; ++elementIndex)
	{
		FEMElement* element = m_elements[elementIndex];
		const Static_Matrix<double,nbNodePerElement*nbDOFPerNode,nbNodePerElement*nbDOFPerNode> &Ke = element->GetGlobalStiffnessMatrix();

		for (int ptI = 0; ptI < nbNodePerElement; ++ptI)
		{
			int nodeI = element->getPointID(ptI);

			for (int ptJ = 0; ptJ < nbNodePerElement; ++ptJ)
			{
				int nodeJ = element->getPointID(ptJ);

				for (int i = 0; i < nbDOFPerNode; i++)
				{
					for (int j = 0; j < nbDOFPerNode; j++)
					{
						m_K[nbDOFPerNode*nodeI + i][nbDOFPerNode*nodeJ + j] += Ke[nbDOFPerNode*ptI + i][nbDOFPerNode*ptJ + j];
					}
				}
			}
		}
	}

	// Integrate the simple Boundary conditions (fixed node) by setting all the related coefficient to 0 (rows and columns)
	Modify_K_withBC();

	printf("FEM[%s]\n",this->getName().c_str());

	// K symmetric?
	bool symmetric = true;
	for (int line = 0; line < nbDOF; ++line)
	{
		for (int col = line+1; col < nbDOF; ++col)
		{
			if( fabs(m_K[line][col]-m_K[col][line]) > 1e-10 )
			{ 
				printf(" > K[%d][%d]=%g != K[%d][%d]=%g\n",line,col,m_K[line][col] , col,line,m_K[col][line]);
				symmetric=false; 
				break; 
			}
		}
		if (!symmetric)
		{
			break;
		}
	}
	cout << " > K is " << (!symmetric ? "not " : "") << "symmetric" << endl;

	// Band of K?
	int bandWidth = 0;
	for (int line = 0 ; line < nbDOF; ++line)
	{
		int startNonNulCol = 0;
		int endNonNulCol = 0;

		int col=0;
		while(col < nbDOF && m_K[line][col] == 0.0)
		{
			col++;
		}
		startNonNulCol = col;

		col = nbDOF - 1;
		while(col >= 0 && m_K[line][col] == 0.0)
		{
			col--;
		}
		endNonNulCol=col;

		int lineBandWidth = endNonNulCol - startNonNulCol + 1;
		if (lineBandWidth > bandWidth) 
		{
			bandWidth=lineBandWidth;
		}
	}
	cout << " > K[0-" << nbDOF - 1 << "][0-" << nbDOF - 1 << "] has a bandwidth of " << bandWidth << endl;

	// K sparse?
	int total0 = 0;
	for (int line=0; line < nbDOF; ++line)
	{
		int nb0 = 0;

		for (int col = 0; col < nbDOF; ++col)
		{
			if (m_K[line][col] == 0.0)
			{
				nb0++;
			}
		}

		total0 += nb0;
	}
	cout << " > K is full of 0 at " << 100 * total0 / double(nbDOF * nbDOF) << "%" << endl;
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::Modify_K_withBC()
{
	const int nbDOF = nbDOFPerNode * m_nbNodes;

	// Integrate the simple Boundary conditions (fixed node) by setting all the related coefficient to 0 (rows and columns)
	for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
	{
		int nodeIndex = m_BC[bcIndex];

		for (int dofIndex = 0; dofIndex < nbDOF; ++dofIndex)
		{
			for (int i = 0; i < nbDOFPerNode; ++i)
			{
				// Reset entire row and column to 0
				m_K[nbDOFPerNode*nodeIndex + i][dofIndex] = 0.0;  
				m_K[dofIndex][nbDOFPerNode*nodeIndex + i] = 0.0;
			}
		}

		// Reset diagonal to 1 (identity)
		for (int i = 0; i < nbDOFPerNode; ++i)
		{
			m_K[nbDOFPerNode*nodeIndex + i][nbDOFPerNode*nodeIndex + i] = m_BCmass;
		}
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::prepare_BackwardEuler_Matrices()
{
	// Compute the internal matrix
	computeLHS_BackwardEuler(m_dt, m_Ut.getPointer(), m_Vt.getPointer());
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::prepare_BackwardEuler_directSolver()
{
	const int nbDOF = nbDOFPerNode * m_nbNodes;

	// Compute LU decomposition of MDK
	SQ_ASSERT(m_MDK.getNbElem() != 0, "Can't decompose m_MDK because it wasn't allocated!");
	SQ_ASSERT(m_K_LU.getNbElem() != 0, "Can't decompose into m_LU because it wasn't allocated!");

	mat_copy<double,double>(m_MDK, m_K_LU, nbDOF, nbDOF);

	// Set the Boundary conditions on the stiffness matrix m_K_LU (to leave m_K intact in case we need it)
	for(int bcID=0 ; bcID<m_nbBC ; ++bcID)
	{
		int nodeID = m_BC[bcID];

		for(int dof=0 ; dof<nbDOFPerNode ; dof++)
		{
			for(int i=0 ; i<nbDOFPerNode*m_nbNodes ; i++)
			{
				m_K_LU[nbDOFPerNode*nodeID+dof][i] = 0.0;
				m_K_LU[i][nbDOFPerNode*nodeID+dof] = 0.0;

				//RtK [nbDOFPerNode*nodeID+dof][i] = 0.0;
				//RtK [i][nbDOFPerNode*nodeID+dof] = 0.0;
			}
			m_K_LU[nbDOFPerNode*nodeID+dof][nbDOFPerNode*nodeID+dof] = m_BCmass;
			//RtK [nbDOFPerNode*nodeID+dof][nbDOFPerNode*nodeID+dof] = m_BCmass;
		}
	}

	directSolver_prepare();
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::solve_BackwardEuler_directSolver()
{
	// Add the RHS equations of the backward Euler scheme
	addRHS_BackwardEuler(m_dt, m_Ut.getPointer(), m_Vt.getPointer(), m_F.getPointer());

	// Make sure we won't have any displacement for the BC => null force for all the BC nodes
	for(int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
	{
		int nodeIndex = m_BC[bcIndex];

		for (int i = 0; i < nbDOFPerNode; ++i)
		{
			m_F[nbDOFPerNode*nodeIndex + i] = 0.0;
		}
	}

	for (int i=0 ; i<getNbDOF() ; i++)
	{
		if ( m_F [i]<1e-15 && m_F [i]>-1e-15 && m_F [i]!=0.0 )  m_F[i]=0.0;
	}

	directSolver_solve();

	for (int i=0 ; i<getNbDOF() ; i++)
	{
		if ( m_Ut[i]<1e-15 && m_Ut[i]>-1e-15 && m_Ut[i]!=0.0) m_Ut[i]=0.0;
	}

	// Compute the updated node positions
	if(m_updatedModel)
	{
		vec_generic_add<double, double>(m_xt.getPointer(), m_Ut.getPointer(), getNbDOF()); // In dynamic, we do not scale Ut by increment

		// Compute the new Vt
		vec_generic_scale<double,double>(m_Ut.getPointer(), m_Vt.getPointer(), getNbDOF(), 1.0/m_dt); // U(t) = x(t)-x(t-dt)
	}
	else
	{
		vec_generic_add<double, double, double>(m_x0.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), getNbDOF());
		
		// Compute the new Vt
		vec_generic_sub<double,double,double>(m_Ut.getPointer(), m_Ut_minus_dt.getPointer(), m_Vt.getPointer(), getNbDOF()); // U(t) = x(t)-x(0)
		vec_generic_scale<double,double>(m_Vt.getPointer(), getNbDOF(), 1.0/m_dt);
	}

	// Update the deformed mesh
	updateDeformedMesh();
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::solve_BackwardEuler_CG()
{
	const double epsilon = m_CG_epsilon;
	const double epsilonSQ = epsilon*epsilon;

	const int nbDOF = nbDOFPerNode * m_nbNodes;
	
	// Ut will be modified with the new Ut...we need to store it away first !
	vec_generic_copy<double,double>(m_Ut_minus_dt.getPointer(), m_Ut.getPointer(), nbDOF);

	// Add the RHS equations of the backward Euler scheme
	addRHS_BackwardEuler_CG(m_dt , m_Ut.getPointer(), m_Vt.getPointer(), m_F.getPointer());

	// We can't touch F, so let copy it 1st to U to modify some values and have the final result in it !
	vec_generic_copy<double,double>(m_FwithBC.getPointer(), m_F.getPointer(), nbDOF);

	// Make sure we won't have any displacement for the BC => null force for all the BC nodes
	for (int bcIndex = 0; bcIndex < m_nbBC; bcIndex++)
	{ 
		int nodeIndex = m_BC[bcIndex];

		for (int i = 0; i < nbDOFPerNode; ++i)
		{
			m_FwithBC[nbDOFPerNode*nodeIndex + i] = 0.0;
		}
	}

	// Initialization
	// q = (M/dt2 + D/dt + K) U
	MatVecProduct_MDK_u(m_dt , m_RayleightDamping_Mass_coef, m_RayleightDamping_Stif_coef, m_Ut.getPointer(), m_xt.getPointer(), m_x0.getPointer(), 
		m_q.getPointer());

	// r = F + (-1)q = F - Ku
	vec_generic_triadic<double,double,double>(m_FwithBC.getPointer(), m_q.getPointer(), m_r.getPointer(), -1.0, nbDOF);

	// d = r
	vec_generic_copy<double,double>(m_d.getPointer(), m_r.getPointer(), nbDOF);

	// deltaNew = r.r
	double deltaNew = vec_generic_dotProduct<double,double,double>(m_r.getPointer(), m_r.getPointer(), nbDOF);

	// delta0 = deltaNew
	double delta0 = deltaNew;

	int nbIter = 0;
	int nbIterMax = m_CG_maxIteration;
	while (nbIter < nbIterMax && deltaNew > epsilonSQ * delta0)
	{
		// q = Kd
		MatVecProduct_MDK_u(m_dt, m_RayleightDamping_Mass_coef, m_RayleightDamping_Stif_coef, m_d.getPointer(), m_xt.getPointer(), m_x0.getPointer(), 
			m_q.getPointer());

		// alpha = deltaNew/(d.q)
		double d_q = vec_generic_dotProduct<double,double,double>( m_d.getPointer() , m_q.getPointer() , nbDOF);

		if( d_q==0.0 )
		{
			cerr << "FEM::solve_BackwardEuler_CG d.q = 0 => can't divide to find alpha !" << endl;
		}

		double alpha = deltaNew / d_q;

		// U = U + alpha.d
		vec_generic_triadic<double,double,double>(m_Ut.getPointer(), m_d.getPointer(), alpha, nbDOF);

		// r = r - alpha.q
		vec_generic_triadic<double,double,double>(m_r.getPointer(), m_q.getPointer(), -alpha, nbDOF);

		// deltaOld = deltaNew
		double deltaOld = deltaNew;

		// deltaNew = r.r
		deltaNew = vec_generic_dotProduct<double,double,double>(m_r.getPointer(), m_r.getPointer(), nbDOF);

		// beta = deltaNew/deltaOld
		double beta = deltaNew / deltaOld;

		// d = r + beta.d
		vec_generic_triadic<double,double,double,double>(m_r.getPointer(), m_d.getPointer(), m_d.getPointer(), beta, nbDOF);

		nbIter++;
	}
	if (nbIter == nbIterMax)
	{
		if (m_convergenceVerbosity >= 0)
		{
			cerr << "FEM::solve_BackwardEuler_CG did not converge in " << nbIter << " iterations - Error = " << sqrt(deltaNew / delta0) << endl;
		}
	}
	else if (nbIter == 0)
	{
		if (m_convergenceVerbosity >= 1)
		{
			cout << "FEM::solve_BackwardEuler_CG converged before any iterations - (Abs) Error = " << sqrt(deltaNew) << endl;
		}
	}
	else
	{
		if (m_convergenceVerbosity >= 1)
		{
			cout << "FEM::solve_BackwardEuler_CG converged in " << nbIter << " iterations - Error = " << sqrt(deltaNew / delta0) << endl;
		}
	}

	// Compute the updated node positions
	if(m_updatedModel)
	{
		vec_generic_add<double, double>(m_xt.getPointer(), m_Ut.getPointer(), nbDOF); // In dynamic, we do not scale Ut by increment

		// Compute the new Vt
		vec_generic_scale<double,double>(m_Ut.getPointer(), m_Vt.getPointer(), nbDOF, 1.0/m_dt); // U(t) = x(t)-x(t-dt)
	}
	else
	{
		vec_generic_add<double, double, double>(m_x0.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), nbDOF);

		// Compute the new Vt
		vec_generic_sub<double,double,double>(m_Ut.getPointer(), m_Ut_minus_dt.getPointer(), m_Vt.getPointer(), nbDOF); // U(t) = x(t)-x(0)
		vec_generic_scale<double,double>(m_Vt.getPointer(), nbDOF, 1.0/m_dt);
	}

	// Update the deformed mesh
	updateDeformedMesh();
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::directSolver_prepare(void)
{
	int m=getNbDOF(), n=getNbDOF();
	int lda=getNbDOF();
	int info;
	dgetrf(&m, &n, m_K_LU.getPointer(), &lda, m_LUpermutation.getPointer(), &info);

	if (info < 0)
	{
		cerr << "MKL > LU decomposition for FEM (static) failed: parameter " << -info << " has a bad value" << endl;
	}
	else if (info > 0)
	{
		cerr << "MKL > LU decomposition for FEM (static) failed: U["<<info<<"]["<<info<<"] = 0 " << endl;
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::directSolver_solve(void)
{
	// Backup m_Ut into m_Ut_minus_dt
	vec_generic_copy(m_Ut_minus_dt.getPointer(), m_Ut.getPointer(), getNbDOF());

	// Copy m_F into m_Ut to work directly with m_Ut
	vec_generic_copy(m_Ut.getPointer(), m_F.getPointer(), getNbDOF());

	char trans='N';
	int n=getNbDOF();
	int nrhs=1;
	int lda=m_K_LU.getNbColumn();
	int ldb=m_Ut.getSize();
	int info;
	dgetrs( &trans, &n, &nrhs, m_K_LU.getPointer(), &lda, m_LUpermutation.getPointer(), m_Ut.getPointer(), &ldb, &info );

	if(info<0)
	{
		cerr << "MKL > LU decomposition for FEM (BE) failed (FEM::solve_BackwardEuler_LU): parameter " << -info << " has a bad value" << endl;
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::directSolver_computeCompliance(void)
{
	// NOTE: directSolver_prepare() MUST HAVE BEEN CALLED BEFORE
	mat_copy<double,double>(m_K_LU, m_C , getNbDOF(), getNbDOF());

	// Compute the system inverse matrix using the LU decomposition
	{
		int n = getNbDOF();
		int lda = m_C.getNbColumn();
		int info;
		int lwork;

		// This first call to dgetri() just asks for the "ideal" size of the work array.
		// the ideal size is returned in idelaLwork
		double idealLwork = 0;
		lwork = -1;
		dgetri(&n, m_C.getPointer(), &lda, m_LUpermutation.getPointer(), &idealLwork, &lwork, &info);

		lwork = static_cast<int>(idealLwork);
		Dynamic_Vector<double> work(lwork);
		dgetri(&n, m_C.getPointer(), &lda, m_LUpermutation.getPointer(), work.getPointer() , &lwork , &info);

		if(info < 0)
		{
			cerr << "MKL > LU-based inversion for FEM (BE) compliance matrix failed: parameter " << -info << " has a bad value" << endl;
		}
		else if (info > 0)
		{
			cerr << "MKL > LU-based inversion for FEM (BE) compliance matrix failed: U["<<info<<"]["<<info<<"] = 0 " << endl;
		}
	}

	// Set the BC compliance diagonal value to what we want them to be...more likely 0
	for(int i=0 ; i<getNbBC() ; i++)
	{
		int nodeID = getBC(i);
		for(int DOF=0 ; DOF<nbDOFPerNode ; DOF++)
		{
			for(int line=0 ; line<getNbDOF() ; line++)
			{
				m_C[line][nbDOFPerNode*nodeID+DOF] = 0.0;
				m_C[nbDOFPerNode*nodeID+DOF][line] = 0.0;
			}
			m_C[nbDOFPerNode*nodeID+DOF][nbDOFPerNode*nodeID+DOF] = m_BCinvMass;
		}
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::prepare_Static_directSolver()
{
	const int nbDOF = getNbDOF();

	// Compute LU decomposition of K.
	// We do this on each and every call; it's the caller's responsibility not to call this method if it's not needed!
	SQ_ASSERT(m_K.getNbElem(), "Can't decompose m_K because it wasn't allocated!");
	SQ_ASSERT(m_K_LU.getNbElem(), "Can't decompose into m_K_LU because it wasn't allocated!");

	mat_copy<double,double>(m_K, m_K_LU, nbDOF, nbDOF);
	
	// Set the Boundary conditions on the stiffness matrix m_K_LU (to leave m_K intact in case we need it)
	for(int bcID=0 ; bcID<m_nbBC ; ++bcID)
	{
		int nodeID = m_BC[bcID];

		for(int dof=0 ; dof<nbDOFPerNode ; dof++)
		{
			for(int i=0 ; i<nbDOFPerNode*m_nbNodes ; i++)
			{
				m_K_LU[nbDOFPerNode*nodeID+dof][i] = 0.0;
				m_K_LU[i][nbDOFPerNode*nodeID+dof] = 0.0;

				//RtK [nbDOFPerNode*nodeID+dof][i] = 0.0;
				//RtK [i][nbDOFPerNode*nodeID+dof] = 0.0;
			}
			m_K_LU[nbDOFPerNode*nodeID+dof][nbDOFPerNode*nodeID+dof] = m_BCmass;
			//RtK [nbDOFPerNode*nodeID+dof][nbDOFPerNode*nodeID+dof] = m_BCmass;
		}
	}

	directSolver_prepare();
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::solve_Static_directSolver()
{
	// Make sure we won't have any displacement for the BC => null force for all the BC nodes
	for(int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
	{
		int nodeID = m_BC[bcIndex];
		for (int i = 0; i < nbDOFPerNode; ++i)
		{
			m_F[nbDOFPerNode*nodeID + i] = 0.0;
		}	
	}

	directSolver_solve();

	// Compute the updated node positions
	if(m_updatedModel && m_corotational)
	{
		// TRICK: If we are in co-rotational and updated model, we apply only a part of the displacement
		vec_generic_triadic<double,double,double>(m_xt.getPointer(), m_Ut.getPointer(), m_staticCorot_DispScaleFactor, getNbDOF());
	}
	else
	{
		vec_generic_add<double, double, double>(m_x0.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), getNbDOF());
	}

	// Update the deformed mesh
	updateDeformedMesh();
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::solve_StaticCG(bool useGlobalMatrix)
{
	const double epsilonSQ = m_CG_epsilon * m_CG_epsilon;

	const int nbDOF = nbDOFPerNode * m_nbNodes;

	double deltaNew;
	double delta0;
	
	// We can't touch F, so copy it to FwithBC to modify some values
	vec_generic_copy<double,double>(m_FwithBC.getPointer(), m_F.getPointer(), nbDOF);

	// Make sure we won't have any displacement for the Boundary Conditions => null force for all the BC nodes
	for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
	{
		int nodeIndex = m_BC[bcIndex];
		for (int i = 0; i < 3; ++i)
		{
			m_FwithBC[3*nodeIndex + i] = 0.0;
		}
	}

	// Initialization
	// q = KU
	MatVecProduct_K_u(m_Ut.getPointer(), m_xt.getPointer(), m_x0.getPointer(), m_q.getPointer(), useGlobalMatrix);
	// r = F + (-1)q = F - Ku
	vec_generic_triadic<double,double,double>(m_FwithBC.getPointer(), m_q.getPointer(), m_r.getPointer(), -1.0, nbDOF);

	// d = r
	vec_generic_copy<double,double>(m_d.getPointer(), m_r.getPointer(), nbDOF);

	// deltaNew = r.r
	deltaNew = vec_generic_dotProduct<double,double,double>(m_r.getPointer(), m_r.getPointer(), nbDOF);

	// delta0 = deltaNew
	delta0 = deltaNew;

	int nbIterations = 0;
	while (nbIterations < m_CG_maxIteration && deltaNew > epsilonSQ * delta0)
	{
		// q = Kd
		MatVecProduct_K_u(m_d.getPointer(), m_xt.getPointer(), m_x0.getPointer(), m_q.getPointer(), useGlobalMatrix);

		// alpha = deltaNew/(d.q)
		double d_q = vec_generic_dotProduct<double,double,double>(m_d.getPointer(), m_q.getPointer(), nbDOF);
		if( d_q == 0.0 )
		{
			cerr << "FEM::solve_StaticCG d.q = 0 => can't divide to find alpha !" << endl;
		}

		double alpha = deltaNew / d_q;

		// U = U + alpha.d
		vec_generic_triadic<double,double,double>(m_Ut.getPointer(), m_d.getPointer(), alpha, nbDOF);

		// r = r - alpha.q
		vec_generic_triadic<double,double,double>(m_r.getPointer(), m_q.getPointer(), -alpha, nbDOF);

		// deltaOld = deltaNew
		double deltaOld = deltaNew;

		// deltaNew = r.r
		deltaNew = vec_generic_dotProduct<double,double,double>(m_r.getPointer(), m_r.getPointer(), nbDOF);

		double beta = deltaNew / deltaOld;
		
		// d = r + beta.d
		vec_generic_triadic<double,double,double,double>(m_r.getPointer(), m_d.getPointer(), m_d.getPointer(), beta, nbDOF);

		nbIterations++;
	}

	if (nbIterations == m_CG_maxIteration)
	{
		if (m_convergenceVerbosity >= 0)
			cerr << "FEM::solve_StaticCG did not converge in " << nbIterations << " iterations - Error = " << sqrt(deltaNew/delta0) << endl;
	}
	else if (nbIterations == 0)
	{
		if (m_convergenceVerbosity >= 1)
			cout << "FEM::solve_StaticCG converged before any iterations - (Abs) Error = " << sqrt(deltaNew) << endl;
	}
	else
	{
		if (m_convergenceVerbosity >= 1)
			cout << "FEM::solve_StaticCG converged in " << nbIterations << " iterations - Error = " << sqrt(deltaNew / delta0) << endl;
	}

	// Compute the updated node positions
	if(m_updatedModel && m_corotational)
	{
		// TRICK: If we are in co-rotational and updated model, we apply only a part of the displacement
		vec_generic_triadic<double,double,double>(m_xt.getPointer(), m_Ut.getPointer(), m_staticCorot_DispScaleFactor, nbDOF);
	}
	else
	{
		vec_generic_add<double, double, double>(m_x0.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), nbDOF);
	}

	// Update the deformed mesh
	updateDeformedMesh();
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::prepareSolver()
{
	if (simulationIsStatic())
	{
		if (simulationUsesPrecomputedDecomposition() || simulationComputesComplianceMatrix())
		{
			prepare_Static_directSolver(); // directSolver_prepare(); is called at the end of this method !
		}
	}
	else if (simulationIsDynamic())
	{
		prepare_BackwardEuler_Matrices();
		if (simulationUsesPrecomputedDecomposition() || simulationComputesComplianceMatrix())
		{
			prepare_BackwardEuler_directSolver(); // directSolver_prepare(); is called at the end of this method !
		}
	}

	if (simulationComputesComplianceMatrix())
	{
		// directSolver_prepare() must have been called before this...look up, it's the case no matter what !
		directSolver_computeCompliance();
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::solveOneStep()
{
	//printf("FEM1D f_global = [");
	//for(int i=0 ; i<getNbDOF() ; i++)
	//{
	//	printf(" %g", m_F[i] );
	//}
	//printf(" ]\n");

	preOneStep();

	if (simulationIsStatic())
	{
		if (simulationUsesPrecomputedDecomposition())
		{
			if (m_corotational)
			{
				// In linear case, we simply use the compliance matrix C=K^-1; C is fixed over time and therefore does not need to be recomputed.
				// But for co-rotational, we need to update things every time.
				// We suppose that Stiffness matrix has been recomputed at this point
				prepare_Static_directSolver();
				if (simulationComputesComplianceMatrix())
				{
					directSolver_computeCompliance();
				}
			}
			solve_Static_directSolver();
		}
		else if (simulationUsesConjugateGradient())
		{
			bool useGlobalMatrix = false;
			solve_StaticCG(useGlobalMatrix);
		}
	}
	else if (simulationIsDynamic())
	{
		if (simulationUsesPrecomputedDecomposition())
		{
			if(m_corotational)
			{
				// In linear case, we simply use the compliance matrix C=MDK^-1; C is fixed over time and therefore does not need to be recomputed.
				// But for co-rotational, we need to update things every time.
				// We suppose that all matrices have been updated at this point (M, K, MDK, MD)
				prepare_BackwardEuler_directSolver();
				if (simulationComputesComplianceMatrix())
				{
					directSolver_computeCompliance();
				}
			}
			solve_BackwardEuler_directSolver();
		}
		else if (simulationUsesConjugateGradient())
		{
			bool useGlobalMatrix = false;
			solve_BackwardEuler_CG(); /// @Note: This is inconsistent in form with the static method in that not using globalMatrix (sparse like multiplication) is done by default with no flag setting
		}
	}

	postOneStep(m_Ut.getPointer(), m_Vt.getPointer());

	//printf("FEM1D U(t+dt)-U(t) = [");
	//for(int i=0 ; i<getNbDOF() ; i++)
	//{
	//	printf(" %g", m_Ut[i]-m_Ut_minus_dt[i] );
	//}
	//printf(" ]\n");
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::addConstraintDisplacement(const int offset, const double* Uc, double scale)
{
	const int nbDOF = getNbDOF();

	// Scale displacement Uc
	vec_generic_scale(Uc + offset, m_Uc_scaled.getPointer(), nbDOF, scale);

	// Zero out displacement for boundary conditions
	for (int i = 0; i < m_nbBC; i++)
	{
		for (int j = 0; j < nbDOFPerNode; j++)
		{
			m_Uc_scaled[nbDOFPerNode * m_BC[i] + j] = 0.0;
		}
	}
	
	double normSQ = vec_generic_normSQ<double,double>( m_Uc_scaled.getPointer(), nbDOF );
	if(normSQ!=0.0)
	{
		// Update the internal displacement Ut
		vec_generic_add<double,double>(m_Ut.getPointer(), m_Uc_scaled.getPointer(), nbDOF);

		// Update the position m_xt
		vec_generic_add<double, double>(m_xt.getPointer(), m_Uc_scaled.getPointer(), nbDOF);
	
		// Update the velocities
		if(m_updatedModel)
		{
			// Compute the new Vt [in updated Lagrangian, Ut = x(t)-x(t-dt)]
			vec_generic_scale<double,double>(m_Ut.getPointer(), m_Vt.getPointer(), nbDOF, 1.0/m_dt); // U(t) = x(t)-x(t-dt)
		}
		else
		{
			// Compute the new Vt [in total Lagrangian, Ut = x(t)-x(0)]
			vec_generic_sub<double,double,double>(m_Ut.getPointer(), m_Ut_minus_dt.getPointer(), m_Vt.getPointer(), nbDOF); // U(t) = x(t)-x(0)
			vec_generic_scale<double,double>(m_Vt.getPointer(), nbDOF, 1.0/m_dt);
		}

		// Update the deformed mesh
		updateDeformedMesh();
		postOneStep(m_Uc_scaled.getPointer(), m_Vt.getPointer());
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::setLastDeformedMesh(const Pt3D* points)
{
	vec_generic_copy<double,double>((double*)m_lastDeformedMesh, (double*)points, 3 * m_nbNodes);
}
template <int nbNodePerElement, int nbDOFPerNode> 
const Pt3D* Fem<nbNodePerElement, nbDOFPerNode>::getLastDeformedMesh() const
{ 
	return m_lastDeformedMesh; 
}

/*!
Ma + Dv + Ku = F
Backward Euler = { v(t+dt) = v(t) + dt*a(t+dt)
                 { u(t+dt) = x(t+dt) - x(0) = x(t) + dt*v(t+dt)
=> { a(t+dt) = ( v(t+dt) - v(t) ) / dt = ( x(t+dt) - 2x(t) + x(t-dt) ) / dt2
   { v(t+dt) = ( x(t+dt) - x(t) ) / dt
(M/dt2 + D/dt + K)u = F + D/dt x(t) + M/dt2 x(t) + M/dt (x(t)-x(t-dt))/dt
(M/dt2 + D/dt + K)u = F + (M/dt2 + D/dt) x(t) + M/dt v(t)
*/
template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::computeLHS_BackwardEuler(const double dt, const double* xt, const double* vt)
{
	const int nbDOF = nbDOFPerNode * m_nbNodes;

	SQ_ASSERT(m_nbNodes > 0, "Can't assemble any matrix before defining the number of nodes in the mesh!");

	if (! simulationNeedsFullMDx())
	{
		return;
	}

	SQ_ASSERT(m_M.getNbElem() != 0,   "Can't assemble from m_M because it wasn't allocated!");
	SQ_ASSERT(m_K.getNbElem() != 0,   "Can't assemble from m_K because it wasn't allocated!");
	SQ_ASSERT(m_MDK.getNbElem() != 0, "Can't assemble m_MDK because it wasn't allocated!");
	SQ_ASSERT(m_MD.getNbElem() != 0,  "Can't assemble m_MD because it wasn't allocated!");

	SQ_ASSERT(dt > 0, "You can't use a zero time step for Backward Euler integration!");
	double inv_dt2 = 1.0 / (dt*dt);
	double inv_dt  = 1.0 / dt;

	double MD_massCoef=(inv_dt2 + m_RayleightDamping_Mass_coef*inv_dt);
	double MD_StifCoef=(          m_RayleightDamping_Stif_coef*inv_dt);
	for (int i = 0; i < nbDOF; ++i)
	{
		for (int j = 0; j < nbDOF; ++j)
		{
			//m_MDK[i][j] = (inv_dt2 + m_RayleightDamping_Mass_coef*inv_dt) * m_M[i][j]  +  (1.0 + m_RayleightDamping_Stif_coef*inv_dt) * m_K[i][j];
			//m_MD [i][j] = (inv_dt2 + m_RayleightDamping_Mass_coef*inv_dt) * m_M[i][j]  +  (      m_RayleightDamping_Stif_coef*inv_dt) * m_K[i][j];
			m_MD[i][j]  = MD_massCoef * m_M[i][j]  +  MD_StifCoef * m_K[i][j];
		}
	}
	// MDK=MD+K
	mat_add(m_MD, m_K, m_MDK, nbDOF, nbDOF);

	// Integrate the simple Boundary conditions (fixed node) by setting all the related coefficient to 0 (rows and columns)
	for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
	{
		int nodeIndex = m_BC[bcIndex];

		for (int dof = 0; dof < nbDOF; dof++)
		{
			for (int i = 0; i < nbDOFPerNode; ++i)
			{
				m_MDK[nbDOFPerNode*nodeIndex + i][dof] = 0.0;  
				m_MDK[dof][nbDOFPerNode*nodeIndex + i] = 0.0;

				m_MD[nbDOFPerNode*nodeIndex + i][dof] = 0.0;
				m_MD[dof][nbDOFPerNode*nodeIndex + i] = 0.0;
			}
		}

		for (int i = 0; i < nbDOFPerNode; ++i)
		{
			m_MDK[nbDOFPerNode*nodeIndex + i][nbDOFPerNode*nodeIndex + i] = m_BCmass;
			m_MD[nbDOFPerNode*nodeIndex + i][nbDOFPerNode*nodeIndex + i] = m_BCmass;
		}
	}
}

//! Note that ut can be total or updated
//! the method will handle it according to the variable m_updatedModel
//! Note: In case of updated model, only vt is used, so ut does not matter
template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::addRHS_BackwardEuler(const double dt, const double* ut, const double* vt, double* F)
{
	const int nbDOF = nbDOFPerNode * m_nbNodes;

	SQ_ASSERT(m_nbNodes > 0, "Can't assemble any matrix before defining the number of nodes in the mesh!");

	if( m_updatedModel )
	{
		// Add RHS implicit forces...
		// (M/dt2 + D/dt + RKRt).[x(t+dt)-x(t)] = Fext - Fint + M/dt.v(t)

		const char trans = 'N';
		int m = nbDOF;
		int n = nbDOF;
		int lda = nbDOF;
		double alpha = 1.0/dt; // m_M/dt * v
		int incx = 1;
		int incy = 1;
		double beta = 1.0; //  1.0 * F
		// F += M*vt/dt
		dgemv(&trans, &m, &n, &alpha, m_M.getPointer(), &lda, vt, &incx, &beta, F, &incy);

	}else{
		// Add RHS implicit forces...
		// (M/dt2 + D/dt + K).[x(t+dt)-x(0)] = F + (M/dt2+D/dt).[x(t)-x(0)] + M/dt.v(t)

		const char trans = 'N';
		int m = nbDOF;
		int n = nbDOF;
		int lda = nbDOF;
		double alpha = 1.0; // 1.0 * m_MD * u
		int incx = 1;
		int incy = 1;
		double beta = 1.0; //  1.0 * F
		// F += MD*ut
		dgemv(&trans, &m, &n, &alpha, m_MD.getPointer(), &lda, ut, &incx, &beta, F, &incy);

		alpha = 1.0/dt; // m_M/dt * v
		// F += M*vt/dt
		dgemv(&trans, &m, &n, &alpha, m_M.getPointer(), &lda, vt, &incx, &beta, F, &incy);
	}
}

template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::addRHS_BackwardEuler_CG(const double dt, const double* ut, const double* vt, double* F)
{
	SQ_ASSERT(m_nbNodes > 0, "Can't assemble any matrix before defining the number of nodes in the mesh!");

	addMatVecProduct_MD_x_Mt_v(dt, m_RayleightDamping_Mass_coef, m_RayleightDamping_Stif_coef, ut, vt, m_xt.getPointer(), m_x0.getPointer(), F);
}

/*!
This should be equivalent to:
for (int i = 0; i < nbDOFPerNode*m_nbPts; i++)
	for (int j = 0; j < nbDOFPerNode*m_nbPts; j++)
		result[i] += m_K[i][j] * u[j];
*/
template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::MatVecProduct_K_u(const double* u, const double* x, const double* x0, double* result, bool useGlobalMatrix)
{
	const int nbDOF = nbDOFPerNode * m_nbNodes;

	vec_generic_null<double>(result, nbDOF);

	if (!useGlobalMatrix)
	{
		for (int elementID = 0; elementID < m_nbElements; ++elementID) 
		{
			m_elements[elementID]->MatVecProduct_K_u(u, x, x0, result);
		}
	}
	else
	{
		mat_vec_mul<double>(m_K, u, result, nbDOF, nbDOF);
	}

	// K is 0 on rows and columns of boundary conditions
	// The displacement of all boundary conditions is 0, so this handles the 0 columns.
	// Here we handle the rows, where the product will be 0.
	for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
	{ 
		int nodeID = m_BC[bcIndex]; 
		for (int i = 0; i < nbDOFPerNode; ++i)
		{
			result[nbDOFPerNode * nodeID + i] = 0.0;
		}
	}
}

/*!
This should be equivalent to:
for (int i = 0; i < nbDOFPerNode*m_nbNodes; i++)
	for (int j = 0; j < nbDOFPerNode*m_nbNodes; j++)
		result[i] += ( (1 + rayleighDampingStiffnessCoefficient/dt)*m_K[i][j] + (1/(dt*dt) + rayleighDampingMassCoefficient/_dt)*m_M[i][j] ) * u[j];
*/
template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::MatVecProduct_MDK_u(double dt, double rayleighDampingMassCoefficient, double rayleighDampingStiffnessCoefficient,
	const double* u, const double* x, const double* x0, double* result)
{
	const int nbDOF = nbDOFPerNode * m_nbNodes;

	vec_generic_null<double>(result, nbDOF);
	for(int elementID = 0; elementID < m_nbElements; ++elementID) 
	{
		m_elements[elementID]->MatVecProduct_MDK_u(dt, rayleighDampingMassCoefficient, rayleighDampingStiffnessCoefficient,
			u, x, x0, result);
	}
	
	// Global stiffness matrix K is 0 on rows and columns of boundary conditions
	// The displacement (u) of all boundary conditions is 0, so this handles the 0 columns.
	// Here we handle the rows, where the product will be 0.
	for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
	{ 
		int nodeID = m_BC[bcIndex]; 
		for (int i = 0; i < nbDOFPerNode; ++i)
		{
			result[nbDOFPerNode * nodeID + i] = 0.0;
		}
	}
}

/*!
This should be equivalent to:
for(int i=0 ; i<3*m_nbPts ; i++)
	for(int j=0 ; j<3*m_nbPts ; j++)
	{
		m_MD[i][j] = (1/(dt*dt) + m_RayleightDamping_Mass_coef/dt)*m_M[i][j] + (m_RayleightDamping_Stif_coef/dt)*m_K[i][j];
		result[i] += (m_MD[i][j] * x[j]) + (m_M[i][j]/dt * v[j]);
	}
*/
template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::addMatVecProduct_MD_x_Mt_v(double dt, double rayleighDampingMassCoefficient, double rayleighDampingStiffnessCoefficient,
	const double* ut, const double* vt, const double* x, const double* x0, double* result)
{
	SQ_ASSERT_WARNING(!m_updatedModel,"BackwardEuler Corotational is not fully implemented...should be adding M.v/dt to the external force instead of MD.x + M.v/dt");
	for (int elementID = 0 ; elementID < m_nbElements ; elementID++) 
	{
		m_elements[elementID]->addMatVecProduct_MD_x_Mt_v(dt, rayleighDampingMassCoefficient, rayleighDampingStiffnessCoefficient, 
			x0, x, ut, vt, result);
	}
}

//! Updates the mesh point positions with the current node positions (xt)
template <int nbNodePerElement, int nbDOFPerNode> 
void Fem<nbNodePerElement, nbDOFPerNode>::updateDeformedMesh()
{
	for (int nodeIndex = 0; nodeIndex < m_nbNodes; ++nodeIndex)
	{
		for (int i = 0; i < 3; ++i)
		{
			m_deformedMesh[nodeIndex][i] = m_xt[nbDOFPerNode*nodeIndex + i];
		}
	}
}