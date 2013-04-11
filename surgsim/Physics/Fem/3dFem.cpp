#include "3dFem.h"

#include <assert.h>
#include <time.h>
#include <math.h>

#include <iomanip>
#include <mkl.h>
#include <mkl_spblas.h>
#include <mkl_dss.h>

#include <SqAssert.h>
#include "Logger.h"

#define USE_SPARSE_LU_SOLVER 1

FEMTetrahedra::FEMTetrahedra(double dt, bool usesPrecomputedDecomposition, bool isDynamic, bool isUsingMLCP) :
	Fem<4,3>(dt, usesPrecomputedDecomposition, isDynamic, isUsingMLCP),
	//m_rotationPerNode(0),
	//m_rotationPerNodeQ(0),
	m_nSurfacePts(0),
	m_undeformedSurfaceMesh(0),
	m_deformedSurfaceMesh(0),
	m_lastDeformedSurfaceMesh(0),
	m_surfacePtTetIds(0),
	m_surfacePtTetWeights(0),
	m_nSurfaceTrisAllocated(16),
	m_nSurfaceTris(0),
	m_surfaceTris((SurfaceTriangle**)malloc(sizeof(SurfaceTriangle*) * 16)),
	m_surfaceTriTetIds((int*)malloc(sizeof(int) * 16)),
	m_surfaceTriTetWeights((TriangleWeights*)malloc(sizeof(TriangleWeights) * 16)),
	m_nodeDuplication(0),
	m_nbDuplicatedNodes(0),
	m_nbBTSFsAllocated(16),
	m_nbBTSFs(0),
	m_BTSFs((BarycentricTetrahedronSpringForce**) malloc(sizeof(BarycentricTetrahedronSpringForce*) * 16))
	//m_corotational(false)
{
	m_name = "FEMTetrahedra";
}

FEMTetrahedra::FEMTetrahedra(int nbPts, Pt3D* pts, double dt, bool usesPrecomputedDecomposition, bool isDynamic, bool isUsingMLCP) :
	Fem<4, 3>(nbPts, pts, dt, usesPrecomputedDecomposition, isDynamic, isUsingMLCP),
	//m_rotationPerNode(0),
	//m_rotationPerNodeQ(0),
	m_nSurfacePts(0),
	m_undeformedSurfaceMesh(0),
	m_deformedSurfaceMesh(0),
	m_lastDeformedSurfaceMesh(0),
	m_surfacePtTetIds(0)
	,m_surfacePtTetWeights(0),
	m_nSurfaceTrisAllocated(16),
	m_nSurfaceTris(0),
	m_surfaceTris((SurfaceTriangle**)malloc(sizeof(SurfaceTriangle*) * 16)),
	m_surfaceTriTetIds((int*)malloc(sizeof(int) * 16)),
	m_surfaceTriTetWeights((TriangleWeights*)malloc(sizeof(TriangleWeights) * 16)),
	m_nodeDuplication(0),
	m_nbDuplicatedNodes(0),
	m_nbBTSFsAllocated(16),
	m_nbBTSFs(0),
	m_BTSFs((BarycentricTetrahedronSpringForce**) malloc(sizeof(BarycentricTetrahedronSpringForce*) * 16))
	//m_corotational(false)
{
	m_name = "FEMTetrahedra";
}

FEMTetrahedra::FEMTetrahedra(int nbPts , Pt3D* pts, int nbSurfacePts, Pt3D* surfacePts, double dt, bool usesPrecomputedDecomposition, bool isDynamic, bool isUsingMLCP) :
	Fem<4, 3>(nbPts, pts, dt, usesPrecomputedDecomposition, isDynamic, isUsingMLCP),
	//m_rotationPerNode(0),
	//m_rotationPerNodeQ(0),
	m_nSurfacePts(nbSurfacePts),
	m_undeformedSurfaceMesh((Pt3D*)malloc(sizeof(Pt3D) * nbSurfacePts)),
	m_deformedSurfaceMesh((Pt3D*)malloc(sizeof(Pt3D) * nbSurfacePts)),
	m_lastDeformedSurfaceMesh((Pt3D*)malloc(sizeof(Pt3D) * nbSurfacePts)),
	m_surfacePtTetIds((int*)malloc(sizeof(int)*nbSurfacePts)),
	m_surfacePtTetWeights((PointWeights*)malloc(sizeof(PointWeights) * nbSurfacePts)),
	m_nSurfaceTrisAllocated(16),
	m_nSurfaceTris(0),
	m_surfaceTris((SurfaceTriangle**)malloc(sizeof(SurfaceTriangle*) * 16)),
	m_surfaceTriTetIds((int*)malloc(sizeof(int) * 16)),
	m_surfaceTriTetWeights((TriangleWeights*)malloc(sizeof(TriangleWeights) * 16)),
	m_nodeDuplication(0),
	m_nbDuplicatedNodes(0),
	m_nbBTSFsAllocated(16),
	m_nbBTSFs(0),
	m_BTSFs((BarycentricTetrahedronSpringForce**) malloc(sizeof(BarycentricTetrahedronSpringForce*) * 16))
	//m_corotational(false)
{
	m_name = "FEMTetrahedra";

	vec_generic_copy<double,double>((double*)m_undeformedSurfaceMesh, (double*)surfacePts, 3*m_nSurfacePts);
	vec_generic_copy<double,double>((double*)m_deformedSurfaceMesh, (double*)m_undeformedSurfaceMesh, 3*m_nSurfacePts);
	vec_generic_copy<double,double>((double*)m_lastDeformedSurfaceMesh, (double*)m_deformedSurfaceMesh, 3*m_nSurfacePts);

	// Initialize arrays to store per-particle barycentric weights (and index of corresponding tet)
	for (int i = 0; i < m_nSurfacePts; ++i)
	{
		m_surfacePtTetIds[i] = -1;
		for (int j = 0; j < 4; ++j)
		{
			m_surfacePtTetWeights[i][j] = 0;
		}
	}
}

FEMTetrahedra::~FEMTetrahedra()
{
	//if (m_rotationPerNode)
	//{
	//	delete [] m_rotationPerNode;
	//	m_rotationPerNode = 0;
	//}
	//if(m_rotationPerNodeQ)
	//{
	//	delete [] m_rotationPerNodeQ;
	//	m_rotationPerNodeQ = 0;
	//}
	if (m_deformedSurfaceMesh)
	{
		free(m_deformedSurfaceMesh);
		m_deformedSurfaceMesh = 0;
	}
	if (m_undeformedSurfaceMesh)
	{
		free(m_undeformedSurfaceMesh);
		m_undeformedSurfaceMesh = 0;
	}
	if (m_lastDeformedSurfaceMesh)
	{
		free(m_lastDeformedSurfaceMesh);
		m_lastDeformedSurfaceMesh = 0;
	}

	if (m_nodeDuplication)
	{
		delete [] m_nodeDuplication;
	}

	if (m_surfaceTris)
	{
		free(m_surfaceTris);
		m_surfaceTris = 0;
	}

	if (m_BTSFs)
	{
		for (int i = 0; i < m_nbBTSFs; ++i)
		{
			delete m_BTSFs[i];
		}
		free(m_BTSFs);
		m_BTSFs = 0;
	}
}

//void FEMTetrahedra::setCorotationalModel()
//{
//	m_corotational = true;
//	for (int tetID = 0; tetID < getNbTetrahedron(); tetID++)
//	{
//		const_cast<Tetrahedron*>(getTetrahedron(tetID))->setCorotational();
//	}
//};
//void FEMTetrahedra::setNonCorotationalModel()
//{
//	m_corotational = false;
//	for (int tetID = 0; tetID < getNbTetrahedron(); tetID++)
//	{
//		const_cast<Tetrahedron*>(getTetrahedron(tetID))->setNonCorotational();
//	}
//};
//bool FEMTetrahedra::isCorotationalModel()
//{
//	return m_corotational;
//};

//void FEMTetrahedra::allocateMatrixVector()
void FEMTetrahedra::allocate()
{
	const int nbDOF = 3 * m_nbNodes;

	Fem<4,3>::allocate();

	//if (simulationNeedsFullK())
	//{
	//	// Do not test if we have a co-rotational model here...as this method is called when loading the mesh...and at this stage, the co-rotational flag might not be set yet !!
	//	m_RK.resize(nbDOF, nbDOF);
	//}

	if (simulationComputesComplianceMatrix())
	{
		if (simulationUsesPrecomputedDecomposition())
		{
			m_C_withBTSF.resize(nbDOF, nbDOF);
		}
	}

	//m_rotationPerNode = new Static_Matrix<double,3,3>[m_nbNodes];
	//m_rotationPerNodeQ = new SqQuaternion<double>[m_nbNodes];
};

const Dynamic_Matrix<double>& FEMTetrahedra::getInitialComplianceMatrix()
{
	return m_C;
};

const Dynamic_Matrix<double>& FEMTetrahedra::getComplianceMatrix()
{
	if (m_C_withBTSF.getNbElem() > 1)
	{
		return m_C_withBTSF;
	}
	else if (m_C.getNbElem() > 1)
	{
		static bool printMessage = true;
		if (printMessage)
		{
			std::cerr << "WARNING: using the initial compliance matrix instead of an updated one," << std::endl << "  which is unavailable." << std::endl;
			printMessage = false;
		}
		return m_C;
	}
	else
	{
		SQ_FAILURE("the compliance matrix is not available");
	}
}

bool FEMTetrahedra::loadMesh(string filename, double E, double nu, string cutFilename, string cutEdgesFileName)
{
	printf("FEMTetrahedra: Loading mesh %s\n", filename.c_str());

	// Load cut data from file
	{
		FILE* f = fopen(filename.c_str(), "rt");

		if (!f)
		{
			cerr << "  FEMTetrahedra::loadMesh failed to open the file " << filename.c_str() << endl;
			return false;
		}

		m_name = filename;

		if (fscanf(f,"%d particles", &m_nbNodes) != 1)
		{
			cerr << "  FEMTetrahedra::loadMesh failed in reading the number of nodes" << endl;
			return false;
		}
		printf("  > %4d initial nodes\n", m_nbNodes);
		m_undeformedMesh = (Pt3D*)malloc(sizeof(Pt3D) * m_nbNodes);

		for (int node = 0; node < m_nbNodes; node++)
		{
			if (fscanf(f,"%lf %lf %lf",&m_undeformedMesh[node][0],&m_undeformedMesh[node][1],&m_undeformedMesh[node][2]) != 3)
			{
				cerr << "  FEMTetrahedra::loadMesh failed in reading the node number " << node << endl;
				return false;
			}
			printf("  >> Nodes 0..%04d initialized\r",node);
		}
		printf("\n");
		m_deformedMesh = (Pt3D*)realloc((void*)m_deformedMesh, sizeof(Pt3D) * m_nbNodes);
		vec_generic_copy<double,double>((double*)m_deformedMesh, (double*)m_undeformedMesh, 3 * m_nbNodes);

		m_lastDeformedMesh = (Pt3D*)realloc((void*)m_lastDeformedMesh, sizeof(Pt3D) * m_nbNodes);
		vec_generic_copy<double,double>((double*)m_lastDeformedMesh, (double*)m_undeformedMesh, 3 * m_nbNodes);

		m_isUndeformedMeshOwned = true;

		if (fscanf(f,"%d tetrahedra", &m_nbElements) != 1)
		{
			cerr << "  FEMTetrahedra::loadMesh failed in reading the number of tetrahedron" << endl;
			return false;
		}
		printf("  > %4d tetrahedra\n", m_nbElements);
		m_nbElementsAllocated = m_nbElements;
		m_elements = (FEMElement**)realloc((void*)m_elements, sizeof(FEMElement*) * m_nbElements);

		for (int tetIndex = 0; tetIndex < m_nbElements; tetIndex++)
		{
			int i, j, k, l;
			if (fscanf(f, "%d %d %d %d", &i, &j, &k, &l) != 4) // Be careful here, PhysBAM index start at 1 !!
			{
				cerr << "  FEMTetrahedra::loadMesh failed in reading the tetrahedron number " << tetIndex << endl;
				return false;
			}
			Tetrahedron* tet = new Tetrahedron();
			tet->setID(tetIndex); // Set the global ID
			tet->setMechanicalParameters_E_nu(E,nu); // Need to be set first !
			tet->setPoints(m_undeformedMesh , i-1 , j-1 , k-1 , l-1); // This call needs the E and nu to compute Ke
			m_elements[tetIndex] = tet;
			printf("  >> Tetrahedra 0..%04d initialized\r", tetIndex);
		}
		m_areElementsOwned=true; // The instance owns the tets themselves...will need to release them on exit !
		printf("\n");

		if (fscanf(f, "%d bc", &m_nbBC) != 1)
		{
			cerr << "  FEMTetrahedra::loadMesh failed in reading the number of boundary conditions" << endl;
			return false;
		}
		printf("  > %4d boundary conditions\n", m_nbBC);
		m_nbBCAllocated = m_nbBC;
		m_BC = (int*)realloc((void*)m_BC, sizeof(int)*m_nbBC);

		for (int bc = 0; bc < m_nbBC; bc++)
		{
			int i;
			if (fscanf(f, "%d", &i) != 1) // Be careful here, PhysBAM index start at 1 !!
			{
				cerr << "  FEMTetrahedra::loadMesh failed in reading the boundary condition " << bc << endl;
				return false;
			}
			m_BC[bc] = i-1;
			printf("  >> Boundary conditions 0..%04d initialized\r", bc);
		}
		printf("\n");

		fclose(f);
	}

	removeUnusedMeshNodes();

	if (cutFilename.compare(string("")) != 0)
	{
		applyCut(cutFilename); // Change the nodes structure and update the tets
	}

	if (cutEdgesFileName.compare(string("")) != 0)
	{
		loadCutEdges(cutEdgesFileName); // Cut edges are useful for smoothing normals (rendering and contact response)
	}

	initializeAfterLoad();

	printf("> Mesh successfully loaded !\n");

	return true;
}

void FEMTetrahedra::addTetrahedron(Tetrahedron* tet)
{
	addElement(tet);
}
int FEMTetrahedra::getNbTetrahedron() const
{
	return m_nbElements;
}
const Tetrahedron* FEMTetrahedra::getTetrahedron(int i) const
{
	return static_cast<const Tetrahedron*>(m_elements[i]);
}
int FEMTetrahedra::findTetrahedronWithNodes(int idA, int idB, int idC, int idD) const
{
	for (int t = 0; t < m_nbElements; ++t)
	{
		const Tetrahedron* tet = static_cast<const Tetrahedron*>(m_elements[t]);
		if (tet->containsPointID(idA) && ((idB == -1) || tet->containsPointID(idB)) &&
		    ((idC == -1) || tet->containsPointID(idC)) && ((idD == -1) || tet->containsPointID(idD)))
		{
			return t;
		}
	}
	return -1;
}

int FEMTetrahedra::getNbBarycentricTetrahedronSprings() const
{
	return m_nbBTSFs;
}
const BarycentricTetrahedronSpringForce* FEMTetrahedra::getBarycentricTetrahedronSpring(int i) const
{
	return m_BTSFs[i];
}

void FEMTetrahedra::addSurfaceTriangle(const int ptIds[3], const int tetId, const double tetWeights[3][4])
{
	// Reallocate triangles if necessary.
	if (m_nSurfaceTrisAllocated == m_nSurfaceTris)
	{
		m_nSurfaceTrisAllocated *= 2;
		m_surfaceTris = (SurfaceTriangle**)realloc((void*)m_surfaceTris, sizeof(SurfaceTriangle*) * m_nSurfaceTrisAllocated);

		m_surfaceTriTetIds = (int*)realloc((void*)m_surfaceTriTetIds, sizeof(int) * m_nSurfaceTrisAllocated);

		m_surfaceTriTetWeights = (TriangleWeights*)realloc((void*)m_surfaceTriTetWeights, sizeof(TriangleWeights) * m_nSurfaceTrisAllocated);
	}

	SurfaceTriangle* triangle = new SurfaceTriangle(ptIds[0], ptIds[1], ptIds[2]);
	triangle->setID(m_nSurfaceTris);
	m_surfaceTris[m_nSurfaceTris] = triangle;

	// Set triangle tet ID and weights. These should only be set once.
	m_surfaceTriTetIds[m_nSurfaceTris] = tetId;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			m_surfaceTriTetWeights[m_nSurfaceTris][i][j] = tetWeights[i][j];
		}
	}

	// Set point tet IDs and weights. These can be set for a single point multiple times, it just uses the last.
	for (int i = 0; i < 3; i++)
	{
		m_surfacePtTetIds[ptIds[i]] = tetId;

		for (int j = 0; j < 4; j++)
		{
			m_surfacePtTetWeights[ptIds[i]][j] = tetWeights[i][j];
		}
	}

	// Set the initial surface mesh positions
	const Tetrahedron* tet = getTetrahedron(tetId);
	for (int i = 0; i < 3; i++)
	{
		tet->calculateEmbeddedLocation(m_undeformedMesh, tetWeights[i], m_undeformedSurfaceMesh[ptIds[i]]);
		m_deformedSurfaceMesh[ptIds[i]]     = m_undeformedSurfaceMesh[ptIds[i]];
		m_lastDeformedSurfaceMesh[ptIds[i]] = m_undeformedSurfaceMesh[ptIds[i]];
	}

	m_nSurfaceTris++;
}

int FEMTetrahedra::getNbSurfaceTriangles() const
{
	return m_nSurfaceTris;
}

int FEMTetrahedra::getNbSurfacePoints() const
{
	return m_nSurfacePts;
}

void FEMTetrahedra::getSurfaceTriangle(const int triID, int ptID[3]) const
{
	SurfaceTriangle* triangle = m_surfaceTris[triID];

	for (int i = 0; i < 3; i++)
	{
		ptID[i] = triangle->pointIds[i];
	}
}

void FEMTetrahedra::getSurfaceTriangleTetWeights(const int triID, int& tetID, double tetWeights[3][4]) const
{
	tetID = m_surfaceTriTetIds[triID];

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			tetWeights[i][j] = m_surfaceTriTetWeights[triID][i][j];
		}
	}
}

void FEMTetrahedra::getSurfacePointTetWeights(const int ptID, int& tetID, double tetWeights[4]) const
{
	tetID = m_surfacePtTetIds[ptID];

	for (int j = 0; j < 4; j++)
	{
		tetWeights[j] = m_surfacePtTetWeights[ptID][j];
	}
}


int FEMTetrahedra::getTetContainingPoint(const Pt3D* mesh, const Pt3D point, double tetBaryCoord[4]) const
{
	for (int i = 0; i < m_nbElements; i++)
	{
		const Tetrahedron* tet = static_cast<const Tetrahedron*>(m_elements[i]);
		tet->calculateBarycentricCoordinates(mesh, point, tetBaryCoord);

		bool valid = true;
		for (int j = 0; j < 4; j++)
		{
			if (tetBaryCoord[j] < 0)
			{
				valid = false;
				break;
			}
		}

		if (valid)
		{
			return i;
		}
	}

	return -1; // did not find a tet
}

double FEMTetrahedra::computeVolume() const
{
	double volume=0;
	for (int tetID = 0; tetID < getNbTetrahedron(); tetID++)
	{
		volume += getTetrahedron(tetID)->Volume(getDeformedMesh());
	}
	return volume;
}


int FEMTetrahedra::addBarycentricTetrahedronSpring(BarycentricTetrahedronSpringForce* btsf)
{
	if (m_nbBTSFsAllocated == m_nbBTSFs)
	{
		m_nbBTSFsAllocated *= 2;
		m_BTSFs = (BarycentricTetrahedronSpringForce**) realloc((void*)m_BTSFs, sizeof(BarycentricTetrahedronSpringForce*) * m_nbBTSFsAllocated);
	}
	int slot = m_nbBTSFs;
	m_nbBTSFs++;
	m_BTSFs[slot] = btsf;
	return slot;
}

int FEMTetrahedra::addBarycentricTetrahedronSpring(int tetIndex, const double* weights, double stiffness, const Pt3D& anchor, int externalBtscIndex)
{
	// XXX TODO: eliminate constructing and deleting the individual BTSFs!
	return addBarycentricTetrahedronSpring(new BarycentricTetrahedronSpringForce(*static_cast<Tetrahedron*>(m_elements[tetIndex]), weights, stiffness, anchor, externalBtscIndex));
}

int FEMTetrahedra::addBarycentricTetrahedronSpring(const int* ptIds, const double* weights, double stiffness, const Pt3D& anchor, int externalBtscIndex)
{
	Tetrahedron temporaryTet(m_undeformedMesh, ptIds[0], ptIds[1], ptIds[2], ptIds[3]);
	// XXX TODO: eliminate constructing and deleting the individual BTSFs!
	return addBarycentricTetrahedronSpring(new BarycentricTetrahedronSpringForce(temporaryTet, weights, stiffness, anchor, externalBtscIndex));
}

void FEMTetrahedra::clearBarycentricTetrahedronSprings()
{
	// XXX TODO: eliminate constructing and deleting the individual BTSFs!
	for (int i = 0;  i < m_nbBTSFs;  ++i)
	{
		delete m_BTSFs[i];
	}
	m_nbBTSFs = 0;
}

void FEMTetrahedra::Assemble_K()
{
	const int nbDOF = 3 * m_nbNodes;

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
		const Static_Matrix<double,12,12>& Ke = element->GetGlobalStiffnessMatrix();

		for (int ptI = 0; ptI < 4; ++ptI)
		{
			int nodeI = element->getPointID(ptI);

			for (int ptJ = 0; ptJ < 4; ++ptJ)
			{
				int nodeJ = element->getPointID(ptJ);

				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						m_K[3*nodeI + i][3*nodeJ + j] += Ke[3*ptI + i][3*ptJ + j];
					}
				}
			}
		}
	}

	// Integrate the simple Boundary conditions (fixed node) by setting all the related coefficient to 0 (rows and columns)
	Modify_K_withBC();

	// K symmetric?
	bool symmetric = true;
	for (int line = 0; line < nbDOF; ++line)
	{
		for (int col = line+1; col < nbDOF; ++col)
		{
			if (m_K[line][col] != m_K[col][line])
			{
				symmetric=false;
				break;
			}
		}
		if (!symmetric)
		{
			break;
		}
	}
	cout << "K is " << (!symmetric ? "not " : "") << "symmetric" << endl;

	// Band of K?
	int bandWidth = 0;
	for (int line = 0 ; line < nbDOF; ++line)
	{
		int startNonNulCol = 0;
		int endNonNulCol = 0;

		int col=0;
		while (col < nbDOF && m_K[line][col] == 0.0)
		{
			col++;
		}
		startNonNulCol = col;

		col = nbDOF - 1;
		while (col >= 0 && m_K[line][col] == 0.0)
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
	cout << "BAND ? K[0-" << nbDOF - 1 << "][0-" << nbDOF - 1 << "] has a bandwidth of " << bandWidth << endl;

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
	cout << "SPARSE ? K is full of 0 at " << 100 * total0 / double(nbDOF * nbDOF) << "%" << endl;
}

void FEMTetrahedra::Assemble_K_corotational()
{
	SQ_ASSERT(m_nbNodes > 0, "Can't assemble K matrix before defining the number of nodes in the mesh!");

	// Erase the stiffness matrix
	mat_null<double>(m_K);
	mat_null<double>(m_RK);

	//int* nbTetsPerNode = new int[m_nbNodes];
	//memset(nbTetsPerNode, 0, m_nbNodes*sizeof(int));

	// Fill up the stiffness matrix by assembling all the local element by element stiffness matrices
	for (int tetIndex = 0; tetIndex < m_nbElements; ++tetIndex)
	{
		const Tetrahedron* tet = static_cast<Tetrahedron*>(m_elements[tetIndex]);
		const Static_Matrix<double,12,12>& RKeGlobalRt = tet->GetGlobalStiffnessMatrix();
		const Static_Matrix<double,12,12>& RKeGlobal   = tet->Get_RKeGlobal();

		const int index[] = { tet->getPointID(0) , tet->getPointID(1) , tet->getPointID(2) , tet->getPointID(3) };

		for (int ptI = 0; ptI < 4; ++ptI)
		{
			const int nodeI = index[ptI];

			for (int ptJ=0; ptJ < 4; ++ptJ)
			{
				const int nodeJ = index[ptJ];

				for (int i = 0; i < 3; ++i)
				{
					for (int j = 0; j < 3; ++j)
					{
						m_K[3*nodeI + i][3*nodeJ + j] += RKeGlobalRt[3*ptI + i][3*ptJ + j];

						m_RK[3*nodeI + i][3*nodeJ + j] += RKeGlobal[3*ptI + i][3*ptJ + j];
					}
				}
			}
		}
	}
}

void FEMTetrahedra::computePerNodeCoRotation(void)
{
	const bool debugThisFnc = false;

	SQ_ASSERT(m_nbNodes > 0, "Can't compute node co-rotation before defining the number of nodes in the mesh!");

	// Erase the per-node rotation matrices
	for (int nodeIndex = 0; nodeIndex < m_nbNodes; ++nodeIndex)
	{
		m_rotationPerNodeQ[nodeIndex].setNull();
	}

	int* nbTetsPerNode = new int[m_nbNodes];
	memset(nbTetsPerNode, 0, m_nbNodes*sizeof(int));

	// Assemble rotations per-node to express the Compliance matrix (Compliance warping)
	for (int tetIndex = 0; tetIndex < m_nbElements; tetIndex++)
	{
		const Tetrahedron* tet = static_cast<Tetrahedron*>(m_elements[tetIndex]);
		const SqQuaternion<double>& tetQ = tet->getCorotationalQuaternion();
		const int nodeIndex[] = { tet->getPointID(0) , tet->getPointID(1) , tet->getPointID(2) , tet->getPointID(3) };
		for (int ptIndex = 0; ptIndex<4; ++ptIndex)
		{
			m_rotationPerNodeQ[nodeIndex[ptIndex]] += tetQ; // Note: technically an SLERP would be more correct
			++nbTetsPerNode[nodeIndex[ptIndex]];
		}
	}
	// Averaging & Normalizing the quaternions & Set the rotation matrices (easier to access than the quaternions)
	for (int nodeIndex = 0; nodeIndex < m_nbNodes; ++nodeIndex)
	{
		if (nbTetsPerNode[nodeIndex] > 0)
		{
			m_rotationPerNodeQ[nodeIndex] /= static_cast<double>(nbTetsPerNode[nodeIndex]);
			m_rotationPerNodeQ[nodeIndex].normalize(); // do not normalize directly
			SqMatrix33<double> rotationPerNode(m_rotationPerNodeQ[nodeIndex]);
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					m_rotationPerNode[nodeIndex][i][j] = rotationPerNode(i, j);
				}
			}

		}
	}
	if (debugThisFnc)
	{
		for (int nodeIndex = 0; nodeIndex < m_nbNodes; ++nodeIndex)
		{
			vector<int> nodeToTet;
			for (int tetIndex = 0; tetIndex < m_nbElements; ++tetIndex)
			{
				const Tetrahedron* tet = static_cast<Tetrahedron*>(m_elements[tetIndex]);
				const int index[] = { tet->getPointID(0) , tet->getPointID(1) , tet->getPointID(2) , tet->getPointID(3) };
				if (index[0] == nodeIndex || index[1] == nodeIndex || index[2] == nodeIndex || index[3] == nodeIndex)
				{
					nodeToTet.push_back(tetIndex);
				}
			}
			cout << setprecision(3) << fixed << "Node # " << nodeIndex << " has axis \t" << m_rotationPerNodeQ[nodeIndex].getAxis() <<
			     " and angle \t" << m_rotationPerNodeQ[nodeIndex].getAngle()/M_PI*180 << char(248) <<
			     " from tets with rotations:" << endl;
			for (vector<int>::iterator it = nodeToTet.begin(); it != nodeToTet.end(); ++it)
			{
				cout << "\t\t\t" << m_elements[*it]->getCorotationalQuaternion().getAxis() <<
				     " and angle \t" << m_elements[*it]->getCorotationalQuaternion().getAngle()/M_PI*180 << char(248) << endl;
			}
			cout << endl;
		}
		cout << endl << endl;
	}
}

void FEMTetrahedra::addRHS_BackwardEuler(const double dt , const double* xt, const double* vt, double* F)
{
	const int nbDOF = 3 * m_nbNodes;
	SQ_ASSERT(m_nbNodes > 0, "Can't assemble any matrix before defining the number of nodes in the mesh!");

	if (!m_corotational)
	{
		const char trans = 'N';
		int m = nbDOF;
		int n = nbDOF;
		int lda = nbDOF;
		double alpha = 1.0; // 1.0 * m_MD * x
		int incx = 1;
		int incy = 1;
		double beta = 1.0; //  1.0 * F
		// F += MD*xt
		dgemv(&trans, &m, &n, &alpha, m_MD.getPointer(), &lda, xt, &incx, &beta, F, &incy);

		alpha = 1.0/dt; // m_M/dt * v
		// F += M*vt/dt
		dgemv(&trans, &m, &n, &alpha, m_M.getPointer(), &lda, vt, &incx, &beta, F, &incy);
	}
	else
	{
		const char trans = 'N';
		int m = nbDOF;
		int n = nbDOF;
		int lda = nbDOF;
		double alpha = 1.0 / dt; // m_M/dt * v(t)
		int incx = 1;
		int incy = 1;
		double beta = 1.0; //  1.0 * F

		// F += M*vt/dt
		dgemv(&trans, &m, &n, &alpha, m_M.getPointer(), &lda, vt, &incx, &beta, F, &incy);
	}
}

/*!
This should be equivalent to:
for (int i = 0; i < nbDOFPerNode*m_nbPts; i++)
	for (int j = 0; j < nbDOFPerNode*m_nbPts; j++)
		result[i] += m_K[i][j] * u[j];
*/
void FEMTetrahedra::MatVecProduct_K_u(const double* u, const double* x, const double* x0, double* result, bool useGlobalMatrix)
{
	const int nbDOF = 3 * m_nbNodes;

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

	for (int btsfIndex = 0; btsfIndex < m_nbBTSFs; btsfIndex++)
	{
		m_BTSFs[btsfIndex]->add_to_K_u(u, result);
	}

	// K is 0 on rows and columns of boundary conditions
	// The displacement of all boundary conditions is 0, so this handles the 0 columns.
	// Here we handle the rows, where the product will be 0.
	for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
	{
		int nodeID = m_BC[bcIndex];
		for (int i = 0; i < 3; ++i)
		{
			result[3 * nodeID + i] = 0.0;
		}
	}
};

/*!
This should be equivalent to:
for (int i = 0; i < nbDOFPerNode*m_nbNodes; i++)
	for (int j = 0; j < nbDOFPerNode*m_nbNodes; j++)
		result[i] += ( (1 + rayleighDampingStiffnessCoefficient/dt)*m_K[i][j] + (1/(dt*dt) + rayleighDampingMassCoefficient/_dt)*m_M[i][j] ) * u[j];
*/
void FEMTetrahedra::MatVecProduct_MDK_u(double dt, double rayleighDampingMassCoefficient, double rayleighDampingStiffnessCoefficient,
                                        const double* u, const double* x, const double* x0, double* result)
{
	vec_generic_null<double>(result, 3 * m_nbNodes);
	for (int elementID = 0; elementID < m_nbElements; ++elementID)
	{
		m_elements[elementID]->MatVecProduct_MDK_u(dt, rayleighDampingMassCoefficient, rayleighDampingStiffnessCoefficient,
		                                           u, x, x0, result);
	}

	for (int btsfIndex = 0; btsfIndex < m_nbBTSFs; btsfIndex++)
	{
		m_BTSFs[btsfIndex]->add_to_MDK_u(dt, rayleighDampingMassCoefficient, rayleighDampingStiffnessCoefficient, u, result);
	}

	// Global stiffness matrix K is 0 on rows and columns of boundary conditions
	// The displacement (u) of all boundary conditions is 0, so this handles the 0 columns.
	// Here we handle the rows, where the product will be 0.
	for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
	{
		int nodeID = m_BC[bcIndex];
		for (int i = 0; i < 3; ++i)
		{
			result[3 * nodeID + i] = 0.0;
		}
	}
};

void FEMTetrahedra::addBTSF_LHS(Dynamic_Matrix<double>& Kinv) // K in static, MDK in dynamic
{
	const int nbDOF = 3 * m_nbNodes;

	for (int btsfIndex = 0; btsfIndex < m_nbBTSFs; ++btsfIndex)
	{
		double u[4];
		double v[4];
		int ptID[4];
		m_BTSFs[btsfIndex]->get_ShermanMorrison_uv(u, v, ptID);

		// Update Kinv
		// Sherman-Morrison scheme:
		// (A + u.v^T)^-1 = A^-1 - (A^-1.u.v^T.A^-1)/(1 + v^T.A^-1.u)
		// We apply it on a per axis basis
		static Dynamic_Vector<double> Kinv_u[3];  // For each axis
		static Dynamic_Vector<double> v_Kinv[3];  // For each axis
		double v_Kinv_u[3];                       // For each axis

		for (int i = 0; i < 3; ++i)
		{
			Kinv_u[i].resizeUpIfNecessary(nbDOF);
		}

		for (int dof = 0; dof < nbDOF; ++dof)
		{
			for (int i = 0; i < 3; ++i)
			{
				Kinv_u[i][dof] = 0.0;

				v_Kinv[i][dof] = 0.0;

				for (int ptIndex = 0; ptIndex < 4; ++ptIndex)
				{
					Kinv_u[i][dof] += Kinv[dof][3*ptID[0] + i] * u[ptIndex];

					v_Kinv[i][dof] += Kinv[3*ptID[0] + i][dof] * v[ptIndex];
				}
			}
		}

		for (int i = 0; i < 3; ++i)
		{
			v_Kinv_u[i] = v[0]*Kinv_u[i][3*ptID[0] + i] + v[1]*Kinv_u[i][3*ptID[1] + i] + v[2]*Kinv_u[i][3*ptID[2] + i] + v[3]*Kinv_u[i][3*ptID[3] + i];
		}

		double denominator[3] = { 1.0/(1.0 + v_Kinv_u[0]), 1.0/(1.0 + v_Kinv_u[1]), 1.0/(1.0 + v_Kinv_u[2]) };

		for (int row = 0; row < nbDOF; ++row)
		{
			for (int column = 0; column < nbDOF; ++column)
			{
				for (int i = 0; i < 3; ++i)
				{
					Kinv[row][column] -= Kinv_u[i][row]*v_Kinv[i][column] * denominator[i];
				}
			}
		}
	}
}

// Solve (M/dt + D/dt + K).u = F + + (M/dt2 + D/dt) x(t) + M/dt v(t) using LU
void FEMTetrahedra::solve_BackwardEuler_directSolver()
{
	SQ_ASSERT(m_C.getNbElem() != 0, "Can't extend m_C because it wasn't allocated!");

	const int nbDOF = 3 * m_nbNodes;

	if (!m_corotational)
	{
		// Ut will be modified with the new Ut...we need to store it away first !
		vec_generic_copy<double,double>(m_Ut_minus_dt.getPointer(), m_Ut.getPointer(), nbDOF);

		// Add the RHS equations of the backward Euler scheme
		addRHS_BackwardEuler(m_dt, m_Ut.getPointer(), m_Vt.getPointer(), m_F.getPointer());

		// Add the Barycentric Spring Forces in the LHS (the mesh point anchor)
		mat_copy<double,double>(m_C, m_C_withBTSF, nbDOF, nbDOF);
		addBTSF_LHS(m_C_withBTSF);

		// We can't touch F, so let copy it 1st to U to modify some values and have the final result in it !
		vec_generic_copy<double,double>(m_Ut.getPointer(), m_F.getPointer(), nbDOF);

		// Make sure we won't have any displacement for the BC => null force for all the BC nodes
		for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
		{
			int nodeIndex = m_BC[bcIndex];
			for (int i = 0; i < 3; ++i)
			{
				m_Ut[3*nodeIndex + i] = 0.0;
			}
		}

		static Dynamic_Vector<double> Utmp;
		Utmp.resizeUpIfNecessary(m_Ut.getSize());

		mat_vec_mul<double>(m_C_withBTSF, m_Ut.getPointer(), Utmp.getPointer(), nbDOF, nbDOF);
		vec_generic_copy<double,double>(m_Ut.getPointer(), Utmp.getPointer(), nbDOF);
	}
	else
	{
		// Ut will be modified with the new Ut...we need to store it away first !
		vec_generic_copy<double,double>(m_Ut_minus_dt.getPointer(), m_Ut.getPointer(), nbDOF);

		// Add the RHS equations of the backward Euler scheme
		addRHS_BackwardEuler(m_dt, m_Ut.getPointer(), m_Vt.getPointer(), m_F.getPointer());

		// RK.(R^t.x(t+dt)-x(0))=F
		// RKR^t.x(t+dt) = F + RK.x(0)
		// or, using a local displacement U(t+dt) between x(t) and x(t+dt):
		// RKR^t.U(t+dt) = F + RK.x(0) - RKR^t.x(t)
		static Dynamic_Vector<double> Utmp;
		Utmp.resizeUpIfNecessary(m_Ut.getSize());

		mat_vec_mul<double>(m_RK, m_x0.getPointer(), Utmp.getPointer(), nbDOF, nbDOF);
		vec_generic_add<double,double>(m_F.getPointer(), Utmp.getPointer(), nbDOF);

		mat_vec_mul<double>(m_K, m_xt.getPointer(), Utmp.getPointer(), nbDOF, nbDOF);
		vec_generic_sub<double,double>(m_F.getPointer(), Utmp.getPointer(), nbDOF);

		// We can't touch F, so let copy it 1st to U to modify some values and have the final result in it !
		vec_generic_copy<double,double>(m_Ut.getPointer(), m_F.getPointer(), nbDOF);

		// Make sure we won't have any displacement for the BC => null force for all the BC nodes
		for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
		{
			int nodeIndex = m_BC[bcIndex];
			for (int i = 0; i < 3; ++i)
			{
				m_Ut[3*nodeIndex + i] = 0.0;
			}
		}

		Modify_K_withBC();
		prepare_BackwardEuler_Matrices();       // Re-compute MDK & MD
		prepare_BackwardEuler_directSolver();    // Compute LU decomposition of MDK

		char trans='N';
		int nrhs = 1;
		int info;

		dgetrs(&trans, &nbDOF, &nrhs, m_K_LU.getPointer(), &nbDOF, m_LUpermutation.getPointer(), m_Ut.getPointer(), &nbDOF, &info);
		if (info < 0)
		{
			printf("FEMTetrahedra::solve_Static_LU could not solve linear system using the LU decomposition. Parameter %d invalid !\n", -info);
		}
	}

	// Compute the internal deformed mesh
	if (!m_corotational)
	{
		vec_generic_add<double, double, double>(m_x0.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), nbDOF);

		// Compute the new Vt
		vec_generic_sub<double,double,double>(m_Ut.getPointer(), m_Ut_minus_dt.getPointer(), m_Vt.getPointer(), nbDOF);
		vec_generic_scale<double,double>(m_Vt.getPointer(), nbDOF, 1.0/m_dt);
	}
	else
	{
		vec_generic_add<double, double, double>(m_xt.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), nbDOF);

		// Compute the new Vt = (X(t+dt)-X(t) )/dt  =  U/dt
		vec_generic_scale<double,double>(m_Ut.getPointer(), m_Vt.getPointer(), nbDOF, 1.0/m_dt);
	}

	// Update the deformed mesh
	updateDeformedMesh();
}

// Solve (M/dt + D/dt + K).u = F + + (M/dt2 + D/dt) x(t) + M/dt v(t) using CG
void FEMTetrahedra::solve_BackwardEuler_CG()
{
	const int nbDOF = 3 * m_nbNodes;

	int nbIter = 0;
	int nbIterMax = m_CG_maxIteration;
	double deltaNew;
	double delta0;
	double epsilon = m_CG_epsilon;
	double epsilonSQ=epsilon*epsilon;

	// Ut will be modified with the new Ut...we need to store it away first !
	vec_generic_copy<double,double>(m_Ut_minus_dt.getPointer(), m_Ut.getPointer(), nbDOF);

	// Add the RHS equations of the backward Euler scheme
	addRHS_BackwardEuler_CG(m_dt, m_Ut.getPointer(), m_Vt.getPointer(), m_F.getPointer());

	// We can't touch F, so let copy it 1st to U to modify some values and have the final result in it !
	vec_generic_copy<double,double>(m_FwithBC.getPointer(), m_F.getPointer(), nbDOF);

	// Make sure we won't have any displacement for the BC => null force for all the BC nodes
	for (int bc=0 ; bc<m_nbBC ; bc++)
	{
		int& I = m_BC[bc];
		m_FwithBC[3*I]=m_FwithBC[3*I+1]=m_FwithBC[3*I+2]=0.0;
	}

	// Initialization
	// q = (M/dt2 + D/dt + K) U
	MatVecProduct_MDK_u(m_dt, m_RayleightDamping_Mass_coef, m_RayleightDamping_Stif_coef, m_Ut.getPointer(), m_xt.getPointer(), m_x0.getPointer(),
	                    m_q.getPointer());

	// r = F + (-1)q = F - Ku
	vec_generic_triadic<double,double,double>(m_FwithBC.getPointer(), m_q.getPointer(), m_r.getPointer(), -1.0, nbDOF);

	// d = r
	vec_generic_copy<double,double>(m_d.getPointer(), m_r.getPointer(), nbDOF);

	// deltaNew = r.r
	deltaNew = vec_generic_dotProduct<double,double,double>(m_r.getPointer(), m_r.getPointer(), nbDOF);

	// delta0 = deltaNew
	delta0 = deltaNew;

	while (nbIter < nbIterMax && deltaNew > epsilonSQ * delta0)
	{
		// q = Kd
		MatVecProduct_MDK_u(m_dt, m_RayleightDamping_Mass_coef, m_RayleightDamping_Stif_coef, m_d.getPointer(), m_xt.getPointer(), m_x0.getPointer(),
		                    m_q.getPointer());

		// alpha = deltaNew/(d.q)
		double d_q = vec_generic_dotProduct<double,double,double>(m_d.getPointer(), m_q.getPointer(), nbDOF);
		if (d_q == 0.0)
		{
			cerr << "FEMTetrahedra::solve_BackwardEuler_CG d.q = 0 => can't divide to find alpha !" << endl;
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
			cerr << "FEMTetrahedra::solve_BackwardEuler_CG did not converge in " << nbIter << " iterations - Error = " << sqrt(deltaNew / delta0) << endl;
		}
	}
	else if (nbIter == 0)
	{
		if (m_convergenceVerbosity >= 1)
		{
			cout << "FEMTetrahedra::solve_BackwardEuler_CG converged before any iterations - (Abs) Error = " << sqrt(deltaNew) << endl;
		}
	}
	else
	{
		if (m_convergenceVerbosity >= 1)
		{
			cout << "FEMTetrahedra::solve_BackwardEuler_CG converged in " << nbIter << " iterations - Error = " << sqrt(deltaNew / delta0) << endl;
		}
	}

	// Compute the internal deformed mesh
	if (!m_corotational)
	{
		vec_generic_add<double, double, double>(m_x0.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), nbDOF);

		// Compute the new Vt
		// Linear deformation in total Lagrangian => Ut = x(t)-x(0)
		// Vt = [ U(t)-U(t-1) ]/dt   =   [ x(t)-x(t-1) ]/dt
		vec_generic_sub<double,double,double>(m_Ut.getPointer(), m_Ut_minus_dt.getPointer(), m_Vt.getPointer(), nbDOF);
		vec_generic_scale<double,double>(m_Vt.getPointer(), nbDOF, 1.0/m_dt);
	}
	else
	{
		vec_generic_add<double, double, double>(m_xt.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), nbDOF);

		// Compute the new Vt = U(t)/dt
		// Corotational => Updated Lagrangian => Ut = x(t)-x(t-1)
		// Vt = U(t)/dt   =   [ x(t)-x(t-1) ]/dt
		vec_generic_scale<double,double>(m_Ut.getPointer(), m_Vt.getPointer(), nbDOF, 1.0/m_dt);
	}

	// Update the deformed mesh
	updateDeformedMesh();
}


// Solve K.U=F for a given F using LU decomposition
void FEMTetrahedra::prepare_Static_directSolver()
{
	const int nbDOF = 3 * m_nbNodes;

	// Compute LU decomposition of K
	SQ_ASSERT(m_K.getNbElem(), "Can't decompose m_K because it wasn't allocated!");
	SQ_ASSERT(m_K_LU.getNbElem(), "Can't decompose into m_LU because it wasn't allocated!");

	int m = nbDOF, n = nbDOF;
	int lda = nbDOF;
	int info;
	mat_copy<double,double>(m_K, m_K_LU, nbDOF, nbDOF);
	dgetrf(&m, &n, m_K_LU.getPointer(), &lda, m_LUpermutation.getPointer(), &info);

	if (info < 0)
	{
		cerr << "MKL > LU decomposition for FEM (static) failed: parameter " << -info << " has a bad value" << endl;
	}
	else if (info > 0)
	{
		cerr << "MKL > LU decomposition for FEM (static) failed: U[" << info << "][" << info << "] = 0 " << endl;
	}
}

// Solve K.U=F for a given F using LU decomposition
void FEMTetrahedra::solve_Static_directSolver()
{
	const int nbDOF = 3 * m_nbNodes;

	if (m_corotational)
	{
		// RK.(R^t.x(t+dt)-x(0)) = F
		// RKR^t.x(t+dt) = F + RK.x(0)
		// or, using a local displacement U(t+dt) between x(t) and x(t+dt):
		// RKR^t.U(t+dt) = F + RK.x(0) - RKR^t.x(t)
		// We are going to work only in U...so let start copying F into U
		vec_generic_copy<double,double>(m_Ut.getPointer(), m_F.getPointer(), nbDOF);

		static Dynamic_Vector<double> Utmp;
		Utmp.resizeUpIfNecessary(m_Ut.getSize());

		mat_vec_mul<double>(m_RK, m_x0.getPointer(), Utmp.getPointer(), nbDOF, nbDOF);
		vec_generic_add<double,double>(m_Ut.getPointer(), Utmp.getPointer(), nbDOF);

		mat_vec_mul<double>(m_K, m_xt.getPointer(), Utmp.getPointer(), nbDOF, nbDOF);
		vec_generic_sub<double,double>(m_Ut.getPointer(), Utmp.getPointer(), nbDOF);

		// Apply boundary condition
		Modify_K_withBC();

		// Make sure we won't have any displacement for the BC => null force for all the BC nodes
		for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
		{
			int nodeIndex = m_BC[bcIndex];
			for (int i = 0; i < 3; ++i)
			{
				m_Ut[3*nodeIndex + i] = 0.0;
			}
		}

#if USE_SPARSE_LU_SOLVER

		// Store K and RK in MKL CSR format
		m_K_sparse.set(m_K.getPointer(), m_K.getNbRow(), m_K.getNbColumn());

		_MKL_DSS_HANDLE_t dss_handle;
		const _INTEGER_t dss_opt = MKL_DSS_DEFAULTS;
		if (dss_create(dss_handle, dss_opt) != MKL_DSS_SUCCESS)
		{
			Logger::instance()->Log("FEMTetrahedra::solve_Static_LU", Logger::Severe, "Could not create a valid direct sparse solver");
		}
		const _INTEGER_t dss_define_opt = MKL_DSS_NON_SYMMETRIC;
		const _INTEGER_t nRowsK = m_K_sparse.getNumRows();
		const _INTEGER_t nColsK = m_K_sparse.getNumColumns();
		const _INTEGER_t nNonZeroK = m_K_sparse.getNumNonZero();
		if (dss_define_structure(dss_handle, dss_define_opt, m_K_sparse.getRowIndices(), nRowsK,
		                         nColsK, m_K_sparse.getColumnIndices(), nNonZeroK) != MKL_DSS_SUCCESS)
		{
			Logger::instance()->Log("FEMTetrahedra::solve_Static_LU", Logger::Severe, "Could not define the structure of the direct sparse solver");
		}
		const _INTEGER_t dss_reorder_opt = MKL_DSS_DEFAULTS;
		if (dss_reorder(dss_handle, dss_reorder_opt, NULL) != MKL_DSS_SUCCESS)
		{
			Logger::instance()->Log("FEMTetrahedra::solve_Static_LU", Logger::Severe, "Could not reorder the structure of the direct sparse solver");
		}
		const _INTEGER_t dss_factor_opt = MKL_DSS_POSITIVE_DEFINITE;
		if (dss_factor_real(dss_handle, dss_factor_opt, m_K_sparse.getValues()) != MKL_DSS_SUCCESS)
		{
			Logger::instance()->Log("FEMTetrahedra::solve_Static_LU", Logger::Severe, "Could not factor the matrix of the direct sparse solver");
		}
		const _INTEGER_t dss_solve_opt = MKL_DSS_DEFAULTS;
		const _INTEGER_t nRhs = 1;
		if (dss_solve_real(dss_handle, dss_solve_opt, m_Ut.getPointer(), nRhs, Utmp.getPointer()) !=  MKL_DSS_SUCCESS)
		{
			Logger::instance()->Log("FEMTetrahedra::solve_Static_LU", Logger::Severe, "Direct sparse solver found not solution");
		}
		for (int i = 0; i < nbDOF; ++i)
		{
			m_Ut[i] = Utmp[i];
		}
		if (dss_delete(dss_handle, dss_opt) != MKL_DSS_SUCCESS)
		{
			Logger::instance()->Log("FEMTetrahedra::solve_Static_LU", Logger::Severe, "Could not delete a valid direct sparse solver");
		}

#else
		prepare_Static_directSolver();
		char trans='N';
		int nrhs=1;
		int info;
		dgetrs(&trans, &nbDOF, &nrhs, m_K_LU.getPointer(), &nbDOF, m_LUpermutation.getPointer(), U.getPointer(), &nbDOF, &info);
		if (info<0)
		{
			printf("FEMTetrahedra::solve_Static_LU could not solve linear system using the LU decomposition. Parameter %d invalid !\n",-info);
		}
		cout << "Solution = " << U << endl;
#endif
		// Compute the internal deformed mesh
		// U contains x(t+dt), but U should contains the displacement from last known position x(t): U = x(t+dt)-x(t)
		vec_generic_add<double, double, double>(m_xt.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), nbDOF);
	}
	else
	{
		// We are going to work only in U...so let start copying F into U
		vec_generic_copy<double,double>(m_Ut.getPointer(), m_F.getPointer(), nbDOF);

		// Add the Barycentric Spring Forces in the LHS (the mesh point anchor)
		mat_copy<double,double>(m_C, m_C_withBTSF, nbDOF, nbDOF);
		addBTSF_LHS(m_C_withBTSF);

		// Make sure we won't have any displacement for the BC => null force for all the BC nodes
		for (int bcIndex = 0; bcIndex < m_nbBC; ++bcIndex)
		{
			int nodeIndex = m_BC[bcIndex];
			for (int i = 0; i < 3; ++i)
			{
				m_Ut[3*nodeIndex + i] = 0.0;
			}
		}

		static Dynamic_Vector<double> Utmp;
		Utmp.resizeUpIfNecessary(m_Ut.getSize());

		// K.U=F => U=K^-1.F = C.F (C = Compliance matrix = K^-1)
		mat_vec_mul<double>(m_C, m_Ut.getPointer(), Utmp.getPointer(), nbDOF, nbDOF);
		vec_generic_copy<double,double>(m_Ut.getPointer(), Utmp.getPointer(), nbDOF);

		// Compute the internal deformed mesh
		vec_generic_add<double, double, double>(m_x0.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), nbDOF);
	}

	// Update the deformed mesh
	updateDeformedMesh();
}

// Solve K.U=F for a given F, and a starting guess of U, using Conjugate Gradient
void FEMTetrahedra::solve_StaticCG(bool useGlobalMatrix)
{
	const int nbDOF = 3 * m_nbNodes;
	int nbIter = 0;
	int nbIterMax = m_CG_maxIteration;
	double deltaNew;
	double delta0;
	double epsilon = m_CG_epsilon;
	double epsilonSQ = epsilon*epsilon;

	// We can't touch F, so let copy it 1st to FwithBC to modify some values !
	vec_generic_copy<double,double>(m_FwithBC.getPointer(), m_F.getPointer(), nbDOF);

	// Make sure we won't have any displacement for the BC => null force for all the BC nodes
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

	while (nbIter < nbIterMax && deltaNew > epsilonSQ*delta0)
	{
		// q = Kd
		MatVecProduct_K_u(m_d.getPointer(), m_xt.getPointer(), m_x0.getPointer(), m_q.getPointer(), useGlobalMatrix);

		// alpha = deltaNew/(d.q)
		double d_q = vec_generic_dotProduct<double,double,double>(m_d.getPointer(), m_q.getPointer(), nbDOF);
		if (d_q==0.0)
		{
			cerr << "FEMTetrahedra::solve_StaticCG d.q = 0 => can't divide to find alpha !" << endl;
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
			cerr << "FEMTetrahedra::solve_StaticCG did not converge in " << nbIter << " iterations - Error = " << sqrt(deltaNew / delta0) << endl;
		}
	}
	else if (nbIter == 0)
	{
		if (m_convergenceVerbosity >= 1)
		{
			cout << "FEMTetrahedra::solve_StaticCG converged before any iterations - (Abs) Error = " << sqrt(deltaNew) << endl;
		}
	}
	else
	{
		if (m_convergenceVerbosity >= 1)
		{
			cout << "FEMTetrahedra::solve_StaticCG converged in " << nbIter << " iterations - Error = " << sqrt(deltaNew / delta0) << endl;
		}
	}

	// Compute the internal deformed mesh
	if (!m_corotational)
	{
		vec_generic_add<double, double, double>(m_x0.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), nbDOF);
	}
	else
	{
		vec_generic_add<double, double, double>(m_xt.getPointer(), m_Ut.getPointer(), m_xt.getPointer(), nbDOF);
	}

	// Update the deformed mesh
	updateDeformedMesh();
}

void FEMTetrahedra::solveOneStep()
{
	if (m_corotational)
	{
		for (int tetID=0; tetID < getNbTetrahedron(); ++tetID)
		{
			const_cast<Tetrahedron*>(getTetrahedron(tetID))->UpdateGlobalStiffnessMatrix(getUndeformedMesh(), getDeformedMesh());
		}
	}

	if (simulationIsStatic())
	{
		if (simulationUsesPrecomputedDecomposition())
		{
			if (m_corotational)
			{
				Assemble_K_corotational();
				computePerNodeCoRotation();
			}
			solve_Static_directSolver();
		}
		else if (simulationUsesConjugateGradient())
		{
			bool useGlobalMatrix=false;
			if (useGlobalMatrix)
			{
				Assemble_K_corotational();
			}
			if (m_corotational)
			{
				computePerNodeCoRotation();
			}
			solve_StaticCG(useGlobalMatrix);
		}
	}
	else if (simulationIsDynamic())
	{
		if (simulationUsesPrecomputedDecomposition())
		{
			if (m_corotational)
			{
				Assemble_K_corotational();     // Re-compute K global and RK global
				computePerNodeCoRotation();
			}
			solve_BackwardEuler_directSolver();
		}
		else if (simulationUsesConjugateGradient())
		{
			bool useGlobalMatrix = false;
			if (useGlobalMatrix)
			{
				Assemble_K_corotational();
			}
			if (m_corotational)
			{
				computePerNodeCoRotation();
			}
			solve_BackwardEuler_CG(); /// @Note: This is inconsistent in form with the static method in that not using globalMatrix (sparse like multiplication) is done by default with no flag setting
		}
	}
}

void FEMTetrahedra::updateStressStrains()
{
	for (int i = 0; i < Tetrahedron::NUM_STRESS_STRAIN_MODES; ++i)
	{
		m_maximumStressStrainValue[i] = -std::numeric_limits<double>::max();
		m_minimumStressStrainValue[i] = std::numeric_limits<double>::max();
	}

	for (int i = 0; i < m_nbElements; i++)
	{
		Tetrahedron* tet = static_cast<Tetrahedron*>(m_elements[i]);

		tet->updateStressAndStrain(m_deformedMesh);

		for (int mode = 0; mode < Tetrahedron::NUM_STRESS_STRAIN_MODES; ++mode)
		{
			double value = tet->getStressStrainValue((Tetrahedron::StressStrainMode)mode);
			if (value > m_maximumStressStrainValue[mode])
			{
				m_maximumStressStrainValue[mode] = value;
			}
			if (value < m_minimumStressStrainValue[mode])
			{
				m_minimumStressStrainValue[mode] = value;
			}
		}
	}
}
double FEMTetrahedra::getMaximumStressStrainValue(Tetrahedron::StressStrainMode mode) const
{
	return m_maximumStressStrainValue[mode];
}
double FEMTetrahedra::getMinimumStressStrainValue(Tetrahedron::StressStrainMode mode) const
{
	return m_minimumStressStrainValue[mode];
}

ostream& operator <<(ostream& o, FEMTetrahedra& fem)
{
	bool textMode = false;
	if (textMode)
	{
		const Pt3D* pts = fem.getDeformedMesh();
		o << fem.m_nbNodes << endl;
		for (int i = 0; i < fem.m_nbNodes; i++)
		{
			o << pts[i][0] << " " << pts[i][1] << " " << pts[i][2] << endl;
		}
	}
	else
	{
		const Pt3D* pts = fem.m_deformedMesh;
		o.write((const char*)&fem.m_nbNodes, sizeof(int));
		o.write((const char*)pts, fem.m_nbNodes*sizeof(Pt3D));
	}

	return o;
}

istream& operator >>(istream& i, FEMTetrahedra& fem)
{
	bool textMode = false;
	if (textMode)
	{
		i >> fem.m_nbNodes;
		if (i.eof())
		{
			cerr << "EOF reached" << endl;
			return i;
		}

		if (!fem.m_deformedMesh)
		{
			fem.m_deformedMesh = (Pt3D*)malloc(sizeof(Pt3D) * fem.m_nbNodes);
		}
		for (int id = 0; id < fem.m_nbNodes; id++)
		{
			i >> fem.m_deformedMesh[id][0] >> fem.m_deformedMesh[id][1] >> fem.m_deformedMesh[id][2];
			if (i.eof())
			{
				cerr << "EOF reached" << endl;
				return i;
			}
		}
	}
	else
	{
		i.read((char*)&fem.m_nbNodes, sizeof(int));
		if (i.eof())
		{
			cerr << "EOF reached" << endl;
			return i;
		}
		if (!fem.m_deformedMesh)
		{
			fem.m_deformedMesh = (Pt3D*)malloc(sizeof(Pt3D) * fem.m_nbNodes);
		}
		i.read((char*)fem.m_deformedMesh, fem.m_nbNodes*sizeof(Pt3D));
		if (i.eof())
		{
			cerr << "EOF reached" << endl;
			return i;
		}
	}

	return i;
}