#include "FEMTetrahedra.h"

void FEMTetrahedra::updateDeformedMesh()
{
	FEM<4,3>::updateDeformedMesh();

	// The mesh has changed, we need to update the surface
	clampDeformedSurfacePoints();
}

void FEMTetrahedra::clampSurfacePoints(const Pt3D* volumeMesh, Pt3D* surfaceMesh) const
{
	for (int i = 0; i < m_nSurfacePts; i++)
	{
		const Tetrahedron* tet = getTetrahedron(m_surfacePtTetIds[i]);

		Pt3D point;
		tet->calculateEmbeddedLocation(volumeMesh, m_surfacePtTetWeights[i], point);

		for (int j = 0; j < 3; j++)
		{
			surfaceMesh[i][j] = point[j];
		}
	}
}

void FEMTetrahedra::clampDeformedSurfacePoints()
{
	clampSurfacePoints(m_deformedMesh, m_deformedSurfaceMesh);
}

void FEMTetrahedra::setLastDeformedSurfaceMesh(const Pt3D* points)
{
	vec_generic_copy<double,double>((double*)m_lastDeformedSurfaceMesh , (double*)points , 3*m_nSurfacePts);
}
const Pt3D* FEMTetrahedra::getLastDeformedSurfaceMesh() const
{
	return m_lastDeformedSurfaceMesh;
}
const Pt3D* FEMTetrahedra::getDeformedSurfaceMesh() const
{
	return m_deformedSurfaceMesh;
}
const Pt3D* FEMTetrahedra::getUndeformedSurfaceMesh() const
{
	return m_undeformedSurfaceMesh;
}

void FEMTetrahedra::resizeSurfaceMesh(int numPoints)
{
	m_nSurfacePts = numPoints;
	m_undeformedSurfaceMesh = (Pt3D*)realloc(m_undeformedSurfaceMesh, sizeof(Pt3D)*m_nSurfacePts);
	m_deformedSurfaceMesh = (Pt3D*)realloc(m_deformedSurfaceMesh, sizeof(Pt3D)*m_nSurfacePts);
	m_lastDeformedSurfaceMesh = (Pt3D*)realloc(m_lastDeformedSurfaceMesh, sizeof(Pt3D)*m_nSurfacePts);

	m_surfacePtTetIds = (int*)realloc(m_surfacePtTetIds, sizeof(int)*m_nSurfacePts);
	m_surfacePtTetWeights = (PointWeights*)realloc(m_surfacePtTetWeights, sizeof(PointWeights)*m_nSurfacePts);
}

void FEMTetrahedra::buildSurfaceTrisFromTets()
{
	std::vector<SurfaceTriangle> potentialTriangles;
	std::vector<int> potentialTriTetIds;

	int triNodeID[3];

	// Collect all tetrahedron triangles.
	for (int i = 0; i < m_nbElements; i++)
	{
		const Tetrahedron* tet = getTetrahedron(i);

		// treat the 1st triangle CBA
		triNodeID[0] = tet->getPointID(2);
		triNodeID[1] = tet->getPointID(1);
		triNodeID[2] = tet->getPointID(0);

		potentialTriangles.push_back(SurfaceTriangle(triNodeID[0], triNodeID[1], triNodeID[2]));
		potentialTriTetIds.push_back(i);

		// treat the 2nd triangle ABD
		triNodeID[0] = tet->getPointID(0);
		triNodeID[1] = tet->getPointID(1);
		triNodeID[2] = tet->getPointID(3);
		potentialTriangles.push_back(SurfaceTriangle(triNodeID[0], triNodeID[1], triNodeID[2]));
		potentialTriTetIds.push_back(i);

		// treat the 3rd triangle DCA
		triNodeID[0] = tet->getPointID(3);
		triNodeID[1] = tet->getPointID(2);
		triNodeID[2] = tet->getPointID(0);
		potentialTriangles.push_back(SurfaceTriangle(triNodeID[0], triNodeID[1], triNodeID[2]));
		potentialTriTetIds.push_back(i);

		// treat the 4th triangle BCD
		triNodeID[0] = tet->getPointID(1);
		triNodeID[1] = tet->getPointID(2);
		triNodeID[2] = tet->getPointID(3);
		potentialTriangles.push_back(SurfaceTriangle(triNodeID[0], triNodeID[1], triNodeID[2]));
		potentialTriTetIds.push_back(i);
	}

	// Filter only the triangles that occur once - these are surface triangles.
	std::vector<SurfaceTriangle> filteredTriangles;
	std::vector<int> filteredTriTetIds;

	for (int i = 0; i < (int)potentialTriangles.size(); i++)
	{
		SurfaceTriangle& tri1 = potentialTriangles[i];

		int numOccurences = 0;
		for (int j = 0; j < (int)potentialTriangles.size(); j++)
		{
			SurfaceTriangle& tri2 = potentialTriangles[j];

			// Check if the triangle contains all the same points - if so, it is the same triangle (just possibly facing other direction).
			if ((tri1.pointIds[0] == tri2.pointIds[0] || tri1.pointIds[0] == tri2.pointIds[1] || tri1.pointIds[0] == tri2.pointIds[2]) &&
			    (tri1.pointIds[1] == tri2.pointIds[0] || tri1.pointIds[1] == tri2.pointIds[1] || tri1.pointIds[1] == tri2.pointIds[2]) &&
			    (tri1.pointIds[2] == tri2.pointIds[0] || tri1.pointIds[2] == tri2.pointIds[1] || tri1.pointIds[2] == tri2.pointIds[2]))
			{
				numOccurences++;
			}
		}

		if (numOccurences == 1)
		{
			filteredTriangles.push_back(potentialTriangles[i]);
			filteredTriTetIds.push_back(potentialTriTetIds[i]);
		}
	}

	// Add and map the nodes to surface points.
	std::map<int, int> mapNodeToPoint;
	int numPoints = 0;

	for (int i = 0; i < (int)filteredTriangles.size(); i++)
	{
		SurfaceTriangle& tri = filteredTriangles[i];

		for (int j = 0; j < 3; j++)
		{
			int node = tri.pointIds[j];

			if (mapNodeToPoint.find(node) == mapNodeToPoint.end())
			{
				mapNodeToPoint.insert(std::pair<int, int>(node, numPoints));
				numPoints++;
			}
		}
	}

	resizeSurfaceMesh(numPoints);

	// Store surface points and add triangles.
	for (int i = 0; i < (int)filteredTriangles.size(); i++)
	{
		SurfaceTriangle& tri = filteredTriangles[i];

		int pointIds[3];
		int tetId = filteredTriTetIds[i];

		const Tetrahedron* tet = getTetrahedron(tetId);
		double weights[3][4];

		for (int j = 0; j < 3; j++)
		{
			int node = tri.pointIds[j];
			pointIds[j] = mapNodeToPoint[node];

			for (int k = 0; k < 4; k++)
			{
				weights[j][k] = 0.0;
				if (node == tet->getPointID(k))
				{
					weights[j][k] = 1.0;
				}
			}
		}

		addSurfaceTriangle(pointIds, tetId, weights);
	}
}

void FEMTetrahedra::mapTriToTet(const int triID, const double triBaryCoord[3], int& tetID, double tetBaryCoord[4]) const
{
	tetID = m_surfaceTriTetIds[triID];

	for (int i = 0; i < 4; i++)
	{
		tetBaryCoord[i] = 0.0;
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			tetBaryCoord[j] += triBaryCoord[i] * m_surfaceTriTetWeights[triID][i][j];
		}
	}
}

void FEMTetrahedra::eraseTriMLCP_FRICTIONLESS_3D_CONTACT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset, int firstLineID, const double* n, double scale,
    int triID, const double triBaryCoord[3])
{
	int tetID;
	double tetBaryCoord[4];
	mapTriToTet(triID, triBaryCoord, tetID, tetBaryCoord);

	eraseTetMLCP_FRICTIONLESS_3D_CONTACT(H, CHt, offset, firstLineID, n, scale, tetID, tetBaryCoord);
}
void FEMTetrahedra::eraseTriMLCP_FRICTIONAL_3D_CONTACT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset, int firstLineID, const double* n, const double* t1, const double* t2, double scale,
    int triID, const double triBaryCoord[3])
{
	int tetID;
	double tetBaryCoord[4];
	mapTriToTet(triID, triBaryCoord, tetID, tetBaryCoord);

	eraseTetMLCP_FRICTIONAL_3D_CONTACT(H, CHt, offset, firstLineID, n, t1, t2, scale, tetID, tetBaryCoord);
}
void FEMTetrahedra::eraseTriMLCP_BILATERAL_3D_CONSTRAINT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset, int firstLineID, double scale,
    int triID, const double triBaryCoord[3])
{
	int tetID;
	double tetBaryCoord[4];
	mapTriToTet(triID, triBaryCoord, tetID, tetBaryCoord);

	eraseTetMLCP_BILATERAL_3D_CONSTRAINT(H, CHt, offset, firstLineID, scale, tetID, tetBaryCoord);
}
void FEMTetrahedra::eraseTriMLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset, int firstLineID, const double* n1, const double* n2, double scale,
    int triID, const double triBaryCoord[3])
{
	int tetID;
	double tetBaryCoord[4];
	mapTriToTet(triID, triBaryCoord, tetID, tetBaryCoord);

	eraseTetMLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT(H, CHt, offset, firstLineID, n1, n2, scale, tetID, tetBaryCoord);
}
void FEMTetrahedra::eraseTriMLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset, int firstLineID, const double* n1, const double* n2, const double* t, double scale,
    int triID, const double triBaryCoord[3])
{
	int tetID;
	double tetBaryCoord[4];
	mapTriToTet(triID, triBaryCoord, tetID, tetBaryCoord);

	eraseTetMLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT(H, CHt, offset, firstLineID, n1, n2, t, scale, tetID, tetBaryCoord);
}

void FEMTetrahedra::buildTriMLCP_FRICTIONLESS_3D_CONTACT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, const double* n, double scale,
    int triID, const double triBaryCoord[3], bool fillUpViolationONLY)
{
	int tetID;
	double tetBaryCoord[4];
	mapTriToTet(triID, triBaryCoord, tetID, tetBaryCoord);

	buildTetMLCP_FRICTIONLESS_3D_CONTACT(E, H, CHt, HCHt, offset, firstLineID, n, scale, tetID, tetBaryCoord, fillUpViolationONLY);
}
void FEMTetrahedra::buildTriMLCP_FRICTIONAL_3D_CONTACT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, const double* n, const double* t1, const double* t2, double scale,
    int triID, const double triBaryCoord[3], bool fillUpViolationONLY)
{
	int tetID;
	double tetBaryCoord[4];
	mapTriToTet(triID, triBaryCoord, tetID, tetBaryCoord);

	buildTetMLCP_FRICTIONAL_3D_CONTACT(E, H, CHt, HCHt, offset, firstLineID, n, t1, t2, scale, tetID, tetBaryCoord, fillUpViolationONLY);
}
void FEMTetrahedra::buildTriMLCP_BILATERAL_3D_CONSTRAINT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, double scale,
    int triID, const double triBaryCoord[3], bool fillUpViolationONLY)
{
	int tetID;
	double tetBaryCoord[4];
	mapTriToTet(triID, triBaryCoord, tetID, tetBaryCoord);

	buildTetMLCP_BILATERAL_3D_CONSTRAINT(E, H, CHt, HCHt, offset, firstLineID, scale, tetID, tetBaryCoord, fillUpViolationONLY);
}
void FEMTetrahedra::buildTriMLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, const double* n1, const double* n2, double scale,
    int triID, const double triBaryCoord[3], bool fillUpViolationONLY)
{
	int tetID;
	double tetBaryCoord[4];
	mapTriToTet(triID, triBaryCoord, tetID, tetBaryCoord);

	buildTetMLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT(E, H, CHt, HCHt, offset, firstLineID, n1, n2, scale, tetID, tetBaryCoord, fillUpViolationONLY);
}
void FEMTetrahedra::buildTriMLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, const double* n1, const double* n2, const double* t, double scale,
    int triID, const double triBaryCoord[3], bool fillUpViolationONLY)
{
	int tetID;
	double tetBaryCoord[4];
	mapTriToTet(triID, triBaryCoord, tetID, tetBaryCoord);

	buildTetMLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT(E, H, CHt, HCHt, offset, firstLineID, n1, n2, t, scale, tetID, tetBaryCoord, fillUpViolationONLY);
}

//##################################################
//##################################################
// ### MLCP Frictionless 3D Contact
//##################################################

void FEMTetrahedra::eraseTetMLCP_FRICTIONLESS_3D_CONTACT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset, int firstLineID, const double* n, double scale,
    int tetID, const double tetBaryCoord[4])
{
	const Tetrahedron* tet = getTetrahedron(tetID);
	int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

	// Erase entries in H
	for (int i=0 ; i<4 ; i++)
	{
		H[firstLineID][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID][offset + 3*nodeID[i]+2] = 0.0;
	}

	// Erase entries in CHt
	for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
	{
		CHt[offset + CHt_line][firstLineID] = 0.0;
	}
}

// 3D contact without Friction
// Fill up the line of the contact matrix H the given contact info:
// - n the normal plane of contact
// - (tetID, tetBaryCoord) the tetrahedron point in contact
//   tetID in [0, m_nTets-1]
//   tetBaryCoord in ([0,1], [0,1], [0,1], [0,1])
// + build the C.Ht matrix accordingly
// Suppose H and CHt set to 0 prior to the call
void FEMTetrahedra::buildTetMLCP_FRICTIONLESS_3D_CONTACT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, const double* n, double scale,
    int tetID, const double tetBaryCoord[4], bool fillUpViolationONLY)
{
	const Tetrahedron* tet = getTetrahedron(tetID);
	const Dynamic_Matrix<double>& C = getComplianceMatrixForLCP();
	int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

	Pt3D point;
	tet->calculateEmbeddedLocation(m_deformedMesh, tetBaryCoord, point);
	E[firstLineID] += n[3]*scale;
	E[firstLineID] += scale * (n[0]*point[0] + n[1]*point[1] + n[2]*point[2]);

	if (fillUpViolationONLY)
	{
		return;
	}

	// If one of the points is a boundary condition, then treat this as fixed; otherwise objects will penetrate
	for (int i = 0; i < m_nbBC; ++i)
	{
		if (m_BC[i] == nodeID[0] || m_BC[i] == nodeID[1] || m_BC[i] == nodeID[2] || m_BC[i] == nodeID[3])
		{
			return;
		}
	}

	// Fill up H with just the non null values
	for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
	{
		int nodeColumnInC;
		if (m_corotational)
		{
			nodeColumnInC = m_constrainedNodesToColumnsInWarpedC[nodeID[i]];
		}
		else
		{
			nodeColumnInC = 3*nodeID[i];
		}

		H[firstLineID][offset + 3*nodeID[i]+0] += scale*n[0] * tetBaryCoord[i]; // nx
		H[firstLineID][offset + 3*nodeID[i]+1] += scale*n[1] * tetBaryCoord[i]; // ny
		H[firstLineID][offset + 3*nodeID[i]+2] += scale*n[2] * tetBaryCoord[i]; // nz

		// Fill up CHt by computing the minimum matrix-vector operation
		for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
		{
			CHt[offset + CHt_line][firstLineID] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID][offset + 3*nodeID[i]+2];
		}
	}

	// Compute the current HCHt
	// NOTE: HCHt is symmetric => we compute the last line and reflect it on the last column
	for (int col=0 ; col<firstLineID ; col++)
	{
		double coef=0;
		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
		{
			coef +=  H[firstLineID][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			         H[firstLineID][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			         H[firstLineID][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
		}
		HCHt[firstLineID  ][col] += coef;
		HCHt[col][firstLineID  ] += coef;
	}

	double& coef=HCHt[firstLineID][firstLineID];
	for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
	{
		coef += H[firstLineID][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][firstLineID] +
		        H[firstLineID][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][firstLineID] +
		        H[firstLineID][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][firstLineID];
	}
}

//##################################################
//##################################################
// ### MLCP Frictional 3D Contact
//##################################################

void FEMTetrahedra::eraseTetMLCP_FRICTIONAL_3D_CONTACT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset , int firstLineID, const double* n, const double* t1, const double* t2, double scale,
    int tetID, const double tetBaryCoord[4])
{
	const Tetrahedron* tet = getTetrahedron(tetID);
	int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

	// Erase entries in H
	for (int i=0 ; i<4 ; i++)
	{
		H[firstLineID][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID][offset + 3*nodeID[i]+2] = 0.0;

		H[firstLineID+1][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID+1][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID+1][offset + 3*nodeID[i]+2] = 0.0;

		H[firstLineID+2][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID+2][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID+2][offset + 3*nodeID[i]+2] = 0.0;
	}

	// Erase entries in CHt
	for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
	{
		CHt[offset + CHt_line][firstLineID] = 0.0;
		CHt[offset + CHt_line][firstLineID+1] = 0.0;
		CHt[offset + CHt_line][firstLineID+2] = 0.0;
	}
}

// 3D contact with Friction
// Fill up the line (3*numContact..3*numContact+2) of the contact matrix with the given contact info:
// - n,t1,t2 the normal plane and the 2 tangential plane of contact
// - (tetID, tetBaryCoord) the tetrahedron point in contact
//   tetID in [0, m_nTets-1]
//   tetBaryCoord in ([0,1], [0,1], [0,1], [0,1])
// + build the C.Ht matrix accordingly
// Suppose H and CHt set to 0 prior to the call
void FEMTetrahedra::buildTetMLCP_FRICTIONAL_3D_CONTACT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, const double* n, const double* t1, const double* t2, double scale,
    int tetID, const double tetBaryCoord[4], bool fillUpViolationONLY)
{
	const Tetrahedron* tet = getTetrahedron(tetID);
	const Dynamic_Matrix<double>& C = getComplianceMatrixForLCP();
	int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

	Pt3D point;
	tet->calculateEmbeddedLocation(m_deformedMesh, tetBaryCoord, point);
	E[firstLineID  ] += scale * (n[0]*point[0] +  n[1]*point[1] +  n[2]*point[2] +  n[3]);
	E[firstLineID+1] += scale * (t1[0]*point[0] + t1[1]*point[1] + t1[2]*point[2] + t1[3]);
	E[firstLineID+2] += scale * (t2[0]*point[0] + t2[1]*point[1] + t2[2]*point[2] + t2[3]);

	if (fillUpViolationONLY)
	{
		return;
	}

	// If one of the points is a boundary condition, then treat this as fixed; otherwise objects will penetrate
	for (int i = 0; i < m_nbBC; ++i)
	{
		if (m_BC[i] == nodeID[0] || m_BC[i] == nodeID[1] || m_BC[i] == nodeID[2] || m_BC[i] == nodeID[3])
		{
			return;
		}
	}

	// Fill up H with just the non null values
	for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
	{
		H[firstLineID][offset + 3*nodeID[i]+0] += scale*n[0] * tetBaryCoord[i]; // nx
		H[firstLineID][offset + 3*nodeID[i]+1] += scale*n[1] * tetBaryCoord[i]; // ny
		H[firstLineID][offset + 3*nodeID[i]+2] += scale*n[2] * tetBaryCoord[i]; // nz

		H[firstLineID+1][offset + 3*nodeID[i]+0] += scale*t1[0] * tetBaryCoord[i]; // t1x
		H[firstLineID+1][offset + 3*nodeID[i]+1] += scale*t1[1] * tetBaryCoord[i]; // t1y
		H[firstLineID+1][offset + 3*nodeID[i]+2] += scale*t1[2] * tetBaryCoord[i]; // t1z

		H[firstLineID+2][offset + 3*nodeID[i]+0] += scale*t2[0] * tetBaryCoord[i]; // t2x
		H[firstLineID+2][offset + 3*nodeID[i]+1] += scale*t2[1] * tetBaryCoord[i]; // t2y
		H[firstLineID+2][offset + 3*nodeID[i]+2] += scale*t2[2] * tetBaryCoord[i]; // t2z
	}

	for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
	{
		int nodeColumnInC;
		if (m_corotational)
		{
			nodeColumnInC = m_constrainedNodesToColumnsInWarpedC[nodeID[i]];
		}
		else
		{
			nodeColumnInC = 3*nodeID[i];
		}

		// Fill up CHt by computing the minimum matrix-vector operation
		for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
		{
			CHt[offset + CHt_line][firstLineID] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID][offset + 3*nodeID[i]+2];
			CHt[offset + CHt_line][firstLineID+1] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID+1][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID+1][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID+1][offset + 3*nodeID[i]+2];
			CHt[offset + CHt_line][firstLineID+2] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID+2][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID+2][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID+2][offset + 3*nodeID[i]+2];
		}
	}

	// Compute the current HCHt
	// NOTE: HCHt is symmetric => we compute the last line and reflect it on the last column
	for (int col=0 ; col<firstLineID ; col++)
	{
		double coef[3]= {0,0,0};

		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
		{
			coef[0] +=  H[firstLineID][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			            H[firstLineID][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			            H[firstLineID][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
			coef[1] +=  H[firstLineID+1][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			            H[firstLineID+1][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			            H[firstLineID+1][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
			coef[2] +=  H[firstLineID+2][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			            H[firstLineID+2][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			            H[firstLineID+2][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
		}

		HCHt[firstLineID  ][col] += coef[0];
		HCHt[col][firstLineID  ] += coef[0];

		HCHt[firstLineID+1][col] += coef[1];
		HCHt[col][firstLineID+1] += coef[1];

		HCHt[firstLineID+2][col] += coef[2];
		HCHt[col][firstLineID+2] += coef[2];
	}
	for (int col=firstLineID ; col<=firstLineID+2 ; col++)
	{
		double& coef1=HCHt[firstLineID  ][col];
		double& coef2=HCHt[firstLineID+1][col];
		double& coef3=HCHt[firstLineID+2][col];

		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
		{
			coef1 += H[firstLineID][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			         H[firstLineID][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			         H[firstLineID][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
			coef2 += H[firstLineID+1][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			         H[firstLineID+1][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			         H[firstLineID+1][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
			coef3 += H[firstLineID+2][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			         H[firstLineID+2][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			         H[firstLineID+2][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
		}
	}
}

//##################################################
//##################################################
// ### MLCP Bilateral 3D constraint (anchor point)
//##################################################

void FEMTetrahedra::eraseTetMLCP_BILATERAL_3D_CONSTRAINT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset , int firstLineID, double scale,
    int tetID, const double tetBaryCoord[4])
{
	const Tetrahedron* tet = getTetrahedron(tetID);
	int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

	// Erase entries in H
	for (int i=0 ; i<4 ; i++)
	{
		H[firstLineID][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID][offset + 3*nodeID[i]+2] = 0.0;

		H[firstLineID+1][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID+1][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID+1][offset + 3*nodeID[i]+2] = 0.0;

		H[firstLineID+2][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID+2][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID+2][offset + 3*nodeID[i]+2] = 0.0;
	}

	// Erase entries in CHt
	for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
	{
		CHt[offset + CHt_line][firstLineID] = 0.0;
		CHt[offset + CHt_line][firstLineID+1] = 0.0;
		CHt[offset + CHt_line][firstLineID+2] = 0.0;
	}
}

void FEMTetrahedra::buildTetMLCP_BILATERAL_3D_CONSTRAINT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, double scale,
    int tetID, const double tetBaryCoord[4], bool fillUpViolationONLY)
{
	// JL 2011-10-26 add comment
	// The MLCP solver expect the data to be presented as follow:
	// 0 <= HCHt.lambda + E >=0
	//
	// The constrained system is written
	// (M  Ht).(DOF    ) = ( F  )
	// (H   0) (-lambda)   ( E1 )
	// with H.DOF - E1 = 0 being the constraints equations
	//
	// Which developed splitting the DOF in free motion and correction give:
	// { M.DOF_f = F
	// { M.DOF_c = Ht.lambda
	// { H.DOF_c = E1-H.DOF_f <=> HCHt.lambda = E1-H.DOF_f
	// to be ready for the MLCP, the data will be presented as:
	// HCHt.lambda + E where E = H.DOF_f-E1 (=constraint equation evaluated after free motion)
	//
	// Here is the constraint considering this object as the 1st one (positive scale):
	// P(t+dt) - P_other_object = 0
	// FEM3D is a total Lagrangian model P(t+dt) = P(0) + U(t+dt)
	// But free motion already has been done and the correction terms are related to the free motion
	// so here we compute a displacement Up(t+dt) such that:
	// P(t) + Up(t+dt) - P_other_object = 0
	// H.U - P_other_object + P(t) = 0
	// So as far as the FEM3D is concerned, its part of the equation is:
	// H=(coefs bary)  E=P(t)
	//   (coefs bary)
	//   (coefs bary)
	const Tetrahedron* tet = getTetrahedron(tetID);
	const Dynamic_Matrix<double>& C = getComplianceMatrixForLCP();
	int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

	Pt3D point;
	tet->calculateEmbeddedLocation(m_deformedMesh, tetBaryCoord, point);
	double p[3]= { point[0], point[1], point[2] };
	E[firstLineID  ] += p[0]*scale;
	E[firstLineID+1] += p[1]*scale;
	E[firstLineID+2] += p[2]*scale;

	if (fillUpViolationONLY)
	{
		return;
	}

	// Fill up H with just the non null values
	for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
	{
		H[firstLineID][offset + 3*nodeID[i]+0] += scale * tetBaryCoord[i]; // Px
		H[firstLineID+1][offset + 3*nodeID[i]+1] += scale * tetBaryCoord[i]; // Py
		H[firstLineID+2][offset + 3*nodeID[i]+2] += scale * tetBaryCoord[i]; // Pz

		int nodeColumnInC;
		if (m_corotational)
		{
			nodeColumnInC = m_constrainedNodesToColumnsInWarpedC[nodeID[i]];
		}
		else
		{
			nodeColumnInC = 3*nodeID[i];
		}

		// Fill up CHt by computing the minimum matrix-vector operation
		for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
		{
			CHt[offset + CHt_line][firstLineID] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID][offset + 3*nodeID[i]+2];

			CHt[offset + CHt_line][firstLineID+1] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID+1][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID+1][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID+1][offset + 3*nodeID[i]+2];

			CHt[offset + CHt_line][firstLineID+2] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID+2][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID+2][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID+2][offset + 3*nodeID[i]+2];
		}
	}

	// Compute the current HCHt
	// NOTE: HCHt is symmetric => we compute the last line and reflect it on the last column
	for (int col=0 ; col<firstLineID ; col++)
	{
		double coef[3]= {0,0,0};

		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
		{
			coef[0] +=  H[firstLineID][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			            H[firstLineID][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			            H[firstLineID][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];

			coef[1] +=  H[firstLineID+1][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			            H[firstLineID+1][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			            H[firstLineID+1][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];

			coef[2] +=  H[firstLineID+2][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			            H[firstLineID+2][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			            H[firstLineID+2][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
		}

		HCHt[firstLineID  ][col] += coef[0];
		HCHt[col][firstLineID  ] += coef[0];

		HCHt[firstLineID+1][col] += coef[1];
		HCHt[col][firstLineID+1] += coef[1];

		HCHt[firstLineID+2][col] += coef[2];
		HCHt[col][firstLineID+2] += coef[2];
	}
	for (int col=firstLineID ; col<=firstLineID+2 ; col++)
	{
		double& coef1=HCHt[firstLineID  ][col];
		double& coef2=HCHt[firstLineID+1][col];
		double& coef3=HCHt[firstLineID+2][col];

		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
		{
			coef1 += H[firstLineID][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			         H[firstLineID][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			         H[firstLineID][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];

			coef2 += H[firstLineID+1][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			         H[firstLineID+1][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			         H[firstLineID+1][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];

			coef3 += H[firstLineID+2][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			         H[firstLineID+2][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			         H[firstLineID+2][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
		}
	}
}

//##################################################
//##################################################
// ### MLCP Bilateral Frictionless Sliding constraint (point sliding on a line)
//##################################################

void FEMTetrahedra::eraseTetMLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset , int firstLineID, const double* n1, const double* n2, double scale,
    int tetID, const double tetBaryCoord[4])
{
	const Tetrahedron* tet = getTetrahedron(tetID);
	int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

	// Erase entries in H
	for (int i=0 ; i<4 ; i++)
	{
		H[firstLineID][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID][offset + 3*nodeID[i]+2] = 0.0;

		H[firstLineID+1][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID+1][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID+1][offset + 3*nodeID[i]+2] = 0.0;
	}

	// Erase entries in CHt
	for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
	{
		CHt[offset + CHt_line][firstLineID] = 0.0;
		CHt[offset + CHt_line][firstLineID+1] = 0.0;
	}
}
void FEMTetrahedra::buildTetMLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, const double* n1, const double* n2, double scale,
    int tetID, const double tetBaryCoord[4], bool fillUpViolationONLY)
{
	// We should not have these sliding constraints on the FEM, but should we ever need them, code is included below.
	const Tetrahedron* tet = getTetrahedron(tetID);
	const Dynamic_Matrix<double>& C = getComplianceMatrixForLCP();
	int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

	Pt3D point;
	tet->calculateEmbeddedLocation(m_deformedMesh, tetBaryCoord, point);
	E[firstLineID  ] += scale * (n1[0]*point[0] + n1[1]*point[1] + n1[2]*point[2] + n1[3]);
	E[firstLineID+1] += scale * (n2[0]*point[0] + n2[1]*point[1] + n2[2]*point[2] + n2[3]);

	if (fillUpViolationONLY)
	{
		return;
	}

	// Fill up H with just the non null values
	for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
	{
		H[firstLineID  ][offset + 3*nodeID[i]+0] += scale*n1[0] * tetBaryCoord[i]; // n1x
		H[firstLineID  ][offset + 3*nodeID[i]+1] += scale*n1[1] * tetBaryCoord[i]; // n1y
		H[firstLineID  ][offset + 3*nodeID[i]+2] += scale*n1[2] * tetBaryCoord[i]; // n1z

		H[firstLineID+1][offset + 3*nodeID[i]+0] += scale*n2[0] * tetBaryCoord[i]; // n2x
		H[firstLineID+1][offset + 3*nodeID[i]+1] += scale*n2[1] * tetBaryCoord[i]; // n2y
		H[firstLineID+1][offset + 3*nodeID[i]+2] += scale*n2[2] * tetBaryCoord[i]; // n2z

		int nodeColumnInC;
		if (m_corotational)
		{
			nodeColumnInC = m_constrainedNodesToColumnsInWarpedC[nodeID[i]];
		}
		else
		{
			nodeColumnInC = 3*nodeID[i];
		}

		// Fill up CHt by computing the minimum matrix-vector operation
		for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
		{
			CHt[offset + CHt_line][firstLineID  ] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID  ][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID  ][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID  ][offset + 3*nodeID[i]+2];
			CHt[offset + CHt_line][firstLineID+1] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID+1][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID+1][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID+1][offset + 3*nodeID[i]+2];
		}
	}

	// Compute the current HCHt
	// NOTE: HCHt is symmetric => we compute the last line and reflect it on the last column
	for (int col=0 ; col<firstLineID ; col++)
	{
		double coef[2]= {0.0 , 0.0};

		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
		{
			coef[0] +=
			    H[firstLineID  ][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			    H[firstLineID  ][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			    H[firstLineID  ][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
			coef[1] +=
			    H[firstLineID+1][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			    H[firstLineID+1][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			    H[firstLineID+1][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
		}

		HCHt[firstLineID  ][col] += coef[0];
		HCHt[col][firstLineID  ] += coef[0];

		HCHt[firstLineID+1][col] += coef[1];
		HCHt[col][firstLineID+1] += coef[1];
	}
	for (int col=firstLineID ; col<=firstLineID+1 ; col++)
	{
		double& coef1=HCHt[firstLineID  ][col];
		double& coef2=HCHt[firstLineID+1][col];

		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
		{
			coef1 +=
			    H[firstLineID  ][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			    H[firstLineID  ][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			    H[firstLineID  ][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
			coef2 +=
			    H[firstLineID+1][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			    H[firstLineID+1][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			    H[firstLineID+1][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
		}
	}
}


//##################################################
//##################################################
// ### MLCP Bilateral Frictional Sliding constraint (point sliding on a line)
//##################################################

void FEMTetrahedra::eraseTetMLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset , int firstLineID, const double* n1, const double* n2, const double* t, double scale,
    int tetID, const double tetBaryCoord[4])
{
	const Tetrahedron* tet = getTetrahedron(tetID);
	int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

	// Erase entries in H
	for (int i=0 ; i<4 ; i++)
	{
		H[firstLineID][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID][offset + 3*nodeID[i]+2] = 0.0;

		H[firstLineID+1][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID+1][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID+1][offset + 3*nodeID[i]+2] = 0.0;

		H[firstLineID+2][offset + 3*nodeID[i]+0] = 0.0;
		H[firstLineID+2][offset + 3*nodeID[i]+1] = 0.0;
		H[firstLineID+2][offset + 3*nodeID[i]+2] = 0.0;
	}

	// Erase entries in CHt
	for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
	{
		CHt[offset + CHt_line][firstLineID] = 0.0;
		CHt[offset + CHt_line][firstLineID+1] = 0.0;
		CHt[offset + CHt_line][firstLineID+2] = 0.0;
	}
}


void FEMTetrahedra::buildTetMLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, const double* n1, const double* n2, const double* t, double scale,
    int tetID, const double tetBaryCoord[4], bool fillUpViolationONLY)
{
	const Tetrahedron* tet = getTetrahedron(tetID);
	const Dynamic_Matrix<double>& C = getComplianceMatrixForLCP();
	int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

	Pt3D point;
	tet->calculateEmbeddedLocation(m_deformedMesh, tetBaryCoord, point);
	E[firstLineID  ] += scale * (n1[0]*point[0] + n1[1]*point[1] + n1[2]*point[2] + n1[3]);
	E[firstLineID+1] += scale * (n2[0]*point[0] + n2[1]*point[1] + n2[2]*point[2] + n2[3]);
	E[firstLineID+2] += scale * (t[0]*point[0] +  t[1]*point[1] +  t[2]*point[2] +  t[3]);

	if (fillUpViolationONLY)
	{
		return;
	}

	// Fill up H with just the non null values
	for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
	{
		H[firstLineID][offset + 3*nodeID[i]+0] += scale*n1[0] * tetBaryCoord[i]; // n1x
		H[firstLineID][offset + 3*nodeID[i]+1] += scale*n1[1] * tetBaryCoord[i]; // n1y
		H[firstLineID][offset + 3*nodeID[i]+2] += scale*n1[2] * tetBaryCoord[i]; // n1z

		H[firstLineID+1][offset + 3*nodeID[i]+0] += scale*n2[0] * tetBaryCoord[i]; // n2x
		H[firstLineID+1][offset + 3*nodeID[i]+1] += scale*n2[1] * tetBaryCoord[i]; // n2y
		H[firstLineID+1][offset + 3*nodeID[i]+2] += scale*n2[2] * tetBaryCoord[i]; // n2z

		H[firstLineID+2][offset + 3*nodeID[i]+0] += scale*t[0] * tetBaryCoord[i]; // tx
		H[firstLineID+2][offset + 3*nodeID[i]+1] += scale*t[1] * tetBaryCoord[i]; // ty
		H[firstLineID+2][offset + 3*nodeID[i]+2] += scale*t[2] * tetBaryCoord[i]; // tz

		int nodeColumnInC;
		if (m_corotational)
		{
			nodeColumnInC = m_constrainedNodesToColumnsInWarpedC[nodeID[i]];
		}
		else
		{
			nodeColumnInC = 3*nodeID[i];
		}

		// Fill up CHt by computing the minimum matrix-vector operation
		for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
		{
			CHt[offset + CHt_line][firstLineID] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID][offset + 3*nodeID[i]+2];
			CHt[offset + CHt_line][firstLineID+1] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID+1][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID+1][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID+1][offset + 3*nodeID[i]+2];
			CHt[offset + CHt_line][firstLineID+2] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID+2][offset + 3*nodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID+2][offset + 3*nodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID+2][offset + 3*nodeID[i]+2];
		}
	}

	// Compute the current HCHt
	// NOTE: HCHt is symmetric => we compute the last line and reflect it on the last column
	for (int col=0 ; col<firstLineID ; col++)
	{
		double coef[3]= {0,0,0};

		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
		{
			coef[0] +=  H[firstLineID][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			            H[firstLineID][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			            H[firstLineID][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
			coef[1] +=  H[firstLineID+1][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			            H[firstLineID+1][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			            H[firstLineID+1][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
			coef[2] +=  H[firstLineID+2][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			            H[firstLineID+2][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			            H[firstLineID+2][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
		}

		HCHt[firstLineID  ][col] += coef[0];
		HCHt[col][firstLineID  ] += coef[0];

		HCHt[firstLineID+1][col] += coef[1];
		HCHt[col][firstLineID+1] += coef[1];

		HCHt[firstLineID+2][col] += coef[2];
		HCHt[col][firstLineID+2] += coef[2];
	}
	for (int col=firstLineID ; col<=firstLineID+2 ; col++)
	{
		double& coef1=HCHt[firstLineID  ][col];
		double& coef2=HCHt[firstLineID+1][col];
		double& coef3=HCHt[firstLineID+2][col];

		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
		{
			coef1 +=
			    H[firstLineID  ][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			    H[firstLineID  ][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			    H[firstLineID  ][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
			coef2 +=
			    H[firstLineID+1][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			    H[firstLineID+1][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			    H[firstLineID+1][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
			coef3 +=
			    H[firstLineID+2][offset + 3*nodeID[i]+0] * CHt[offset + 3*nodeID[i]+0][col] +
			    H[firstLineID+2][offset + 3*nodeID[i]+1] * CHt[offset + 3*nodeID[i]+1][col] +
			    H[firstLineID+2][offset + 3*nodeID[i]+2] * CHt[offset + 3*nodeID[i]+2][col];
		}
	}
}

//##################################################
//##################################################
// ### MLCP Bilateral Frictional Sliding constraint (point sliding on a line)
//##################################################

void FEMTetrahedra::eraseTetMLCP_BILATERAL_DIRECTION_CONSTRAINT(
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    int offset, int firstLineID, double scale,
    const double n1[3], const double n2[3], int fromTetID, const double fromTetBaryCoord[4], int toTetID, const double toTetBaryCoord[4])
{
	// Erase for both from and to tetIDs.
	{
		const Tetrahedron* tet = getTetrahedron(fromTetID);
		int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

		// Erase entries in H
		for (int i=0 ; i<4 ; i++)
		{
			H[firstLineID][offset + 3*nodeID[i]+0] = 0.0;
			H[firstLineID][offset + 3*nodeID[i]+1] = 0.0;
			H[firstLineID][offset + 3*nodeID[i]+2] = 0.0;

			H[firstLineID+1][offset + 3*nodeID[i]+0] = 0.0;
			H[firstLineID+1][offset + 3*nodeID[i]+1] = 0.0;
			H[firstLineID+1][offset + 3*nodeID[i]+2] = 0.0;

			H[firstLineID+2][offset + 3*nodeID[i]+0] = 0.0;
			H[firstLineID+2][offset + 3*nodeID[i]+1] = 0.0;
			H[firstLineID+2][offset + 3*nodeID[i]+2] = 0.0;
		}
	}
	{
		const Tetrahedron* tet = getTetrahedron(toTetID);
		int nodeID[4]= {tet->getPointID(0),tet->getPointID(1),tet->getPointID(2),tet->getPointID(3)};

		// Erase entries in H
		for (int i=0 ; i<4 ; i++)
		{
			H[firstLineID][offset + 3*nodeID[i]+0] = 0.0;
			H[firstLineID][offset + 3*nodeID[i]+1] = 0.0;
			H[firstLineID][offset + 3*nodeID[i]+2] = 0.0;

			H[firstLineID+1][offset + 3*nodeID[i]+0] = 0.0;
			H[firstLineID+1][offset + 3*nodeID[i]+1] = 0.0;
			H[firstLineID+1][offset + 3*nodeID[i]+2] = 0.0;

			H[firstLineID+2][offset + 3*nodeID[i]+0] = 0.0;
			H[firstLineID+2][offset + 3*nodeID[i]+1] = 0.0;
			H[firstLineID+2][offset + 3*nodeID[i]+2] = 0.0;
		}
	}


	// Erase entries in CHt
	for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
	{
		CHt[offset + CHt_line][firstLineID] = 0.0;
		CHt[offset + CHt_line][firstLineID+1] = 0.0;
		CHt[offset + CHt_line][firstLineID+2] = 0.0;
	}
}

void FEMTetrahedra::buildTetMLCP_BILATERAL_DIRECTION_CONSTRAINT(
    Dynamic_Vector<double>& E,
    Dynamic_Matrix<double>& H,
    Dynamic_Matrix<double>& CHt,
    Dynamic_Matrix<double>& HCHt,
    int offset, int firstLineID, double scale,
    const double n1[3], const double n2[3], int fromTetID, const double fromTetBaryCoord[4], int toTetID, const double toTetBaryCoord[4], bool fillUpViolationONLY)
{
	const Dynamic_Matrix<double>& C = getComplianceMatrixForLCP();

	// Get both point positions.
	const Tetrahedron* fromTet = getTetrahedron(fromTetID);
	int fromNodeID[4]= {fromTet->getPointID(0),fromTet->getPointID(1),fromTet->getPointID(2),fromTet->getPointID(3)};
	Pt3D fromPoint;
	fromTet->calculateEmbeddedLocation(m_deformedMesh, fromTetBaryCoord, fromPoint);

	const Tetrahedron* toTet = getTetrahedron(toTetID);
	int toNodeID[4]= {toTet->getPointID(0),toTet->getPointID(1),toTet->getPointID(2),toTet->getPointID(3)};
	Pt3D toPoint;
	toTet->calculateEmbeddedLocation(m_deformedMesh, toTetBaryCoord, toPoint);

	E[firstLineID  ] += (vec_generic_dotProduct<double,double,double>(toPoint , n1 , 3) -
	                     vec_generic_dotProduct<double,double,double>(fromPoint , n1 , 3))*scale;
	E[firstLineID+1] += (vec_generic_dotProduct<double,double,double>(toPoint , n2 , 3) -
	                     vec_generic_dotProduct<double,double,double>(fromPoint , n2 , 3))*scale;

	if (fillUpViolationONLY)
	{
		return;
	}

	// Fill up H with just the non null values
	for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the from tet !
	{
		H[firstLineID][offset + 3*fromNodeID[i]+0] -= scale*n1[0] * fromTetBaryCoord[i]; // n1x
		H[firstLineID][offset + 3*fromNodeID[i]+1] -= scale*n1[1] * fromTetBaryCoord[i]; // n1y
		H[firstLineID][offset + 3*fromNodeID[i]+2] -= scale*n1[2] * fromTetBaryCoord[i]; // n1z

		H[firstLineID+1][offset + 3*fromNodeID[i]+0] -= scale*n2[0] * fromTetBaryCoord[i]; // n2x
		H[firstLineID+1][offset + 3*fromNodeID[i]+1] -= scale*n2[1] * fromTetBaryCoord[i]; // n2y
		H[firstLineID+1][offset + 3*fromNodeID[i]+2] -= scale*n2[2] * fromTetBaryCoord[i]; // n2z
	}
	for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the to tet !
	{
		H[firstLineID][offset + 3*toNodeID[i]+0] += scale*n1[0] * toTetBaryCoord[i]; // n1x
		H[firstLineID][offset + 3*toNodeID[i]+1] += scale*n1[1] * toTetBaryCoord[i]; // n1y
		H[firstLineID][offset + 3*toNodeID[i]+2] += scale*n1[2] * toTetBaryCoord[i]; // n1z

		H[firstLineID+1][offset + 3*toNodeID[i]+0] += scale*n2[0] * toTetBaryCoord[i]; // n2x
		H[firstLineID+1][offset + 3*toNodeID[i]+1] += scale*n2[1] * toTetBaryCoord[i]; // n2y
		H[firstLineID+1][offset + 3*toNodeID[i]+2] += scale*n2[2] * toTetBaryCoord[i]; // n2z
	}


	for (int i=0 ; i<4 ; i++)
	{
		int nodeColumnInC;
		if (m_corotational)
		{
			nodeColumnInC = m_constrainedNodesToColumnsInWarpedC[fromNodeID[i]];
		}
		else
		{
			nodeColumnInC = 3*fromNodeID[i];
		}

		// Fill up CHt by computing the minimum matrix-vector operation
		for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
		{
			CHt[offset + CHt_line][firstLineID] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID][offset + 3*fromNodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID][offset + 3*fromNodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID][offset + 3*fromNodeID[i]+2];
			CHt[offset + CHt_line][firstLineID+1] +=
			    C[CHt_line][nodeColumnInC+0]*H[firstLineID+1][offset + 3*fromNodeID[i]+0] +
			    C[CHt_line][nodeColumnInC+1]*H[firstLineID+1][offset + 3*fromNodeID[i]+1] +
			    C[CHt_line][nodeColumnInC+2]*H[firstLineID+1][offset + 3*fromNodeID[i]+2];
		}
	}
	if (fromTetID != toTetID)
	{
		for (int i=0 ; i<4 ; i++)
		{
			int nodeColumnInC;
			if (m_corotational)
			{
				nodeColumnInC = m_constrainedNodesToColumnsInWarpedC[toNodeID[i]];
			}
			else
			{
				nodeColumnInC = 3*toNodeID[i];
			}

			// Fill up CHt by computing the minimum matrix-vector operation
			for (int CHt_line=0 ; CHt_line<getNbDOF() ; CHt_line++)
			{
				CHt[offset + CHt_line][firstLineID] +=
				    C[CHt_line][nodeColumnInC+0]*H[firstLineID][offset + 3*toNodeID[i]+0] +
				    C[CHt_line][nodeColumnInC+1]*H[firstLineID][offset + 3*toNodeID[i]+1] +
				    C[CHt_line][nodeColumnInC+2]*H[firstLineID][offset + 3*toNodeID[i]+2];
				CHt[offset + CHt_line][firstLineID+1] +=
				    C[CHt_line][nodeColumnInC+0]*H[firstLineID+1][offset + 3*toNodeID[i]+0] +
				    C[CHt_line][nodeColumnInC+1]*H[firstLineID+1][offset + 3*toNodeID[i]+1] +
				    C[CHt_line][nodeColumnInC+2]*H[firstLineID+1][offset + 3*toNodeID[i]+2];
			}
		}
	}

	// Compute the current HCHt
	// NOTE: HCHt is symmetric => we compute the last line and reflect it on the last column
	for (int col=0 ; col<firstLineID ; col++)
	{
		double coef[2]= {0,0};

		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the from tet !
		{
			coef[0] +=  H[firstLineID][offset + 3*fromNodeID[i]+0] * CHt[offset + 3*fromNodeID[i]+0][col] +
			            H[firstLineID][offset + 3*fromNodeID[i]+1] * CHt[offset + 3*fromNodeID[i]+1][col] +
			            H[firstLineID][offset + 3*fromNodeID[i]+2] * CHt[offset + 3*fromNodeID[i]+2][col];
			coef[1] +=  H[firstLineID+1][offset + 3*fromNodeID[i]+0] * CHt[offset + 3*fromNodeID[i]+0][col] +
			            H[firstLineID+1][offset + 3*fromNodeID[i]+1] * CHt[offset + 3*fromNodeID[i]+1][col] +
			            H[firstLineID+1][offset + 3*fromNodeID[i]+2] * CHt[offset + 3*fromNodeID[i]+2][col];
		}
		if (fromTetID != toTetID)
		{
			for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the to tet !
			{
				coef[0] +=  H[firstLineID][offset + 3*toNodeID[i]+0] * CHt[offset + 3*toNodeID[i]+0][col] +
				            H[firstLineID][offset + 3*toNodeID[i]+1] * CHt[offset + 3*toNodeID[i]+1][col] +
				            H[firstLineID][offset + 3*toNodeID[i]+2] * CHt[offset + 3*toNodeID[i]+2][col];
				coef[1] +=  H[firstLineID+1][offset + 3*toNodeID[i]+0] * CHt[offset + 3*toNodeID[i]+0][col] +
				            H[firstLineID+1][offset + 3*toNodeID[i]+1] * CHt[offset + 3*toNodeID[i]+1][col] +
				            H[firstLineID+1][offset + 3*toNodeID[i]+2] * CHt[offset + 3*toNodeID[i]+2][col];
			}
		}

		HCHt[firstLineID  ][col] += coef[0];
		HCHt[col][firstLineID  ] += coef[0];

		HCHt[firstLineID+1][col] += coef[1];
		HCHt[col][firstLineID+1] += coef[1];
	}
	for (int col=firstLineID ; col<=firstLineID+1 ; col++)
	{
		double& coef1=HCHt[firstLineID  ][col];
		double& coef2=HCHt[firstLineID+1][col];

		for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
		{
			coef1 += H[firstLineID][offset + 3*fromNodeID[i]+0] * CHt[offset + 3*fromNodeID[i]+0][col] +
			         H[firstLineID][offset + 3*fromNodeID[i]+1] * CHt[offset + 3*fromNodeID[i]+1][col] +
			         H[firstLineID][offset + 3*fromNodeID[i]+2] * CHt[offset + 3*fromNodeID[i]+2][col];
			coef2 += H[firstLineID+1][offset + 3*fromNodeID[i]+0] * CHt[offset + 3*fromNodeID[i]+0][col] +
			         H[firstLineID+1][offset + 3*fromNodeID[i]+1] * CHt[offset + 3*fromNodeID[i]+1][col] +
			         H[firstLineID+1][offset + 3*fromNodeID[i]+2] * CHt[offset + 3*fromNodeID[i]+2][col];
		}
		if (fromTetID != toTetID)
		{
			for (int i=0 ; i<4 ; i++) // Loop over the 4 nodes of the tet !
			{
				coef1 += H[firstLineID][offset + 3*toNodeID[i]+0] * CHt[offset + 3*toNodeID[i]+0][col] +
				         H[firstLineID][offset + 3*toNodeID[i]+1] * CHt[offset + 3*toNodeID[i]+1][col] +
				         H[firstLineID][offset + 3*toNodeID[i]+2] * CHt[offset + 3*toNodeID[i]+2][col];
				coef2 += H[firstLineID+1][offset + 3*toNodeID[i]+0] * CHt[offset + 3*toNodeID[i]+0][col] +
				         H[firstLineID+1][offset + 3*toNodeID[i]+1] * CHt[offset + 3*toNodeID[i]+1][col] +
				         H[firstLineID+1][offset + 3*toNodeID[i]+2] * CHt[offset + 3*toNodeID[i]+2][col];
			}
		}
	}
}

void FEMTetrahedra::setConstrainedNodes(const std::set<int>& nodes)
{
	m_constrainedNodesToColumnsInWarpedC.clear();
	int column = 0;
	for (std::set<int>::const_iterator it = nodes.begin(); it != nodes.end(); it++)
	{
		m_constrainedNodesToColumnsInWarpedC[*it] = column;
		column += 3;
	}
}

void FEMTetrahedra::computePartialWarpedComplianceMatrix()
{
	const int nbDOF = 3 * m_nbNodes;

	m_warpedC.resizeUpIfNecessary(nbDOF, 3 * (unsigned int)m_constrainedNodesToColumnsInWarpedC.size());

	const Dynamic_Matrix<double>& C = getInitialComplianceMatrix();

	for (std::map<int, int>::iterator it = m_constrainedNodesToColumnsInWarpedC.begin(); it != m_constrainedNodesToColumnsInWarpedC.end(); it++)
	{
		int node = it->first;
		int column = 3 * node;
		int outputColumn = it->second;

		// Build the 3 columns of RC for this node.
		static Dynamic_Matrix<double> partialRC(nbDOF, 3);

		for (int n = 0; n < m_nbNodes; n++)
		{
			Static_Matrix<double, 3, 3>& rotation = m_rotationPerNode[n];

			int row = 3 * n;

			// Multiply rotation by the 3x3 submatrix of C.
			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 3; c++)
				{
					partialRC[row + r][c] = 0.0;
					for (int i = 0; i < 3; i++)
					{
						partialRC[row + r][c] += rotation[r][i] * C[row + i][column + c];
					}
				}
			}
		}

		// Now multiply the partial RC with Rt to get the partial warped C.
		Static_Matrix<double, 3, 3>& rotation = m_rotationPerNode[node];
		for (int r = 0; r < 3*m_nbNodes; r++)
		{
			for (int c = 0; c < 3; c++)
			{
				m_warpedC[r][outputColumn+c] = 0.0;
				for (int i = 0; i < 3; i++)
				{
					m_warpedC[r][outputColumn+c] += partialRC[r][i] * rotation[c][i]; // Rt[r][c] = R[c][r]
				}
			}
		}
	}
}

const Dynamic_Matrix<double>& FEMTetrahedra::getComplianceMatrixForLCP()
{
	if (m_corotational)
	{
		return m_warpedC;
	}
	else
	{
		return getComplianceMatrix();
	}
}