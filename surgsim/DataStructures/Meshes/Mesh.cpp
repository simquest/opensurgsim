#include "Mesh.h"

using SurgSim::DataStructures::Mesh;
using SurgSim::Math::Vector3d;

Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

void Mesh::loadFrom_PlainTriMesh(string name,
	int nbPts,
	const double *pts,
	int nbTri,
	const int *tri_ptID,
	double scale)
{
	int nbVertices = nbPts;
	int nbTriangles = nbTri;

	// Set the transformation
	const btQuaternion quaternion( 0, 0, 0, 1 );
	const btVector3 origin( 0 , 0 , 0 );
	m_from.setOrigin(origin);
	m_from.setRotation(quaternion);
	m_to.setOrigin(origin);
	m_to.setRotation(quaternion);

	//char buf[256];
	//sprintf(buf,"loadFrom_PlainTriMesh nbPts=%d nbTri=%d",nbPts,nbTri);
	m_name = name;//string(buf);
	// 1) Create all the vertices
	for(int vv=0 ; vv< nbVertices ; ++vv)
	{
		double p[3];
		p[0] = pts[3*vv  ] * scale;
		p[1] = pts[3*vv+1] * scale;
		p[2] = pts[3*vv+2] * scale;
		// Triangles are defined relative to the Node id...so we need to create all node as is !
		// If we test if the node already exist, we will screw up the nodeID in the triangle list...
		/*if( doesThisVertexAlreadyExist(p)==-1 ) */
		createNewVertex(p);
	}

	// 2) Create all edges and triangles
	for(int tt=0 ; tt<nbTriangles ; ++tt)
	{
		int vID[3]={ tri_ptID[3*tt] , tri_ptID[3*tt+1] , tri_ptID[3*tt+2] };

		int eID[3]={-1,-1,-1};
		if( (eID[0]=doesThisEdgeAlreadyExist(vID[0],vID[1]))==-1 ) eID[0]=createNewEdge(vID[0],vID[1]);
		if( (eID[1]=doesThisEdgeAlreadyExist(vID[0],vID[2]))==-1 ) eID[1]=createNewEdge(vID[0],vID[2]);
		if( (eID[2]=doesThisEdgeAlreadyExist(vID[1],vID[2]))==-1 ) eID[2]=createNewEdge(vID[1],vID[2]);

		createNewTriangle(vID,eID);
	}

	internalProcessWithVertexAndTriangle();
}

void Mesh::loadFromParticles(string name, int nbPts, const double *pts)
{
	// Set the transformation
	m_from.setIdentity();
	m_to.setIdentity();

	//char buf[256];
	//sprintf(buf,"loadFromParticles nbPts=%d",nbPts);
	m_name = name;//string(buf);
	// 1) Create all the vertices
	double p[3];
	for(int vv = 0 ; vv < nbPts ; ++vv)
	{
		p[0] = pts[3*vv+0];
		p[1] = pts[3*vv+1];
		p[2] = pts[3*vv+2];
		createNewVertex(p);
	}

	// 2) Create all edges and triangles
	int nbTriangles = nbPts/3;
	int vID[3]={-1,-1,-1};
	for(int tt = 0; tt < nbTriangles; ++tt)
	{
		vID[0]= tt*3+0;
		vID[1]= tt*3+1;
		vID[2]= tt*3+2;

		int eID[3]={-1,-1,-1};
		if( (eID[0]=doesThisEdgeAlreadyExist(vID[0],vID[1]))==-1 ) eID[0]=createNewEdge(vID[0],vID[1]);
		if( (eID[1]=doesThisEdgeAlreadyExist(vID[0],vID[2]))==-1 ) eID[1]=createNewEdge(vID[0],vID[2]);
		if( (eID[2]=doesThisEdgeAlreadyExist(vID[1],vID[2]))==-1 ) eID[2]=createNewEdge(vID[1],vID[2]);

		createNewTriangle(vID,eID);
	}

	internalProcessWithVertexAndTriangle();
}

void Mesh::internalProcessWithVertexAndTriangle()
{
	// 3) Compute normals (vertex, edge, triangle)
	computeNormal();

	// 4) Compute triangle's neighbors
	for (int tID = 0;  tID < numTriangles();  ++tID)
	{
		TriangleTopology& t        = mutableTriangleTopology(tID);
		const EdgeTopology&     e0 = getEdgeTopology(t.edgeID[0]);
		const EdgeTopology&     e1 = getEdgeTopology(t.edgeID[1]);
		const EdgeTopology&     e2 = getEdgeTopology(t.edgeID[2]);
		const VertexTopology&   v0 = getVertex(t.vertexID[0]);
		const VertexTopology&   v1 = getVertex(t.vertexID[1]);
		const VertexTopology&   v2 = getVertex(t.vertexID[2]);

		//if(e0.nbTriangles>2)
		//	cerr << "Mesh::formTrianglesSTL error while computing triangle "<<tID<<"'s neighborhood. Edge " << t.edgeID[0] << " has more than 2 triangles !" << endl;
		//else
		{
			for(int i=0 ; i<e0.nbTriangles ; i++)
				if(e0.triangleIDs[i]!=t.ID) t.triangleID_sharing_edge[0]=e0.triangleIDs[i];
		}

		//if(e1.nbTriangles>2)
		//	cerr << "Mesh::formTrianglesSTL error while computing triangle "<<tID<<"'s neighborhood. Edge " << t.edgeID[1] << " has more than 2 triangles !" << endl;
		//else
		{
			for(int i=0 ; i<e1.nbTriangles ; i++)
				if(e1.triangleIDs[i]!=t.ID) t.triangleID_sharing_edge[1]=e1.triangleIDs[i];
		}

		//if(e2.nbTriangles>2)
		//	cerr << "Mesh::formTrianglesSTL error while computing triangle "<<tID<<"'s neighborhood. Edge " << t.edgeID[2] << " has more than 2 triangles !" << endl;
		//else
		{
			for(int i=0 ; i<e2.nbTriangles ; i++)
				if(e2.triangleIDs[i]!=t.ID) t.triangleID_sharing_edge[2]=e2.triangleIDs[i];
		}

		for(int i=0 ; i<v0.nbTriangles ; i++)
		{
			if(v0.triangleIDs[i]!=t.ID)
			{
				t.triangleIDs_sharing_vertexID[0][ t.nbTriangle_sharing_vertexID[0] ] = v0.triangleIDs[i];
				t.nbTriangle_sharing_vertexID[0]++;
			}
		}

		for(int i=0 ; i<v1.nbTriangles ; i++)
		{
			if(v1.triangleIDs[i]!=t.ID)
			{
				t.triangleIDs_sharing_vertexID[1][ t.nbTriangle_sharing_vertexID[1] ] = v1.triangleIDs[i];
				t.nbTriangle_sharing_vertexID[1]++;
			}
		}

		for(int i=0 ; i<v2.nbTriangles ; i++)
		{
			if(v2.triangleIDs[i]!=t.ID)
			{
				t.triangleIDs_sharing_vertexID[2][ t.nbTriangle_sharing_vertexID[2] ] = v2.triangleIDs[i];
				t.nbTriangle_sharing_vertexID[2]++;
			}
		}
	}
}

void Mesh::setVertexPositions(const std::vector<Vector3d>& positions, bool doUpdate)
{
	// TODO: assert that the number of positions matches vertices
	m_vertexPositions = positions;

	update();
}

void Mesh::setVertexPositions(const Mesh &mesh, bool doUpdate)
{
	// TODO: check that meshes have same number of vertices

	m_vertexPositions = mesh.getVertexPositions();

	update();
}

void Mesh::setVertexPositions(const Mesh& mesh1, double percent1, const Mesh& mesh2, double percent2, bool doUpdate)
{
	// TODO: check that meshes have same number of vertices

	for (unsigned int i = 0; i < m_vertexPositions.size(); ++i)
	{
		m_vertexPositions[i] = mesh1.getVertexPosition(i) * percent1 + mesh2.getVertexPosition(i) * percent2;
	}

	update();
}
