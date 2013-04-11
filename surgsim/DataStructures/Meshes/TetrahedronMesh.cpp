#include "Mesh.h"

#include <SurgSim/DataStructures/Edge.h>
#include <SurgSim/DataStructures/Triangle.h>
#include <SurgSim/DataStructures/Vertex.h>

#include <math.h>
#include <fstream>

using SurgSim::DataStructures::Edge;
using SurgSim::DataStructures::Mesh;
using SurgSim::DataStructures::Triangle;
using SurgSim::DataStructures::Vertex;
using SurgSim::Math::Vector3d;

Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

void Mesh::reset()
{
	m_vertices.clear();
	m_edges.clear();
	m_triangles.clear();
}

static inline bool sameEdge(unsigned int edge1vertex1, unsigned int edge1vertex2, 
	unsigned int edge2vertex1, unsigned int edge2vertex2)
{ 
	return ( edge1vertex1 == edge2vertex1 && edge1vertex2 == edge2vertex2 ) || 
		( edge1vertex1 == edge2vertex2 && edge1vertex2 == edge2vertex1 );
}

static inline bool sharesEdge(const Triangle& triangle1, const Triangle& triangle2)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (triangle1.edges[i] == triangle2.edges[j])
			{
				return true;
			}
		}
	}
}

unsigned int Mesh::createNewVertex(const Vector3d& position)
{
	Vertex vertex(position);

	m_vertices.push_back(vertex);

	return m_vertices.size() - 1;
}

unsigned int Mesh::createNewEdge(unsigned int vertex0, unsigned int vertex1)
{
	Edge edge(vertex0, vertex1);
	m_edges.push_back(edge);

	return m_edges.size() - 1;
}

unsigned int Mesh::createNewTriangle(unsigned int vertices[3], unsigned int edges[3])
{
	Triangle triangle(vertices, edges);

	m_triangles.push_back(triangle);

	unsigned int triangleId = m_triangles.size() - 1;

	for (int i = 0; i < 3; ++i)
	{
		registerTriangleInVertex(vertices[i] , triangleId);
		registerTriangleInEdge(edges[i], triangleId);
	}

	registerEdgeInVertex(vertices[0], edges[0]); 
	registerEdgeInVertex(vertices[0], edges[1]);

	registerEdgeInVertex(vertices[1], edges[0]);
	registerEdgeInVertex(vertices[1], edges[2]);

	registerEdgeInVertex(vertices[2], edges[1]);
	registerEdgeInVertex(vertices[2], edges[2]);

	return triangleId;
}

void Mesh::registerTriangleInVertex(unsigned int vertexId, unsigned int triangleId)
{
	Vertex& vertex = m_vertices[vertexId];

	// Look up first if the triangleId is already registered!
	for (auto it = vertex.triangles.cbegin(); it != vertex.triangles.cend(); ++it)
	{
		if (*it == triangleId)
		{
			return;
		}
	}

	vertex.triangles.push_back(triangleId);
}

void Mesh::registerEdgeInVertex(unsigned int vertexId, unsigned int edgeId)
{
	Vertex& vertex = m_vertices[vertexId];

	// Look up first if the triangleId is already registered !
	for (auto it = vertex.edges.cbegin(); it != vertex.edges.cend(); ++it)
	{
		if (*it == edgeId)
		{
			return;
		}
	}

	vertex.edges.push_back(edgeId);
}

void Mesh::loadFromSTLFile
	(
	const char *filename,
	const double trans[3], const double orientationQuat[4]
) 
{
	int eof=0, counter=0; 
	double cxyz[3][3];
	double normal[3];
	//double normalX,normalY,normalZ;
	double offsetX = 0.0;		// NO OFFSET IN X
	double offsetY = 0.0;		// NO OFFSET IN Y
	double offsetZ = 0.0;		// NO OFFSET IN Z
	double m_Theta=0;			// NO ROTATION AROUND Y !!
	double m_ScaleFactor=1.0;	// NO SCALING FACTOR !!


	double cosine = cos(m_Theta*M_PI/180.0);
	double sine   = sin(m_Theta*M_PI/180.0);

	char s1[256], s2[256];
	FILE *fileSTL; 

	// Bounding Box 
	double gridMin[3], gridMax[3];
	gridMin[0] = DBL_MAX; gridMax[0] = -DBL_MAX;
	gridMin[1] = DBL_MAX; gridMax[1] = -DBL_MAX;
	gridMin[2] = DBL_MAX; gridMax[2] = -DBL_MAX;

	m_from = btTransform( btQuaternion(orientationQuat[0],orientationQuat[1],orientationQuat[2],orientationQuat[3]) , btVector3(trans[0],trans[1],trans[2]) );
	m_to   = m_from;

	m_name = string(filename);
	fprintf(stderr,"opening input file %s \n",filename);

	fileSTL = fopen(filename, "r");
	if(fileSTL == NULL)
	{
		//EcWARN("ERROR in opening input file \n"); 
		exit(0);
	}

	// Anything before the solid definition can be skipped !
	do{
		eof = fscanf(fileSTL,"%s",s1);
	}while( eof!=EOF && strcmp(s1,"solid")!=0 );

	if( eof==EOF ) return;

	// Here, we entered the 'solid' definition.
	// 1st thing, the solid name...
	char buf[1024];
	fgets(buf,1024,fileSTL);
	eof = feof(fileSTL);
	//eof = fscanf(fileSTL,"%s",s1);

	while(eof != EOF)
	{
		// read 'facet normal nx ny nz' 
		eof = fscanf(fileSTL, "%s %s %lf %lf %lf",s1,s2,&normal[0],&normal[1],&normal[2]);
		if( strcmp(s1,"endsolid")!=0 && (strcmp(s1,"facet")!=0 || strcmp(s2,"normal")!=0) )
		{
			char buf[256]="";
			sprintf(buf,"Mesh STL Parser found unexpected tokens %s %s instead of 'facet normal' or 'endsolid ...'   aborting the loading of the file %s\n",s1,s2,filename);
			SQ_WARNING(buf);
			this->reset();
			return;
		}

		if(strcmp(s1,"endsolid")!=0)
		{
			// add the facet normal vector
			//SqVector normal=SqVector(normalX,normalY,normalZ);

			int vID[3]={-1,-1,-1};
			int eID[3]={-1,-1,-1};

			eof = fscanf(fileSTL, "%s %s",s1,s2); // outer loop
			if(strcmp(s1,"outer")!=0 || strcmp(s2,"loop")!=0)
			{
				char buf[256]="";
				sprintf(buf,"Mesh STL Parser found unexpected tokens %s %s instead of 'outer loop'....aborting the loading of the file %s\n",s1,s2,filename);
				SQ_WARNING(buf);
				this->reset();
				return;
			}

			for(int jj = 0; jj < 3; ++jj)
			{
				// real vertices's point
				eof = fscanf(fileSTL, "%s %lf %lf %lf", s1,&cxyz[jj][0], &cxyz[jj][1], &cxyz[jj][2]);
				if(strcmp(s1,"vertex")!=0 )
				{
					char buf[256]="";
					sprintf(buf,"Mesh STL Parser found unexpected tokens %s instead of 'vertex'....aborting the loading of the file %s\n",s1,filename);
					SQ_WARNING(buf);
					this->reset();
					return;
				}

				double transf_x = cxyz[jj][0]*cosine - cxyz[jj][2]*sine;
				double transf_y = cxyz[jj][1];
				double transf_z = cxyz[jj][0]*sine   + cxyz[jj][2]*cosine;

				/*double transf_x = cxyz[jj][0]*cosine - cxyz[jj][2]*sine;
				double transf_y = cxyz[jj][0]*sine   + cxyz[jj][2]*cosine;
				double transf_z = cxyz[jj][1];*/

				//m_XyzVertices[counter*3+jj].x = transf_x*m_ScaleFactor + offsetX;
				//m_XyzVertices[counter*3+jj].y = transf_y*m_ScaleFactor + offsetY;
				//m_XyzVertices[counter*3+jj].z = transf_z*m_ScaleFactor + offsetZ;
				double p[3];
				p[0] = transf_x*m_ScaleFactor + offsetX;
				p[1] = transf_y*m_ScaleFactor + offsetY;
				p[2] = transf_z*m_ScaleFactor + offsetZ;

				auto existingIt = std::find_if(m_vertices.begin(), m_vertices.end(), Vertex::PositionComparator(p));
				if (existingIt == m_vertices.end())
				{
					vID[jj]=doesThisVertexAlreadyExist(p);
					if( vID[jj]==-1 ) vID[jj] = createNewVertex(p);

					if(eof != EOF)
					{ 
						for(int kk = 0; kk < 3; ++kk)
						{
							if(cxyz[jj][kk]*m_ScaleFactor > gridMax[kk]) 
							{
								gridMax[kk] = cxyz[jj][kk]*m_ScaleFactor;
							}
							if(cxyz[jj][kk]*m_ScaleFactor < gridMin[kk]) 
							{
								gridMin[kk] = cxyz[jj][kk]*m_ScaleFactor;
							}
						}
					}	
				}
				counter++;

				// Verify that the triangle is defined CCW
				// (v0v1 crossProduct v0v2).normal > 0
				{
					const double* v0 = getVertexLocalPosition(vID[0]);
					const double* v1 = getVertexLocalPosition(vID[1]);
					const double* v2 = getVertexLocalPosition(vID[2]);
					double v0v1[3]={ v1[0]-v0[0] , v1[1]-v0[1] , v1[2]-v0[2] };
					double v0v2[3]={ v2[0]-v0[0] , v2[1]-v0[1] , v2[2]-v0[2] };
					double v0v1_v0v2[3]={
						v0v1[1]*v0v2[2] - v0v1[2]*v0v2[1],
						v0v1[2]*v0v2[0] - v0v1[0]*v0v2[2],
						v0v1[0]*v0v2[1] - v0v1[1]*v0v2[0]
					};
					if( v0v1_v0v2[0]*normal[0] + v0v1_v0v2[1]*normal[1] + v0v1_v0v2[2]*normal[2]<0 )
					{
						cerr << "Mesh not defined CCW" << endl;
					}
				}

				// Retrieve the edges info
				eID[0]=doesThisEdgeAlreadyExist( vID[0] , vID[1] );
				if( eID[0]==-1 ) eID[0] = createNewEdge( vID[0] , vID[1] );

				eID[1]=doesThisEdgeAlreadyExist( vID[0] , vID[2] );
				if( eID[1]==-1 ) eID[1] = createNewEdge( vID[0] , vID[2] );

				eID[2]=doesThisEdgeAlreadyExist( vID[1] , vID[2] );
				if( eID[2]==-1 ) eID[2] = createNewEdge( vID[1] , vID[2] );

				// Create the triangle info
				int triID = createNewTriangle(vID, eID);

				eof = fscanf(fileSTL, "%s", s1); // endloop
				eof = fscanf(fileSTL, "%s", s2); // endfacet
				if(strcmp(s1,"endloop")!=0 || strcmp(s2,"endfacet")!=0)
				{
					char buf[256]="";
					sprintf(buf,"Mesh STL Parser found unexpected tokens %s %s instead of 'endloop endfacet'....aborting the loading of the file %s\n",s1,s2,filename);
					SQ_WARNING(buf);
					this->reset();
					return;
				}
			}
			else
			{
				fclose(fileSTL);

				// set the end of file
				eof = EOF;

				double averageX = (gridMax[0]-gridMin[0])/2;
				double averageY = (gridMax[1]-gridMin[1])/2;
				double averageZ = (gridMax[2]-gridMin[2])/2;
			}
		}

		internalProcessWithVertexAndTriangle();
	}
}
void Mesh::writeToSTLFile(const std::string& filename) const
{
	ofstream output(filename);

	output << "solid " << getName() << std::endl;

	for (int t = 0; t < numTriangles(); ++t)
	{
		const double* normal = getTriangleLocalNormal(t);
		output << "facet normal " << normal[0] << " " << normal[1] << " " << normal[2] << std::endl;

		output << "outer loop" << std::endl;

		const TriangleTopology& triangle = getTriangleTopology(t);
		for (int v = 0; v < 3; ++v)
		{
			const double* position = getVertexLocalPosition(triangle.vertexID[v]);
			output << "vertex " << position[0] << " " << position[1] << " " << position[2] << std::endl;
		}

		output << "endloop" << std::endl;
		output << "endfacet" << std::endl;
	}

	output << "endsolid " << getName() << std::endl;

	output.close();
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

void Mesh::doUpdateTriangles()
{
	for (auto it = m_triangles.begin(); it != m_triangles.end(); ++it)
	{
		(*it)->update(*this);
	}
}

void Mesh::doUpdateEdges()
{
	for (auto it = m_edges.begin(); it != m_edges.end(); ++it)
	{
		(*it)->update(*this);
	}
}

void Mesh::doUpdateVertices()
{
	for (auto it = m_vertices.begin(); it != m_vertices.end(); ++it)
	{
		(*it)->update(*this);
	}
}

void Mesh::updatePositions(const std::vector<Vector3d>& positions)
{
	// TODO: assert that the number of positions matches vertices

	auto positionIt = positions.cbegin();
	auto vertexIt = m_vertices.begin();
	for ( ; positionIt != positions.end() && vertexIt != m_vertices.end(); ++positionIt, ++vertexIt)
	{
		vertexIt->position = *positionIt;
	}

	computeNormals();
}

void Mesh::updatePositionFromOtherMesh(const Mesh &rhs, bool updateNormalAndAabb)
{
	assert(numVertices() == rhs.numVertices());

	for (int vv=0; vv < numVertices(); ++vv)
	{
		double* position           = mutableVertexLocalPosition(vv);
		const double* rhs_position = rhs.getVertexLocalPosition(vv);
		position[0] = rhs_position[0];
		position[1] = rhs_position[1];
		position[2] = rhs_position[2];
	}

	if (updateNormalAndAabb)
	{
		for (int ii = 0;  ii < numTriangles();  ++ii)
		{
			TriangleGeometry& tri           = m_triangleGeometry[ii];
			const TriangleGeometry& rhs_tri = rhs.m_triangleGeometry[ii];
			for (int jj = 0;  jj < 3;  ++jj)
			{
				tri.normal[jj] = rhs_tri.normal[jj];
				tri.m_aabb.min[jj] = rhs_tri.m_aabb.min[jj];
				tri.m_aabb.max[jj] = rhs_tri.m_aabb.max[jj];
			}
		}
	}
}

void Mesh::updatePositionFromTwoMeshes(const Mesh& m1, double percent1, const Mesh& m2, double percent2, bool updateNormalAndAabb)
{
	// TODO: check that meshes have same number of vertices

	for 
	{
		double* position          = mutableVertexLocalPosition(vv);
		const double* m1_position = m1.getVertexLocalPosition(vv);
		const double* m2_position = m2.getVertexLocalPosition(vv);
		for (int ii = 0;  ii < 3;  ++ii)
		{
			position[ii] = m1_position[ii]*percent1 + m2_position[ii]*percent2;
		}
	}

	if (updateNormal)
	{
		computeNormal();
	}
}

bool Mesh::writeMeshToFile(char *filename)
{
	ofstream filestr;
	filestr.open (filename, fstream::out);

	if (filestr.fail())
	{
		return false;
	}

	int nbVertices  = numVertices();
	int nbTriangles = numTriangles();

	const char vertexPosTag[256] = "vertexPos";
	const char triIdTag[256] = "triId";
	const char transformFrom[256] = "transformFrom";
	const char transformTo[256] = "transformTo";

	// write vertex position
	for (int vv = 0; vv < nbVertices; ++vv)
	{
		const double* position = getVertexLocalPosition(vv);
		filestr << vertexPosTag << " " << position[0] << " " << position[1] << " " << position[2] << endl;
	}

	// write triangle id
	for (int tt = 0; tt < nbTriangles; ++tt)
	{
		const TriangleTopology& tri = getTriangleTopology(tt);
		filestr << triIdTag << " " << tri.vertexID[0] << " " << tri.vertexID[1] << " " << tri.vertexID[2] << endl;
	}

	// write transform
	btScalar matFrom[16], matTo[16];
	m_from.getOpenGLMatrix(matFrom);
	m_to.getOpenGLMatrix(matTo);

	filestr << transformFrom;
	for (int ii = 0 ; ii < 16; ++ii) filestr << " " << matFrom[ii] ;
	filestr << endl ;

	filestr << transformTo;
	for (int ii = 0 ; ii < 16; ++ii) filestr << " " << matTo[ii] ;
	filestr << endl ;


	filestr.close();

	return true;
}

bool Mesh::readMeshFromFile(char *filename)
{
	ifstream filestr;
	filestr.open (filename, fstream::in);
	if (filestr.fail())
	{
		return false;
	}

	double vertexPos[3];
	int triId[3];
	char tagChar[256];

	std::vector<double> allVertex;
	std::vector<int> allTri;
	btScalar matFrom[16], matTo[16];

	while(!filestr.eof())
	{
		filestr >> tagChar;
		if(!filestr.eof() && strcmp(tagChar,"vertexPos")==0)
		{
			filestr >> vertexPos[0];
			filestr >> vertexPos[1];
			filestr >> vertexPos[2];

			allVertex.push_back(vertexPos[0]);
			allVertex.push_back(vertexPos[1]);
			allVertex.push_back(vertexPos[2]);
		}
		else if(!filestr.eof() && strcmp(tagChar,"triId")==0)
		{
			filestr >> triId[0];
			filestr >> triId[1];
			filestr >> triId[2];

			allTri.push_back(triId[0]);
			allTri.push_back(triId[1]);
			allTri.push_back(triId[2]);
		}
		else if(!filestr.eof() && strcmp(tagChar,"transformFrom")==0)
		{
			for (int ii = 0 ; ii < 16; ++ii) filestr >> matFrom[ii] ;
		}
		else if(!filestr.eof() && strcmp(tagChar,"transformTo")==0)
		{
			for (int ii = 0 ; ii < 16; ++ii) filestr >> matTo[ii] ;
		}
	}

	filestr.close();

	// reset existing data
	m_topology->m_vertices.resize(0);
	m_topology->m_edges.resize(0);
	m_topology->m_triangles.resize(0);
	m_vertexGeometry.resize(0);
	m_edgeGeometry.resize(0);
	m_triangleGeometry.resize(0);
	m_from.setIdentity();
	m_to.setIdentity();
	m_from.setFromOpenGLMatrix(matFrom);
	m_to.setFromOpenGLMatrix(matTo);


	int nbVertices = (int)allVertex.size()/3;
	int nbTriangles = (int)allTri.size()/3;

	// 1) Create all the vertices
	for(int vv=0 ; vv< nbVertices ; ++vv)
	{
		createNewVertex(&(allVertex[vv*3]));
	}

	// 2) Create all edges and triangles
	int vID[3]={-1,-1,-1};
	int eID[3]={-1,-1,-1};
	for(int tt=0 ; tt<nbTriangles ; ++tt)
	{
		vID[0] = allTri[tt*3+0];
		vID[1] = allTri[tt*3+1];
		vID[2] = allTri[tt*3+2];

		if( (eID[0]=doesThisEdgeAlreadyExist(vID[0],vID[1]))==-1 ) eID[0]=createNewEdge(vID[0],vID[1]);
		if( (eID[1]=doesThisEdgeAlreadyExist(vID[0],vID[2]))==-1 ) eID[1]=createNewEdge(vID[0],vID[2]);
		if( (eID[2]=doesThisEdgeAlreadyExist(vID[1],vID[2]))==-1 ) eID[2]=createNewEdge(vID[1],vID[2]);

		createNewTriangle(vID,eID);
	}

	internalProcessWithVertexAndTriangle();
	return true;
}
