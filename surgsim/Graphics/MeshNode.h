#ifndef MESH_OBJECT_NODE_H
#define MESH_OBJECT_NODE_H

#include <osg/Switch>
#include <osg/Geode>

#include "ObjectNode.h"
#include "Graphics/InterfaceObjects/MeshObject.h"

//! Abstract node for displaying mesh objects
template <int numVerticesPerElement>
class MeshObjectNode : public ObjectNode
{
public:
	//! Initializes a mesh object node
	/*!
	\param meshObject Interface object to retrieve updates
	*/
	MeshObjectNode(const MeshObject<numVerticesPerElement>* const meshObject);

	//! Updates the node
	virtual void update();

	//! Gets the node for displaying the mesh with solid faces
	/*
	\return Solid node
	*/
	osg::Node* getSolidNode();
	//! Gets the node for displaying the mesh points
	/*
	\return Points node
	*/
	osg::Node* getPointsNode();
	//! Gets the node for displaying the mesh wireframe
	/*
	\return Wireframe node
	*/
	osg::Node* getWireframeNode();

protected:
	//! Destructor
	virtual ~MeshObjectNode();

	//! Display mode switch to change visibility of solid, wireframe, and points
	osg::ref_ptr<osg::Switch> m_displayModeSwitch;

	//! Transform by pose
	osg::ref_ptr<osg::MatrixTransform> m_poseTransform;
	
	//! Mesh geometry
	osg::ref_ptr<osg::Geometry> m_meshGeometry;

	//! Geode to display mesh with solid faces
	osg::ref_ptr<osg::Geode> m_solidGeode;
	//! Geode to display mesh wireframe
	osg::ref_ptr<osg::Geode> m_wireframeGeode;
	//! Geode to display mesh points
	osg::ref_ptr<osg::Geode> m_pointsGeode;

	// Mesh X,Y,Z position
	double m_position[3];
	// Mesh orientation as quaternion X,Y,Z,W
	double m_orientation[4];

	//! Mesh vertex positions
	std::vector<double> m_vertices;
	//! Mesh element vertex IDs
	std::vector<size_t> m_elements;
	//! Mesh colors
	std::vector<double> m_colors;
	//! Mesh color binding
	typename MeshObject<numVerticesPerElement>::Binding m_colorBinding;
	//! Mesh texture coordinates
	std::vector<double> m_textureCoords;
	//! Mesh normals
	std::vector<double> m_normals;
	//! Mesh normal binding
	typename MeshObject<numVerticesPerElement>::Binding m_normalBinding;

	//! Whether to show the mesh with solid faces
	bool m_showSolid;
	//! Whether to show the mesh points
	bool m_showPoints;
	//! Whether to show the mesh wireframe
	bool m_showWireframe;

	//! Size of points
	double m_pointSize;
	//! Thickness of wireframe lines
	double m_lineThickness;

	//! Color of points
	double m_pointColor[4];
	//! Color of wireframe lines
	double m_lineColor[4];

	//! Whether the pose has changed
	bool m_isPoseDirty;
	//! Whether the mesh has changed
	bool m_isMeshDirty;
	//! Whether the display mode has changed
	bool m_isDisplayModeDirty;
	//! Whether the wireframe and point settings have changed
	bool m_areWireframeAndPointSettingsDirty;

	//! Current generation of the pose data, to determine if data in the interface object is new
	unsigned int m_poseGeneration;
	//! Current generation of the mesh data, to determine if data in the interface object is new
	unsigned int m_meshGeneration;
	//! Current generation of the display mode data, to determine if data in the interface object is new
	unsigned int m_displayModeGeneration;
	//! Current generation of the wireframe and point settings data, to determine if data in the interface object is new
	unsigned int m_wireframeAndPointSettingsGeneration;
};

#include "MeshObjectNode.inl"

#endif