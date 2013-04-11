// #include "MeshObjectNode.h"
// 
// #include <osg/MatrixTransform>
// #include <osg/PolygonMode>
// #include <osg/Point>
// #include <osg/PointSprite>
// #include <osg/LineWidth>
// #include <osg/Material>
// #include <osg/Array>
// 
// #include "Graphics/DrawingUtilities.h"
// 
// template <int numVerticesPerElement>
// MeshObjectNode<numVerticesPerElement>::MeshObjectNode(const MeshObject<numVerticesPerElement>* const meshObject) : ObjectNode(meshObject),
// 	m_showSolid(true),
// 	m_showPoints(false), 
// 	m_showWireframe(false),
// 	m_pointSize(3.0),
// 	m_lineThickness(2.0),
// 	m_colorBinding(MeshObject<numVerticesPerElement>::BIND_OVERALL),
// 	m_normalBinding(MeshObject<numVerticesPerElement>::BIND_PER_VERTEX),
// 	m_isPoseDirty(false),
// 	m_isMeshDirty(false),
// 	m_isDisplayModeDirty(false),
// 	m_areWireframeAndPointSettingsDirty(false),
// 	m_poseGeneration(0),
// 	m_meshGeneration(0),
// 	m_displayModeGeneration(0),
// 	m_wireframeAndPointSettingsGeneration(0)
// {
// 	for (int i = 0; i < 4; ++i)
// 	{
// 		m_pointColor[i] = 1.0;
// 		m_lineColor[i] = 1.0;
// 	}
// 
// 	for (int i = 0; i < 3; ++i)
// 	{
// 		m_position[i] = 0.0;
// 		m_orientation[i] = 0.0;
// 	}
// 	m_orientation[3] = 1.0;
// 
// 	m_poseTransform = new osg::MatrixTransform();
// 	m_poseTransform->setDataVariance(osg::Object::DYNAMIC);
// 	addChild(m_poseTransform);
// 
// 	m_displayModeSwitch = new osg::Switch();
// 	m_displayModeSwitch->setDataVariance(osg::Object::DYNAMIC);
// 	m_poseTransform->addChild(m_displayModeSwitch);
// 
// 	// Solid
// 	{
// 		m_solidGeode = new osg::Geode();
// 		m_solidGeode->setDataVariance(osg::Object::DYNAMIC);
// 		m_displayModeSwitch->addChild(m_solidGeode);
// 	}
// 	
// 	// Wireframe
// 	{
// 		m_wireframeGeode = new osg::Geode();
// 		m_wireframeGeode->setDataVariance(osg::Object::DYNAMIC);
// 		m_displayModeSwitch->addChild(m_wireframeGeode);
// 
// 		osg::StateSet* state = m_wireframeGeode->getOrCreateStateSet();
// 		osg::ref_ptr<osg::PolygonMode> polyMode;
// 
// 		polyMode = static_cast<osg::PolygonMode*>(state->getAttribute(osg::StateAttribute::POLYGONMODE));
// 		if (!polyMode) 
// 		{
// 			polyMode = new osg::PolygonMode();
// 		}
// 		state->setAttributeAndModes(polyMode, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);  
// 
// 		polyMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
// 
// 		osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth();
// 		lineWidth->setWidth(m_lineThickness);
// 		state->setAttributeAndModes(lineWidth, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
// 
// 		osg::ref_ptr<osg::Material> material = static_cast<osg::Material*>(state->getAttribute(osg::StateAttribute::MATERIAL));
// 		if (!material)
// 		{
// 			material = new osg::Material();
// 		}
// 		state->setAttributeAndModes(material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
// 		material->setColorMode(osg::Material::OFF);
// 		material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(m_lineColor[0], m_lineColor[1], m_lineColor[2], m_lineColor[3]));
// 		material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(m_lineColor[0], m_lineColor[1], m_lineColor[2], m_lineColor[3]));
// 
// 		state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
// 	}
// 
// 	// Points
// 	{
// 		m_pointsGeode = new osg::Geode();
// 		m_pointsGeode->setDataVariance(osg::Object::DYNAMIC);
// 		m_displayModeSwitch->addChild(m_pointsGeode);
// 
// 		osg::StateSet* state = m_pointsGeode->getOrCreateStateSet();
// 		osg::ref_ptr<osg::PolygonMode> polyMode;
// 
// 		polyMode = static_cast<osg::PolygonMode*>(state->getAttribute(osg::StateAttribute::POLYGONMODE));
// 
// 		if (!polyMode) 
// 		{
// 			polyMode = new osg::PolygonMode();
// 		}
// 		state->setAttributeAndModes(polyMode, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);  
// 
// 		state->setTextureAttributeAndModes(0, new osg::PointSprite(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
// 		polyMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT);
// 
// 		osg::ref_ptr<osg::Point> point = new osg::Point();
// 		point->setSize(m_pointSize);
// 		state->setAttributeAndModes(point, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
// 
// 		osg::ref_ptr<osg::Material> material = static_cast<osg::Material*>(state->getAttribute(osg::StateAttribute::MATERIAL));
// 		if (!material)
// 		{
// 			material = new osg::Material();
// 		}
// 		state->setAttributeAndModes(material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
// 		material->setColorMode(osg::Material::OFF);
// 		material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(m_pointColor[0], m_pointColor[1], m_pointColor[2], m_pointColor[3]));
// 		material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(m_pointColor[0], m_pointColor[1], m_pointColor[2], m_pointColor[3]));
// 
// 		state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
// 	}	
// 
// 	update();
// }
// 
// template <int numVerticesPerElement>
// MeshObjectNode<numVerticesPerElement>::~MeshObjectNode()
// {
// }
// 
// template <int numVerticesPerElement>
// void MeshObjectNode<numVerticesPerElement>::update()
// {
// 	ObjectNode::update();
// 
// 	if (!m_isVisible)
// 	{
// 		return;
// 	}
// 
// 	const MeshObject<numVerticesPerElement>* const meshObject = static_cast<const MeshObject<numVerticesPerElement>* const>(m_interfaceObject);
// 
// 	m_isPoseDirty = meshObject->getPose(m_position, m_orientation, m_poseGeneration);
// 	m_isMeshDirty = meshObject->getMesh(m_vertices, m_elements, m_colors, m_colorBinding, m_textureCoords, m_normals, m_normalBinding, m_meshGeneration);
// 	m_isDisplayModeDirty = meshObject->getDisplayMode(m_showSolid, m_showPoints, m_showWireframe, m_displayModeGeneration);
// 	m_areWireframeAndPointSettingsDirty = meshObject->getWireframeAndPointSettings(m_pointColor, m_pointSize, m_lineColor, m_lineThickness,
// 		m_wireframeAndPointSettingsGeneration);
// 
// 	// Update pose
// 	if (m_isPoseDirty)
// 	{
// 		osg::Matrix matrix;
// 
// 		osg::Quat quaternion(m_orientation[0], m_orientation[1], m_orientation[2], m_orientation[3]);
// 		matrix.makeRotate(quaternion);
// 		matrix.setTrans(m_position[0], m_position[1], m_position[2]);
// 
// 		m_poseTransform->setMatrix(matrix);
// 	}
// 
// 	// Update display mode
// 	if (m_isDisplayModeDirty)
// 	{
// 		m_displayModeSwitch->setChildValue(m_solidGeode, m_showSolid);
// 		m_displayModeSwitch->setChildValue(m_pointsGeode, m_showPoints);
// 		m_displayModeSwitch->setChildValue(m_wireframeGeode, m_showWireframe);
// 	}
// 
// 	// Update wireframe and point settings
// 	if (m_areWireframeAndPointSettingsDirty)
// 	{
// 		// Wireframe
// 		{
// 			osg::StateSet* state = m_wireframeGeode->getOrCreateStateSet();
// 			osg::ref_ptr<osg::PolygonMode> polyMode;
// 
// 			polyMode = static_cast<osg::PolygonMode*>(state->getAttribute(osg::StateAttribute::POLYGONMODE));
// 			polyMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
// 
// 			osg::ref_ptr<osg::LineWidth> lineWidth = static_cast<osg::LineWidth*>(state->getAttribute(osg::StateAttribute::LINEWIDTH));
// 			lineWidth->setWidth(m_lineThickness);
// 
// 			osg::ref_ptr<osg::Material> material = static_cast<osg::Material*>(state->getAttribute(osg::StateAttribute::MATERIAL));
// 			material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(m_lineColor[0], m_lineColor[1], m_lineColor[2], m_lineColor[3]));
// 			material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(m_lineColor[0], m_lineColor[1], m_lineColor[2], m_lineColor[3]));
// 		
// 			// Enable transparency if needed
// 			if (m_lineColor[3] < 1.0)
// 			{
// 				state->setMode(GL_BLEND, osg::StateAttribute::ON);
// 				state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
// 			}
// 			else
// 			{
// 				state->setMode(GL_BLEND, osg::StateAttribute::INHERIT);
// 				state->setRenderingHint(osg::StateSet::INHERIT_RENDERBIN_DETAILS);
// 			}
// 		}
// 		
// 		// Points
// 		{
// 			osg::StateSet* state = m_pointsGeode->getOrCreateStateSet();
// 			osg::ref_ptr<osg::PolygonMode> polyMode;
// 
// 			polyMode = static_cast<osg::PolygonMode*>(state->getAttribute(osg::StateAttribute::POLYGONMODE));
// 			polyMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT);
// 
// 			osg::ref_ptr<osg::Point> point = static_cast<osg::Point*>(state->getAttribute(osg::StateAttribute::POINT));
// 			point->setSize(m_pointSize);
// 
// 			osg::ref_ptr<osg::Material> material = static_cast<osg::Material*>(state->getAttribute(osg::StateAttribute::MATERIAL));
// 			material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(m_pointColor[0], m_pointColor[1], m_pointColor[2], m_pointColor[3]));
// 			material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(m_pointColor[0], m_pointColor[1], m_pointColor[2], m_pointColor[3]));
// 		
// 			// Enable transparency if needed
// 			if (m_pointColor[3] < 1.0)
// 			{
// 				state->setMode(GL_BLEND, osg::StateAttribute::ON);
// 				state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
// 			}
// 			else
// 			{
// 				state->setMode(GL_BLEND, osg::StateAttribute::INHERIT);
// 				state->setRenderingHint(osg::StateSet::INHERIT_RENDERBIN_DETAILS);
// 			}
// 		}
// 	}
// }
// 
// template <int numVerticesPerElement>
// osg::Node* MeshObjectNode<numVerticesPerElement>::getSolidNode()
// {
// 	return m_solidGeode;
// }
// template <int numVerticesPerElement>
// osg::Node* MeshObjectNode<numVerticesPerElement>::getPointsNode()
// {
// 	return m_pointsGeode;
// }
// template <int numVerticesPerElement>
// osg::Node* MeshObjectNode<numVerticesPerElement>::getWireframeNode()
// {
// 	return m_wireframeGeode;
// }