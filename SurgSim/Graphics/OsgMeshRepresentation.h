// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_GRAPHICS_OSGMESHREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGMESHREPRESENTATION_H

#include <memory>

#include <osg/Array>
#include <osg/ref_ptr>

#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/MeshRepresentation.h"

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace osg
{
class Geode;
class Geometry;
class DrawElementsUInt;
}

namespace SurgSim
{
namespace Graphics
{
class Mesh;

SURGSIM_STATIC_REGISTRATION(OsgMeshRepresentation);

/// Implementation of a MeshRepresentation for rendering under osg.
class OsgMeshRepresentation : public OsgRepresentation, public MeshRepresentation
{
public:
	/// Constructor.
	/// \param	name	The name.
	explicit OsgMeshRepresentation(const std::string& name);

	/// Destructor
	~OsgMeshRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgMeshRepresentation);

	std::shared_ptr<Mesh> getMesh() override;

	void setMesh(std::shared_ptr<SurgSim::Framework::Asset> mesh) override;

	void loadMesh(const std::string& fileName) override;

	void setShape(std::shared_ptr<SurgSim::Math::Shape> shape) override;

	void setUpdateOptions(int val) override;
	int getUpdateOptions() const override;

	osg::ref_ptr<osg::Geometry> getOsgGeometry() const;

protected:
	void doUpdate(double dt) override;

	/// \note If m_filename is set, m_mesh will be overwritten with the mesh loaded from the external file.
	bool doInitialize() override;

private:
	/// Indicates which elements of the mesh should be updated on every frame
	int m_updateOptions;

	/// The mesh.
	std::shared_ptr<Mesh> m_mesh;

	/// File name of the external file which contains the mesh to be used by this class.
	std::string m_filename;

	///@{
	/// Osg structures
	osg::ref_ptr<osg::Switch> m_meshSwitch;
	osg::ref_ptr<osg::Geometry> m_geometry;
	///@}

	/// Updates the internal arrays in accordance to the sizes given in the mesh
	/// \param geometry [out] The geometry that carries the data
	/// \return updateOptions value that indicates which of the structures where updated in size and
	/// 		will have to be updated independent of the value set in setUpdateOptions()
	int updateOsgArrays(osg::Geometry* geometry);

	/// Copies the attributes for each mesh vertex in the appropriate osg structure, this will only be done
	/// for the data as is indicated by updateOptions
	/// \param geometry [out] The geometry that carries the data
	/// \param updateOptions Set of flags indicating whether a specific vertex attribute should be updated
	void updateVertices(osg::Geometry* geometry, int updateOptions);

	/// Updates the normals.
	/// \param geometry [out] The geometry that carries the data
	void updateNormals(osg::Geometry* geometry);

	/// Updates the triangles.
	/// \param geometry [out] The geometry that carries the data
	void updateTriangles(osg::Geometry* geometry);

	/// Gets data variance for a given update option.
	/// \param	updateOption	The update option.
	/// \return	The data variance.
	osg::Object::DataVariance getDataVariance(int updateOption);

	/// Create the appropriate geometry nodes
	void buildGeometry();

	/// Cache for the update count pull from the mesh
	size_t m_updateCount;

};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_OSGMESHREPRESENTATION_H
