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

#ifndef SURGSIM_GRAPHICS_OSGSKELETONREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGSKELETONREPRESENTATION_H

#include <boost/thread.hpp>
#include <map>
#include <osg/MatrixTransform>
#include <osg/ref_ptr>
#include <osgUtil/UpdateVisitor>
#include <string>

#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/SkeletonRepresentation.h"

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace osg
{
class Node;
class Shader;
};

namespace SurgSim
{
namespace Graphics
{

struct BoneData;
class OsgModel;

/// Skeleton representation is used to move a mesh based on the movements of
/// pre-selected control points (bones).
class OsgSkeletonRepresentation : public OsgRepresentation, public SkeletonRepresentation
{
public:
	/// Constructor.
	/// \param	name	The name of the representation.
	explicit OsgSkeletonRepresentation(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgSkeletonRepresentation);

	void loadModel(const std::string& fileName) override;

	void setModel(std::shared_ptr<SurgSim::Framework::Asset> model) override;

	std::shared_ptr<Model> getModel() const override;

	/// Set the file containing the skinning shader.
	/// \param fileName The file containing the skinning shader.
	void setSkinningShaderFileName(const std::string& fileName);

	/// \return The file containing the skinning shader.
	std::string getSkinningShaderFileName() const;

	void setBonePose(const std::string& name, const SurgSim::Math::RigidTransform3d& pose) override;

	SurgSim::Math::RigidTransform3d getBonePose(const std::string& name) const override;

	void setNeutralBonePose(const std::string& name, const SurgSim::Math::RigidTransform3d& pose) override;

	SurgSim::Math::RigidTransform3d getNeutralBonePose(const std::string& name) const override;

protected:
	void setNeutralBonePoses(const std::map<std::string, SurgSim::Math::UnalignedRigidTransform3d>& poses) override;

	std::map<std::string, SurgSim::Math::UnalignedRigidTransform3d> getNeutralBonePoses() const override;

	void doUpdate(double dt) override;

	bool doInitialize() override;

private:
	/// Setup the bones with the model and skinning shader
	bool setupBones();

	/// The logger for this class.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	/// The model containing the bone and mesh information.
	std::shared_ptr<OsgModel> m_model;

	/// The named map of the bones in this skeleton.
	std::shared_ptr<std::map<std::string, BoneData>> m_bones;

	/// Mutex to access m_bones safely.
	mutable boost::shared_mutex m_mutex;

	/// The skeleton which is read from the mesh file.
	osg::ref_ptr<osg::Node> m_skeleton;

	/// The file containing the skinning shader.
	std::string m_skinningShaderFileName;

	/// The hardware skinning shader.
	osg::ref_ptr<osg::Shader> m_skinningShader;

	/// Tree updater which updates the position of the bones.
	osg::ref_ptr<osgUtil::UpdateVisitor> m_updateVisitor;

	/// Parameter to keep track of the skeleton's frame count. Set to the UpdateVisitor.
	size_t m_frameCount;

	/// The root node of the skeleton tree.
	osg::ref_ptr<osg::Node> m_root;

	/// The first MatrixTransform node
	osg::ref_ptr<osg::MatrixTransform> m_base;
};

};  // namespace Graphics
};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGSKELETONREPRESENTATION_H
