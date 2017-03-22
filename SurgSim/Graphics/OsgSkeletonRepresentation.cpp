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

#include "SurgSim/Graphics/OsgSkeletonRepresentation.h"

#include <algorithm>
#include <boost/thread.hpp>
#include <map>
#include <osg/Geode>
#include <osg/Quat>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osg/Shader>
#include <osg/Vec3>
#include <osgAnimation/Bone>
#include <osgAnimation/RigGeometry>
#include <osgAnimation/RigTransformHardware>
#include <osgAnimation/Skeleton>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/UpdateBone>
#include <osgDB/ReadFile>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/OsgModel.h"
#include "SurgSim/Graphics/OsgRigidTransformConversions.h"
#include "SurgSim/Graphics/OsgProgram.h"


/// Local data structure to store the bones and their references to the transforms.
struct SurgSim::Graphics::BoneData
{
	osg::ref_ptr<osgAnimation::Bone> osgBone;
	osg::ref_ptr<osgAnimation::StackedQuaternionElement> osgRotation;
	osg::ref_ptr<osgAnimation::StackedTranslateElement> osgTranslation;
	SurgSim::Math::UnalignedRigidTransform3d neutralPose;
	SurgSim::Math::UnalignedRigidTransform3d pose;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	BoneData() :
		osgBone(nullptr),
		osgRotation(nullptr),
		osgTranslation(nullptr),
		neutralPose(SurgSim::Math::UnalignedRigidTransform3d::Identity()),
		pose(SurgSim::Math::UnalignedRigidTransform3d::Identity())
	{
	}
};

namespace
{

/// Visitor to collect information about the tree that we are using, collect all the
/// osgAnimation::Bone nodes in it and add stacked transform elements to them so
/// we can manipulate them, also switches the skinning to Hardware
class BoneBuilder : public osg::NodeVisitor
{
public:
	/// Constructor
	/// \param hardwareShader The shader which does the skinning.
	/// \param bones The data structure to store the found bones
	BoneBuilder(osg::Shader* hardwareShader,
			std::shared_ptr<std::map<std::string, SurgSim::Graphics::BoneData>> bones) :
		NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN),
		m_shader(hardwareShader),
		m_bones(bones)
	{
	}

	/// Traverse the given tree and collect the bone data in them.
	/// Also setup for hardware skinning, if a hardware shader is provided.
	/// \param node The root node of the skeleton tree that needs to be traversed.
	virtual void apply(osg::Node& node) override // NOLINT
	{
		// Look for the transformation root ..
		if (m_rootTransform == nullptr)
		{
			m_rootTransform = dynamic_cast<osg::MatrixTransform*>(&node);
		}

		// Parse the bone data.
		osgAnimation::Bone* bone = dynamic_cast<osgAnimation::Bone*>(&node);
		if (bone != nullptr)
		{
			auto newBone = m_bones->find(bone->getName());
			if (newBone == m_bones->end())
			{
				newBone = m_bones->emplace(std::make_pair(bone->getName(), SurgSim::Graphics::BoneData())).first;
			}
			else
			{
				if (newBone->second.osgBone != nullptr)
				{
					SURGSIM_ASSERT(m_bones->find(bone->getName()) == m_bones->end())
						<< "There already exists a bone with name, " << bone->getName()
						<< ", in this skeleton. Cannot create a duplicate.";
				}
			}

			newBone->second.osgBone = bone;
			newBone->second.osgRotation = new osgAnimation::StackedQuaternionElement("OssRotation");
			newBone->second.osgRotation->setQuaternion(osg::Quat());
			newBone->second.osgTranslation = new osgAnimation::StackedTranslateElement("OssTranslation");

			osgAnimation::UpdateMatrixTransform* callback =
				dynamic_cast<osgAnimation::UpdateMatrixTransform*>(bone->getUpdateCallback());
			if (callback == nullptr)
			{
				bone->setDefaultUpdateCallback();
				callback = dynamic_cast<osgAnimation::UpdateMatrixTransform*>(bone->getUpdateCallback());
			}
			SURGSIM_ASSERT(callback != nullptr) << "Could neither find nor create the appropriate BoneUpdate callback";

			// Push these transformations onto the stack so we can manipulate them
			callback->getStackedTransforms().push_back(newBone->second.osgRotation);
			callback->getStackedTransforms().push_back(newBone->second.osgTranslation);
		}

		// Setup for hardware skinning.
		if (m_shader != nullptr)
		{
			osg::Geode* meshGeode = dynamic_cast<osg::Geode*>(&node);
			if (nullptr != meshGeode)
			{
				osgAnimation::RigTransformHardware* rigTransform = new osgAnimation::RigTransformHardware();
				osg::Shader* shader = new osg::Shader(*m_shader);
				rigTransform->setShader(shader);

				for (size_t i = 0; i < meshGeode->getNumDrawables(); ++i)
				{
					auto rigGeometry = dynamic_cast<osgAnimation::RigGeometry*>(meshGeode->getDrawable(i));
					if (nullptr != rigGeometry)
					{
						rigGeometry->setRigTransformImplementation(rigTransform);
					}
				}
			}
		}
		traverse(node);
	}

	/// \return The root node of the skeleton on which the transform is set.
	osg::ref_ptr<osg::MatrixTransform> getRootTransform()
	{
		return m_rootTransform;
	}

private:
	/// The root bone of the skeleton where the global transform is set.
	osg::ref_ptr<osg::MatrixTransform> m_rootTransform;

	/// The hardware skinning shader.
	osg::ref_ptr<osg::Shader> m_shader;

	/// The bone data from the skeleton.
	std::shared_ptr<std::map<std::string, SurgSim::Graphics::BoneData>>  m_bones;
};

};

namespace SurgSim
{
namespace Graphics
{

SURGSIM_REGISTER(SurgSim::Framework::Component,
				 SurgSim::Graphics::OsgSkeletonRepresentation, OsgSkeletonRepresentation);

OsgSkeletonRepresentation::OsgSkeletonRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	SkeletonRepresentation(name),
	m_logger(SurgSim::Framework::Logger::getLogger("Graphics/OsgSkeletonRepresentation")),
	m_bones(std::make_shared<std::map<std::string, BoneData>>())
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OsgSkeletonRepresentation, std::string,
									  SkinningShaderFileName, getSkinningShaderFileName, setSkinningShaderFileName);
}

void OsgSkeletonRepresentation::loadModel(const std::string& fileName)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot load model after OsgSkeletonRepresentation had been initialized.";

	auto model = std::make_shared<OsgModel>();
	model->load(fileName);
	setModel(model);
}

void OsgSkeletonRepresentation::setModel(std::shared_ptr<SurgSim::Framework::Asset> model)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set model after OsgSkeletonRepresentation had been initialized.";

	auto osgModel = std::dynamic_pointer_cast<OsgModel>(model);
	SURGSIM_ASSERT(model == nullptr || osgModel != nullptr) << "OsgSkeletonRepresentation expects an OsgModel.";
	m_model = osgModel;
}

std::shared_ptr<Model> OsgSkeletonRepresentation::getModel() const
{
	return m_model;
}

void OsgSkeletonRepresentation::setSkinningShaderFileName(const std::string& fileName)
{
	m_skinningShaderFileName = fileName;
}

std::string OsgSkeletonRepresentation::getSkinningShaderFileName() const
{
	return m_skinningShaderFileName;
}

void OsgSkeletonRepresentation::setBonePose(const std::string& name, const SurgSim::Math::RigidTransform3d& pose)
{
	boost::unique_lock<boost::shared_mutex> lock(m_mutex);

	auto found = m_bones->find(name);
	if (found != m_bones->end())
	{
		found->second.pose = pose;
	}
	else if (isInitialized())
	{
		SURGSIM_FAILURE() << "Bone with name " << name << " is not present in mesh.";
	}
	else
	{
		auto newBone = m_bones->emplace(std::make_pair(name, BoneData())).first;
		newBone->second.pose = pose;
	}
}

SurgSim::Math::RigidTransform3d OsgSkeletonRepresentation::getBonePose(const std::string& name) const
{
	SurgSim::Math::RigidTransform3d pose = SurgSim::Math::RigidTransform3d::Identity();
	boost::shared_lock<boost::shared_mutex> lock(m_mutex);

	auto found = m_bones->find(name);
	if (found != m_bones->end())
	{
		pose = found->second.pose;
	}
	else if (isInitialized())
	{
		SURGSIM_FAILURE() << "Bone with name " << name << " is not present in mesh.";
	}

	return std::move(pose);
}

void OsgSkeletonRepresentation::setNeutralBonePose(const std::string& name,
												   const SurgSim::Math::RigidTransform3d& pose)
{
	boost::unique_lock<boost::shared_mutex> lock(m_mutex);

	auto found = m_bones->find(name);
	if (found != m_bones->end())
	{
		found->second.neutralPose = pose;
	}
	else if (isInitialized())
	{
		SURGSIM_FAILURE() << "Bone with name " << name << " is not present in mesh.";
	}
	else
	{
		auto newBone = m_bones->emplace(std::make_pair(name, BoneData())).first;
		newBone->second.neutralPose = pose;
	}
}

SurgSim::Math::RigidTransform3d OsgSkeletonRepresentation::getNeutralBonePose(const std::string& name) const
{
	SurgSim::Math::RigidTransform3d pose = SurgSim::Math::RigidTransform3d::Identity();
	boost::shared_lock<boost::shared_mutex> lock(m_mutex);

	auto found = m_bones->find(name);
	if (found != m_bones->end())
	{
		pose = found->second.neutralPose;
	}
	else if (isInitialized())
	{
		SURGSIM_FAILURE() << "Bone with name " << name << " is not present in mesh.";
	}

	return std::move(pose);
}

void OsgSkeletonRepresentation::setNeutralBonePoses(const std::map<std::string, SurgSim::Math::UnalignedRigidTransform3d>& poses)
{
	for (auto& pose : poses)
	{
		setNeutralBonePose(pose.first, pose.second);
	}
}

std::map<std::string, SurgSim::Math::UnalignedRigidTransform3d> OsgSkeletonRepresentation::getNeutralBonePoses() const 
{
	std::map<std::string, SurgSim::Math::UnalignedRigidTransform3d> neutralBonePoses;
	boost::shared_lock<boost::shared_mutex> lock(m_mutex);

	for (auto& bone : *m_bones)
	{
		neutralBonePoses[bone.first] = bone.second.neutralPose;
	}
	return neutralBonePoses;
}

void OsgSkeletonRepresentation::doUpdate(double dt)
{
	{
		boost::shared_lock<boost::shared_mutex> lock(m_mutex);
		for (auto& bone : *m_bones)
		{
			std::pair<osg::Quat, osg::Vec3d> pose = toOsg(bone.second.pose * bone.second.neutralPose);
			bone.second.osgRotation->setQuaternion(pose.first);
			bone.second.osgTranslation->setTranslate(pose.second);
		}
	}

	// Update the position of the rest of the bones.
	m_base->accept(*m_updateVisitor);
	++m_frameCount;
	m_updateVisitor->setTraversalNumber(m_frameCount);
}

bool OsgSkeletonRepresentation::doInitialize()
{
	std::string shaderFilename;
	if (m_skinningShaderFileName.empty())
	{
		SURGSIM_LOG_SEVERE(m_logger) << getName() << ": Skinning shader file not set.";
		return false;
	}

	getRuntime()->getApplicationData()->tryFindFile(m_skinningShaderFileName, &shaderFilename);
	if (shaderFilename.empty())
	{
		SURGSIM_LOG_SEVERE(m_logger) << getName() << ": Skinning shader file (" << m_skinningShaderFileName
			<< ") not found.";
		return false;
	}

	m_skinningShader = new osg::Shader(osg::Shader::VERTEX);
	if (!m_skinningShader->loadShaderSourceFromFile(shaderFilename))
	{
		SURGSIM_LOG_SEVERE(m_logger) << getName() << ": Error loading shader (" << shaderFilename << ").";
		return false;
	}

	if (m_model == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << getName() << ": Model is not set.";
		return false;
	}

	m_skeleton = dynamic_cast<osg::Node*>(m_model->getOsgNode().get());
	if (m_skeleton == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << getName() << ": Model does not have a valid osgNode.";
		return false;
	}

	if (!setupBones())
	{
		SURGSIM_LOG_SEVERE(m_logger) << getName() << ": Could not find any bones in model.";
		return false;
	}

	// Setup the transform updater for the skeleton.
	m_updateVisitor = new osgUtil::UpdateVisitor();
	m_frameCount = 0;
	m_updateVisitor->setTraversalNumber(m_frameCount);
	m_base->accept(*m_updateVisitor);

	// Add the bone skeleton as a child to m_transform
	m_transform->addChild(m_base.get());

	return true;
}

bool OsgSkeletonRepresentation::setupBones()
{
	boost::unique_lock<boost::shared_mutex> lock(m_mutex);

	BoneBuilder builder(m_skinningShader, m_bones);
	builder.traverse(*m_skeleton.get());
	m_base = builder.getRootTransform();

	for (auto bone = m_bones->begin(); bone != m_bones->end();)
	{
		if (bone->second.osgBone == nullptr)
		{
			SURGSIM_FAILURE() << "Bone with name " << bone->first << " is not present in mesh.";
			bone = m_bones->erase(bone);
		}
		else
		{
			++bone;
		}
	}

	return (m_bones->size() != 0);
}

}; // Graphics
}; // SurgSim
