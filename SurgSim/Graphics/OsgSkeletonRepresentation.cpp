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

#include <map>

#include <boost/thread.hpp>

#include <osg/Quat>
#include <osg/Vec3>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>

#include <osgAnimation/Skeleton>
#include <osgAnimation/RigGeometry>
#include <osgAnimation/RigTransformHardware>
#include <osgAnimation/UpdateBone>

#include <osgDB/ReadFile>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgModel.h"
#include "SurgSim/Graphics/OsgRigidTransformConversions.h"
#include "SurgSim/Graphics/OsgProgram.h"

namespace
{

/// Visitor to collect information about the tree that we are using, collect all the
/// osgAnimation::Bone nodes in it and add stacked transform elements to them so
/// we can manipulate them, also switches the skinning to Hardware
class BoneMapCreator : public osg::NodeVisitor
{
public:
	/// Single argument constructor.
	/// \param hardwareShader The shader which does the skinning.
	explicit BoneMapCreator(osg::Shader* hardwareShader) :
		NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN), m_shader(hardwareShader) {}

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
			SurgSim::Graphics::OsgSkeletonBoneData data;
			data.bone = bone;
			osgAnimation::UpdateMatrixTransform* callback =
				dynamic_cast<osgAnimation::UpdateMatrixTransform*>(bone->getUpdateCallback());

			// If we can't find this callback, replace with the default
			if (callback == nullptr)
			{
				bone->setDefaultUpdateCallback();
				callback = dynamic_cast<osgAnimation::UpdateMatrixTransform*>(bone->getUpdateCallback());
			}

			SURGSIM_ASSERT(callback != nullptr) << "Could neither find nor create the appropriate BoneUpdate callback";

			// Push these transformations onto the stack so we can manipulate them
			data.rotation = new osgAnimation::StackedQuaternionElement("OssRotation");
			callback->getStackedTransforms().push_back(data.rotation);
			data.rotation->setQuaternion(osg::Quat());
			data.translation = new osgAnimation::StackedTranslateElement("OssTranslation");
			callback->getStackedTransforms().push_back(data.translation);

			SURGSIM_ASSERT(m_boneData.find(bone->getName()) == m_boneData.end())
					<< "There already exists a bone with name, " << bone->getName()
					<< ", in this skeleton. Cannot create a duplicate.";

			m_boneData[bone->getName()] = data;
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

				for (size_t i = 0; i < meshGeode->getDrawableList().size(); ++i)
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

	/// \return The bone data that had been collected.
	std::map<std::string, SurgSim::Graphics::OsgSkeletonBoneData> getMap()
	{
		return m_boneData;
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
	std::map<std::string, SurgSim::Graphics::OsgSkeletonBoneData> m_boneData;
};

}

namespace SurgSim
{
namespace Graphics
{
SURGSIM_REGISTER(SurgSim::Framework::Component,
				 SurgSim::Graphics::OsgSkeletonRepresentation, OsgSkeletonRepresentation);

OsgSkeletonRepresentation::OsgSkeletonRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	SkeletonRepresentation(name)
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

void OsgSkeletonRepresentation::setSkinningShaderFileName(std::string fileName)
{
	m_skinningShaderFileName = fileName;
}

std::string OsgSkeletonRepresentation::getSkinningShaderFileName()
{
	return m_skinningShaderFileName;
}

void OsgSkeletonRepresentation::setBonePose(const std::string& name, const SurgSim::Math::RigidTransform3d& pose)
{
	auto boneData = m_boneData.find(name);
	if (boneData != m_boneData.end())
	{
		boost::lock_guard<boost::mutex> lock(m_boneDataMutex);
		boneData->second.pose = pose;
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "OsgSkeletonRepresentation::setBonePose(): Bone with name, " << name << ", is not present in mesh."
				<< "Cannot set pose.";
	}
}

void OsgSkeletonRepresentation::doUpdate(double dt)
{
	{
		boost::lock_guard<boost::mutex> lock(m_boneDataMutex);
		// Update the pose of all the bones.
		for (auto& boneData : m_boneData) // NOLINT
		{
			std::pair<osg::Quat, osg::Vec3d> pose = toOsg(boneData.second.pose);
			boneData.second.rotation->setQuaternion(pose.first);
			boneData.second.translation->setTranslate(pose.second);
		}
	}

	// Update the position of the rest of the bones.
	m_base->accept(*m_updateVisitor);
	++m_frameCount;
	m_updateVisitor->setTraversalNumber(m_frameCount);
}

bool OsgSkeletonRepresentation::doInitialize()
{
	bool result = true;

	std::string shaderFilename;
	if (m_skinningShaderFileName.empty())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
				<< "OsgSkeletonRepresentation::doInitialize(): Skinning shader file not set.";
		result = false;
	}
	else
	{
		getRuntime()->getApplicationData()->tryFindFile(m_skinningShaderFileName, &shaderFilename);
	}

	if (result)
	{
		if (shaderFilename.empty())
		{
			SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
					<< "OsgSkeletonRepresentation::doInitialize(): Skinning shader file (" << m_skinningShaderFileName
					<< ") not found.";
			result = false;
		}
		else
		{
			m_skinningShader = new osg::Shader(osg::Shader::VERTEX);
			result = m_skinningShader->loadShaderSourceFromFile(shaderFilename);
		}
	}

	if (result)
	{
		if (m_model == nullptr || std::dynamic_pointer_cast<OsgModel>(m_model) == nullptr)
		{
			SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
					<< "OsgSkeletonRepresentation::doInitialize(): model is not set.";
			result = false;
		}
		else
		{
			m_skeleton = dynamic_cast<osg::Node*>(std::dynamic_pointer_cast<OsgModel>(m_model)->getOsgNode().get());
			result = nullptr != m_skeleton;
			SURGSIM_ASSERT(result)
					<< "OsgSkeletonRepresentation::doInitialize(): model does not have a valid osgNode.";
		}
	}

	if (result)
	{
		// Traverse the skeleton and collect bone data.
		BoneMapCreator mapCreator(m_skinningShader);
		mapCreator.traverse(*m_skeleton.get());
		m_boneData = mapCreator.getMap();
		if (m_boneData.size() == 0)
		{
			SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
					<< "Could not find any osgAnimation::Bone nodes in tree with root <" + m_skeleton->getName() + ">";
		}
		m_base = mapCreator.getRootTransform();

		// Setup the transform updater for the skeleton.
		m_updateVisitor = new osgUtil::UpdateVisitor();
		m_frameCount = 0;
		m_updateVisitor->setTraversalNumber(m_frameCount);
		m_base->accept(*m_updateVisitor);

		// Add the bone skeleton as a child to m_transform
		m_transform->addChild(m_base.get());
	}

	return result;
}

}; // Graphics
}; // SurgSim
