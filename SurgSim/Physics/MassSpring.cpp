// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include "SurgSim/Physics/MassSpring.h"

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/MassSpringPlyReaderDelegate.h"
#include "SurgSim/Physics/Spring.h"

namespace SurgSim
{
namespace Physics
{

SURGSIM_REGISTER(SurgSim::Framework::Asset, SurgSim::Physics::MassSpring, MassSpring)

MassSpring::MassSpring()
{
}

void MassSpring::addMass(const std::shared_ptr<Mass> mass)
{
	m_masses.push_back(mass);
}

void MassSpring::addSpring(const std::shared_ptr<Spring> spring)
{
	m_springs.push_back(spring);
}

size_t MassSpring::getNumMasses() const
{
	return m_masses.size();
}

size_t MassSpring::getNumSprings() const
{
	return m_springs.size();
}

const std::vector<std::shared_ptr<Mass>>& MassSpring::getMasses() const
{
	return m_masses;
}

const std::vector<std::shared_ptr<Spring>>& MassSpring::getSprings() const
{
	return m_springs;
}

const std::shared_ptr<Mass>& MassSpring::getMass(size_t nodeId) const
{
	SURGSIM_ASSERT(nodeId < getNumMasses()) << "Invalid node id to request a mass from";
	return m_masses[nodeId];
}

const std::shared_ptr<Spring>& MassSpring::getSpring(size_t springId) const
{
	SURGSIM_ASSERT(springId < getNumSprings()) << "Invalid spring id";
	return m_springs[springId];
}

size_t MassSpring::addBoundaryCondition(size_t boundaryCondition)
{
	m_boundaryConditions.push_back(boundaryCondition);
	return m_boundaryConditions.size() - 1;
}

const std::vector<size_t>& MassSpring::getBoundaryConditions() const
{
	return m_boundaryConditions;
}

std::vector<size_t>& MassSpring::getBoundaryConditions()
{
	return m_boundaryConditions;
}

size_t MassSpring::getBoundaryCondition(size_t id) const
{
	return m_boundaryConditions[id];
}

size_t MassSpring::addElement(const std::vector<size_t>& nodeIds)
{
	SURGSIM_ASSERT((m_nodeIds.size() == 0) || (nodeIds.size() == getNumNodesPerElement())) <<
		"Cannot add an element with " << nodeIds.size() << " nodes to a MassSpring that already has an element with " <<
		getNumNodesPerElement() << " nodes.";
	m_nodeIds.push_back(nodeIds);
	return m_nodeIds.size();
}

const std::vector<size_t>& MassSpring::getNodeIds(size_t index) const
{
	return m_nodeIds[index];
}

size_t MassSpring::getNumElements() const
{
	return m_nodeIds.size();
}

size_t MassSpring::getNumNodesPerElement() const
{
	SURGSIM_ASSERT(m_nodeIds.size() > 0) <<
		"Cannot get the number of nodes per element of a MassSpring before adding elements.";
	return m_nodeIds[0].size();
}

void MassSpring::setRadius(double radius)
{
	m_radius.setValue(radius);
}

const DataStructures::OptionalValue<double>& MassSpring::getRadius() const
{
	return m_radius;
}

void MassSpring::setThickness(double thickness)
{
	m_thickness.setValue(thickness);
}

const DataStructures::OptionalValue<double>& MassSpring::getThickness() const
{
	return m_thickness;
}

bool MassSpring::save(const std::string& fileName, double physicsLength) const
{
	std::fstream out(fileName, std::ios::out);

	if (out.is_open())
	{
		out << "ply" << std::endl;
		out << "format ascii 1.0" << std::endl;
		out << "comment Created by OpenSurgSim, www.opensurgsim.org" << std::endl;
		out << "element vertex " << getNumVertices() << std::endl;
		out << "property double x\nproperty double y\nproperty double z\nproperty double mass" << std::endl;
		if (getNumElements() > 0)
		{
			if (getNumNodesPerElement() == 2)
			{
				out << "element 1d_element " << getNumElements() << std::endl;
				out << "property list uint uint vertex_indices" << std::endl;
				out << "element radius 1" << std::endl;
				out << "property double value" << std::endl;
			}
			else if (getNumNodesPerElement() == 3)
			{
				out << "element 2d_element " << getNumElements() << std::endl;
				out << "property list uint uint uint vertex_indices" << std::endl;
				out << "element thickness 1" << std::endl;
				out << "property double value" << std::endl;
			}
			else if (getNumNodesPerElement() == 4)
			{
				out << "element 3d_element " << getNumElements() << std::endl;
				out << "property list uint uint uint uint vertex_indices" << std::endl;
			}
			else if (getNumNodesPerElement() == 8)
			{
				out << "element 3d_element " << getNumElements() << std::endl;
				out << "property list uint uint uint uint uint uint uint uint vertex_indices" << std::endl;
			}
		}
		bool hasLinearSprings = (getNumSprings() > 0) &&
			(std::dynamic_pointer_cast<LinearSpring>(getSpring(0)) != nullptr);
		if (hasLinearSprings)
		{
			out << "element spring " << getNumSprings() << std::endl;
			out << "property list uint uint vertex_indices" << std::endl;
			out << "property double stiffness" << std::endl;
			out << "property double damping" << std::endl;
		}
		out << "element boundary_condition " << getBoundaryConditions().size() << std::endl;
		out << "property uint vertex_index" << std::endl;
		out << "end_header" << std::endl;
		for (const auto& vertex : getVertices())
		{
			out << vertex.position[0] << " " << vertex.position[1] << " " << vertex.position[2] << " " <<
				vertex.data.getMass() << std::endl;
		}

		for (size_t i = 0; i < getNumElements(); ++i)
		{
			const auto& nodeIds = getNodeIds(i);
			out << nodeIds.size();
			for (const auto& nodeId : nodeIds)
			{
				out << " " << nodeId;
			}
			out << std::endl;
		}

		if (getNumNodesPerElement() == 2)
		{
			if (m_radius.hasValue())
			{
				out << m_radius.getValue() << std::endl;
			}
			else
			{
				out << physicsLength << std::endl;
			}
		}
		if (getNumNodesPerElement() == 3)
		{
			if (m_thickness.hasValue())
			{
				out << m_thickness.getValue() << std::endl;
			}
			else
			{
				out << physicsLength << std::endl;
			}
		}
		if (hasLinearSprings)
		{
			for (const auto& spring : getSprings())
			{
				const auto& linearSpring = std::static_pointer_cast<LinearSpring>(spring);
				out << "2";
				for (const auto& nodeId : linearSpring->getNodeIds())
				{
					out << " " << nodeId;
				}
				out << " " << linearSpring->getStiffness() << " " << linearSpring->getDamping() << std::endl;
			}
		}
		for (const auto& boundary : getBoundaryConditions())
		{
			out << boundary << std::endl;
		}

		if (out.bad())
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__
				<< "There was a problem writing " << fileName;
		}

		out.close();
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__
			<< "Could not open " << fileName << " for writing.";
		return false;
	}
	return true;
}

bool MassSpring::doLoad(const std::string & filePath)
{
	SurgSim::DataStructures::PlyReader reader(filePath);
	if (!reader.isValid())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "'" << filePath << "' is an invalid .ply file.";
		return false;
	}

	auto delegate = std::make_shared<MassSpringPlyReaderDelegate>(this->shared_from_this());
	if (!reader.parseWithDelegate(delegate))
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "The input file '" << filePath << "' does not have the property required by MassSpring.";
		return false;
	}

	return true;
}

} // namespace Physics
} // namespace SurgSim
