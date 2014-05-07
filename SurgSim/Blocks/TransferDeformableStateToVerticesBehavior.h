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

#ifndef SURGSIM_BLOCKS_TRANSFERDEFORMABLESTATETOVERTICESBEHAVIOR_H
#define SURGSIM_BLOCKS_TRANSFERDEFORMABLESTATETOVERTICESBEHAVIOR_H

#include "SurgSim/Framework/Behavior.h"

namespace SurgSim
{

namespace Math
{
class OdeState;
}

namespace DataStructures
{
template <class VertexData>
class Vertices;
}

namespace Blocks
{

/// Behavior to copy positions of an ode state to a vertices based objects.
/// For example, this behavior is used to send a deformable representation state to a point cloud in graphics.
template <class VertexData>
class TransferDeformableStateToVerticesBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	/// \param	from	OdeState to get the positions from
	/// \param	to	Vertices to set the positions into
	TransferDeformableStateToVerticesBehavior(const std::string& name,
		std::shared_ptr<SurgSim::Math::OdeState> from,
		std::shared_ptr<SurgSim::DataStructures::Vertices<VertexData>> to);

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt);

private:
	/// Initialize the behavior
	virtual bool doInitialize() override;

	/// Wakeup the behavior, which simply do a copy (same as update)
	virtual bool doWakeUp() override;

	/// Transfer the data from an OdeState m_from to Vertices m_to
	/// \param doInitialization True if the recipient should be initialized if needed, False otherwise
	/// \note if doInitialization is true and Vertices is empty, it will be filled accordingly
	/// \note with a default vertex data instanciation (if VertexData type is not 'void')
	void transfer(bool doInitialization = false);

	/// Deformable representation state to get the positions from
	std::shared_ptr<SurgSim::Math::OdeState> m_from;

	/// Vertices to set the positions into
	std::shared_ptr<SurgSim::DataStructures::Vertices<VertexData>> m_to;
};

};  // namespace Blocks

};  // namespace SurgSim

#include "SurgSim/Blocks/TransferDeformableStateToVerticesBehavior-inl.h"

#endif  // SURGSIM_BLOCKS_TRANSFERDEFORMABLESTATETOVERTICESBEHAVIOR_H
