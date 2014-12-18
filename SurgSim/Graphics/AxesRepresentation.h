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

#ifndef SURGSIM_GRAPHICS_AXESREPRESENTATION_H
#define SURGSIM_GRAPHICS_AXESREPRESENTATION_H

#include "SurgSim/Graphics/Representation.h"

namespace SurgSim
{
namespace Graphics
{

/// Displays the coordinate axes, as three lines from the origin
/// default size is 1.0, the X/Y/Z axis are indicated by R/G/B respectively
class AxesRepresentation : public virtual Representation
{
public:

	/// Constructor
	explicit AxesRepresentation(const std::string& name) : Representation(name)
	{
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(AxesRepresentation, double, Size, getSize, setSize);
	};

	/// Sets the size of the shown axes.
	/// \param	val	The value.
	virtual void setSize(double val) = 0;

	/// Gets the current size.
	/// \return	The size.
	virtual double getSize() = 0;
private:

};

}; // Graphics
}; // SurgSim

#endif