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

#ifndef SURGSIM_GRAPHICS_CURVEREPRESENTATION_H
#define SURGSIM_GRAPHICS_CURVEREPRESENTATION_H

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Graphics/Representation.h"

namespace SurgSim
{

namespace Graphics
{

/// This implements a graphical object to draw an interpolated curve, it accepts a series of control points, the
/// number of segments in the curve will depend on the number of control points and the value returned from
/// getSubdivisions().
/// This class also provides the ad-hoc "Vertices" property, this means it can receive a
/// \sa DataStructures::VerticesPlain structure as a property via setValue()
class CurveRepresentation : public virtual Representation
{
public:
	/// Constructor
	/// \param name the name of the representation
	explicit CurveRepresentation(const std::string& name);

	/// Sets the number of intermediate points the get generated between each two control points
	/// \param num number of interpolated points
	virtual void setSubdivisions(size_t num) = 0;

	/// \return the number of interpolated points between control points
	virtual size_t getSubdivisions() const = 0;

	/// Sets the tension (tau) parameter of the Catmull Rom interpolation, needs to be between 0.0 and 1.0
	/// \param tension the tension
	virtual void setTension(double tension) = 0;

	/// \return the tension currently used
	virtual double getTension() const = 0;

	/// Sets the color for the curve
	/// \param color the new color to be used
	virtual void setColor(const SurgSim::Math::UnalignedVector4d& color) = 0;

	/// \return the current color for this curve
	virtual Math::UnalignedVector4d getColor() const = 0;

	/// Sets the line width to be used for drawing this curve
	/// \param width the new width to be used
	virtual void setWidth(double width) = 0;

	/// \return the current width
	virtual double getWidth() const = 0;

	/// Sets up whether to use anti aliasing on the curve or not
	/// \param val if true anti aliasing will be turned on
	virtual void setAntiAliasing(bool val) = 0;

	/// \return whether the curve is currently being drawn with anti aliasing
	virtual bool isAntiAliasing() const = 0;

	/// Updates the control points for this class, this will cause a new curve to be generated on the next update
	/// \note this method is threadsafe
	/// \throws if the number of control points is < 2
	/// \param vertices new vertices to be used as control points
	void updateControlPoints(const DataStructures::VerticesPlain& vertices);

	/// Updates the control points for this class, this will cause a new curve to be generated on the next update
	/// move support.
	/// \note this method is threadsafe
	/// \throws if the number of control points is < 2
	/// \param vertices new vertices to be used as control points
	void updateControlPoints(DataStructures::VerticesPlain&& vertices);

protected:

	/// Container control points, threadsafe access when updating.
	Framework::LockedContainer<DataStructures::VerticesPlain> m_locker;

};

}
}

#endif
