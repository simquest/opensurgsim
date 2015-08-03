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

#ifndef SURGSIM_GRAPHICS_OSGCURVE_H
#define SURGSIM_GRAPHICS_OSGCURVE_H

#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/CurveRepresentation.h"

#include <osg/Array>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace osg
{
class Geometry;
class DrawArrays;
}

namespace SurgSim
{

namespace Graphics
{

class OsgCurveRepresentation : public OsgRepresentation, public CurveRepresentation
{
public:
	/// Constructor
	OsgCurveRepresentation(const std::string& name);

	virtual bool doInitialize() override;

	virtual bool doWakeUp() override;

	virtual void doUpdate(double dt) override;

	void setSubdivision(size_t num);

	size_t getSubdivision() const;

	void setTension(double tension);

	double getTension() const;

	void setColor(const SurgSim::Math::Vector4d& color);

	Math::Vector4d getColor() const;

private:
	void updateGraphics(const ControlPointType& controlPoints);
	osg::ref_ptr<osg::Geometry> m_geometry;
	osg::ref_ptr<osg::Vec3Array> m_vertexData;
	osg::ref_ptr<osg::DrawArrays> m_drawArrays;

	Math::Vector4d m_color;

	size_t m_subdivision;
	double m_tension;
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}
}

#endif
