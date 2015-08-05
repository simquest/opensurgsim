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

#include "SurgSim/Graphics/OsgCurveRepresentation.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/DataStructures/Vertices.h"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Array>
#include <osg/PositionAttitudeTransform>

namespace
{

std::vector<SurgSim::Math::Vector3d> retrievePoints(
	const SurgSim::DataStructures::VerticesPlain& points)
{
	SURGSIM_ASSERT(points.getNumVertices() >= 2) << "Cannot apply CatmullRom with less than 2 points";

	std::vector<SurgSim::Math::Vector3d> result;
	result.reserve(points.getNumVertices() + 2);

	// Interpolate the 1st point (ghost) as the symmetric of P1 from P0: P-1 = P0 + P1P0
	result.push_back(2.0 * points.getVertexPosition(0) - points.getVertexPosition(1));
	for (size_t i = 0; i < points.getNumVertices(); ++i)
	{
		result.push_back(points.getVertexPosition(i));
	}
	// Interpolate the last point (ghost) as the symmetric of Pn-1 from Pn: Pn+1 = Pn + Pn-1Pn
	result.push_back(2.0 * points.getVertexPosition(points.getNumVertices() - 1) -
					 points.getVertexPosition(points.getNumVertices() - 2));

	return result;
}

SurgSim::Math::Vector3d computePoint(double s, const std::vector<SurgSim::Math::Vector3d>& x, double tau = 0.4)
{
	size_t numPoints = x.size();
	SURGSIM_ASSERT(s >= 0.0 && s <= static_cast<double>(numPoints - 3)) << "Out of range abscissa s=" << s <<
			" expected in [0.." << numPoints - 3 << "]";

	// We look for the 4 points corresponding to the requested segment
	// Keeping in mind that any upper bound (1.0 2.0...) should be considered the upper bound of the lower segment
	// and not the lower bound of the upper segment, with a special care to the abscissa s=0.0.
	size_t firstPointIndex = static_cast<size_t>(s);
	if (static_cast<double>(firstPointIndex) == s && s != 0.0)
	{
		firstPointIndex--;
	}
	SURGSIM_ASSERT(firstPointIndex < numPoints - 3);
	// We compute the local abscissa in the requested segment within [0..1].
	double localAbscissa = s - static_cast<double>(firstPointIndex);
	double SQlocalAbscissa = localAbscissa * localAbscissa;
	double CUlocalAbscissa = SQlocalAbscissa * localAbscissa;

	// 1st) Retrieve the 4 points on which the interpolation will happen
	const SurgSim::Math::Vector3d p[4] =
	{x[firstPointIndex] , x[firstPointIndex + 1], x[firstPointIndex + 2], x[firstPointIndex + 3]};

	return p[1] +
		   localAbscissa * (tau * (p[2] - p[0])) +
		   SQlocalAbscissa * (2.0 * tau * p[0] + (tau - 3.0) * p[1] + (3.0 - 2.0 * tau) * p[2] - tau * p[3]) +
		   CUlocalAbscissa * (-tau * p[0] + (2.0 - tau) * p[1] + (tau - 2.0) * p[2] + tau * p[3]);
}

}

namespace SurgSim
{

namespace Graphics
{

OsgCurveRepresentation::OsgCurveRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	CurveRepresentation(name),
	m_subdivision(100),
	m_tension(0.4)
{
	osg::Geode* geode = new osg::Geode();
	m_geometry = new osg::Geometry();
	m_vertexData = new osg::Vec3Array;

	m_geometry->setVertexArray(m_vertexData);

	// Normals
	m_normalData = new osg::Vec3Array;
	m_geometry->setNormalArray(m_normalData, osg::Array::BIND_PER_VERTEX);

	// At this stage there are no vertices in there
	m_drawArrays = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, m_vertexData->size());
	m_geometry->addPrimitiveSet(m_drawArrays);
	m_geometry->setUseDisplayList(false);
	m_geometry->setDataVariance(osg::Object::DYNAMIC);

	setColor(Math::Vector4d(1.0, 1.0, 1.0, 1.0));

	geode->addDrawable(m_geometry);
	m_transform->addChild(geode);
}

bool OsgCurveRepresentation::doInitialize()
{
	return true;
}

bool OsgCurveRepresentation::doWakeUp()
{
	return true;
}

void OsgCurveRepresentation::doUpdate(double dt)
{
	DataStructures::VerticesPlain controlPoints;
	if (m_locker.tryTakeChanged(&controlPoints))
	{
		updateGraphics(controlPoints);
	}
}

void OsgCurveRepresentation::setSubdivisions(size_t num)
{
	m_subdivision = num;
}

size_t OsgCurveRepresentation::getSubdivisions() const
{
	return m_subdivision;
}

void OsgCurveRepresentation::setTension(double tension)
{
	m_tension = tension;
}

double OsgCurveRepresentation::getTension() const
{
	return m_tension;
}

void OsgCurveRepresentation::updateGraphics(const DataStructures::VerticesPlain& controlPoints)
{
	const double stepsize = 1.0 / m_subdivision;

	auto points = retrievePoints(controlPoints);

	size_t numPoints = static_cast<size_t>(static_cast<double>(points.size() - 3) / stepsize);

	std::vector<SurgSim::Math::Vector3d> interpolated;
	interpolated.reserve(numPoints);

	double s = 0.0;
	while (s <= static_cast<double>(points.size() - 3))
	{
		interpolated.push_back(computePoint(s, points, m_tension));
		s += stepsize;
	}

	size_t vertexCount = interpolated.size();
	if (m_vertexData->size() != vertexCount)
	{
		m_vertexData->resize(vertexCount);
		m_normalData->resize(vertexCount);

		m_drawArrays->set(osg::PrimitiveSet::LINE_STRIP, 0, vertexCount);
		m_drawArrays->dirty();
	}

	size_t count = interpolated.size();

	Math::Vector3d vertex0;
	Math::Vector3d vertex1;
	Math::Vector3d normal;

	for (size_t i = 0; i < count; ++i)
	{
		vertex0 = interpolated[i];
		vertex1 = (i < count - 1) ? interpolated[i + 1] : points.back();

		// The normal carries the info about the segment vector
		// This should be a vertex attribute, rather than hidden
		normal = vertex1 - vertex0;

		(*m_normalData)[i][0] = normal[0];
		(*m_normalData)[i][1] = normal[1];
		(*m_normalData)[i][2] = normal[2];

		(*m_vertexData)[i][0] = vertex0[0];
		(*m_vertexData)[i][1] = vertex0[1];
		(*m_vertexData)[i][2] = vertex0[2];
	}

	m_vertexData->dirty();
	m_normalData->dirty();
}

void OsgCurveRepresentation::setColor(const SurgSim::Math::Vector4d& color)
{
	// Set the color of the particles to one single color by default
	osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(m_geometry->getColorArray());
	if (colors == nullptr)
	{
		colors = new osg::Vec4Array(1);
		m_geometry->setColorArray(colors);
		m_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	}
	(*colors)[0] = SurgSim::Graphics::toOsg(color);
	m_color = color;
}

Math::Vector4d OsgCurveRepresentation::getColor() const
{
	return m_color;
}

}
}