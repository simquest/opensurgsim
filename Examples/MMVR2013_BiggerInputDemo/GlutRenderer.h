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

#ifndef GLUT_RENDERER_H
#define GLUT_RENDERER_H

#include <vector>
#include <GL/glut.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/RigidTransform.h>

struct GlutRenderObject
{
	SurgSim::Math::RigidTransform3d pose;

	GlutRenderObject(const SurgSim::Math::RigidTransform3d& pose) : pose(pose)
	{
	}

	virtual void draw() const = 0;
};

struct GlutSquare : public GlutRenderObject
{
	/// The unit direction along one of the pairs edges of the square.
	SurgSim::Math::Vector3d planeDirectionX;
	/// The unit direction along the other pair of edges of the square.
	SurgSim::Math::Vector3d planeDirectionY;
	double halfSize;
	SurgSim::Math::Vector3d color;

	GlutSquare(double halfSize, const SurgSim::Math::Vector3d& color, const SurgSim::Math::RigidTransform3d& pose) : GlutRenderObject(pose),
		halfSize(halfSize), color(color), planeDirectionX(1.0, 0.0, 0.0), planeDirectionY(0.0, 0.0, 1.0)
	{
	}

	virtual void draw() const;
};

struct GlutAxes : GlutRenderObject
{
	double length;
	double width;

	GlutAxes(double length, double width, const SurgSim::Math::RigidTransform3d& pose) : GlutRenderObject(pose), length(length), width(width)
	{
	}

	virtual void draw() const;
};

struct GlutSphere : GlutRenderObject
{
	double radius;
	SurgSim::Math::Vector3d color;

	GlutSphere(double radius, const SurgSim::Math::Vector3d& color, const SurgSim::Math::RigidTransform3d& pose) :
	GlutRenderObject(pose), radius(radius), color(color), quadratic(gluNewQuadric())
	{
	}

	virtual void draw() const;

private:
	GLUquadric* quadratic;
};

struct GlutTool : public GlutRenderObject
{
	std::shared_ptr<GlutSphere> sphere;
	std::shared_ptr<GlutAxes> axes;

	GlutTool(std::shared_ptr<GlutSphere> sphere, std::shared_ptr<GlutAxes> axes, 
		const SurgSim::Math::RigidTransform3d& pose) : GlutRenderObject(pose), sphere(sphere), axes(axes)
	{
	}

	virtual void draw() const;
};

struct GlutCamera 
{
	SurgSim::Math::Vector3d eye;
	SurgSim::Math::Vector3d center;
	SurgSim::Math::Vector3d up;
	double fovY;
	double near;
	double far;

	GlutCamera(const SurgSim::Math::Vector3d& eye, const SurgSim::Math::Vector3d& center, 
		const SurgSim::Math::Vector3d& up, const double fovY, double near, double far) : eye(eye), center(center), 
		up(up), fovY(fovY), near(near), far(far)
	{
	}
};

class GlutRenderer
{
public:
	static void initialize();

	static void run()
	{
		initialize();
		glutMainLoop(); 
	}

	static void setCamera(std::shared_ptr<GlutCamera> camera)
	{
		m_camera = camera;
	}

	static void addObject(std::shared_ptr<GlutRenderObject> object)
	{
		m_objects.push_back(object);
	}

private:
	static int m_width;
	static int m_height;

	static std::shared_ptr<GlutCamera> m_camera;
	static std::vector< std::shared_ptr<GlutRenderObject> > m_objects;

	static void reshape(GLint width, GLint height)
	{
		m_width = width;
		m_height = height;
		glViewport(0, 0, m_width, m_height);
	}

	static void display();

	static void drawObjects()
	{
		for (auto it = m_objects.cbegin(); it != m_objects.cend(); ++it)
		{
			(*it)->draw();
		}
	}
};


#endif // GLUT_RENDERER_H
