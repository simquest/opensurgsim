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

/// Abstract definition of an object that can render itself with Glut.
struct GlutRenderObject
{
	/// Pose (rotation and translation) of the object.
	SurgSim::Math::RigidTransform3d pose;

	/// Constructor initializes pose as identity (no rotation or translation)
	GlutRenderObject() : pose(SurgSim::Math::RigidTransform3d::Identity())
	{
	}

	/// Pure virtual draw method for subclasses to define how to draw themselves with Glut.
	virtual void draw() const = 0;
};

/// Square with center at local origin.
struct GlutSquare : public GlutRenderObject
{
	/// The unit direction along one of the pairs edges of the square.
	SurgSim::Math::Vector3d planeDirectionX;
	/// The unit direction along the other pair of edges of the square.
	SurgSim::Math::Vector3d planeDirectionY;
	/// One half of the edge length of the square, in meters.
	double halfSize;
	/// Color of the square.
	SurgSim::Math::Vector3d color;

	/// Constructor
	/// \param halfSize One half of the edge length of the square, in meters.
	/// \param color Color of the square.
	/// \param planeDirectionX The unit direction along one of the pairs edges of the square, default is X-axis.
	/// \param planeDirectionY The unit direction along the other pair of edges of the square, default is Y-axis.
	GlutSquare(double halfSize, const SurgSim::Math::Vector3d& color,
		const SurgSim::Math::Vector3d& planeDirectionX = SurgSim::Math::Vector3d(1.0, 0.0, 0.0),
		const SurgSim::Math::Vector3d& planeDirectionY = SurgSim::Math::Vector3d(0.0, 1.0, 0.0)) : 
	GlutRenderObject(), halfSize(halfSize), color(color), planeDirectionX(planeDirectionX), 
		planeDirectionY(planeDirectionY)
	{
	}

	/// Draws the square with Glut.
	virtual void draw() const;
};

/// Axes with center at local origin, red axis along the local X-axis, green axis along the local Y-axis, and blue axis
/// along the local Z-axis.
struct GlutAxes : GlutRenderObject
{
	/// Length of each axis, in meters.
	double length;
	/// Width of each axis, in pixels.
	double width;

	/// Constructor
	/// \param length Length of each axis, in meters.
	/// \param Width of each axis, in pixels.
	GlutAxes(double length, double width) : GlutRenderObject(), length(length), width(width)
	{
	}

	/// Draws the axes with Glut.
	virtual void draw() const;
};

/// Sphere with center at local origin.
struct GlutSphere : GlutRenderObject
{
	/// Radius of the sphere, in meters.
	double radius;
	/// Color of the sphere.
	SurgSim::Math::Vector3d color;

	/// Constructor
	/// \param radius Radius of the sphere, in meters.
	/// \param color Color of the sphere.
	GlutSphere(double radius, const SurgSim::Math::Vector3d& color) :
	radius(radius), color(color), quadratic(gluNewQuadric())
	{
	}

	/// Draws the sphere with Glut.
	virtual void draw() const;

private:
	/// GLU quadric object for the quadric operations required to build the sphere.
	GLUquadric* quadratic;
};

/// Group of objects which provides a transform hierarchy.
struct GlutGroup : public GlutRenderObject
{
	/// Children of this group.
	std::vector< std::shared_ptr<GlutRenderObject> > children;

	/// Constructor. The group is initialized with no children.
	GlutGroup() : GlutRenderObject()
	{
	}

	/// Draws the group with Glut and iterates through its children to draw them.
	virtual void draw() const;
};

/// Camera which controls the view of the scene.
struct GlutCamera 
{
	/// Eye position.
	SurgSim::Math::Vector3d eye;
	/// Center (look at) position.
	SurgSim::Math::Vector3d center;
	/// Up direction.
	SurgSim::Math::Vector3d up;
	/// Field of view angle (in degrees) in the vertical direction.
	double fovY;
	/// Near clipping plane distance from camera, in meters.
	double near;
	/// Far clipping plane distance from camera, in meters.
	double far;

	/// Constructor
	/// \param eye Eye position.
	/// \param center Center (look at) position.
	/// \param up Up direction.
	/// \param fovY Field of view angle (in degrees) in the vertical direction.
	/// \param near Near clipping plane distance from camera, in meters.
	/// \param far Far clipping plane distance from camera, in meters.
	GlutCamera(const SurgSim::Math::Vector3d& eye, const SurgSim::Math::Vector3d& center, 
		const SurgSim::Math::Vector3d& up, const double fovY, double near, double far) : eye(eye), center(center), 
		up(up), fovY(fovY), near(near), far(far)
	{
	}
};

/// Simple static class renderer built on Glut.
class GlutRenderer
{
public:
	/// Initializes and runs the Glut main loop. This function will block until the Glut graphics window is closed.
	static void run()
	{
		initialize();
		glutMainLoop(); 
	}

	/// Sets the camera used to control the view of the scene.
	/// \param camera View camera.
	static void setCamera(std::shared_ptr<GlutCamera> camera)
	{
		m_camera = camera;
	}

	/// Adds an object to the scene.
	/// \param object Scene object.
	static void addObject(std::shared_ptr<GlutRenderObject> object)
	{
		m_objects.push_back(object);
	}

private:
	/// Width of the window.
	static int m_width;
	/// Height of the window.
	static int m_height;

	/// Camera which controls the view of the scene.
	static std::shared_ptr<GlutCamera> m_camera;
	/// Objects in the scene.
	static std::vector< std::shared_ptr<GlutRenderObject> > m_objects;

	/// Initializes the Glut window.
	static void initialize();

	/// Glut reshape function which handles the resizing of the window.
	static void reshape(GLint width, GLint height)
	{
		m_width = width;
		m_height = height;
		glViewport(0, 0, m_width, m_height);
	}

	/// Glut display function which handles the drawing of the scene.
	static void display();

	/// Iterates through the scene objects to draw them.
	static void drawObjects()
	{
		for (auto it = m_objects.cbegin(); it != m_objects.cend(); ++it)
		{
			(*it)->draw();
		}
	}
};


#endif // GLUT_RENDERER_H
