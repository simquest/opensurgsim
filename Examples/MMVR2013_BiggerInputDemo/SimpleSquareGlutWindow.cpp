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

#include "SimpleSquareGlutWindow.h"

#include <GL/glut.h>

using SurgSim::Input::DataGroup;

using SurgSim::Math::Vector3d;
using SurgSim::Math::RigidTransform3d;

class Renderer
{
public:
	static void initialize()
	{
		char* argv[1];
		int argc = 1;
		argv[0] = _strdup("GlutWindow");
		glutInit(&argc, argv); 
		glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA); 
		glutInitWindowPosition(100, 100); 
		glutInitWindowSize(m_width, m_height); 
		glutCreateWindow("MMVR2013 Bigger Input Demo"); 

		glDisable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glShadeModel(GL_SMOOTH);
		glClearColor(0.1, 0.1, 0.1, 1.0);

		glutDisplayFunc(display);
		glutReshapeFunc(reshape);
	}

	static void run()
	{
		initialize();
		glutMainLoop(); 
	}

	static void setSquare(const Vector3d& center, double halfSize, const Vector3d& normal, Vector3d& color)
	{
		m_square.set(center, halfSize, normal, color);
	}

	static void setLocalTipPoint(const Vector3d& point)
	{
		m_localTipPoint = point;
	}
	static void setTipSphereColor(const Vector3d& color)
	{
		m_tipSphereColor = color;
	}
	static void setTipSphereRadius(double radius)
	{
		m_tipSphereRadius = radius;
	}
	static void setAxisLength(double length)
	{
		m_axisLength = length;
	}
	static void setAxisWidth(double width)
	{
		m_axisWidth = width;
	}
	static void setPose(const RigidTransform3d& pose)
	{
		m_pose = pose;
	}

	static void setCamera(const Vector3d& eye, const Vector3d& center, const Vector3d& up, double fovY,
		double near, double far)
	{
		m_camera.set(eye, center, up, fovY, near, far);
	}

private:
	/// The unit direction along one of the pairs edges of the square.
	static Vector3d m_planeDirectionX;
	/// The unit direction along the other pair of edges of the square.
	static Vector3d m_planeDirectionY;

	static int m_width;
	static int m_height;

	struct Square
	{
		Vector3d center;
		double halfSize;
		Vector3d normal;
		Vector3d color;

		Square(const Vector3d& center, double halfSize, const Vector3d& normal, const Vector3d& color)
		{
			set(center, halfSize, normal, color);
		}

		void set(const Vector3d& center, double halfSize, const Vector3d& normal, const Vector3d& color)
		{
			this->center = center;
			this->halfSize = halfSize;
			this->normal = normal;
			this->color = color;
		}
	};
	static Square m_square;

	static RigidTransform3d m_pose;
	static Vector3d m_localTipPoint;
	static Vector3d m_tipSphereColor;
	static double m_tipSphereRadius;
	static double m_axisLength;
	static double m_axisWidth;

	static GLUquadric* quadratic;

	static Vector3d m_lightPosition;

	struct Camera 
	{
		Vector3d eye;
		Vector3d center;
		Vector3d up;
		double fovY;
		double near;
		double far;

		Camera(const Vector3d& eye, const Vector3d& center, const Vector3d& up, const double fovY,
			double near, double far)
		{
			set(eye, center, up, fovY, near, far);
		}

		void set(const Vector3d& eye, const Vector3d& center, const Vector3d& up, const double fovY,
			double near, double far)
		{
			this->eye = eye,
			this->center = center;
			this->up = up;
			this->fovY = fovY;
			this->near = near;
			this->far = far;
		}
	};
	static Camera m_camera;

	static void reshape(GLint width, GLint height)
	{
		m_width = width;
		m_height = height;
		glViewport(0, 0, m_width, m_height);
	}

	static void display()
	{
		glViewport(0, 0, (GLsizei) m_width, (GLsizei) m_height);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(m_camera.fovY, (float)m_width / m_height, m_camera.near, m_camera.far);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(m_camera.eye.x(), m_camera.eye.y(), m_camera.eye.z(),
			m_camera.center.x(), m_camera.center.y(), m_camera.center.z(),
			m_camera.up.x(), m_camera.up.y(), m_camera.up.z());

		glEnable(GL_LIGHT0);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		drawObjects();
		glutSwapBuffers();

		glutPostRedisplay();
	}

	static void drawObjects()
	{
		drawSquare();
		drawAxes();
		drawTipPoint();
	}

	static void drawSquare()
	{
		glEnable(GL_LIGHTING);

		Vector3d squarePoints[4];
		squarePoints[0] = m_square.center - (m_square.halfSize * m_planeDirectionX) + (m_square.halfSize * m_planeDirectionY);
		squarePoints[1] = m_square.center + (m_square.halfSize * m_planeDirectionX) + (m_square.halfSize * m_planeDirectionY);
		squarePoints[2] = m_square.center + (m_square.halfSize * m_planeDirectionX) - (m_square.halfSize * m_planeDirectionY);
		squarePoints[3] = m_square.center - (m_square.halfSize * m_planeDirectionX) - (m_square.halfSize * m_planeDirectionY);

		glColor3d(m_square.color.x(), m_square.color.y(), m_square.color.z());
		glBegin(GL_QUADS);
		glNormal3d(m_square.normal.x(), m_square.normal.y(), m_square.normal.z());
		for (int i = 0; i < 4; ++i)
		{
			glVertex3d(squarePoints[i].x(), squarePoints[i].y(), squarePoints[i].z());
		}
		glEnd();
	}

	static void drawAxes()
	{
		glDisable(GL_LIGHTING);

		Vector3d axesPoints[4];
		axesPoints[0] = m_pose * Vector3d(0.0, 0.0, 0.0);
		axesPoints[1] = m_pose * (m_axisLength * Vector3d(1.0, 0.0, 0.0));
		axesPoints[2] = m_pose * (m_axisLength * Vector3d(0.0, 1.0, 0.0));
		axesPoints[3] = m_pose * (m_axisLength * Vector3d(0.0, 0.0, 1.0));

		glLineWidth(m_axisWidth);
		glBegin(GL_LINES);
		glColor3d(1.0, 0.0, 0.0);
		glVertex3d(axesPoints[0].x(), axesPoints[0].y(), axesPoints[0].z());
		glVertex3d(axesPoints[1].x(), axesPoints[1].y(), axesPoints[1].z());
		glColor3d(0.0, 1.0, 0.0);
		glVertex3d(axesPoints[0].x(), axesPoints[0].y(), axesPoints[0].z());
		glVertex3d(axesPoints[2].x(), axesPoints[2].y(), axesPoints[2].z());
		glColor3d(0.0, 0.0, 1.0);
		glVertex3d(axesPoints[0].x(), axesPoints[0].y(), axesPoints[0].z());
		glVertex3d(axesPoints[3].x(), axesPoints[3].y(), axesPoints[3].z());
		glEnd();
	}

	static void drawTipPoint()
	{
		glEnable(GL_LIGHTING);

		Vector3d tipPoint = m_pose * m_localTipPoint;

		glColor3d(m_tipSphereColor.x(), m_tipSphereColor.y(), m_tipSphereColor.z());

		glPushMatrix();
		glTranslated(tipPoint.x(), tipPoint.y(), tipPoint.z());
		gluQuadricOrientation(quadratic, GLU_OUTSIDE);
		gluSphere(quadratic, m_tipSphereRadius, 32, 32);
		glPopMatrix();
	}
};

Renderer::Square Renderer::m_square(Vector3d(0.0, 0.0, 0.0), 0.01, Vector3d(0.0, 0.0, 1.0), Vector3d(1.0, 1.0, 1.0));

Vector3d Renderer::m_localTipPoint(0.0, 0.0, 0.0);
RigidTransform3d Renderer::m_pose(RigidTransform3d::Identity());
Vector3d Renderer::m_tipSphereColor(1.0, 1.0, 1.0);
double Renderer::m_tipSphereRadius = 0.01;
double Renderer::m_axisLength = 0.01;
double Renderer::m_axisWidth = 1.0;

int Renderer::m_width = 1024;
int Renderer::m_height = 768;

Vector3d Renderer::m_planeDirectionX(1.0, 0.0, 0.0);
Vector3d Renderer::m_planeDirectionY(0.0, 0.0, 1.0);

GLUquadric* Renderer::quadratic = gluNewQuadric();

Renderer::Camera Renderer::m_camera(Vector3d(0.0, 0.0, 1.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0), 45.0, 
	0.1, 1.0);


SimpleSquareGlutWindow::SimpleSquareGlutWindow()
{
	Renderer::setSquare(Vector3d(0.0, 0.0, 0.0), 0.050, Vector3d(0.0, 1.0, 0.0), Vector3d(1.0, 1.0, 1.0));
	Renderer::setTipSphereColor(Vector3d(1.0, 1.0, 1.0));
	Renderer::setTipSphereRadius(0.010);
	Renderer::setLocalTipPoint(Vector3d(0.0, 0.0, 0.0));
	Renderer::setPose(RigidTransform3d::Identity());
	Renderer::setAxisLength(0.025);
	Renderer::setAxisWidth(5.0);
	Renderer::setCamera(Vector3d(-0.15, 0.15, 0.3), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0), 45.0, 0.001, 1.0);

	m_renderThread = boost::thread(boost::ref(Renderer::run));
}

SimpleSquareGlutWindow::~SimpleSquareGlutWindow()
{
	if (m_renderThread.joinable())
	{
		m_renderThread.join();
	}
}

void SimpleSquareGlutWindow::handleInput(const std::string& device, const DataGroup& inputData)
{
	RigidTransform3d devicePose;
	bool button0, button1, button2, button3;
	if (! inputData.poses().get("pose", devicePose) || 
		! inputData.booleans().get("button0", button0) ||
		! inputData.booleans().get("button1", button1) ||
		! inputData.booleans().get("button2", button2) ||
		! inputData.booleans().get("button3", button3))
	{
		return;
	}
	
	Renderer::setPose(devicePose);

	Vector3d tipPointColor(1.0, 1.0, 1.0);
	if (button0)
	{
		tipPointColor.x() = 0.0;
	}
	if (button1)
	{
		tipPointColor.y() = 0.0;
	}
	if (button2)
	{
		tipPointColor.z() = 0.0;
	}
	if (button3)
	{
		tipPointColor *= 0.5;
	}
	Renderer::setTipSphereColor(tipPointColor);
}

bool SimpleSquareGlutWindow::requestOutput(const std::string& device, DataGroup* outputData)
{
	return true;
}
