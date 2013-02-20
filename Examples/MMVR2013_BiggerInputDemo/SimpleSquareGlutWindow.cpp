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
		glDisable(GL_LIGHTING);
		glShadeModel(GL_FLAT);
		glClearColor(0.1, 0.1, 0.1, 1.0);

		glutDisplayFunc(display);
		glutReshapeFunc(reshape);
	}

	static void run()
	{
		initialize();
		glutMainLoop(); 
	}

	static void setSquare(const Vector3d& center, double halfSize, const Vector3d& normal)
	{
		m_squareCenter = center;
		m_squareHalfSize = halfSize;
		m_squareNormal = normal;
	}
	static void setSquareColor(const Vector3d& color)
	{
		m_squareColor = color;
	}

	static void setLocalTipPoint(const Vector3d& point)
	{
		m_localTipPoint = point;
	}
	static void setTipPointColor(const Vector3d& color)
	{
		m_tipPointColor = color;
	}
	static void setAxisLength(double length)
	{
		m_axisLength = length;
	}
	static void setPose(const RigidTransform3d& pose)
	{
		m_pose = pose;
	}

private:
	/// The unit direction along one of the pairs edges of the square.
	static Vector3d m_planeDirectionX;
	/// The unit direction along the other pair of edges of the square.
	static Vector3d m_planeDirectionY;

	static int m_width;
	static int m_height;

	static Vector3d m_squareColor;
	static Vector3d m_squareCenter;
	static double m_squareHalfSize;
	static Vector3d m_squareNormal;

	static RigidTransform3d m_pose;
	static Vector3d m_localTipPoint;
	static Vector3d m_tipPointColor;
	static double m_axisLength;

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
		gluPerspective(45.0, (float)m_width / m_height, 0.001, 1.0);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(-0.15, 0.15, 0.3,  0.0, 0.0, 0.0,  0.0, 1.0, 0.0);

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
		Vector3d m_squarePoints[4];
		m_squarePoints[0] = m_squareCenter - (m_squareHalfSize * m_planeDirectionX) + (m_squareHalfSize * m_planeDirectionY);
		m_squarePoints[1] = m_squareCenter + (m_squareHalfSize * m_planeDirectionX) + (m_squareHalfSize * m_planeDirectionY);
		m_squarePoints[2] = m_squareCenter + (m_squareHalfSize * m_planeDirectionX) - (m_squareHalfSize * m_planeDirectionY);
		m_squarePoints[3] = m_squareCenter - (m_squareHalfSize * m_planeDirectionX) - (m_squareHalfSize * m_planeDirectionY);

		glColor3d(m_squareColor.x(), m_squareColor.y(), m_squareColor.z());
		glBegin(GL_QUADS);
		for (int i = 0; i < 4; ++i)
		{
			glVertex3d(m_squarePoints[i].x(), m_squarePoints[i].y(), m_squarePoints[i].z());
		}
		glEnd();
	}

	static void drawAxes()
	{
		Vector3d axesPoints[4];
		axesPoints[0] = m_pose * Vector3d(0.0, 0.0, 0.0);
		axesPoints[1] = m_pose * (m_axisLength * Vector3d(1.0, 0.0, 0.0));
		axesPoints[2] = m_pose * (m_axisLength * Vector3d(0.0, 1.0, 0.0));
		axesPoints[3] = m_pose * (m_axisLength * Vector3d(0.0, 0.0, 1.0));

		glLineWidth(3.0);
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
		Vector3d tipPoint = m_pose * m_localTipPoint;

		glColor3d(m_tipPointColor.x(), m_tipPointColor.y(), m_tipPointColor.z());
		glPointSize(5.0);

		glBegin(GL_POINT);
		glVertex3d(tipPoint.x(), tipPoint.y(), tipPoint.z());
		glEnd();
	}
};

Vector3d Renderer::m_squareCenter(0.0, 0.0, 0.0);
Vector3d Renderer::m_squareNormal(0.0, 0.0, 1.0);
double Renderer::m_squareHalfSize = 0.01;
Vector3d Renderer::m_squareColor = Vector3d(1.0, 1.0, 1.0);

Vector3d Renderer::m_localTipPoint(0.0, 0.0, 0.0);
RigidTransform3d Renderer::m_pose(RigidTransform3d::Identity());
Vector3d Renderer::m_tipPointColor(1.0, 1.0, 1.0);
double Renderer::m_axisLength = 0.01;

int Renderer::m_width = 1024;
int Renderer::m_height = 768;

Vector3d Renderer::m_planeDirectionX(1, 0, 0);
Vector3d Renderer::m_planeDirectionY(0, 0, 1);

SimpleSquareGlutWindow::SimpleSquareGlutWindow()
{
	Renderer::setSquare(Vector3d(0, 0, 0), 0.050, Vector3d(0, -1, 0));
	Renderer::setTipPointColor(Vector3d(1.0, 1.0, 1.0));
	Renderer::setLocalTipPoint(Vector3d(0.0, 0.0, 0.0));
	Renderer::setPose(RigidTransform3d::Identity());
	Renderer::setAxisLength(0.01);

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
	Renderer::setTipPointColor(tipPointColor);
}

bool SimpleSquareGlutWindow::requestOutput(const std::string& device, DataGroup* outputData)
{
	return true;
}
