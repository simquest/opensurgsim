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

#include "SurgSim/Testing/VisualTestCommon/GlutRenderer.h"

#include <string>

using SurgSim::Math::Vector3d;
using SurgSim::Math::RigidTransform3d;


int GlutRenderer::m_width = 1024;
int GlutRenderer::m_height = 768;

std::shared_ptr<GlutCamera> GlutRenderer::m_camera = nullptr;
std::vector< std::shared_ptr<GlutRenderObject> > GlutRenderer::m_objects =
	std::vector< std::shared_ptr<GlutRenderObject> >();


GlutRenderObject::~GlutRenderObject()
{
}

void GlutSquare::draw()
{
	glEnable(GL_LIGHTING);

	glPushMatrix();

	glTranslated(pose.translation().x(), pose.translation().y(), pose.translation().z());
	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(pose.rotation());
	glRotated(angleAxis.angle() * 180.0 / M_PI, angleAxis.axis().x(), angleAxis.axis().y(), angleAxis.axis().z());

	Vector3d squarePoints[4];
	squarePoints[0] = - (halfSize * planeDirectionX) + (halfSize * planeDirectionY);
	squarePoints[1] =   (halfSize * planeDirectionX) + (halfSize * planeDirectionY);
	squarePoints[2] =   (halfSize * planeDirectionX) - (halfSize * planeDirectionY);
	squarePoints[3] = - (halfSize * planeDirectionX) - (halfSize * planeDirectionY);

	Vector3d normal = (squarePoints[1] - squarePoints[0]).cross(squarePoints[2] - squarePoints[0]);
	normal.normalize();

	glColor3d(color.x(), color.y(), color.z());
	glBegin(GL_QUADS);

	// Front side
	glNormal3d(normal.x(), normal.y(), normal.z());
	for (int i = 0; i < 4; ++i)
	{
		glVertex3d(squarePoints[i].x(), squarePoints[i].y(), squarePoints[i].z());
	}
	// Back side
	glNormal3d(- normal.x(), - normal.y(), - normal.z());
	for (int i = 0; i < 4; ++i)
	{
		glVertex3d(squarePoints[3 - i].x(), squarePoints[3 - i].y(), squarePoints[3 - i].z());
	}

	glEnd();

	glPopMatrix();
}

void GlutAxes::draw()
{
	glDisable(GL_LIGHTING);

	glPushMatrix();

	glTranslated(pose.translation().x(), pose.translation().y(), pose.translation().z());
	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(pose.rotation());
	glRotated(angleAxis.angle() * 180.0 / M_PI, angleAxis.axis().x(), angleAxis.axis().y(), angleAxis.axis().z());

	Vector3d axesPoints[4];
	axesPoints[0] = Vector3d(0.0, 0.0, 0.0);
	axesPoints[1] = (length * Vector3d(1.0, 0.0, 0.0));
	axesPoints[2] = (length * Vector3d(0.0, 1.0, 0.0));
	axesPoints[3] = (length * Vector3d(0.0, 0.0, 1.0));

	glLineWidth(width);

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

	glPopMatrix();
}

void GlutSphere::draw()
{
	glEnable(GL_LIGHTING);

	glPushMatrix();

	glTranslated(pose.translation().x(), pose.translation().y(), pose.translation().z());
	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(pose.rotation());
	glRotated(angleAxis.angle() * 180.0 / M_PI, angleAxis.axis().x(), angleAxis.axis().y(), angleAxis.axis().z());

	glColor3d(color.x(), color.y(), color.z());

	gluQuadricOrientation(quadratic, GLU_OUTSIDE);
	gluSphere(quadratic, radius, 32, 32);

	glPopMatrix();
}

void GlutGroup::draw()
{
	glPushMatrix();

	glTranslated(pose.translation().x(), pose.translation().y(), pose.translation().z());
	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(pose.rotation());
	glRotated(angleAxis.angle() * 180.0 / M_PI, angleAxis.axis().x(), angleAxis.axis().y(), angleAxis.axis().z());

	for (auto it = children.cbegin(); it != children.cend(); ++it)
	{
		(*it)->draw();
	}

	glPopMatrix();
}

void GlutRenderer::initialize()
{
	char* argv[1];
	int argc = 1;
	std::string title = "GlutWindow";
	std::vector<char> charString(title.size() + 1);
	std::copy(title.begin(), title.end(), charString.begin());
	argv[0] = &charString[0];
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(m_width, m_height);
	glutCreateWindow("Simple GLUT display");

	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
}
void GlutRenderer::display()
{
	glViewport(0, 0, (GLsizei) m_width, (GLsizei) m_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (m_camera)
	{
		gluPerspective(m_camera->fovY, static_cast<float>(m_width) / m_height, m_camera->zNear, m_camera->zFar);
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (m_camera)
	{
		gluLookAt(m_camera->eye.x(), m_camera->eye.y(), m_camera->eye.z(),
				  m_camera->center.x(), m_camera->center.y(), m_camera->center.z(),
				  m_camera->up.x(), m_camera->up.y(), m_camera->up.z());
	}

	glEnable(GL_LIGHT0);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	drawObjects();
	glutSwapBuffers();

	glutPostRedisplay();
}
