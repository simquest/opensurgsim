#include "CameraImplementation.h"

using SurgSim::Graphics::Osg::CameraImplementation;

CameraImplementation::CameraImplementation() : SurgSim::Graphics::CameraImplementation(), m_camera(new osg::Camera())
{
}

CameraImplementation::~CameraImplementation()
{
}