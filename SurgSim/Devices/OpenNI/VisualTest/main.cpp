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

#include <boost/thread.hpp>
#include <memory>
#include <string>
#include <vector>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/Image.h"
#include "SurgSim/Devices/OpenNI/OpenNIDevice.h"
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Testing/VisualTestCommon/GlutRenderer.h"

using SurgSim::Device::OpenNIDevice;
using SurgSim::Input::DeviceInterface;
using SurgSim::Math::Vector3d;

typedef SurgSim::DataStructures::DataGroup::ImageType ImageType;

struct GlutImage : GlutRenderObject
{
	SurgSim::Framework::LockedContainer<ImageType> image;

	GlutImage(int imageNum, int numImages) :
		m_imageNum(imageNum),
		m_numImages(numImages),
		m_firstRun(true)
	{
	}

	virtual void draw()
	{
		if (m_firstRun)
		{
		   glGenTextures(1, &m_texture);
		   m_firstRun = false;
		}

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(-m_numImages, m_numImages, -1, 1, -1.0, 1.0);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glEnable(GL_TEXTURE_2D);
		glEnable(GL_BLEND);
		glBindTexture(GL_TEXTURE_2D, m_texture);

		ImageType imageData;
		if(image.tryTakeChanged(&imageData))
		{
			float maxPixel = imageData.getAsVector().maxCoeff();
			if (maxPixel > 1.0)
			{
			   float scaleFactor = 1.0 / maxPixel;
			   glPixelTransferf(GL_RED_SCALE, scaleFactor);
			   glPixelTransferf(GL_BLUE_SCALE, scaleFactor);
			   glPixelTransferf(GL_GREEN_SCALE, scaleFactor);
			}
			else
			{
			   glPixelTransferf(GL_RED_SCALE, 1.0);
			   glPixelTransferf(GL_BLUE_SCALE, 1.0);
			   glPixelTransferf(GL_GREEN_SCALE, 1.0);
			}

			switch (imageData.getNumChannels())
			{
			case 1:
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageData.getWidth(), imageData.getHeight(), 0, GL_LUMINANCE,
						GL_FLOAT, imageData.getData());
				break;
			case 3:
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageData.getWidth(), imageData.getHeight(), 0, GL_RGB, GL_FLOAT,
						imageData.getData());
				break;
			}
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		}

		int xMin = -m_numImages + m_imageNum * 2;
		glColor4f(1,1,1,1);
		glBegin(GL_QUADS);

		glTexCoord2f(1, 1);
		glVertex2f(xMin, -1.0);

		glTexCoord2f(0, 1);
		glVertex2f(xMin+2, -1.0);

		glTexCoord2f(0, 0);
		glVertex2f(xMin+2, 1.0);

		glTexCoord2f(1, 0);
		glVertex2f(xMin, 1.0);

		glEnd();
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
	}

private:
	int m_imageNum;
	int m_numImages;
	unsigned int m_texture;
	bool m_firstRun;
};

class ImageGlutWindow : public SurgSim::Input::InputConsumerInterface
{
public:
	explicit ImageGlutWindow(const std::vector<std::string>& imageNames)
	{
		for (size_t i = 0; i < imageNames.size(); i++)
		{
			auto view = std::make_shared<GlutImage>(i, imageNames.size());
			GlutRenderer::addObject(view);
			m_views.insert(std::make_pair(imageNames[i], view));
		}
		m_renderThread = boost::thread(boost::ref(GlutRenderer::run));
	}

	~ImageGlutWindow()
	{
		if (m_renderThread.joinable())
		{
			m_renderThread.join();
		}
	}

	void initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override
	{
		for (auto view : m_views)
		{
			SURGSIM_ASSERT(inputData.images().hasEntry(view.first))
				<< "No image named '" << view.first << "' provided by the device: " << device;
		}
	}

	void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override
	{
		ImageType data;
		for (auto view : m_views)
		{
			if(inputData.images().get(view.first, &data))
			{
				view.second->image.set(std::move(data));
			}
		}
	}

private:
	boost::thread m_renderThread;
	std::unordered_map<std::string, std::shared_ptr<GlutImage>> m_views;
};

int main(int argc, char** argv)
{
	std::shared_ptr<DeviceInterface> device = std::make_shared<OpenNIDevice>("OpenNIDevice");
	if (!device->initialize())
	{
		std::cout << std::endl << "Could not initialize device: " << device->getName()
			<< std::endl << "--- Press Enter to quit the application! ---" << std::endl;
		getc(stdin);
		return 1;
	}

	std::vector<std::string> imagesToDraw;
	imagesToDraw.push_back("color");
	imagesToDraw.push_back("depth");
	std::shared_ptr<ImageGlutWindow> imageWindow = std::make_shared<ImageGlutWindow>(imagesToDraw);
	device->addInputConsumer(imageWindow);

	std::cout << std::endl << "**********************************************************************" << std::endl
		<< "OpenNI Visual Test" << std::endl << std::endl
		<< "When done, press Enter to quit the application." << std::endl
		<< "**********************************************************************" << std::endl;
	getc(stdin);

	std::cout << "Exiting..." << std::endl;
	device->removeInputConsumer(imageWindow);
	device.reset();
	exit(0);
}
