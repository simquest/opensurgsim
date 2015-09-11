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

using SurgSim::Devices::OpenNIDevice;
using SurgSim::Input::DeviceInterface;
using SurgSim::Math::Vector3d;


class ImageGlutWindow : public SurgSim::Input::InputConsumerInterface
{
public:
	explicit ImageGlutWindow(const std::vector<std::string>& imageNames)
	{
		const size_t numImages = imageNames.size();
		const SurgSim::Math::Vector2d yRange(-1.0, 1.0);
		SurgSim::Math::Vector2d xRange;
		for (size_t i = 0; i < numImages; i++)
		{
			xRange = SurgSim::Math::Vector2d(i - numImages, i - numImages + 1) / numImages;
			auto view = std::make_shared<GlutImage>(Eigen::AlignedBox<double, 2>(xRange, yRange));
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
		SurgSim::DataStructures::DataGroup::ImageType data;
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
