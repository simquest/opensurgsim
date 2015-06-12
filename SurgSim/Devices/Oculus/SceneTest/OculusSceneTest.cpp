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

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include "SurgSim/Blocks/Blocks.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Input/Input.h"
#include "SurgSim/Math/Math.h"

// Include this for linking only
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Devices/Oculus/OculusDevice.h"
#include "SurgSim/Devices/Oculus/OculusView.h"

using SurgSim;
using Math::Vector3d;

namespace po = boost::program_options;

int main(int argc, char* argv[])
{
	// Parse command-line parameters
	std::string sceneFileName;
	po::options_description visible("Allowed options");
	visible.add_options()("help", "produce help message")
	("config-file", po::value<std::string>(), "The config file to use");

	po::options_description hidden("Hidden options");
	hidden.add_options()("input-file", po::value<std::vector<std::string>>(), "input file");

	po::options_description all("All options");
	all.add(visible).add(hidden);

	po::positional_options_description positional;
	positional.add("input-file", -1);
	po::variables_map variables;
	po::store(po::command_line_parser(argc, argv).options(all).positional(positional).run(), variables);

	if (variables.count("help"))
	{
		std::cout << visible << "\n";
		return 1;
	}

	if (variables.count("input-file") == 0)
	{
		std::cout << "You need to supply one or more input files.\n";
		return 1;
	}

	std::shared_ptr<Framework::Runtime> runtime;

	if (variables.count("config-file") == 1)
	{
		runtime = std::make_shared<Framework::Runtime>(variables["config-file"].as<std::string>());
	}
	else
	{
		runtime = std::make_shared<Framework::Runtime>();
	}

	auto data = runtime->getApplicationData();

	runtime->addManager(std::make_shared<SurgSim::Graphics::OsgManager>());
	runtime->addManager(std::make_shared<SurgSim::Framework::BehaviorManager>());

	std::string sceneFile;
	if (variables.count("scene") == 1)
	{
		runtime->loadScene(variables["config-file"].as<std::string>());
	}

	std::array<std::string, 2> devices =
	{
		"SurgSim::Device::OculusDevice",
		"SurgSim::Device::IdentityPoseDevice"
	};

	std::shared_ptr<SurgSim::Input::DeviceInterface> device;
	for (auto deviceName : devices)
	{
		try
		{
			device = SurgSim::Input::DeviceInterface::getFactory().create(deviceName, "Tracker");
		}
		catch (std::exception e)
		{
			continue;
		}
		if (device->initialize())
		{
			break;
		}
		else
		{
			device = nullptr;
		}
	}
	SURGSIM_ASSERT(device != nullptr) << "Could not initialize any kind of tracking device.";

	auto inputManager = std::make_shared<SurgSim::Input::InputManager>();
	runtime->addManager(inputManager);
	inputManager->addDevice(device);

	auto scene = runtime->getScene();

	auto files = variables["input-file"].as<std::vector<std::string>>();
	for (auto file : files)
	{
		auto appData = runtime->getApplicationData();
		std::string path;

#ifdef WIN32
		// Fix windows backslashes coming in from the command-line, these may be absolute paths
		std::replace_if(file.begin(), file.end(), [](const char& c)
		{
			return c == '\\';
		}, '/');
#endif

		// In this case we circumvent the assertion that will stop execution if the file can't be found and
		// try to show as many scenery objects as possible
		if (appData->tryFindFile(file, &path))
		{
			runtime->addSceneElements(file);
		}
		else
		{
			SURGSIM_LOG_WARNING(Framework::Logger::getDefaultLogger())
					<< "Can't find " << file;
		}
	}

	runtime->execute();
	return 0;
}
