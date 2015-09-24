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

#include <memory>

#include <boost/program_options.hpp>
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Math/Math.h"

using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::Logger;
using SurgSim::Framework::Runtime;
using SurgSim::Graphics::OsgAxesRepresentation;
using SurgSim::Graphics::OsgSceneryRepresentation;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Math::Vector3d;

namespace po = boost::program_options;

namespace
{

std::shared_ptr<OsgViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	std::array<int, 2> position = {x, y};
	std::array<int, 2> dimensions = {width, height};
	viewElement->getView()->setPosition(position);
	viewElement->getView()->setDimensions(dimensions);

	return viewElement;
}
}

int main(int argc, char* argv[])
{

	// Parse commandline parameters
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

	std::shared_ptr<Runtime> runtime;

	if (variables.count("config-file") == 1)
	{
		runtime = std::make_shared<Runtime>(variables["config-file"].as<std::string>());
	}
	else
	{
		runtime = std::make_shared<Runtime>();
	}

	auto data = runtime->getApplicationData();

	runtime->addManager(std::make_shared<SurgSim::Graphics::OsgManager>());
	runtime->addManager(std::make_shared<SurgSim::Framework::BehaviorManager>());

	auto scene = runtime->getScene();

	auto viewElement = createView("View", 40, 40, 1024, 768);
	scene->addSceneElement(viewElement);
	viewElement->enableManipulator(true);
	viewElement->setManipulatorParameters(Vector3d(0, 0, 2), Vector3d(0, 0, 0));

	auto element = std::make_shared<BasicSceneElement>("Graphics");

	auto axes = std::make_shared<OsgAxesRepresentation>("axes");
	element->addComponent(axes);

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
			auto graphics = std::make_shared<OsgSceneryRepresentation>(file);
			graphics->loadModel(file);
			element->addComponent(graphics);
		}
		else
		{
			SURGSIM_LOG_WARNING(Logger::getDefaultLogger())
					<< "Can't find " << file;
		}
	}

	scene->addSceneElement(element);

	runtime->execute();

	return 0;
}
