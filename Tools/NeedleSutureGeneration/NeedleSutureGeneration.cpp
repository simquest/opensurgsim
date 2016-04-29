// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#include <cmath>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/CatmullRom.h"
#include "SurgSim/Math/Vector.h"

/// Function to generate points on a circle with radius 'radius'.
/// \note The points always start from the origin and go on the circle to 'angle'.
/// \param subdivisions Number of interpolated points on the circle.
/// \param radius Radius of the circle on which points are generated.
/// \param angle How much (of the circle) should points occupy, in radian.
/// \return A list of points on the circle.
std::vector<SurgSim::Math::Vector3d> generateNeedle(size_t subdivisions, double radius, double angle)
{
	SURGSIM_ASSERT(0 < angle && angle < 2.0 * M_PI) << __FUNCTION__ << " 'angle' must be in the range (0, 2PI)";
	SURGSIM_ASSERT(2 <= subdivisions) << __FUNCTION__ << " 'subdivisions' must be at least 2";

	double increment = angle / static_cast<double>(subdivisions);
	double controlAngle = 0.0;

	std::vector<SurgSim::Math::Vector3d> vertices(subdivisions+1);
	for (size_t i = 0; i < subdivisions; ++i)
	{
		vertices[i] = radius * SurgSim::Math::Vector3d(std::cos(controlAngle) - 1.0, std::sin(controlAngle), 0.0);
		controlAngle += increment;
	}
	vertices[subdivisions] =
		radius * SurgSim::Math::Vector3d(std::cos(controlAngle) - 1.0, std::sin(controlAngle), 0.0);

	return vertices;
}

/// Function to generate a suture, starting from the origin and wraps around 'axis' in circles.
/// A circle (formed by the suture when wraps 'axis') is defined by four markers.
/// Each marker is rotated 45 degrees along 'axis' from previous one.
/// 'subdivisions' controls how many points to interpolate between each pair of markers.
/// In the returned list, the origin will always be the first point.
/// \param subdivisions Number of points between makers.
/// \param radius Radius of the circle the suture forms.
/// \param length Length of the suture.
/// \param circles Number of circles the suture forms wrapping around 'axis'.
/// \param axis The axis around which the suture wraps.
/// \return A list of points representing the suture.
std::vector<SurgSim::Math::Vector3d> generateSuture(size_t subdivisions, double radius, double length, size_t circles,
										const SurgSim::Math::Vector3d& axis = SurgSim::Math::Vector3d(1.0, 0.0, 0.0))
{
	// Increment on X-axis.
	double increment = length / circles / 4.0;

	SurgSim::Math::Vector3d direction = axis;
	direction.normalize();
	auto rotateXToAxis = SurgSim::Math::makeRotationQuaternion(
											 -std::acos(direction.dot(SurgSim::Math::Vector3d(1.0, 0.0, 0.0))),
													  direction.cross(SurgSim::Math::Vector3d(1.0, 0.0, 0.0)));

	std::vector<SurgSim::Math::Vector3d> controlPoints;
	controlPoints.push_back(SurgSim::Math::Vector3d(0.0, 0.0, 0.0) - increment * direction);
	controlPoints.push_back(SurgSim::Math::Vector3d(0.0, 0.0, 0.0));

	// Used to control the rotation between markers.
	SurgSim::Math::Quaterniond rotation =
								SurgSim::Math::makeRotationQuaternion(M_PI_2, SurgSim::Math::Vector3d(1.0, 0.0, 0.0));
	auto point = SurgSim::Math::Vector3d(increment, radius, 0.0);
	for (size_t i = 0; i < circles * 4; ++i)
	{
		controlPoints.push_back(rotateXToAxis * point);
		point = rotation * point;
		point.x() += increment;
	}

	std::vector<SurgSim::Math::Vector3d> result;
	SurgSim::Math::CatmullRom::interpolate(subdivisions, controlPoints, &result);

	return result;
}

/// Save a list of needle and a list of suture as one ply file.
/// Points in 'needle' will be output first, as the order they appear in 'needle' and
/// points in 'suture' will be output second, as the order they appear in 'suture'.
/// The caller of this function needs to make sure
/// the needle and the suture connects (smoothly) at needle[last] and suture[begin].
/// \param fileName Name of the ply file.
/// \param needle List of points representing the needle.
/// \param suture List of points representing the suture.
/// \param asPhysics Boolean to decide whether or not to save physics properties.
/// \param needleMassDensity Mass density of needle.
/// \param needlepoissonRatio Poisson ratio of needle.
/// \param needleYoungModulus Young modulus of needle.
/// \param sutureMassDensity Mass density of suture.
/// \param suturepoissonRatio Poisson ratio of suture.
/// \param sutureYoungModulus Young modulus of suture.
/// \param radius Radius used for fem1d beam.
void saveNeedleSuturePly(const std::string& fileName,
						 const std::vector<SurgSim::Math::Vector3d>& needle,
						 const std::vector<SurgSim::Math::Vector3d>& suture,
						 bool asPhysics,
						 double needleMassDensity, double needlePoissonRatio, double needleYoungModulus,
						 double sutureMassDensity, double suturePoissonRatio, double sutureYoungModulus,
						 double radius = 0.0001)
{
	std::ofstream out(fileName);

	size_t numOfPoints = needle.size() + suture.size();
	if (out.is_open())
	{
		out << "ply" << std::endl;
		out << "format ascii 1.0" << std::endl;
		out << "comment Created by OpenSurgSim, www.opensurgsim.org" << std::endl;
		out << "element vertex " << numOfPoints << std::endl;
		out << "property float x\nproperty float y\nproperty float z" << std::endl;
		if (asPhysics)
		{
			out << "element 1d_element " << numOfPoints - 1 << std::endl;
			out << "property list uint uint vertex_indices" << std::endl;
			out << "property double mass_density" << std::endl;
			out << "property double poisson_ratio" << std::endl;
			out << "property double young_modulus" << std::endl;
		}
		out << "element radius 1" << std::endl;
		out << "property double value" << std::endl;
		out << "element boundary_condition 0" << std::endl;
		out << "property uint vertex_index" << std::endl;
		out << "end_header" << std::endl;
		for (const auto& vertex : needle)
		{
			out << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
		}
		for (const auto& vertex : suture)
		{
			out << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
		}

		if (asPhysics)
		{
			int index = 0;
			for (const auto& vertex : needle)
			{
				out << "2 " << index << " " << index + 1 << " " <<
					needleMassDensity << " " << needlePoissonRatio << " " << needleYoungModulus << std::endl;
				++index;
			}
			for (const auto& vertex : suture)
			{
				out << "2 " << index << " " << index + 1 << " " <<
					sutureMassDensity << " " << suturePoissonRatio << " " << sutureYoungModulus << std::endl;
				++index;
			}
		}

		out << radius << std::endl;

		if (out.bad())
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__
				<< "There was a problem writing " << fileName;
		}

		out.close();
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__
			<< "Could not open " << fileName << " for writing.";
	}
};

// Utility to generate a ply file for needle suture.
// Command line input are supported.
int main(int argc, char* argv[])
{
	namespace po = boost::program_options;

	po::options_description commandLine("Allowed options");
	commandLine.add_options()("help", "produce help message")
	("filename", po::value<std::string>()->default_value("needlesuture.ply"),
									"File name to save the generated needle suture.")
	("needleSubdivisions", po::value<int>()->default_value(10),
									"Number of interpolated points for the needle (default 10)")
	("needleRadius", po::value<double>()->default_value(0.019), "Radius of the needle (in m) (default 0.019)")
	("needleAngle", po::value<double>()->default_value(135),
									"How big is the arc (common values: 90, 135, 180, 225) in degrees (default 135)")
	("sutureSubdivisions", po::value<int>()->default_value(20),
									"Number of interpolated points for each pair of markers on the suture (default 20)")
	("sutureRadius", po::value<double>()->default_value(0.01), "Radius of the suture (default 0.01")
	("sutureLength", po::value<double>()->default_value(0.45), "Length of th suture (in m) (default 0.45)")
	("sutureCircles", po::value<int>()->default_value(2), "Number of circles formed by suture going around the axis")
	("savePhysicsProperty", po::value<bool>()->default_value(true),
									"Output physics properties for the needle and suture in ply file")
	("physicsProperty", po::value<std::string>(),
									"File name for physics properties. Note that command line values have priority: "
									"program will use value(s) from command line if specified.")
	("needleMassDensity", po::value<double>(), "Mass density for the needle (default 7500 Kg.m-3)")
	("needlePoissonRatio", po::value<double>(), "Poisson ratio for the needle (default 0.305)")
	("needleYoungModulus", po::value<double>(), "Young Modulus for the needle (default (1.8e11 Pa)")
	("sutureMassDensity", po::value<double>(), "Mass density for the suture (default 900 Kg.m-3)")
	("suturePoissonRatio", po::value<double>(), "Poisson ratio for the suture (default 0.45)")
	("sutureYoungModulus", po::value<double>(), "Young Modulus for the suture (default 1.75e9 Pa)");

	po::variables_map variables;
	try
	{
		po::store(po::parse_command_line(argc, argv, commandLine), variables);
	}
	catch (po::error& e)
	{
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << commandLine << std::endl;
		return 1;
	}

	if (variables.count("help"))
	{
		std::cout << commandLine << "\n";
		return 1;
	}

	double needleMassDensity = 7500, needlePoissonRatio = 0.305, needleYoungModulus = 180 * 1e9;
	double sutureMassDensity = 900,  suturePoissonRatio = 0.45,  sutureYoungModulus = 1.75 * 1e9;
	if (variables.count("physicsProperty"))
	{
		boost::filesystem::path fileName("Data/" + variables["physicsProperty"].as<std::string>());
		boost::filesystem::path filePath = boost::filesystem::complete(fileName);
		std::cout << "File path: " << filePath << std::endl;
		if (!boost::filesystem::exists(filePath))
		{
			std::cout << "Can't find my file!" << std::endl;
			return -1;
		}

		boost::property_tree::ptree parameters;
		boost::property_tree::ini_parser::read_ini(filePath.string(), parameters);
		needleMassDensity  = parameters.get<double>("PhysicsProperty.needleMassDensity");
		needlePoissonRatio = parameters.get<double>("PhysicsProperty.needlePoissonRatio");
		needleYoungModulus = parameters.get<double>("PhysicsProperty.needleYoungModulus");
		sutureMassDensity  = parameters.get<double>("PhysicsProperty.sutureMassDensity");
		suturePoissonRatio = parameters.get<double>("PhysicsProperty.suturePoissonRatio");
		sutureYoungModulus = parameters.get<double>("PhysicsProperty.sutureYoungModulus");
	}

	// Command line values for physics properties will take precedence if specified.
	if (variables.count("needleMassDensity"))  needleMassDensity  = variables["needleMassDensity"].as<double>();
	if (variables.count("needlePoissonRatio")) needlePoissonRatio = variables["needlePoissonRatio"].as<double>();
	if (variables.count("needleYoungModulus")) needleYoungModulus = variables["needleYoungModulus"].as<double>();
	if (variables.count("sutureMassDensity"))  sutureMassDensity  = variables["sutureMassDensity"].as<double>();
	if (variables.count("suturePoissonRatio")) suturePoissonRatio = variables["suturePoissonRatio"].as<double>();
	if (variables.count("sutureYoungModulus")) sutureYoungModulus = variables["sutureYoungModulus"].as<double>();

	auto needlePoints = generateNeedle(variables["needleSubdivisions"].as<int>(),
									   variables["needleRadius"].as<double>(),
									   variables["needleAngle"].as<double>() / 180 * M_PI);

	auto suturePoints = generateSuture(variables["sutureSubdivisions"].as<int>(),
									   variables["sutureRadius"].as<double>(),
									   variables["sutureLength"].as<double>(),
									   variables["sutureCircles"].as<int>(),
									   needlePoints[0] - needlePoints[1]);

	// Points generated are starting from the origin, when constructing the needle suture,
	// the list of needle points need to be reversed and the origin is removed to avoid duplication.
	std::vector<SurgSim::Math::Vector3d> needlePointsReversed(needlePoints.rbegin(), needlePoints.rend()-1);

	saveNeedleSuturePly(variables["filename"].as<std::string>(), needlePointsReversed, suturePoints,
						variables["savePhysicsProperty"].as<bool>(),
						needleMassDensity, needlePoissonRatio, needleYoungModulus,
						sutureMassDensity, suturePoissonRatio, sutureYoungModulus);
	return 0;
}
