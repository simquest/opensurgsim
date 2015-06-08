// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

/// \file
/// Tests for the OsgProgram class.

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Graphics/OsgProgram.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <gtest/gtest.h>

#include <sstream>

namespace
{
/// Sample vertex shader code
const std::string vertexShader =
	"varying vec4 vertColor;\n"
	"void main(void)\n"
	"{\n"
	"	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
	"	vertColor.rgb = gl_Normal;\n"
	"	vertColor.a = 1.0;\n"
	"}\n";

/// Sample geometry shader code
const std::string geometryShader =
	"#version 150\n"
	"#extension GL_EXT_gpu_shader4 : enable\n"
	"#extension GL_EXT_geometry_shader4 : enable\n"
	"layout(triangles) in;\n"
	"layout(triangle_strip, max_vertices=3) out;\n"
	"in vec4 vertColor[3];\n"
	"out vec4 geomColor;\n"
	"void main()\n"
	"{\n"
	"	for (int i = 0; i < gl_VerticesIn; ++i)\n"
	"	{\n"
	"		gl_Position = gl_PositionIn[i];\n"
	"		geomColor = vertColor[i];\n"
	"		EmitVertex();\n"
	"	}\n"
	"	EndPrimitive();\n"
	"};";

/// Sample fragment shader code
const std::string fragmentShader =
	"varying vec4 geomColor;\n"
	"void main(void)\n"
	"{\n"
	"	gl_FragColor = geomColor;\n"
	"}";
}

namespace SurgSim
{
namespace Graphics
{

TEST(OsgProgramTests, InitTest)
{
	OsgProgram program;

	EXPECT_NE(nullptr, program.getOsgProgram());

	EXPECT_EQ(0u, program.getOsgProgram()->getNumShaders());
}

TEST(OsgProgramTests, StateSetTest)
{
	OsgProgram program;

	// Create an OSG state set
	osg::ref_ptr<osg::StateSet> stateSet = new osg::StateSet();

	const osg::StateSet::AttributeList& attributes = stateSet->getAttributeList();
	EXPECT_EQ(0u, attributes.size());

	/// Add the program to the state set
	program.addToStateSet(stateSet.get());

	EXPECT_EQ(1u, attributes.size()) << "State set has no attributes, one program should have been added!";
	EXPECT_EQ(program.getOsgProgram(),
			  attributes.at(osg::StateAttribute::TypeMemberPair(osg::StateAttribute::PROGRAM, 0)).first)
			<< "First attribute in state set should be the added program!";

	/// Remove the program from the state set
	program.removeFromStateSet(stateSet.get());

	EXPECT_EQ(0u, attributes.size())
			<< "State set should no longer have any attributes, the program should have been removed!";
}

TEST(OsgProgramTests, SetShaderSourceTest)
{
	std::shared_ptr<OsgProgram> osgProgram = std::make_shared<OsgProgram>();
	std::shared_ptr<Program> program = osgProgram;

	EXPECT_FALSE(program->hasVertexShader());
	EXPECT_FALSE(program->hasGeometryShader());
	EXPECT_FALSE(program->hasFragmentShader());

	{
		SCOPED_TRACE("Set vertex shader source");
		program->setVertexShaderSource(vertexShader);

		std::string resultVertexShader;
		EXPECT_TRUE(program->hasVertexShader());
		EXPECT_TRUE(program->getVertexShaderSource(&resultVertexShader));
		EXPECT_EQ(vertexShader, resultVertexShader);

		EXPECT_EQ(1u, osgProgram->getOsgProgram()->getNumShaders());
	}

	{
		SCOPED_TRACE("Set geometry shader source");
		program->setGeometryShaderSource(geometryShader);

		std::string resultGeometryShader;
		EXPECT_TRUE(program->hasGeometryShader());
		EXPECT_TRUE(program->getGeometryShaderSource(&resultGeometryShader));
		EXPECT_EQ(geometryShader, resultGeometryShader);

		EXPECT_EQ(2u, osgProgram->getOsgProgram()->getNumShaders());
	}

	{
		SCOPED_TRACE("Set fragment shader source");
		program->setFragmentShaderSource(fragmentShader);

		std::string resultFragmentShader;
		EXPECT_TRUE(program->hasFragmentShader());
		EXPECT_TRUE(program->getFragmentShaderSource(&resultFragmentShader));
		EXPECT_EQ(fragmentShader, resultFragmentShader);

		EXPECT_EQ(3u, osgProgram->getOsgProgram()->getNumShaders());
	}
}

void expectFileContents(const std::string& filePath, const std::string& contents)
{
	boost::filesystem::ifstream fileStream(filePath);
	std::stringstream resultStream(contents);

	ASSERT_FALSE(fileStream.bad());

	std::string fileLine;
	std::string resultLine;
	while (!fileStream.eof() && !resultStream.eof())
	{
		fileStream >> fileLine;
		resultStream >> resultLine;

		// Skip possible trailing newlines
		fileStream >> std::ws;
		resultStream >> std::ws;

		EXPECT_EQ(fileLine, resultLine);
	}
	EXPECT_TRUE(fileStream.eof());
	EXPECT_TRUE(resultStream.eof());
	fileStream.close();
}

TEST(OsgProgramTests, LoadShaderSourceTest)
{
	SurgSim::Framework::ApplicationData data("config.txt");

	std::string vertexShaderPath   = data.findFile("OsgProgramTests/shader.vert");
	std::string geometryShaderPath = data.findFile("OsgProgramTests/shader.geom");
	std::string fragmentShaderPath = data.findFile("OsgProgramTests/shader.frag");

	ASSERT_NE("", vertexShaderPath) << "Could not find vertex shader!";
	ASSERT_NE("", geometryShaderPath) << "Could not find geometry shader!";
	ASSERT_NE("", fragmentShaderPath) << "Could not find fragment shader!";

	std::shared_ptr<OsgProgram> osgProgram = std::make_shared<OsgProgram>();
	std::shared_ptr<Program> program = osgProgram;

	{
		SCOPED_TRACE("Load vertex shader source from file");
		EXPECT_TRUE(program->loadVertexShader(vertexShaderPath));
		std::string resultVertexShader;
		EXPECT_TRUE(program->getVertexShaderSource(&resultVertexShader));
		expectFileContents(vertexShaderPath, resultVertexShader);

		EXPECT_EQ(1u, osgProgram->getOsgProgram()->getNumShaders());
	}

	{
		SCOPED_TRACE("Load geometry shader source from file");
		EXPECT_TRUE(program->loadGeometryShader(geometryShaderPath));
		std::string resultGeometryShader;
		EXPECT_TRUE(program->getGeometryShaderSource(&resultGeometryShader));
		expectFileContents(geometryShaderPath, resultGeometryShader);

		EXPECT_EQ(2u, osgProgram->getOsgProgram()->getNumShaders());
	}

	{
		SCOPED_TRACE("Load fragment shader source from file");
		EXPECT_TRUE(program->loadFragmentShader(fragmentShaderPath));
		std::string resultFragmentShader;
		EXPECT_TRUE(program->getFragmentShaderSource(&resultFragmentShader));
		expectFileContents(fragmentShaderPath, resultFragmentShader);

		EXPECT_EQ(3u, osgProgram->getOsgProgram()->getNumShaders());
	}
}

TEST(OsgProgramTests, ClearShaderTest)
{
	std::shared_ptr<OsgProgram> osgProgram = std::make_shared<OsgProgram>();
	std::shared_ptr<Program> program = osgProgram;

	// Set vertex, geometry, and fragment shaders
	EXPECT_FALSE(program->hasVertexShader());
	program->setVertexShaderSource(vertexShader);
	EXPECT_TRUE(program->hasVertexShader());

	EXPECT_FALSE(program->hasGeometryShader());
	program->setGeometryShaderSource(geometryShader);
	EXPECT_TRUE(program->hasGeometryShader());

	EXPECT_FALSE(program->hasFragmentShader());
	program->setFragmentShaderSource(fragmentShader);
	EXPECT_TRUE(program->hasFragmentShader());

	EXPECT_EQ(3u, osgProgram->getOsgProgram()->getNumShaders());

	{
		SCOPED_TRACE("Clear vertex shader");
		program->clearVertexShader();
		EXPECT_FALSE(program->hasVertexShader());
		EXPECT_TRUE(program->hasGeometryShader());
		EXPECT_TRUE(program->hasFragmentShader());

		EXPECT_EQ(2u, osgProgram->getOsgProgram()->getNumShaders());
	}
	program->setVertexShaderSource(vertexShader);
	EXPECT_TRUE(program->hasVertexShader());

	{
		SCOPED_TRACE("Clear geometry shader");
		program->clearGeometryShader();
		EXPECT_TRUE(program->hasVertexShader());
		EXPECT_FALSE(program->hasGeometryShader());
		EXPECT_TRUE(program->hasFragmentShader());

		EXPECT_EQ(2u, osgProgram->getOsgProgram()->getNumShaders());
	}
	program->setGeometryShaderSource(geometryShader);
	EXPECT_TRUE(program->hasGeometryShader());

	{
		SCOPED_TRACE("Clear fragment shader");
		program->clearFragmentShader();
		EXPECT_TRUE(program->hasVertexShader());
		EXPECT_TRUE(program->hasGeometryShader());
		EXPECT_FALSE(program->hasFragmentShader());

		EXPECT_EQ(2u, osgProgram->getOsgProgram()->getNumShaders());
	}
	program->setFragmentShaderSource(fragmentShader);
	EXPECT_TRUE(program->hasFragmentShader());

	{
		SCOPED_TRACE("Clear the entire shader");
		program->clear();
		EXPECT_FALSE(program->hasVertexShader());
		EXPECT_FALSE(program->hasGeometryShader());
		EXPECT_FALSE(program->hasFragmentShader());

		EXPECT_EQ(0u, osgProgram->getOsgProgram()->getNumShaders());
	}
}

}  // namespace Graphics
}  // namespace SurgSim
