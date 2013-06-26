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
/// Tests for the OsgShader class.

#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Graphics/OsgShader.h>

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

TEST(OsgShaderTests, InitTest)
{
	OsgShader shader;

	EXPECT_NE(nullptr, shader.getOsgProgram());

	EXPECT_EQ(0u, shader.getOsgProgram()->getNumShaders());
}

TEST(OsgShaderTests, StateSetTest)
{
	OsgShader shader;

	// Create an OSG state set
	osg::ref_ptr<osg::StateSet> stateSet = new osg::StateSet();

	const osg::StateSet::AttributeList& attributes = stateSet->getAttributeList();
	EXPECT_EQ(0u, attributes.size());

	/// Add the shader to the state set
	shader.addToStateSet(stateSet.get());

	EXPECT_EQ(1u, attributes.size()) << "State set has no attributes, one shader program should have been added!";
	EXPECT_EQ(shader.getOsgProgram(), attributes.at(osg::StateAttribute::TypeMemberPair(
		osg::StateAttribute::PROGRAM, 0)).first) << "First attribute in state set should be the added shader program!";

	/// Remove the shader from the state set
	shader.removeFromStateSet(stateSet.get());

	EXPECT_EQ(0u, attributes.size()) <<
		"State set should no longer have any attributes, the shader program should have been removed!";
}

TEST(OsgShaderTests, SetShaderSourceTest)
{
	std::shared_ptr<OsgShader> osgShader = std::make_shared<OsgShader>();
	std::shared_ptr<Shader> shader = osgShader;

	EXPECT_FALSE(shader->hasVertexShader());
	EXPECT_FALSE(shader->hasGeometryShader());
	EXPECT_FALSE(shader->hasFragmentShader());

	{
		SCOPED_TRACE("Set vertex shader source");
		shader->setVertexShaderSource(vertexShader);

		std::string resultVertexShader;
		EXPECT_TRUE(shader->hasVertexShader());
		EXPECT_TRUE(shader->getVertexShaderSource(&resultVertexShader));
		EXPECT_EQ(vertexShader, resultVertexShader);

		EXPECT_EQ(1u, osgShader->getOsgProgram()->getNumShaders());
	}

	{
		SCOPED_TRACE("Set geometry shader source");
		shader->setGeometryShaderSource(geometryShader);

		std::string resultGeometryShader;
		EXPECT_TRUE(shader->hasGeometryShader());
		EXPECT_TRUE(shader->getGeometryShaderSource(&resultGeometryShader));
		EXPECT_EQ(geometryShader, resultGeometryShader);

		EXPECT_EQ(2u, osgShader->getOsgProgram()->getNumShaders());
	}

	{
		SCOPED_TRACE("Set fragment shader source");
		shader->setFragmentShaderSource(fragmentShader);

		std::string resultFragmentShader;
		EXPECT_TRUE(shader->hasFragmentShader());
		EXPECT_TRUE(shader->getFragmentShaderSource(&resultFragmentShader));
		EXPECT_EQ(fragmentShader, resultFragmentShader);

		EXPECT_EQ(3u, osgShader->getOsgProgram()->getNumShaders());
	}
}

void expectFileContents(const std::string& filePath, const std::string& contents)
{
	boost::filesystem::ifstream fileStream(filePath);
	std::stringstream resultStream(contents);

	ASSERT_FALSE(fileStream.bad());

	std::string fileLine;
	std::string resultLine;
	while (! fileStream.eof() && ! resultStream.eof())
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

TEST(OsgShaderTests, LoadShaderSourceTest)
{
	ASSERT_TRUE(boost::filesystem::exists("Data"));

	std::vector<std::string> paths;
	paths.push_back("Data/OsgShaderTests");
	SurgSim::Framework::ApplicationData data(paths);

	std::string vertexShaderPath = data.findFile("shader.vert");
	std::string geometryShaderPath = data.findFile("shader.geom");
	std::string fragmentShaderPath = data.findFile("shader.frag");

	ASSERT_NE("", vertexShaderPath) << "Could not find vertex shader!";
	ASSERT_NE("", geometryShaderPath) << "Could not find geometry shader!";
	ASSERT_NE("", fragmentShaderPath) << "Could not find fragment shader!";

	std::shared_ptr<OsgShader> osgShader = std::make_shared<OsgShader>();
	std::shared_ptr<Shader> shader = osgShader;

	{
		SCOPED_TRACE("Load vertex shader source from file");
		EXPECT_TRUE(shader->loadVertexShaderSource(vertexShaderPath));
		std::string resultVertexShader;
		EXPECT_TRUE(shader->getVertexShaderSource(&resultVertexShader));
		expectFileContents(vertexShaderPath, resultVertexShader);

		EXPECT_EQ(1u, osgShader->getOsgProgram()->getNumShaders());
	}

	{
		SCOPED_TRACE("Load geometry shader source from file");
		EXPECT_TRUE(shader->loadGeometryShaderSource(geometryShaderPath));
		std::string resultGeometryShader;
		EXPECT_TRUE(shader->getGeometryShaderSource(&resultGeometryShader));
		expectFileContents(geometryShaderPath, resultGeometryShader);

		EXPECT_EQ(2u, osgShader->getOsgProgram()->getNumShaders());
	}

	{
		SCOPED_TRACE("Load fragment shader source from file");
		EXPECT_TRUE(shader->loadFragmentShaderSource(fragmentShaderPath));
		std::string resultFragmentShader;
		EXPECT_TRUE(shader->getFragmentShaderSource(&resultFragmentShader));
		expectFileContents(fragmentShaderPath, resultFragmentShader);

		EXPECT_EQ(3u, osgShader->getOsgProgram()->getNumShaders());
	}
}

TEST(OsgShaderTests, ClearShaderTest)
{
	std::shared_ptr<OsgShader> osgShader = std::make_shared<OsgShader>();
	std::shared_ptr<Shader> shader = osgShader;

	// Set vertex, geometry, and fragment shaders
	EXPECT_FALSE(shader->hasVertexShader());
	shader->setVertexShaderSource(vertexShader);
	EXPECT_TRUE(shader->hasVertexShader());

	EXPECT_FALSE(shader->hasGeometryShader());
	shader->setGeometryShaderSource(geometryShader);
	EXPECT_TRUE(shader->hasGeometryShader());

	EXPECT_FALSE(shader->hasFragmentShader());
	shader->setFragmentShaderSource(fragmentShader);
	EXPECT_TRUE(shader->hasFragmentShader());

	EXPECT_EQ(3u, osgShader->getOsgProgram()->getNumShaders());

	{
		SCOPED_TRACE("Clear vertex shader");
		shader->clearVertexShader();
		EXPECT_FALSE(shader->hasVertexShader());
		EXPECT_TRUE(shader->hasGeometryShader());
		EXPECT_TRUE(shader->hasFragmentShader());

		EXPECT_EQ(2u, osgShader->getOsgProgram()->getNumShaders());
	}
	shader->setVertexShaderSource(vertexShader);
	EXPECT_TRUE(shader->hasVertexShader());

	{
		SCOPED_TRACE("Clear geometry shader");
		shader->clearGeometryShader();
		EXPECT_TRUE(shader->hasVertexShader());
		EXPECT_FALSE(shader->hasGeometryShader());
		EXPECT_TRUE(shader->hasFragmentShader());

		EXPECT_EQ(2u, osgShader->getOsgProgram()->getNumShaders());
	}
	shader->setGeometryShaderSource(geometryShader);
	EXPECT_TRUE(shader->hasGeometryShader());

	{
		SCOPED_TRACE("Clear fragment shader");
		shader->clearFragmentShader();
		EXPECT_TRUE(shader->hasVertexShader());
		EXPECT_TRUE(shader->hasGeometryShader());
		EXPECT_FALSE(shader->hasFragmentShader());

		EXPECT_EQ(2u, osgShader->getOsgProgram()->getNumShaders());
	}
	shader->setFragmentShaderSource(fragmentShader);
	EXPECT_TRUE(shader->hasFragmentShader());

	{
		SCOPED_TRACE("Clear the entire shader");
		shader->clear();
		EXPECT_FALSE(shader->hasVertexShader());
		EXPECT_FALSE(shader->hasGeometryShader());
		EXPECT_FALSE(shader->hasFragmentShader());

		EXPECT_EQ(0u, osgShader->getOsgProgram()->getNumShaders());
	}
}

}  // namespace Graphics
}  // namespace SurgSim