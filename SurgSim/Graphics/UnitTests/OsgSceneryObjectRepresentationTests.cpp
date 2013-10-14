//// This file is a part of the OpenSurgSim project.
//// Copyright 2012-2013, SimQuest Solutions Inc.
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.
//
///// \file
///// Tests for the OsgSceneryRepresentation class.
//
//#include <SurgSim/Framework/ApplicationData.h>
//#include <SurgSim/Graphics/OsgSceneryRepresentation.h>
//
//#include <boost/filesystem.hpp>
//
//#include <gtest/gtest.h>
//
//
//namespace SurgSim
//{
//namespace Graphics
//{
//
//TEST(OsgObjectTests, InitTest)
//{
//	OsgSceneryRepresentation object("SceneryRepresentation");
//	EXPECT_EQ(nullptr, object.getOsgObject());
//}
//
//TEST(OsgObjectTests, LoadAndUnloadTest)
//{
//	ASSERT_TRUE(boost::filesystem::exists("Data"));
//
//	std::vector<std::string> paths;
//	paths.push_back("Data/OsgObjectTests");
//	SurgSim::Framework::ApplicationData data(paths);
//
//	std::string objectPath = data.findFile("table_extension.osgb");
//	ASSERT_NE("", objectPath) << "Could not find object file!";
//
//	std::shared_ptr<OsgSceneryRepresentation> osgObject = std::make_shared<OsgSceneryRepresentation>("Obj1");
//	std::shared_ptr<SceneryRepresentation> object = osgObject;
//	EXPECT_TRUE(object->loadObject(objectPath)) << "Failed to load object!";
//
//	objectPath.clear();
//	objectPath = data.findFile("table_extension.obj");
//	ASSERT_NE("", objectPath) << "Could not find object file!";
//
//	std::shared_ptr<OsgSceneryRepresentation> osgObject2 = std::make_shared<OsgSceneryRepresentation>("Obj2");
//	std::shared_ptr<SceneryRepresentation> object2 = osgObject2;
//	EXPECT_TRUE(object2->loadObject(objectPath)) << "Failed to load object!";
//}
//
//}  // namespace Graphics
//}  // namespace SurgSim