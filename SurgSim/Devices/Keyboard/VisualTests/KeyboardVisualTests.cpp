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

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <osg/Camera>
#include <osg/Geode>
#include <osgText/Text>
#include <osgViewer/Viewer>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Keyboard/KeyboardDevice.h"
#include "SurgSim/Devices/Keyboard/OsgKeyboardHandler.h"
#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Input/InputConsumerInterface.h"



using SurgSim::DataStructures::DataGroup;

struct TestListener : public SurgSim::Input::InputConsumerInterface
{
	TestListener()
	{
		creatKeyMap();
		createModifierMap();
	}
	void initializeInput(const std::string& device, const DataGroup& inputData) override {}
	void handleInput(const std::string& device, const DataGroup& inputData) override
	{
		int key, modifierMask;
		inputData.integers().get("key", &key);
		inputData.integers().get("modifierMask", &modifierMask);

		if (key != SurgSim::Device::KeyCode::NONE)
		{
			std::cerr << "Key pressed :" << keyMap[key] << std::endl;
			if (modifierMask != SurgSim::Device::ModKeyMask::MODKEY_NONE)
			{
				if (modifierMap[modifierMask] != "")
				{
					std::cerr << "Modifier:" << modifierMap[modifierMask] << std::endl;
				}
				else
				{
					std::cerr << "Modifier: UNDEFINED" << std::endl;
				}
			}
		}
	}

	// keyMap is used to output the corresponding key name for a given key code.
	std::map<int, std::string> keyMap;

	// modifierMap is used to output the corresponding modifier name for a given modifier mask.
	std::map<int, std::string> modifierMap;

	void creatKeyMap()
	{
		keyMap[SurgSim::Device::KeyCode::KEY_SPACE] = "KEY_SPACE";
		keyMap[SurgSim::Device::KeyCode::KEY_0] = "0";
		keyMap[SurgSim::Device::KeyCode::KEY_1] = "1";
		keyMap[SurgSim::Device::KeyCode::KEY_2] = "2";
		keyMap[SurgSim::Device::KeyCode::KEY_3] = "3";
		keyMap[SurgSim::Device::KeyCode::KEY_4] = "4";
		keyMap[SurgSim::Device::KeyCode::KEY_5] = "5";
		keyMap[SurgSim::Device::KeyCode::KEY_6] = "6";
		keyMap[SurgSim::Device::KeyCode::KEY_7] = "7";
		keyMap[SurgSim::Device::KeyCode::KEY_8] = "8";
		keyMap[SurgSim::Device::KeyCode::KEY_9] = "9";
		keyMap[SurgSim::Device::KeyCode::KEY_A] = "a";
		keyMap[SurgSim::Device::KeyCode::KEY_B] = "b";
		keyMap[SurgSim::Device::KeyCode::KEY_C] = "c";
		keyMap[SurgSim::Device::KeyCode::KEY_D] = "d";
		keyMap[SurgSim::Device::KeyCode::KEY_E] = "e";
		keyMap[SurgSim::Device::KeyCode::KEY_F] = "f";
		keyMap[SurgSim::Device::KeyCode::KEY_G] = "g";
		keyMap[SurgSim::Device::KeyCode::KEY_H] = "h";
		keyMap[SurgSim::Device::KeyCode::KEY_I] = "i";
		keyMap[SurgSim::Device::KeyCode::KEY_J] = "j";
		keyMap[SurgSim::Device::KeyCode::KEY_K] = "k";
		keyMap[SurgSim::Device::KeyCode::KEY_L] = "l";
		keyMap[SurgSim::Device::KeyCode::KEY_M] = "m";
		keyMap[SurgSim::Device::KeyCode::KEY_N] = "n";
		keyMap[SurgSim::Device::KeyCode::KEY_O] = "o";
		keyMap[SurgSim::Device::KeyCode::KEY_P] = "p";
		keyMap[SurgSim::Device::KeyCode::KEY_Q] = "q";
		keyMap[SurgSim::Device::KeyCode::KEY_R] = "r";
		keyMap[SurgSim::Device::KeyCode::KEY_S] = "s";
		keyMap[SurgSim::Device::KeyCode::KEY_T] = "t";
		keyMap[SurgSim::Device::KeyCode::KEY_U] = "u";
		keyMap[SurgSim::Device::KeyCode::KEY_V] = "v";
		keyMap[SurgSim::Device::KeyCode::KEY_W] = "w";
		keyMap[SurgSim::Device::KeyCode::KEY_X] = "x";
		keyMap[SurgSim::Device::KeyCode::KEY_Y] = "y";
		keyMap[SurgSim::Device::KeyCode::KEY_Z] = "z";
		keyMap[SurgSim::Device::KeyCode::KEY_EXCLAIM] = "KEY_EXCLAIM";
		keyMap[SurgSim::Device::KeyCode::KEY_QUOTEDBL] = "KEY_QUOTEDBL";
		keyMap[SurgSim::Device::KeyCode::KEY_HASH] = "KEY_HASH";
		keyMap[SurgSim::Device::KeyCode::KEY_DOLLAR] = "KEY_DOLLAR";
		keyMap[SurgSim::Device::KeyCode::KEY_AMPERSAND] = "KEY_AMPERSAND";
		keyMap[SurgSim::Device::KeyCode::KEY_QUOTE] = "KEY_QUOTE";
		keyMap[SurgSim::Device::KeyCode::KEY_LEFTPAREN] = "KEY_LEFTPAREN";
		keyMap[SurgSim::Device::KeyCode::KEY_RIGHTPAREN]= "KEY_RIGHTPAREN";
		keyMap[SurgSim::Device::KeyCode::KEY_ASTERISK] = "KEY_ASTERISK";
		keyMap[SurgSim::Device::KeyCode::KEY_PLUS] = "KEY_PLUS";
		keyMap[SurgSim::Device::KeyCode::KEY_COMMA] = "KEY_COMMA";
		keyMap[SurgSim::Device::KeyCode::KEY_MINUS] = "KEY_MINUS";
		keyMap[SurgSim::Device::KeyCode::KEY_PERIOD] = "KEY_PERIOD";
		keyMap[SurgSim::Device::KeyCode::KEY_SLASH] = "KEY_SLASH";
		keyMap[SurgSim::Device::KeyCode::KEY_COLON] = "KEY_COLON";
		keyMap[SurgSim::Device::KeyCode::KEY_SEMICOLON] = "KEY_SEMICOLON";
		keyMap[SurgSim::Device::KeyCode::KEY_LESS] = "KEY_LESS";
		keyMap[SurgSim::Device::KeyCode::KEY_EQUALS] = "KEY_EQUALS";
		keyMap[SurgSim::Device::KeyCode::KEY_GREATER] = "KEY_GREATER";
		keyMap[SurgSim::Device::KeyCode::KEY_QUESTION] = "KEY_QUESTION";
		keyMap[SurgSim::Device::KeyCode::KEY_AT] = "KEY_AT";
		keyMap[SurgSim::Device::KeyCode::KEY_LEFTBRACKET] = "KEY_LEFTBRACKET";
		keyMap[SurgSim::Device::KeyCode::KEY_BACKSLASH] = "KEY_BACKSLASH";
		keyMap[SurgSim::Device::KeyCode::KEY_RIGHTBRACKET]= "KEY_RIGHTBRACKET";
		keyMap[SurgSim::Device::KeyCode::KEY_CARET] = "KEY_CARET";
		keyMap[SurgSim::Device::KeyCode::KEY_UNDERSCORE] = "KEY_UNDERSCORE";
		keyMap[SurgSim::Device::KeyCode::KEY_BACKQUOTE] = "KEY_BACKQUOTE";
		keyMap[SurgSim::Device::KeyCode::KEY_BACKSPACE] = "KEY_BACKSPACE";
		keyMap[SurgSim::Device::KeyCode::KEY_TAB] = "KEY_TAB";
		keyMap[SurgSim::Device::KeyCode::KEY_LINEFEED] = "KEY_LINEFEED";
		keyMap[SurgSim::Device::KeyCode::KEY_CLEAR] = "KEY_CLEAR";
		keyMap[SurgSim::Device::KeyCode::KEY_RETURN] = "KEY_RETURN";
		keyMap[SurgSim::Device::KeyCode::KEY_PAUSE] = "KEY_PAUSE";
		keyMap[SurgSim::Device::KeyCode::KEY_SCROLL_LOCK] = "KEY_SCROLL_LOCK";
		keyMap[SurgSim::Device::KeyCode::KEY_SYS_REQ] = "KEY_SYS_REQ";
		keyMap[SurgSim::Device::KeyCode::KEY_ESCAPE] = "KEY_ESCAPE";
		keyMap[SurgSim::Device::KeyCode::KEY_DELETE] = "KEY_DELETE";
		keyMap[SurgSim::Device::KeyCode::KEY_HOME] = "KEY_HOME";
		keyMap[SurgSim::Device::KeyCode::KEY_LEFT] = "KEY_LEFT";
		keyMap[SurgSim::Device::KeyCode::KEY_UP] = "KEY_UP";
		keyMap[SurgSim::Device::KeyCode::KEY_RIGHT] = "KEY_RIGHT";
		keyMap[SurgSim::Device::KeyCode::KEY_DOWN] = "KEY_DOWN";
		keyMap[SurgSim::Device::KeyCode::KEY_PRIOR] = "KEY_PRIOR";
		keyMap[SurgSim::Device::KeyCode::KEY_PAGE_UP] = "KEY_PAGE_UP";
		keyMap[SurgSim::Device::KeyCode::KEY_NEXT] = "KEY_NEXT";
		keyMap[SurgSim::Device::KeyCode::KEY_PAGE_DOWN] = "KEY_PAGE_DOWN";
		keyMap[SurgSim::Device::KeyCode::KEY_END] = "KEY_END";
		keyMap[SurgSim::Device::KeyCode::KEY_BEGIN] = "KEY_BEGIN";
		keyMap[SurgSim::Device::KeyCode::KEY_SELECT] = "KEY_SELECT";
		keyMap[SurgSim::Device::KeyCode::KEY_PRINT] = "KEY_PRINT";
		keyMap[SurgSim::Device::KeyCode::KEY_EXECUTE] = "KEY_EXECUTE";
		keyMap[SurgSim::Device::KeyCode::KEY_INSERT] = "KEY_INSERT";
		keyMap[SurgSim::Device::KeyCode::KEY_UNDO] = "KEY_UNDO";
		keyMap[SurgSim::Device::KeyCode::KEY_REDO] = "KEY_REDO";
		keyMap[SurgSim::Device::KeyCode::KEY_MENU] = "KEY_MENU";
		keyMap[SurgSim::Device::KeyCode::KEY_FIND] = "KEY_FIND";
		keyMap[SurgSim::Device::KeyCode::KEY_CANCEL] = "KEY_CANCEL";
		keyMap[SurgSim::Device::KeyCode::KEY_HELP] = "KEY_HELP";
		keyMap[SurgSim::Device::KeyCode::KEY_BREAK] = "KEY_BREAK";
		keyMap[SurgSim::Device::KeyCode::KEY_MODE_SWITCH] = "KEY_MODE_SWITCH";
		keyMap[SurgSim::Device::KeyCode::KEY_SCRIPT_SWITCH] = "KEY_SCRIPT_SWITCH";
		keyMap[SurgSim::Device::KeyCode::KEY_NUM_LOCK] = "KEY_NUM_LOCK";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_SPACE] = "KEY_KP_SPACE";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_TAB] = "KEY_KP_TAB";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_ENTER] = "KEY_KP_ENTER";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_F1] = "KEY_KP_F1";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_F2] = "KEY_KP_F2";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_F3] = "KEY_KP_F3";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_F4] = "KEY_KP_F4";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_HOME] = "KEY_KP_HOME";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_LEFT] = "KEY_KP_LEFT";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_UP] = "KEY_KP_UP";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_RIGHT] = "KEY_KP_RIGHT";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_DOWN] = "KEY_KP_DOWN";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_PRIOR] = "KEY_KP_PRIOR";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_PAGE_UP] = "KEY_KP_PAGE_UP";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_NEXT] = "KEY_KP_NEXT";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_PAGE_DOWN] = "KEY_KP_PAGE_DOWN";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_END] = "KEY_KP_END";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_BEGIN] = "KEY_KP_BEGIN";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_INSERT] = "KEY_KP_INSERT";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_DELETE] = "KEY_KP_DELETE";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_EQUAL] = "KEY_KP_EQUAL";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_MULTIPLY] = "KEY_KP_MULTIPLY";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_ADD] = "KEY_KP_ADD";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_SEPARATOR]= "KEY_KP_SEPARATOR";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_SUBTRACT] = "KEY_KP_SUBTRACT";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_DECIMAL] = "KEY_KP_DECIMAL";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_DIVIDE] = "KEY_KP_DIVIDE";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_0] = "KEY_KP_0";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_1] = "KEY_KP_1";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_2] = "KEY_KP_2";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_3] = "KEY_KP_3";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_4] = "KEY_KP_4";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_5] = "KEY_KP_5";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_6] = "KEY_KP_6";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_7] = "KEY_KP_7";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_8] = "KEY_KP_8";
		keyMap[SurgSim::Device::KeyCode::KEY_KP_9] = "KEY_KP_9";
		keyMap[SurgSim::Device::KeyCode::KEY_F1] = "KEY_F1";
		keyMap[SurgSim::Device::KeyCode::KEY_F2] = "KEY_F2";
		keyMap[SurgSim::Device::KeyCode::KEY_F3] = "KEY_F3";
		keyMap[SurgSim::Device::KeyCode::KEY_F4] = "KEY_F4";
		keyMap[SurgSim::Device::KeyCode::KEY_F5] = "KEY_F5";
		keyMap[SurgSim::Device::KeyCode::KEY_F6] = "KEY_F6";
		keyMap[SurgSim::Device::KeyCode::KEY_F7] = "KEY_F7";
		keyMap[SurgSim::Device::KeyCode::KEY_F8] = "KEY_F8";
		keyMap[SurgSim::Device::KeyCode::KEY_F9] = "KEY_F9";
		keyMap[SurgSim::Device::KeyCode::KEY_F10] = "KEY_F10";
		keyMap[SurgSim::Device::KeyCode::KEY_F11] = "KEY_F11";
		keyMap[SurgSim::Device::KeyCode::KEY_F12] = "KEY_F12";
		keyMap[SurgSim::Device::KeyCode::KEY_F13] = "KEY_F13";
		keyMap[SurgSim::Device::KeyCode::KEY_F14] = "KEY_F14";
		keyMap[SurgSim::Device::KeyCode::KEY_F15] = "KEY_F15";
		keyMap[SurgSim::Device::KeyCode::KEY_F16] = "KEY_F16";
		keyMap[SurgSim::Device::KeyCode::KEY_F17] = "KEY_F17";
		keyMap[SurgSim::Device::KeyCode::KEY_F18] = "KEY_F18";
		keyMap[SurgSim::Device::KeyCode::KEY_F19] = "KEY_F19";
		keyMap[SurgSim::Device::KeyCode::KEY_F20] = "KEY_F20";
		keyMap[SurgSim::Device::KeyCode::KEY_F21] = "KEY_F21";
		keyMap[SurgSim::Device::KeyCode::KEY_F22] = "KEY_F22";
		keyMap[SurgSim::Device::KeyCode::KEY_F23] = "KEY_F23";
		keyMap[SurgSim::Device::KeyCode::KEY_F24] = "KEY_F24";
		keyMap[SurgSim::Device::KeyCode::KEY_F25] = "KEY_F25";
		keyMap[SurgSim::Device::KeyCode::KEY_F26] = "KEY_F26";
		keyMap[SurgSim::Device::KeyCode::KEY_F27] = "KEY_F27";
		keyMap[SurgSim::Device::KeyCode::KEY_F28] = "KEY_F28";
		keyMap[SurgSim::Device::KeyCode::KEY_F29] = "KEY_F29";
		keyMap[SurgSim::Device::KeyCode::KEY_F30] = "KEY_F30";
		keyMap[SurgSim::Device::KeyCode::KEY_F31] = "KEY_F31";
		keyMap[SurgSim::Device::KeyCode::KEY_F32] = "KEY_F32";
		keyMap[SurgSim::Device::KeyCode::KEY_F33] = "KEY_F33";
		keyMap[SurgSim::Device::KeyCode::KEY_F34] = "KEY_F34";
		keyMap[SurgSim::Device::KeyCode::KEY_F35] = "KEY_F35";
		keyMap[SurgSim::Device::KeyCode::KEY_SHIFT_L] = "KEY_SHIFT_L";
		keyMap[SurgSim::Device::KeyCode::KEY_SHIFT_R] = "KEY_SHIFT_R";
		keyMap[SurgSim::Device::KeyCode::KEY_CONTROL_L] = "KEY_CONTROL_L";
		keyMap[SurgSim::Device::KeyCode::KEY_CONTROL_R] = "KEY_CONTROL_R";
		keyMap[SurgSim::Device::KeyCode::KEY_CAPS_LOCK] = "KEY_CAPS_LOCK";
		keyMap[SurgSim::Device::KeyCode::KEY_SHIFT_LOCK] = "KEY_SHIFT_LOCK";
		keyMap[SurgSim::Device::KeyCode::KEY_META_L] = "KEY_META_L";
		keyMap[SurgSim::Device::KeyCode::KEY_META_R] = "KEY_META_R";
		keyMap[SurgSim::Device::KeyCode::KEY_ALT_L] = "KEY_ALT_L";
		keyMap[SurgSim::Device::KeyCode::KEY_ALT_R] = "KEY_ALT_R";
		keyMap[SurgSim::Device::KeyCode::KEY_SUPER_L] = "KEY_SUPER_L";
		keyMap[SurgSim::Device::KeyCode::KEY_SUPER_R] = "KEY_SUPER_R";
		keyMap[SurgSim::Device::KeyCode::KEY_HYPER_L] = "KEY_HYPER_L";
		keyMap[SurgSim::Device::KeyCode::KEY_HYPER_R] = "KEY_HYPER_R";
	};

	void createModifierMap()
	{
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_LEFT_SHIFT] = "KEY_SHIFT_L";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_RIGHT_SHIFT] = "KEY_SHIFT_R";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_LEFT_CTRL] = "KEY_CONTROL_L";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_RIGHT_CTRL] = "KEY_CONTROL_R";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_LEFT_ALT] = "KEY_ALT_L";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_RIGHT_ALT] = "KEY_ALT_R";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_LEFT_META] = "KEY_META_L";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_RIGHT_META] = "KEY_META_R";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_LEFT_SUPER] = "KEY_SUPER_L";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_RIGHT_SUPER] = "KEY_SUPER_R";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_LEFT_HYPER] = "KEY_HYPER_L";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_RIGHT_HYPER] = "KEY_HYPER_R";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_NUM_LOCK] = "KEY_NUM_LOCK";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_CAPS_LOCK] = "KEY_CAPS_LOCK";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_CTRL] = "KEY_CTRL";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_SHIFT] = "KEY_SHIFT";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_ALT] = "KEY_ALT";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_META] = "KEY_META";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_SUPER] = "KEY_SUPER";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_HYPER] = "KEY_HYPER";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_CAPS_SHIFT_L] = "KEY_CAPS_SHIFT_L";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_CAPS_SHIFT_R] = "KEY_CAPS_SHIFT_R";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_CAPS_CONTROL_L] = "KEY_CAPS_CONTROL_L";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_CAPS_CONTROL_R] = "KEY_CAPS_CONTROL_R";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_CAPS_ALT_L] = "KEY_CAPS_ALT_L";
		modifierMap[SurgSim::Device::ModKeyMask::MODKEY_CAPS_ALT_R] = "KEY_CAPS_ALT_R";
	}
};

int main(int argc, char* argv[])
{
	auto toolDevice	 = std::make_shared<SurgSim::Device::KeyboardDevice>("Keyboard");
	toolDevice->initialize();

	osg::ref_ptr<osgGA::GUIEventHandler> keyboardHandler = toolDevice->getKeyboardHandler();
	auto consumer = std::make_shared<TestListener>();
	toolDevice->addInputConsumer(consumer);

	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setText("Press any key in this window \n\nto verify keyboard driver \n\nworks correctly.");
	text->setPosition(osg::Vec3(0.0f, 300.0f, 0.0f));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(text);

	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setProjectionMatrixAsOrtho2D(0, 600 ,0, 400);
	camera->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	camera->addChild(geode);

	osg::ref_ptr<osg::Group> group = new osg::Group;
	group->addChild(camera);

	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
	viewer->setUpViewInWindow(400, 400, 640, 480);
	viewer->addEventHandler(keyboardHandler);
	viewer->setSceneData(group);

	viewer->run();
	return 0;
}
