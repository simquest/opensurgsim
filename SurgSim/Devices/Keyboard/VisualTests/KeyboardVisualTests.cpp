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

#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/Devices/Keyboard/KeyboardDevice.h>
#include <SurgSim/Devices/Keyboard/KeyboardHandler.h>
#include <SurgSim/Devices/Keyboard/KeyCode.h>
#include <SurgSim/Input/InputConsumerInterface.h>

#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/Projection>
#include <osgGA/GUIEventHandler>
#include <osgText/Text>
#include <osgViewer/config/SingleWindow>
#include <osgViewer/Viewer>

using SurgSim::DataStructures::DataGroup;

struct TestListener : public SurgSim::Input::InputConsumerInterface
{
	TestListener()
	{
		creatKeyMap();
		createModifierMap();
	}
	virtual void initializeInput(const std::string& device, const DataGroup& inputData) override {}
	virtual void handleInput(const std::string& device, const DataGroup& inputData) override
	{
		int key, keyMask;
		inputData.integers().get("key", &key);
		inputData.integers().get("key_modifier", &keyMask);

		if (key != -1)
		{
			std::cerr << "Key pressed :" << keyMap[key] << std::endl;
			if (keyMask != 0)
			{
				if (modifierMap[keyMask] != "")
				{
					std::cerr << "Modifier:" << modifierMap[keyMask] << std::endl;
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
	// Note that osgGA doesn't give back the modifier's key code when a modifier and a key are pressed at the same time.
	std::map<int, std::string> modifierMap;

	void creatKeyMap()
	{
		keyMap[0x20]="KEY_SPACE";
		keyMap['0'] = "0";
		keyMap['1'] = "1";
		keyMap['2'] = "2";
		keyMap['3'] = "3";
		keyMap['4'] = "4";
		keyMap['5'] = "5";
		keyMap['6'] = "6";
		keyMap['7'] = "7";
		keyMap['8'] = "8";
		keyMap['9'] = "9";
		keyMap['a'] = "a";
		keyMap['b'] = "b";
		keyMap['c'] = "c";
		keyMap['d'] = "d";
		keyMap['e'] = "e";
		keyMap['f'] = "f";
		keyMap['g'] = "g";
		keyMap['h'] = "h";
		keyMap['i'] = "i";
		keyMap['j'] = "j";
		keyMap['k'] = "k";
		keyMap['l'] = "l";
		keyMap['m'] = "m";
		keyMap['n'] = "n";
		keyMap['o'] = "o";
		keyMap['p'] = "p";
		keyMap['q'] = "q";
		keyMap['r'] = "r";
		keyMap['s'] = "s";
		keyMap['t'] = "t";
		keyMap['u'] = "u";
		keyMap['v'] = "v";
		keyMap['w'] = "w";
		keyMap['x'] = "x";
		keyMap['y'] = "y";
		keyMap['z'] = "z";
		keyMap[0x21] = "KEY_EXCLAIM";
		keyMap[0x22] = "KEY_QUOTEDBL";
		keyMap[0x23] = "KEY_HASH";
		keyMap[0x24] = "KEY_DOLLAR";
		keyMap[0x26] = "KEY_AMPERSAND";
		keyMap[0x27] = "KEY_QUOTE";
		keyMap[0x28] = "KEY_LEFTPAREN";
		keyMap[0x29] = "KEY_RIGHTPAREN";
		keyMap[0x2A] = "KEY_ASTERISK";
		keyMap[0x2B] = "KEY_PLUS";
		keyMap[0x2C] = "KEY_COMMA";
		keyMap[0x2D] = "KEY_MINUS";
		keyMap[0x2E] = "KEY_PERIOD";
		keyMap[0x2F] = "KEY_SLASH";
		keyMap[0x3A] = "KEY_COLON";
		keyMap[0x3B] = "KEY_SEMICOLON";
		keyMap[0x3C] = "KEY_LESS";
		keyMap[0x3D] = "KEY_EQUALS";
		keyMap[0x3E] = "KEY_GREATER";
		keyMap[0x3F] = "KEY_QUESTION";
		keyMap[0x40] = "KEY_AT";
		keyMap[0x5B] = "KEY_LEFTBRACKET";
		keyMap[0x5C] = "KEY_BACKSLASH";
		keyMap[0x5D] = "KEY_RIGHTBRACKET";
		keyMap[0x5E] = "KEY_CARET";
		keyMap[0x5F] = "KEY_UNDERSCORE";
		keyMap[0x60] = "KEY_BACKQUOTE";
		keyMap[0xFF08] = "KEY_BACKSPACE";
		keyMap[0xFF09] = "KEY_TAB";
		keyMap[0xFF0A] = "KEY_LINEFEED";
		keyMap[0xFF0B] = "KEY_CLEAR";
		keyMap[0xFF0D] = "KEY_RETURN";
		keyMap[0xFF13] = "KEY_PAUSE";
		keyMap[0xFF14] = "KEY_SCROLL_LOCK";
		keyMap[0xFF15] = "KEY_SYS_REQ";
		keyMap[0xFF1B] = "KEY_ESCAPE";
		keyMap[0xFFFF] = "KEY_DELETE";
		keyMap[0xFF50] = "KEY_HOME";
		keyMap[0xFF51] = "KEY_LEFT";
		keyMap[0xFF52] = "KEY_UP";
		keyMap[0xFF53] = "KEY_RIGHT";
		keyMap[0xFF54] = "KEY_DOWN";
		keyMap[0xFF55] = "KEY_PRIOR";
		keyMap[0xFF55] = "KEY_PAGE_UP";
		keyMap[0xFF56] = "KEY_NEXT";
		keyMap[0xFF56] = "KEY_PAGE_DOWN";
		keyMap[0xFF57] = "KEY_END";
		keyMap[0xFF58] = "KEY_BEGIN";
		keyMap[0xFF60] = "KEY_SELECT";
		keyMap[0xFF61] = "KEY_PRINT";
		keyMap[0xFF62] = "KEY_EXECUTE";
		keyMap[0xFF63] = "KEY_INSERT";
		keyMap[0xFF65] = "KEY_UNDO";
		keyMap[0xFF66] = "KEY_REDO";
		keyMap[0xFF67] = "KEY_MENU";
		keyMap[0xFF68] = "KEY_FIND";
		keyMap[0xFF69] = "KEY_CANCEL";
		keyMap[0xFF6A] = "KEY_HELP";
		keyMap[0xFF6B] = "KEY_BREAK";
		keyMap[0xFF7E] = "KEY_MODE_SWITCH";
		keyMap[0xFF7E] = "KEY_SCRIPT_SWITCH";
		keyMap[0xFF7F] = "KEY_NUM_LOCK";
		keyMap[0xFF80] = "KEY_KP_SPACE";
		keyMap[0xFF89] = "KEY_KP_TAB";
		keyMap[0xFF8D] = "KEY_KP_ENTER";
		keyMap[0xFF91] = "KEY_KP_F1";
		keyMap[0xFF92] = "KEY_KP_F2";
		keyMap[0xFF93] = "KEY_KP_F3";
		keyMap[0xFF94] = "KEY_KP_F4";
		keyMap[0xFF95] = "KEY_KP_HOME";
		keyMap[0xFF96] = "KEY_KP_LEFT";
		keyMap[0xFF97] = "KEY_KP_UP";
		keyMap[0xFF98] = "KEY_KP_RIGHT";
		keyMap[0xFF99] = "KEY_KP_DOWN";
		keyMap[0xFF9A] = "KEY_KP_PRIOR";
		keyMap[0xFF9A] = "KEY_KP_PAGE_UP";
		keyMap[0xFF9B] = "KEY_KP_NEXT";
		keyMap[0xFF9B] = "KEY_KP_PAGE_DOWN";
		keyMap[0xFF9C] = "KEY_KP_END";
		keyMap[0xFF9D] = "KEY_KP_BEGIN";
		keyMap[0xFF9E] = "KEY_KP_INSERT";
		keyMap[0xFF9F] = "KEY_KP_DELETE";
		keyMap[0xFFBD] = "KEY_KP_EQUAL";
		keyMap[0xFFAA] = "KEY_KP_MULTIPLY";
		keyMap[0xFFAB] = "KEY_KP_ADD";
		keyMap[0xFFAC] = "KEY_KP_SEPARATOR";
		keyMap[0xFFAD] = "KEY_KP_SUBTRACT";
		keyMap[0xFFAE] = "KEY_KP_DECIMAL";
		keyMap[0xFFAF] = "KEY_KP_DIVIDE";
		keyMap[0xFFB0] = "KEY_KP_0";
		keyMap[0xFFB1] = "KEY_KP_1";
		keyMap[0xFFB2] = "KEY_KP_2";
		keyMap[0xFFB3] = "KEY_KP_3";
		keyMap[0xFFB4] = "KEY_KP_4";
		keyMap[0xFFB5] = "KEY_KP_5";
		keyMap[0xFFB6] = "KEY_KP_6";
		keyMap[0xFFB7] = "KEY_KP_7";
		keyMap[0xFFB8] = "KEY_KP_8";
		keyMap[0xFFB9] = "KEY_KP_9";
		keyMap[0xFFBE] = "KEY_F1";
		keyMap[0xFFBF] = "KEY_F2";
		keyMap[0xFFC0] = "KEY_F3";
		keyMap[0xFFC1] = "KEY_F4";
		keyMap[0xFFC2] = "KEY_F5";
		keyMap[0xFFC3] = "KEY_F6";
		keyMap[0xFFC4] = "KEY_F7";
		keyMap[0xFFC5] = "KEY_F8";
		keyMap[0xFFC6] = "KEY_F9";
		keyMap[0xFFC7] = "KEY_F10";
		keyMap[0xFFC8] = "KEY_F11";
		keyMap[0xFFC9] = "KEY_F12";
		keyMap[0xFFCA] = "KEY_F13";
		keyMap[0xFFCB] = "KEY_F14";
		keyMap[0xFFCC] = "KEY_F15";
		keyMap[0xFFCD] = "KEY_F16";
		keyMap[0xFFCE] = "KEY_F17";
		keyMap[0xFFCF] = "KEY_F18";
		keyMap[0xFFD0] = "KEY_F19";
		keyMap[0xFFD1] = "KEY_F20";
		keyMap[0xFFD2] = "KEY_F21";
		keyMap[0xFFD3] = "KEY_F22";
		keyMap[0xFFD4] = "KEY_F23";
		keyMap[0xFFD5] = "KEY_F24";
		keyMap[0xFFD6] = "KEY_F25";
		keyMap[0xFFD7] = "KEY_F26";
		keyMap[0xFFD8] = "KEY_F27";
		keyMap[0xFFD9] = "KEY_F28";
		keyMap[0xFFDA] = "KEY_F29";
		keyMap[0xFFDB] = "KEY_F30";
		keyMap[0xFFDC] = "KEY_F31";
		keyMap[0xFFDD] = "KEY_F32";
		keyMap[0xFFDE] = "KEY_F33";
		keyMap[0xFFDF] = "KEY_F34";
		keyMap[0xFFE0] = "KEY_F35";
		keyMap[0xFFE1] = "KEY_SHIFT_L";
		keyMap[0xFFE2] = "KEY_SHIFT_R";
		keyMap[0xFFE3] = "KEY_CONTROL_L";
		keyMap[0xFFE4] = "KEY_CONTROL_R";
		keyMap[0xFFE5] = "KEY_CAPS_LOCK";
		keyMap[0xFFE6] = "KEY_SHIFT_LOCK";
		keyMap[0xFFE7] = "KEY_META_L";
		keyMap[0xFFE8] = "KEY_META_R";
		keyMap[0xFFE9] = "KEY_ALT_L";
		keyMap[0xFFEA] = "KEY_ALT_R";
		keyMap[0xFFEB] = "KEY_SUPER_L";
		keyMap[0xFFEC] = "KEY_SUPER_R";
		keyMap[0xFFED] = "KEY_HYPER_L";
		keyMap[0xFFEE] = "KEY_HYPER_R";
	};

	void createModifierMap()
	{
		modifierMap[0x0001] = "KEY_SHIFT_L";
		modifierMap[0x0002] = "KEY_SHIFT_R";
		modifierMap[0x0004] = "KEY_CONTROL_L";
		modifierMap[0x0008] = "KEY_CONTROL_R";
		modifierMap[0x0010] = "KEY_ALT_L";
		modifierMap[0x0020] = "KEY_ALT_R";
		modifierMap[0x0040] = "KEY_META_L";
		modifierMap[0x0080] = "KEY_META_R";
		modifierMap[0x0100] = "KEY_SUPER_L";
		modifierMap[0x0200] = "KEY_SUPER_R";
		modifierMap[0x0400] = "KEY_HYPER_L";
		modifierMap[0x0800] = "KEY_HYPER_R";
		modifierMap[0x1000] = "KEY_NUM_LOCK";
		modifierMap[0x2000] = "KEY_CAPS_LOCK";
		modifierMap[(0x2000|0x0001)] = "KEY_CAPS_SHIFT_L";
		modifierMap[(0x2000|0x0002)] = "KEY_CAPS_SHIFT_R";
		modifierMap[(0x2000|0x0004)] = "KEY_CAPS_CONTROL_L";
		modifierMap[(0x2000|0x0008)] = "KEY_CAPS_CONTROL_R";
		modifierMap[(0x2000|0x0010)] = "KEY_CAPS_ALT_L";
		modifierMap[(0x2000|0x0020)] = "KEY_CAPS_ALT_R";
	}
};

int main(int argc, char* argv[])
{
	auto toolDevice	 = std::make_shared<SurgSim::Device::KeyboardDevice>("Keyboard");
	toolDevice->initialize();
	osg::ref_ptr<osgGA::GUIEventHandler> keyboardHandler = toolDevice->getKeyboardHandler();
	std::shared_ptr<TestListener> consumer = std::make_shared<TestListener>();
	toolDevice->addInputConsumer(consumer);

	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
	viewer->apply(new osgViewer::SingleWindow(100, 200, 600, 400));
	viewer->addEventHandler(keyboardHandler);

	osg::ref_ptr<osg::Group> group = new osg::Group;
	viewer->setSceneData(group);

	osg::ref_ptr<osg::MatrixTransform> matrixTransform = new osg::MatrixTransform;
	matrixTransform->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

	osg::ref_ptr<osg::Projection> projection = new osg::Projection;
	projection->setMatrix(osg::Matrix::ortho2D(0, 600, -600, 400));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setFont("./Data/Vera.ttf");
	text->setCharacterSize(28);
	text->setText("Press any key in this window to verify\n\n that keyboard driver works correctly.");

	geode->addDrawable(text);
	projection->addChild(geode);
	matrixTransform->addChild(projection);
	group->addChild(matrixTransform);

	viewer->run();
	return 0;
}
