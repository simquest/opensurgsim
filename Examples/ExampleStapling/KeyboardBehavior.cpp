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

#include "Examples/ExampleStapling/KeyboardBehavior.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Input/InputComponent.h"

std::unordered_map<int, SurgSim::Device::KeyCode> createKeyMap()
{
	static std::unordered_map<int, SurgSim::Device::KeyCode> keyMap;
	keyMap[SurgSim::Device::KeyCode::KEY_SPACE] = SurgSim::Device::KeyCode::KEY_SPACE;
	keyMap[SurgSim::Device::KeyCode::KEY_0] = SurgSim::Device::KeyCode::KEY_0;
	keyMap[SurgSim::Device::KeyCode::KEY_1] = SurgSim::Device::KeyCode::KEY_1;
	keyMap[SurgSim::Device::KeyCode::KEY_2] = SurgSim::Device::KeyCode::KEY_2;
	keyMap[SurgSim::Device::KeyCode::KEY_3] = SurgSim::Device::KeyCode::KEY_3;
	keyMap[SurgSim::Device::KeyCode::KEY_4] = SurgSim::Device::KeyCode::KEY_4;
	keyMap[SurgSim::Device::KeyCode::KEY_5] = SurgSim::Device::KeyCode::KEY_5;
	keyMap[SurgSim::Device::KeyCode::KEY_6] = SurgSim::Device::KeyCode::KEY_6;
	keyMap[SurgSim::Device::KeyCode::KEY_7] = SurgSim::Device::KeyCode::KEY_7;
	keyMap[SurgSim::Device::KeyCode::KEY_8] = SurgSim::Device::KeyCode::KEY_8;
	keyMap[SurgSim::Device::KeyCode::KEY_9] = SurgSim::Device::KeyCode::KEY_9;
	keyMap[SurgSim::Device::KeyCode::KEY_A] = SurgSim::Device::KeyCode::KEY_A;
	keyMap[SurgSim::Device::KeyCode::KEY_B] = SurgSim::Device::KeyCode::KEY_B;
	keyMap[SurgSim::Device::KeyCode::KEY_C] = SurgSim::Device::KeyCode::KEY_C;
	keyMap[SurgSim::Device::KeyCode::KEY_D] = SurgSim::Device::KeyCode::KEY_D;
	keyMap[SurgSim::Device::KeyCode::KEY_E] = SurgSim::Device::KeyCode::KEY_E;
	keyMap[SurgSim::Device::KeyCode::KEY_F] = SurgSim::Device::KeyCode::KEY_F;
	keyMap[SurgSim::Device::KeyCode::KEY_G] = SurgSim::Device::KeyCode::KEY_G;
	keyMap[SurgSim::Device::KeyCode::KEY_H] = SurgSim::Device::KeyCode::KEY_H;
	keyMap[SurgSim::Device::KeyCode::KEY_I] = SurgSim::Device::KeyCode::KEY_I;
	keyMap[SurgSim::Device::KeyCode::KEY_J] = SurgSim::Device::KeyCode::KEY_J;
	keyMap[SurgSim::Device::KeyCode::KEY_K] = SurgSim::Device::KeyCode::KEY_K;
	keyMap[SurgSim::Device::KeyCode::KEY_L] = SurgSim::Device::KeyCode::KEY_L;
	keyMap[SurgSim::Device::KeyCode::KEY_M] = SurgSim::Device::KeyCode::KEY_M;
	keyMap[SurgSim::Device::KeyCode::KEY_N] = SurgSim::Device::KeyCode::KEY_N;
	keyMap[SurgSim::Device::KeyCode::KEY_O] = SurgSim::Device::KeyCode::KEY_O;
	keyMap[SurgSim::Device::KeyCode::KEY_P] = SurgSim::Device::KeyCode::KEY_P;
	keyMap[SurgSim::Device::KeyCode::KEY_Q] = SurgSim::Device::KeyCode::KEY_Q;
	keyMap[SurgSim::Device::KeyCode::KEY_R] = SurgSim::Device::KeyCode::KEY_R;
	keyMap[SurgSim::Device::KeyCode::KEY_S] = SurgSim::Device::KeyCode::KEY_S;
	keyMap[SurgSim::Device::KeyCode::KEY_T] = SurgSim::Device::KeyCode::KEY_T;
	keyMap[SurgSim::Device::KeyCode::KEY_U] = SurgSim::Device::KeyCode::KEY_U;
	keyMap[SurgSim::Device::KeyCode::KEY_V] = SurgSim::Device::KeyCode::KEY_V;
	keyMap[SurgSim::Device::KeyCode::KEY_W] = SurgSim::Device::KeyCode::KEY_W;
	keyMap[SurgSim::Device::KeyCode::KEY_X] = SurgSim::Device::KeyCode::KEY_X;
	keyMap[SurgSim::Device::KeyCode::KEY_Y] = SurgSim::Device::KeyCode::KEY_Y;
	keyMap[SurgSim::Device::KeyCode::KEY_Z] = SurgSim::Device::KeyCode::KEY_Z;
	keyMap[SurgSim::Device::KeyCode::KEY_EXCLAIM] = SurgSim::Device::KeyCode::KEY_EXCLAIM;
	keyMap[SurgSim::Device::KeyCode::KEY_QUOTEDBL] = SurgSim::Device::KeyCode::KEY_QUOTEDBL;
	keyMap[SurgSim::Device::KeyCode::KEY_HASH] = SurgSim::Device::KeyCode::KEY_HASH;
	keyMap[SurgSim::Device::KeyCode::KEY_DOLLAR] = SurgSim::Device::KeyCode::KEY_DOLLAR;
	keyMap[SurgSim::Device::KeyCode::KEY_AMPERSAND] = SurgSim::Device::KeyCode::KEY_AMPERSAND;
	keyMap[SurgSim::Device::KeyCode::KEY_QUOTE] = SurgSim::Device::KeyCode::KEY_QUOTE;
	keyMap[SurgSim::Device::KeyCode::KEY_LEFTPAREN] = SurgSim::Device::KeyCode::KEY_LEFTPAREN;
	keyMap[SurgSim::Device::KeyCode::KEY_RIGHTPAREN]= SurgSim::Device::KeyCode::KEY_RIGHTPAREN;
	keyMap[SurgSim::Device::KeyCode::KEY_ASTERISK] = SurgSim::Device::KeyCode::KEY_ASTERISK;
	keyMap[SurgSim::Device::KeyCode::KEY_PLUS] = SurgSim::Device::KeyCode::KEY_PLUS;
	keyMap[SurgSim::Device::KeyCode::KEY_COMMA] = SurgSim::Device::KeyCode::KEY_COMMA;
	keyMap[SurgSim::Device::KeyCode::KEY_MINUS] = SurgSim::Device::KeyCode::KEY_MINUS;
	keyMap[SurgSim::Device::KeyCode::KEY_PERIOD] = SurgSim::Device::KeyCode::KEY_PERIOD;
	keyMap[SurgSim::Device::KeyCode::KEY_SLASH] = SurgSim::Device::KeyCode::KEY_SLASH;
	keyMap[SurgSim::Device::KeyCode::KEY_COLON] = SurgSim::Device::KeyCode::KEY_COLON;
	keyMap[SurgSim::Device::KeyCode::KEY_SEMICOLON] = SurgSim::Device::KeyCode::KEY_SEMICOLON;
	keyMap[SurgSim::Device::KeyCode::KEY_LESS] = SurgSim::Device::KeyCode::KEY_LESS;
	keyMap[SurgSim::Device::KeyCode::KEY_EQUALS] = SurgSim::Device::KeyCode::KEY_EQUALS;
	keyMap[SurgSim::Device::KeyCode::KEY_GREATER] = SurgSim::Device::KeyCode::KEY_GREATER;
	keyMap[SurgSim::Device::KeyCode::KEY_QUESTION] = SurgSim::Device::KeyCode::KEY_QUESTION;
	keyMap[SurgSim::Device::KeyCode::KEY_AT] = SurgSim::Device::KeyCode::KEY_AT;
	keyMap[SurgSim::Device::KeyCode::KEY_LEFTBRACKET] = SurgSim::Device::KeyCode::KEY_LEFTBRACKET;
	keyMap[SurgSim::Device::KeyCode::KEY_BACKSLASH] = SurgSim::Device::KeyCode::KEY_BACKSLASH;
	keyMap[SurgSim::Device::KeyCode::KEY_RIGHTBRACKET]= SurgSim::Device::KeyCode::KEY_RIGHTBRACKET;
	keyMap[SurgSim::Device::KeyCode::KEY_CARET] = SurgSim::Device::KeyCode::KEY_CARET;
	keyMap[SurgSim::Device::KeyCode::KEY_UNDERSCORE] = SurgSim::Device::KeyCode::KEY_UNDERSCORE;
	keyMap[SurgSim::Device::KeyCode::KEY_BACKQUOTE] = SurgSim::Device::KeyCode::KEY_BACKQUOTE;
	keyMap[SurgSim::Device::KeyCode::KEY_BACKSPACE] = SurgSim::Device::KeyCode::KEY_BACKSPACE;
	keyMap[SurgSim::Device::KeyCode::KEY_TAB] = SurgSim::Device::KeyCode::KEY_TAB;
	keyMap[SurgSim::Device::KeyCode::KEY_LINEFEED] = SurgSim::Device::KeyCode::KEY_LINEFEED;
	keyMap[SurgSim::Device::KeyCode::KEY_CLEAR] = SurgSim::Device::KeyCode::KEY_CLEAR;
	keyMap[SurgSim::Device::KeyCode::KEY_RETURN] = SurgSim::Device::KeyCode::KEY_RETURN;
	keyMap[SurgSim::Device::KeyCode::KEY_PAUSE] = SurgSim::Device::KeyCode::KEY_PAUSE;
	keyMap[SurgSim::Device::KeyCode::KEY_SCROLL_LOCK] = SurgSim::Device::KeyCode::KEY_SCROLL_LOCK;
	keyMap[SurgSim::Device::KeyCode::KEY_SYS_REQ] = SurgSim::Device::KeyCode::KEY_SYS_REQ;
	keyMap[SurgSim::Device::KeyCode::KEY_ESCAPE] = SurgSim::Device::KeyCode::KEY_ESCAPE;
	keyMap[SurgSim::Device::KeyCode::KEY_DELETE] = SurgSim::Device::KeyCode::KEY_DELETE;
	keyMap[SurgSim::Device::KeyCode::KEY_HOME] = SurgSim::Device::KeyCode::KEY_HOME;
	keyMap[SurgSim::Device::KeyCode::KEY_LEFT] = SurgSim::Device::KeyCode::KEY_LEFT;
	keyMap[SurgSim::Device::KeyCode::KEY_UP] = SurgSim::Device::KeyCode::KEY_UP;
	keyMap[SurgSim::Device::KeyCode::KEY_RIGHT] = SurgSim::Device::KeyCode::KEY_RIGHT;
	keyMap[SurgSim::Device::KeyCode::KEY_DOWN] = SurgSim::Device::KeyCode::KEY_DOWN;
	keyMap[SurgSim::Device::KeyCode::KEY_PRIOR] = SurgSim::Device::KeyCode::KEY_PRIOR;
	keyMap[SurgSim::Device::KeyCode::KEY_PAGE_UP] = SurgSim::Device::KeyCode::KEY_PAGE_UP;
	keyMap[SurgSim::Device::KeyCode::KEY_NEXT] = SurgSim::Device::KeyCode::KEY_NEXT;
	keyMap[SurgSim::Device::KeyCode::KEY_PAGE_DOWN] = SurgSim::Device::KeyCode::KEY_PAGE_DOWN;
	keyMap[SurgSim::Device::KeyCode::KEY_END] = SurgSim::Device::KeyCode::KEY_END;
	keyMap[SurgSim::Device::KeyCode::KEY_BEGIN] = SurgSim::Device::KeyCode::KEY_BEGIN;
	keyMap[SurgSim::Device::KeyCode::KEY_SELECT] = SurgSim::Device::KeyCode::KEY_SELECT;
	keyMap[SurgSim::Device::KeyCode::KEY_PRINT] = SurgSim::Device::KeyCode::KEY_PRINT;
	keyMap[SurgSim::Device::KeyCode::KEY_EXECUTE] = SurgSim::Device::KeyCode::KEY_EXECUTE;
	keyMap[SurgSim::Device::KeyCode::KEY_INSERT] = SurgSim::Device::KeyCode::KEY_INSERT;
	keyMap[SurgSim::Device::KeyCode::KEY_UNDO] = SurgSim::Device::KeyCode::KEY_UNDO;
	keyMap[SurgSim::Device::KeyCode::KEY_REDO] = SurgSim::Device::KeyCode::KEY_REDO;
	keyMap[SurgSim::Device::KeyCode::KEY_MENU] = SurgSim::Device::KeyCode::KEY_MENU;
	keyMap[SurgSim::Device::KeyCode::KEY_FIND] = SurgSim::Device::KeyCode::KEY_FIND;
	keyMap[SurgSim::Device::KeyCode::KEY_CANCEL] = SurgSim::Device::KeyCode::KEY_CANCEL;
	keyMap[SurgSim::Device::KeyCode::KEY_HELP] = SurgSim::Device::KeyCode::KEY_HELP;
	keyMap[SurgSim::Device::KeyCode::KEY_BREAK] = SurgSim::Device::KeyCode::KEY_BREAK;
	keyMap[SurgSim::Device::KeyCode::KEY_MODE_SWITCH] = SurgSim::Device::KeyCode::KEY_MODE_SWITCH;
	keyMap[SurgSim::Device::KeyCode::KEY_SCRIPT_SWITCH] = SurgSim::Device::KeyCode::KEY_SCRIPT_SWITCH;
	keyMap[SurgSim::Device::KeyCode::KEY_NUM_LOCK] = SurgSim::Device::KeyCode::KEY_NUM_LOCK;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_SPACE] = SurgSim::Device::KeyCode::KEY_KP_SPACE;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_TAB] = SurgSim::Device::KeyCode::KEY_KP_TAB;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_ENTER] = SurgSim::Device::KeyCode::KEY_KP_ENTER;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_F1] = SurgSim::Device::KeyCode::KEY_KP_F1;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_F2] = SurgSim::Device::KeyCode::KEY_KP_F2;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_F3] = SurgSim::Device::KeyCode::KEY_KP_F3;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_F4] = SurgSim::Device::KeyCode::KEY_KP_F4;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_HOME] = SurgSim::Device::KeyCode::KEY_KP_HOME;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_LEFT] = SurgSim::Device::KeyCode::KEY_KP_LEFT;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_UP] = SurgSim::Device::KeyCode::KEY_KP_UP;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_RIGHT] = SurgSim::Device::KeyCode::KEY_KP_RIGHT;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_DOWN] = SurgSim::Device::KeyCode::KEY_KP_DOWN;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_PRIOR] = SurgSim::Device::KeyCode::KEY_KP_PRIOR;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_PAGE_UP] = SurgSim::Device::KeyCode::KEY_KP_PAGE_UP;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_NEXT] = SurgSim::Device::KeyCode::KEY_KP_NEXT;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_PAGE_DOWN] = SurgSim::Device::KeyCode::KEY_KP_PAGE_DOWN;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_END] = SurgSim::Device::KeyCode::KEY_KP_END;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_BEGIN] = SurgSim::Device::KeyCode::KEY_KP_BEGIN;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_INSERT] = SurgSim::Device::KeyCode::KEY_KP_INSERT;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_DELETE] = SurgSim::Device::KeyCode::KEY_KP_DELETE;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_EQUAL] = SurgSim::Device::KeyCode::KEY_KP_EQUAL;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_MULTIPLY] = SurgSim::Device::KeyCode::KEY_KP_MULTIPLY;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_ADD] = SurgSim::Device::KeyCode::KEY_KP_ADD;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_SEPARATOR]= SurgSim::Device::KeyCode::KEY_KP_SEPARATOR;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_SUBTRACT] = SurgSim::Device::KeyCode::KEY_KP_SUBTRACT;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_DECIMAL] = SurgSim::Device::KeyCode::KEY_KP_DECIMAL;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_DIVIDE] = SurgSim::Device::KeyCode::KEY_KP_DIVIDE;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_0] = SurgSim::Device::KeyCode::KEY_KP_0;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_1] = SurgSim::Device::KeyCode::KEY_KP_1;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_2] = SurgSim::Device::KeyCode::KEY_KP_2;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_3] = SurgSim::Device::KeyCode::KEY_KP_3;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_4] = SurgSim::Device::KeyCode::KEY_KP_4;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_5] = SurgSim::Device::KeyCode::KEY_KP_5;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_6] = SurgSim::Device::KeyCode::KEY_KP_6;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_7] = SurgSim::Device::KeyCode::KEY_KP_7;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_8] = SurgSim::Device::KeyCode::KEY_KP_8;
	keyMap[SurgSim::Device::KeyCode::KEY_KP_9] = SurgSim::Device::KeyCode::KEY_KP_9;
	keyMap[SurgSim::Device::KeyCode::KEY_F1] = SurgSim::Device::KeyCode::KEY_F1;
	keyMap[SurgSim::Device::KeyCode::KEY_F2] = SurgSim::Device::KeyCode::KEY_F2;
	keyMap[SurgSim::Device::KeyCode::KEY_F3] = SurgSim::Device::KeyCode::KEY_F3;
	keyMap[SurgSim::Device::KeyCode::KEY_F4] = SurgSim::Device::KeyCode::KEY_F4;
	keyMap[SurgSim::Device::KeyCode::KEY_F5] = SurgSim::Device::KeyCode::KEY_F5;
	keyMap[SurgSim::Device::KeyCode::KEY_F6] = SurgSim::Device::KeyCode::KEY_F6;
	keyMap[SurgSim::Device::KeyCode::KEY_F7] = SurgSim::Device::KeyCode::KEY_F7;
	keyMap[SurgSim::Device::KeyCode::KEY_F8] = SurgSim::Device::KeyCode::KEY_F8;
	keyMap[SurgSim::Device::KeyCode::KEY_F9] = SurgSim::Device::KeyCode::KEY_F9;
	keyMap[SurgSim::Device::KeyCode::KEY_F10] = SurgSim::Device::KeyCode::KEY_F10;
	keyMap[SurgSim::Device::KeyCode::KEY_F11] = SurgSim::Device::KeyCode::KEY_F11;
	keyMap[SurgSim::Device::KeyCode::KEY_F12] = SurgSim::Device::KeyCode::KEY_F12;
	keyMap[SurgSim::Device::KeyCode::KEY_F13] = SurgSim::Device::KeyCode::KEY_F13;
	keyMap[SurgSim::Device::KeyCode::KEY_F14] = SurgSim::Device::KeyCode::KEY_F14;
	keyMap[SurgSim::Device::KeyCode::KEY_F15] = SurgSim::Device::KeyCode::KEY_F15;
	keyMap[SurgSim::Device::KeyCode::KEY_F16] = SurgSim::Device::KeyCode::KEY_F16;
	keyMap[SurgSim::Device::KeyCode::KEY_F17] = SurgSim::Device::KeyCode::KEY_F17;
	keyMap[SurgSim::Device::KeyCode::KEY_F18] = SurgSim::Device::KeyCode::KEY_F18;
	keyMap[SurgSim::Device::KeyCode::KEY_F19] = SurgSim::Device::KeyCode::KEY_F19;
	keyMap[SurgSim::Device::KeyCode::KEY_F20] = SurgSim::Device::KeyCode::KEY_F20;
	keyMap[SurgSim::Device::KeyCode::KEY_F21] = SurgSim::Device::KeyCode::KEY_F21;
	keyMap[SurgSim::Device::KeyCode::KEY_F22] = SurgSim::Device::KeyCode::KEY_F22;
	keyMap[SurgSim::Device::KeyCode::KEY_F23] = SurgSim::Device::KeyCode::KEY_F23;
	keyMap[SurgSim::Device::KeyCode::KEY_F24] = SurgSim::Device::KeyCode::KEY_F24;
	keyMap[SurgSim::Device::KeyCode::KEY_F25] = SurgSim::Device::KeyCode::KEY_F25;
	keyMap[SurgSim::Device::KeyCode::KEY_F26] = SurgSim::Device::KeyCode::KEY_F26;
	keyMap[SurgSim::Device::KeyCode::KEY_F27] = SurgSim::Device::KeyCode::KEY_F27;
	keyMap[SurgSim::Device::KeyCode::KEY_F28] = SurgSim::Device::KeyCode::KEY_F28;
	keyMap[SurgSim::Device::KeyCode::KEY_F29] = SurgSim::Device::KeyCode::KEY_F29;
	keyMap[SurgSim::Device::KeyCode::KEY_F30] = SurgSim::Device::KeyCode::KEY_F30;
	keyMap[SurgSim::Device::KeyCode::KEY_F31] = SurgSim::Device::KeyCode::KEY_F31;
	keyMap[SurgSim::Device::KeyCode::KEY_F32] = SurgSim::Device::KeyCode::KEY_F32;
	keyMap[SurgSim::Device::KeyCode::KEY_F33] = SurgSim::Device::KeyCode::KEY_F33;
	keyMap[SurgSim::Device::KeyCode::KEY_F34] = SurgSim::Device::KeyCode::KEY_F34;
	keyMap[SurgSim::Device::KeyCode::KEY_F35] = SurgSim::Device::KeyCode::KEY_F35;
	keyMap[SurgSim::Device::KeyCode::KEY_SHIFT_L] = SurgSim::Device::KeyCode::KEY_SHIFT_L;
	keyMap[SurgSim::Device::KeyCode::KEY_SHIFT_R] = SurgSim::Device::KeyCode::KEY_SHIFT_R;
	keyMap[SurgSim::Device::KeyCode::KEY_CONTROL_L] = SurgSim::Device::KeyCode::KEY_CONTROL_L;
	keyMap[SurgSim::Device::KeyCode::KEY_CONTROL_R] = SurgSim::Device::KeyCode::KEY_CONTROL_R;
	keyMap[SurgSim::Device::KeyCode::KEY_CAPS_LOCK] = SurgSim::Device::KeyCode::KEY_CAPS_LOCK;
	keyMap[SurgSim::Device::KeyCode::KEY_SHIFT_LOCK] = SurgSim::Device::KeyCode::KEY_SHIFT_LOCK;
	keyMap[SurgSim::Device::KeyCode::KEY_META_L] = SurgSim::Device::KeyCode::KEY_META_L;
	keyMap[SurgSim::Device::KeyCode::KEY_META_R] = SurgSim::Device::KeyCode::KEY_META_R;
	keyMap[SurgSim::Device::KeyCode::KEY_ALT_L] = SurgSim::Device::KeyCode::KEY_ALT_L;
	keyMap[SurgSim::Device::KeyCode::KEY_ALT_R] = SurgSim::Device::KeyCode::KEY_ALT_R;
	keyMap[SurgSim::Device::KeyCode::KEY_SUPER_L] = SurgSim::Device::KeyCode::KEY_SUPER_L;
	keyMap[SurgSim::Device::KeyCode::KEY_SUPER_R] = SurgSim::Device::KeyCode::KEY_SUPER_R;
	keyMap[SurgSim::Device::KeyCode::KEY_HYPER_L] = SurgSim::Device::KeyCode::KEY_HYPER_L;
	keyMap[SurgSim::Device::KeyCode::KEY_HYPER_R] = SurgSim::Device::KeyCode::KEY_HYPER_R;

	return keyMap;
}
namespace{
	// Maps KeyCode with corresponding integer.
	std::unordered_map<int, SurgSim::Device::KeyCode> keyMap = createKeyMap();
}

KeyboardBehavior::KeyboardBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_totalTime(0.0)
{
}

void KeyboardBehavior::setInputComponent(std::shared_ptr<SurgSim::Input::InputComponent> inputComponent)
{
	m_inputComponent = inputComponent;
}

// Note: This behavior is currently updated by BehaviorManager which runs at 30Hz.
// Since the speed (30Hz) is fast compared with the speed one key is pressed, the graphical representations will be
// set to visible/invisible even with one key press.
void KeyboardBehavior::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;
	m_inputComponent->getData(&dataGroup);

	int key;
	m_totalTime += dt;
	if(dataGroup.integers().get("key", &key))
	{
		auto match = m_keyRegister.find(keyMap[key]);
		if (match != m_keyRegister.end() && m_totalTime > 0.3)
		{
			for(auto it = std::begin(match->second); it != std::end(match->second); ++it)
			{
				(*it)->setVisible(!(*it)->isVisible());
			};
			m_totalTime = 0;
		}
	}
}

bool KeyboardBehavior::doInitialize()
{
	return true;
}

bool KeyboardBehavior::doWakeUp()
{
	return true;
}