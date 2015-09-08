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

#ifndef SURGSIM_DEVICES_KEYBOARD_KEYCODE_H
#define SURGSIM_DEVICES_KEYBOARD_KEYCODE_H

namespace SurgSim
{
namespace Devices
{
	// KeyCode pulled out from osgGA/GUIEventAdapter
	enum KeyCode
	{
		NONE                = -1,
		KEY_SPACE           = 0x20,
		KEY_0               = '0',
		KEY_1               = '1',
		KEY_2               = '2',
		KEY_3               = '3',
		KEY_4               = '4',
		KEY_5               = '5',
		KEY_6               = '6',
		KEY_7               = '7',
		KEY_8               = '8',
		KEY_9               = '9',
		KEY_A               = 'a',
		KEY_B               = 'b',
		KEY_C               = 'c',
		KEY_D               = 'd',
		KEY_E               = 'e',
		KEY_F               = 'f',
		KEY_G               = 'g',
		KEY_H               = 'h',
		KEY_I               = 'i',
		KEY_J               = 'j',
		KEY_K               = 'k',
		KEY_L               = 'l',
		KEY_M               = 'm',
		KEY_N               = 'n',
		KEY_O               = 'o',
		KEY_P               = 'p',
		KEY_Q               = 'q',
		KEY_R               = 'r',
		KEY_S               = 's',
		KEY_T               = 't',
		KEY_U               = 'u',
		KEY_V               = 'v',
		KEY_W               = 'w',
		KEY_X               = 'x',
		KEY_Y               = 'y',
		KEY_Z               = 'z',
		KEY_EXCLAIM         = 0x21,
		KEY_QUOTEDBL        = 0x22,
		KEY_HASH            = 0x23,
		KEY_DOLLAR          = 0x24,
		KEY_AMPERSAND       = 0x26,
		KEY_QUOTE           = 0x27,
		KEY_LEFTPAREN       = 0x28,
		KEY_RIGHTPAREN      = 0x29,
		KEY_ASTERISK        = 0x2A,
		KEY_PLUS            = 0x2B,
		KEY_COMMA           = 0x2C,
		KEY_MINUS           = 0x2D,
		KEY_PERIOD          = 0x2E,
		KEY_SLASH           = 0x2F,
		KEY_COLON           = 0x3A,
		KEY_SEMICOLON       = 0x3B,
		KEY_LESS            = 0x3C,
		KEY_EQUALS          = 0x3D,
		KEY_GREATER         = 0x3E,
		KEY_QUESTION        = 0x3F,
		KEY_AT              = 0x40,
		KEY_LEFTBRACKET     = 0x5B,
		KEY_BACKSLASH       = 0x5C,
		KEY_RIGHTBRACKET    = 0x5D,
		KEY_CARET           = 0x5E,
		KEY_UNDERSCORE      = 0x5F,
		KEY_BACKQUOTE       = 0x60,
		KEY_BACKSPACE       = 0xFF08,        /* back space, back char */
		KEY_TAB             = 0xFF09,
		KEY_LINEFEED        = 0xFF0A,        /* Linefeed, LF */
		KEY_CLEAR           = 0xFF0B,
		KEY_RETURN          = 0xFF0D,        /* Return, enter */
		KEY_PAUSE           = 0xFF13,        /* Pause, hold */
		KEY_SCROLL_LOCK     = 0xFF14,
		KEY_SYS_REQ         = 0xFF15,
		KEY_ESCAPE          = 0xFF1B,
		KEY_DELETE          = 0xFFFF,        /* Delete, rubout */
		KEY_HOME            = 0xFF50,
		KEY_LEFT            = 0xFF51,        /* Move left, left arrow */
		KEY_UP              = 0xFF52,        /* Move up, up arrow */
		KEY_RIGHT           = 0xFF53,        /* Move right, right arrow */
		KEY_DOWN            = 0xFF54,        /* Move down, down arrow */
		KEY_PRIOR           = 0xFF55,        /* Prior, previous */
		KEY_PAGE_UP         = 0xFF55,
		KEY_NEXT            = 0xFF56,        /* Next */
		KEY_PAGE_DOWN       = 0xFF56,
		KEY_END             = 0xFF57,        /* EOL */
		KEY_BEGIN           = 0xFF58,        /* BOL */
		KEY_SELECT          = 0xFF60,        /* Select, mark */
		KEY_PRINT           = 0xFF61,
		KEY_EXECUTE         = 0xFF62,        /* Execute, run, do */
		KEY_INSERT          = 0xFF63,        /* Insert, insert here */
		KEY_UNDO            = 0xFF65,        /* Undo, oops */
		KEY_REDO            = 0xFF66,        /* redo, again */
		KEY_MENU            = 0xFF67,        /* On Windows, this is VK_APPS, the context-menu key */
		KEY_FIND            = 0xFF68,        /* Find, search */
		KEY_CANCEL          = 0xFF69,        /* Cancel, stop, abort, exit */
		KEY_HELP            = 0xFF6A,        /* Help */
		KEY_BREAK           = 0xFF6B,
		KEY_MODE_SWITCH     = 0xFF7E,        /* Character set switch */
		KEY_SCRIPT_SWITCH   = 0xFF7E,        /* Alias for mode_switch */
		KEY_NUM_LOCK        = 0xFF7F,
		KEY_KP_SPACE        = 0xFF80,        /* space */
		KEY_KP_TAB          = 0xFF89,
		KEY_KP_ENTER        = 0xFF8D,        /* enter */
		KEY_KP_F1           = 0xFF91,        /* PF1, KP_A, ... */
		KEY_KP_F2           = 0xFF92,
		KEY_KP_F3           = 0xFF93,
		KEY_KP_F4           = 0xFF94,
		KEY_KP_HOME         = 0xFF95,
		KEY_KP_LEFT         = 0xFF96,
		KEY_KP_UP           = 0xFF97,
		KEY_KP_RIGHT        = 0xFF98,
		KEY_KP_DOWN         = 0xFF99,
		KEY_KP_PRIOR        = 0xFF9A,
		KEY_KP_PAGE_UP      = 0xFF9A,
		KEY_KP_NEXT         = 0xFF9B,
		KEY_KP_PAGE_DOWN    = 0xFF9B,
		KEY_KP_END          = 0xFF9C,
		KEY_KP_BEGIN        = 0xFF9D,
		KEY_KP_INSERT       = 0xFF9E,
		KEY_KP_DELETE       = 0xFF9F,
		KEY_KP_EQUAL        = 0xFFBD,        /* equals */
		KEY_KP_MULTIPLY     = 0xFFAA,
		KEY_KP_ADD          = 0xFFAB,
		KEY_KP_SEPARATOR    = 0xFFAC,       /* separator, often comma */
		KEY_KP_SUBTRACT     = 0xFFAD,
		KEY_KP_DECIMAL      = 0xFFAE,
		KEY_KP_DIVIDE       = 0xFFAF,
		KEY_KP_0            = 0xFFB0,
		KEY_KP_1            = 0xFFB1,
		KEY_KP_2            = 0xFFB2,
		KEY_KP_3            = 0xFFB3,
		KEY_KP_4            = 0xFFB4,
		KEY_KP_5            = 0xFFB5,
		KEY_KP_6            = 0xFFB6,
		KEY_KP_7            = 0xFFB7,
		KEY_KP_8            = 0xFFB8,
		KEY_KP_9            = 0xFFB9,
		KEY_F1              = 0xFFBE,
		KEY_F2              = 0xFFBF,
		KEY_F3              = 0xFFC0,
		KEY_F4              = 0xFFC1,
		KEY_F5              = 0xFFC2,
		KEY_F6              = 0xFFC3,
		KEY_F7              = 0xFFC4,
		KEY_F8              = 0xFFC5,
		KEY_F9              = 0xFFC6,
		KEY_F10             = 0xFFC7,
		KEY_F11             = 0xFFC8,
		KEY_F12             = 0xFFC9,
		KEY_F13             = 0xFFCA,
		KEY_F14             = 0xFFCB,
		KEY_F15             = 0xFFCC,
		KEY_F16             = 0xFFCD,
		KEY_F17             = 0xFFCE,
		KEY_F18             = 0xFFCF,
		KEY_F19             = 0xFFD0,
		KEY_F20             = 0xFFD1,
		KEY_F21             = 0xFFD2,
		KEY_F22             = 0xFFD3,
		KEY_F23             = 0xFFD4,
		KEY_F24             = 0xFFD5,
		KEY_F25             = 0xFFD6,
		KEY_F26             = 0xFFD7,
		KEY_F27             = 0xFFD8,
		KEY_F28             = 0xFFD9,
		KEY_F29             = 0xFFDA,
		KEY_F30             = 0xFFDB,
		KEY_F31             = 0xFFDC,
		KEY_F32             = 0xFFDD,
		KEY_F33             = 0xFFDE,
		KEY_F34             = 0xFFDF,
		KEY_F35             = 0xFFE0,
		/* Modifiers */
		KEY_SHIFT_L         = 0xFFE1,        /* Left shift */
		KEY_SHIFT_R         = 0xFFE2,        /* Right shift */
		KEY_CONTROL_L       = 0xFFE3,        /* Left control */
		KEY_CONTROL_R       = 0xFFE4,        /* Right control */
		KEY_CAPS_LOCK       = 0xFFE5,        /* Caps lock */
		KEY_SHIFT_LOCK      = 0xFFE6,        /* Shift lock */
		KEY_META_L          = 0xFFE7,        /* Left meta */
		KEY_META_R          = 0xFFE8,        /* Right meta */
		KEY_ALT_L           = 0xFFE9,        /* Left alt */
		KEY_ALT_R           = 0xFFEA,        /* Right alt */
		KEY_SUPER_L         = 0xFFEB,        /* Left super */
		KEY_SUPER_R         = 0xFFEC,        /* Right super */
		KEY_HYPER_L         = 0xFFED,        /* Left hyper */
		KEY_HYPER_R         = 0xFFEE         /* Right hyper */
	};

	// Masks pulled out from osgGA/GUIEventAdapter
	enum ModKeyMask
	{
		MODKEY_NONE           = 0,
		MODKEY_LEFT_SHIFT     = 0x0001,
		MODKEY_RIGHT_SHIFT    = 0x0002,
		MODKEY_LEFT_CTRL      = 0x0004,
		MODKEY_RIGHT_CTRL     = 0x0008,
		MODKEY_LEFT_ALT       = 0x0010,
		MODKEY_RIGHT_ALT      = 0x0020,
		MODKEY_LEFT_META      = 0x0040,
		MODKEY_RIGHT_META     = 0x0080,
		MODKEY_LEFT_SUPER     = 0x0100,
		MODKEY_RIGHT_SUPER    = 0x0200,
		MODKEY_LEFT_HYPER     = 0x0400,
		MODKEY_RIGHT_HYPER    = 0x0800,
		MODKEY_NUM_LOCK       = 0x1000,
		MODKEY_CAPS_LOCK      = 0x2000,
		MODKEY_CTRL           = (MODKEY_LEFT_CTRL|MODKEY_RIGHT_CTRL),
		MODKEY_SHIFT          = (MODKEY_LEFT_SHIFT|MODKEY_RIGHT_SHIFT),
		MODKEY_ALT            = (MODKEY_LEFT_ALT|MODKEY_RIGHT_ALT),
		MODKEY_META           = (MODKEY_LEFT_META|MODKEY_RIGHT_META),
		MODKEY_SUPER          = (MODKEY_LEFT_SUPER|MODKEY_RIGHT_SUPER),
		MODKEY_HYPER          = (MODKEY_LEFT_HYPER|MODKEY_RIGHT_HYPER),
		MODKEY_CAPS_SHIFT_L   = (MODKEY_CAPS_LOCK|MODKEY_LEFT_SHIFT),
		MODKEY_CAPS_SHIFT_R   = (MODKEY_CAPS_LOCK|MODKEY_RIGHT_SHIFT),
		MODKEY_CAPS_CONTROL_L = (MODKEY_CAPS_LOCK|MODKEY_LEFT_CTRL),
		MODKEY_CAPS_CONTROL_R = (MODKEY_CAPS_LOCK|MODKEY_RIGHT_CTRL),
		MODKEY_CAPS_ALT_L     = (MODKEY_CAPS_LOCK|MODKEY_LEFT_ALT),
		MODKEY_CAPS_ALT_R     = (MODKEY_CAPS_LOCK|MODKEY_RIGHT_ALT)
	};
};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_KEYBOARD_KEYCODE_H