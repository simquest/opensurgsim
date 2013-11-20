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

#ifndef SURGSIM_DEVICES_KEYBOARD_KEYBOARDCODE_H
#define SURGSIM_DEVICES_KEYBOARD_KEYBOARDCODE_H

namespace SurgSim
{
namespace Device
{
	enum KeyCode
	{
		KEY_SPACE           = 0X20,
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
		KEY_A               = 'A',
		KEY_B               = 'B',
		KEY_C               = 'C',
		KEY_D               = 'D',
		KEY_E               = 'E',
		KEY_F               = 'F',
		KEY_G               = 'G',
		KEY_H               = 'H',
		KEY_I               = 'I',
		KEY_J               = 'J',
		KEY_K               = 'K',
		KEY_L               = 'L',
		KEY_M               = 'M',
		KEY_N               = 'N',
		KEY_O               = 'O',
		KEY_P               = 'P',
		KEY_Q               = 'Q',
		KEY_R               = 'R',
		KEY_S               = 'S',
		KEY_T               = 'T',
		KEY_U               = 'U',
		KEY_V               = 'V',
		KEY_W               = 'W',
		KEY_X               = 'X',
		KEY_Y               = 'Y',
		KEY_Z               = 'Z',
		KEY_EXCLAIM         = 0X21,
		KEY_QUOTEDBL        = 0X22,
		KEY_HASH            = 0X23,
		KEY_DOLLAR          = 0X24,
		KEY_AMPERSAND       = 0X26,
		KEY_QUOTE           = 0X27,
		KEY_LEFTPAREN       = 0X28,
		KEY_RIGHTPAREN      = 0X29,
		KEY_ASTERISK        = 0X2A,
		KEY_PLUS            = 0X2B,
		KEY_COMMA           = 0X2C,
		KEY_MINUS           = 0X2D,
		KEY_PERIOD          = 0X2E,
		KEY_SLASH           = 0X2F,
		KEY_COLON           = 0X3A,
		KEY_SEMICOLON       = 0X3B,
		KEY_LESS            = 0X3C,
		KEY_EQUALS          = 0X3D,
		KEY_GREATER         = 0X3E,
		KEY_QUESTION        = 0X3F,
		KEY_AT              = 0X40,
		KEY_LEFTBRACKET     = 0X5B,
		KEY_BACKSLASH       = 0X5C,
		KEY_RIGHTBRACKET    = 0X5D,
		KEY_CARET           = 0X5E,
		KEY_UNDERSCORE      = 0X5F,
		KEY_BACKQUOTE       = 0X60,
		KEY_BACKSPACE       = 0XFF08,        /* back space, back char */
		KEY_TAB             = 0XFF09,
		KEY_LINEFEED        = 0XFF0A,        /* Linefeed, LF */
		KEY_CLEAR           = 0XFF0B,
		KEY_RETURN          = 0XFF0D,        /* Return, enter */
		KEY_PAUSE           = 0XFF13,        /* Pause, hold */
		KEY_SCROLL_LOCK     = 0XFF14,
		KEY_SYS_REQ         = 0XFF15,
		KEY_ESCAPE          = 0XFF1B,
		KEY_DELETE          = 0XFFFF,        /* Delete, rubout */
		KEY_HOME            = 0XFF50,
		KEY_LEFT            = 0XFF51,        /* Move left, left arrow */
		KEY_UP              = 0XFF52,        /* Move up, up arrow */
		KEY_RIGHT           = 0XFF53,        /* Move right, right arrow */
		KEY_DOWN            = 0XFF54,        /* Move down, down arrow */
		KEY_PRIOR           = 0XFF55,        /* Prior, previous */
		KEY_PAGE_UP         = 0XFF55,
		KEY_NEXT            = 0XFF56,        /* Next */
		KEY_PAGE_DOWN       = 0XFF56,
		KEY_END             = 0XFF57,        /* EOL */
		KEY_BEGIN           = 0XFF58,        /* BOL */
		KEY_SELECT          = 0XFF60,        /* Select, mark */
		KEY_PRINT           = 0XFF61,
		KEY_EXECUTE         = 0XFF62,        /* Execute, run, do */
		KEY_INSERT          = 0XFF63,        /* Insert, insert here */
		KEY_UNDO            = 0XFF65,        /* Undo, oops */
		KEY_REDO            = 0XFF66,        /* redo, again */
		KEY_MENU            = 0XFF67,        /* On Windows, this is VK_APPS, the context-menu key */
		KEY_FIND            = 0XFF68,        /* Find, search */
		KEY_CANCEL          = 0XFF69,        /* Cancel, stop, abort, exit */
		KEY_HELP            = 0XFF6A,        /* Help */
		KEY_BREAK           = 0XFF6B,
		KEY_MODE_SWITCH     = 0XFF7E,        /* Character set switch */
		KEY_SCRIPT_SWITCH   = 0XFF7E,        /* Alias for mode_switch */
		KEY_NUM_LOCK        = 0XFF7F,
		KEY_KP_SPACE        = 0XFF80,        /* space */
		KEY_KP_TAB          = 0XFF89,
		KEY_KP_ENTER        = 0XFF8D,        /* enter */
		KEY_KP_F1           = 0XFF91,        /* PF1, KP_A, ... */
		KEY_KP_F2           = 0XFF92,
		KEY_KP_F3           = 0XFF93,
		KEY_KP_F4           = 0XFF94,
		KEY_KP_HOME         = 0XFF95,
		KEY_KP_LEFT         = 0XFF96,
		KEY_KP_UP           = 0XFF97,
		KEY_KP_RIGHT        = 0XFF98,
		KEY_KP_DOWN         = 0XFF99,
		KEY_KP_PRIOR        = 0XFF9A,
		KEY_KP_PAGE_UP      = 0XFF9A,
		KEY_KP_NEXT         = 0XFF9B,
		KEY_KP_PAGE_DOWN    = 0XFF9B,
		KEY_KP_END          = 0XFF9C,
		KEY_KP_BEGIN        = 0XFF9D,
		KEY_KP_INSERT       = 0XFF9E,
		KEY_KP_DELETE       = 0XFF9F,
		KEY_KP_EQUAL        = 0XFFBD,        /* equals */
		KEY_KP_MULTIPLY     = 0XFFAA,
		KEY_KP_ADD          = 0XFFAB,
		KEY_KP_SEPARATOR    = 0XFFAC,       /* separator, often comma */
		KEY_KP_SUBTRACT     = 0XFFAD,
		KEY_KP_DECIMAL      = 0XFFAE,
		KEY_KP_DIVIDE       = 0XFFAF,
		KEY_KP_0            = 0XFFB0,
		KEY_KP_1            = 0XFFB1,
		KEY_KP_2            = 0XFFB2,
		KEY_KP_3            = 0XFFB3,
		KEY_KP_4            = 0XFFB4,
		KEY_KP_5            = 0XFFB5,
		KEY_KP_6            = 0XFFB6,
		KEY_KP_7            = 0XFFB7,
		KEY_KP_8            = 0XFFB8,
		KEY_KP_9            = 0XFFB9,
		KEY_F1              = 0XFFBE,
		KEY_F2              = 0XFFBF,
		KEY_F3              = 0XFFC0,
		KEY_F4              = 0XFFC1,
		KEY_F5              = 0XFFC2,
		KEY_F6              = 0XFFC3,
		KEY_F7              = 0XFFC4,
		KEY_F8              = 0XFFC5,
		KEY_F9              = 0XFFC6,
		KEY_F10             = 0XFFC7,
		KEY_F11             = 0XFFC8,
		KEY_F12             = 0XFFC9,
		KEY_F13             = 0XFFCA,
		KEY_F14             = 0XFFCB,
		KEY_F15             = 0XFFCC,
		KEY_F16             = 0XFFCD,
		KEY_F17             = 0XFFCE,
		KEY_F18             = 0XFFCF,
		KEY_F19             = 0XFFD0,
		KEY_F20             = 0XFFD1,
		KEY_F21             = 0XFFD2,
		KEY_F22             = 0XFFD3,
		KEY_F23             = 0XFFD4,
		KEY_F24             = 0XFFD5,
		KEY_F25             = 0XFFD6,
		KEY_F26             = 0XFFD7,
		KEY_F27             = 0XFFD8,
		KEY_F28             = 0XFFD9,
		KEY_F29             = 0XFFDA,
		KEY_F30             = 0XFFDB,
		KEY_F31             = 0XFFDC,
		KEY_F32             = 0XFFDD,
		KEY_F33             = 0XFFDE,
		KEY_F34             = 0XFFDF,
		KEY_F35             = 0XFFE0,
		KEY_SHIFT_L         = 0XFFE1,        /* Left shift */
		KEY_SHIFT_R         = 0XFFE2,        /* Right shift */
		KEY_CONTROL_L       = 0XFFE3,        /* Left control */
		KEY_CONTROL_R       = 0XFFE4,        /* Right control */
		KEY_CAPS_LOCK       = 0XFFE5,        /* Caps lock */
		KEY_SHIFT_LOCK      = 0XFFE6,        /* Shift lock */
		KEY_META_L          = 0XFFE7,        /* Left meta */
		KEY_META_R          = 0XFFE8,        /* Right meta */
		KEY_ALT_L           = 0XFFE9,        /* Left alt */
		KEY_ALT_R           = 0XFFEA,        /* Right alt */
		KEY_SUPER_L         = 0XFFEB,        /* Left super */
		KEY_SUPER_R         = 0XFFEC,        /* Right super */
		KEY_HYPER_L         = 0XFFED,        /* Left hyper */
		KEY_HYPER_R         = 0XFFEE         /* Right hyper */
	};
};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_KEYBOARD_KEYBOARDCODE_H