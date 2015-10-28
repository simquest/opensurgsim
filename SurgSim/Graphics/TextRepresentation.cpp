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

#include "SurgSim/Graphics/TextRepresentation.h"
#include "SurgSim/Graphics/Font.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/Math/MathConvert.h"
namespace SurgSim
{
namespace Graphics
{

TextRepresentation::TextRepresentation(const std::string name) : Representation(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TextRepresentation, std::shared_ptr<SurgSim::Framework::Asset>,
									  Font, getFont, setFont);

	SURGSIM_ADD_SETTER(TextRepresentation, std::string, FontFileName, loadFont);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TextRepresentation, std::string, Text, getText, setText);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TextRepresentation, double, FontSize, getFontSize, setFontSize);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TextRepresentation, SurgSim::DataStructures::OptionalValue<double>,
									  MaximumWidth, getOptionalMaximumWidth, setOptionalMaximumWidth);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SurgSim::Graphics::TextRepresentation, SurgSim::Math::Vector4d,
									  Color, getColor, setColor);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SurgSim::Graphics::TextRepresentation, bool, UseScreenSpace,
									  isUsingScreenSpace, setUseScreenSpace);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SurgSim::Graphics::TextRepresentation, bool, DrawBackground,
									  isDrawingBackground, setDrawBackground);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SurgSim::Graphics::TextRepresentation, Math::Vector4d, BackgroundColor,
									  getBackgroundColor, setBackgroundColor);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SurgSim::Graphics::TextRepresentation, double, BackgroundMargin,
									  getBackgroundMargin, setBackgroundMargin);

}

}
}

