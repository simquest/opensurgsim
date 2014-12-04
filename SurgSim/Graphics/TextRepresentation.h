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

#ifndef SURGSIM_GRAPHICS_TEXTREPRESENTATION_H
#define SURGSIM_GRAPHICS_TEXTREPRESENTATION_H

#include <memory>

#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Framework
{
class Asset;
}
namespace Graphics
{

class Font;

/// A text to be displayed on the screen in screen space coordinates, use setPose() to set the position but
/// x,y are presumed to be in screen space with 0|0 being in the lower left corner, has a default font but can also
/// received a separate font.
class TextRepresentation : public virtual Representation
{
public:
	/// Constructor.
	/// \param	name	The name.
	explicit TextRepresentation(const std::string name);

	/// Destructor
	virtual ~TextRepresentation() {}

	/// Load the font with the given file name, this will overwrite the current font
	/// \param fileName the file that should be used to load the font
	virtual void loadFont(std::string fileName) = 0;

	/// Replace the current font with the one passed
	/// \param font the new font to be used
	virtual void setFont(std::shared_ptr<SurgSim::Framework::Asset> font) = 0;

	/// \return the current font
	virtual std::shared_ptr<Font> getFont() const = 0;

	/// Sets the location in screen space.
	/// \param	x,y	The x and y coordinates.
	virtual void setLocation(double x, double y) = 0;

	/// Gets the location in screen space.
	/// \param [out]	x,y	If non-null the x and y coordinates, may throw if null is passed.
	virtual void getLocation(double* x, double* y) const = 0;

	/// Sets the text to be shown on the screen.
	/// \param text the text to be used.
	virtual void setText(const std::string& text) = 0;

	/// \return the current text
	virtual std::string getText() const = 0;

	/// Sets a maximum width to the text display, the text should be broken up into
	/// multiple lines if the it is longer than width, if no value is given, or values <= 0 are used
	/// the width is assumed to be unlimited
	/// \param width the value to be used.
	virtual void setMaximumWidth(double width) = 0;

	/// \return the maximum width to be used if <=0 then the width is considered unlimited
	virtual double getMaximumWidth() = 0;

	/// Set the vertical size of the font
	/// \param size of the font
	virtual void setFontSize(double size) = 0;

	/// \return the current font size
	virtual double getFontSize() const = 0;

	/// Set the color for the text.
	/// \param color the color to be used.
	virtual void setColor(SurgSim::Math::Vector4d color) = 0;

	/// \return the current text color
	virtual SurgSim::Math::Vector4d getColor() const = 0;

protected:
	/// Optionally sets a maximum width to the text display, the text should be broken up into
	/// multiple lines if the it is longer than width, if no value is given, or values <= 0 are used
	/// the width is assumed to be unlimited
	/// \param width The width to be used, if width has no value, the maximum width is assumed to be unlimited
	virtual void setOptionalMaximumWidth(SurgSim::DataStructures::OptionalValue<double> width) = 0;

	/// Get the current status of the width
	/// \return the current maximum width, if there is no value the width is unlimited
	virtual SurgSim::DataStructures::OptionalValue<double> getOptionalMaximumWidth() = 0;
};

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_TEXTREPRESENTATION_H
