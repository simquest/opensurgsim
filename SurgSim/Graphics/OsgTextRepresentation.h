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

#ifndef SURGSIM_GRAPHICS_OSGTEXTREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGTEXTREPRESENTATION_H

#include <string>

#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/TextRepresentation.h"
#include "SurgSim/DataStructures/OptionalValue.h"

#include <osg/Vec3>
#include <boost/thread/mutex.hpp>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace osg
{
class Geode;
}

namespace osgText
{
class Text;
}

namespace SurgSim
{
namespace Framework
{
class Asset;
}
namespace Graphics
{
class OsgFont;

SURGSIM_STATIC_REGISTRATION(OsgTextRepresentation);

/// Osg implementation of the TextRepresentation, to be used with OsgFont assets
class OsgTextRepresentation : public OsgRepresentation, public TextRepresentation
{
public:
	/// Constructor
	/// \param name Name of this OsgInfo
	explicit OsgTextRepresentation(const std::string& name);

	/// Destructor
	~OsgTextRepresentation();

	friend class OsgTextRepresentationTests_MaximumWidth_Test;

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgTextRepresentation);

	void setLocation(double x, double y) override;
	void getLocation(double* x, double* y) const override;

	void setMaximumWidth(double width) override;
	double getMaximumWidth() override;

	void setText(const std::string& text) override;
	std::string getText() const override;

	void loadFont(const std::string& fileName) override;
	void setFont(std::shared_ptr<SurgSim::Framework::Asset> font) override;
	std::shared_ptr<Font> getFont() const override;

	void setColor(SurgSim::Math::Vector4d color) override;
	SurgSim::Math::Vector4d getColor() const override;

	void setFontSize(double size) override;
	double getFontSize() const override;

	void setUseScreenSpace(bool value) override;
	bool isUsingScreenSpace() const override;

	enum Anchor
	{
		ANCHOR_TOP_LEFT,
		ANCHOR_CENTER
	};

	void setAnchor(int anchor);
	int getAnchor() const;

	void setDrawBackground(bool value) override;
	bool isDrawingBackground() const override;

	void setBackgroundColor(Math::Vector4d color) override;
	Math::Vector4d getBackgroundColor() override;

	void setBackgroundMargin(double margin) override;
	double getBackgroundMargin() const override;

	static std::shared_ptr<OsgMaterial> getDefaultMaterial();

protected:

	void doUpdate(double dt) override;
	bool doInitialize() override;

	void setOptionalMaximumWidth(SurgSim::DataStructures::OptionalValue<double> maximum) override;
	SurgSim::DataStructures::OptionalValue<double> getOptionalMaximumWidth() override;

private:
	osg::ref_ptr<osg::Geode> m_geode; ///< node used to render text
	osg::ref_ptr<osgText::Text> m_textNode; ///< node for text display

	std::string m_text; ///< Text set by the user
	std::shared_ptr<OsgFont> m_font; ///< font used for rendering
	SurgSim::DataStructures::OptionalValue<double> m_optionalWidth; ///< information about the maximum width

	double m_characterSize; ///< the font height

	int m_drawMode; ///< the text drawmode, wether to draw a background or not

	boost::mutex m_parameterMutex; ///< protect changes of parameters
	bool m_needUpdate;	///< indicate whether parameters need to be updated

	SurgSim::Math::Vector3d m_offset;
};

}; // Graphics
}; // SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif