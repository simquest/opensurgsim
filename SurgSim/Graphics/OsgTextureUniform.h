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

#ifndef SURGSIM_GRAPHICS_OSGTEXTUREUNIFORM_H
#define SURGSIM_GRAPHICS_OSGTEXTUREUNIFORM_H

/// \note HS-2013-jul-07 This file is included by OsgUniform.h, it is not meant to be used on its own

namespace SurgSim
{

namespace Graphics
{

/// OSG implementation of graphics uniform with a texture value
/// \tparam	T	Texture type
template <class T>
class OsgTextureUniform : public Uniform<std::shared_ptr<T>>, public OsgUniformBase
{
public:
	/// Constructor
	/// \param	name	Name used in shader code to access this uniform
	explicit OsgTextureUniform(const std::string& name);

	/// This is the texture unit from where the search for a free texture unit will start
	/// \param unit lowest texture unit to use, default is 0
	void setMinimumTextureUnit(size_t unit);

	/// \return the value of the lowest texture unit to use for this uniform
	size_t getMinimumTextureUnit() const;

	/// Sets the value of the uniform
	/// \note using this directly might make the state of the uniform incosistent with
	///       the texture that was used to create this uniform
	virtual void set(const std::shared_ptr<T>& value);

	virtual void set(const YAML::Node& node);

	/// Returns the value of the uniform
	virtual const std::shared_ptr<T>& get() const;

	/// Adds this uniform to the OSG state set
	/// \param	stateSet	OSG state set
	virtual void addToStateSet(osg::StateSet* stateSet);

	/// Removes this uniform from the OSG state set
	/// \param	stateSet	OSG state set
	virtual void removeFromStateSet(osg::StateSet* stateSet);

private:
	/// Texture
	std::shared_ptr<T> m_texture;

	osg::ref_ptr<osg::StateSet> m_stateset;

	/// Texture unit
	ptrdiff_t m_unit;

	/// The smallest unit to be used
	size_t m_minimumTextureUnit;
};

/// Specialization of OsgUniform for OsgTexture1d
template <>
class OsgUniform<std::shared_ptr<OsgTexture1d>> : public OsgTextureUniform<OsgTexture1d>
{
public:
	/// Constructor
	/// \param	name	Name used in shader code to access this uniform
	explicit OsgUniform(const std::string& name) : OsgTextureUniform<OsgTexture1d>(name)
	{
	}
};

/// Specialization of OsgUniform for OsgTexture2d
template <>
class OsgUniform<std::shared_ptr<OsgTexture2d>> : public OsgTextureUniform<OsgTexture2d>
{
public:
	/// Constructor
	/// \param	name	Name used in shader code to access this uniform
	explicit OsgUniform(const std::string& name) : OsgTextureUniform<OsgTexture2d>(name)
	{
	}
};

/// Specialization of OsgUniform for OsgTexture3d
template <>
class OsgUniform<std::shared_ptr<OsgTexture3d>> : public OsgTextureUniform<OsgTexture3d>
{
public:
	/// Constructor
	/// \param	name	Name used in shader code to access this uniform
	explicit OsgUniform(const std::string& name) : OsgTextureUniform<OsgTexture3d>(name)
	{
	}
};

/// Specialization of OsgUniform for OsgTextureCubeMap
template <>
class OsgUniform<std::shared_ptr<OsgTextureCubeMap>> : public OsgTextureUniform<OsgTextureCubeMap>
{
public:
	/// Constructor
	/// \param	name	Name used in shader code to access this uniform
	explicit OsgUniform(const std::string& name) : OsgTextureUniform<OsgTextureCubeMap>(name)
	{
	}
};

/// Specialization of OsgUniform for OsgTextureRectangle
template <>
class OsgUniform<std::shared_ptr<OsgTextureRectangle>> : public OsgTextureUniform<OsgTextureRectangle>
{
public:
	/// Constructor
	/// \param	name	Name used in shader code to access this uniform
	explicit OsgUniform(const std::string& name) : OsgTextureUniform<OsgTextureRectangle>(name)
	{
	}
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGTEXTUREUNIFORM_H
