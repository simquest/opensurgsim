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

#ifndef SURGSIM_PRECOMPILEDHEADER_H
#define SURGSIM_PRECOMPILEDHEADER_H

#ifndef __INTELLISENSE__
#ifdef __cplusplus

#include <algorithm>
#include <array>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <math.h>
#include <memory>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/chrono.hpp>
#include <boost/container/static_vector.hpp>
#include <boost/exception/to_string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/function.hpp>
#include <boost/functional/factory.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/ref.hpp>
#include <boost/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/random_generator.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Sparse>

#include <yaml-cpp/yaml.h>

#include <osg/ref_ptr>
#include <osg/Array>
#include <osg/FrameBufferObject>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Matrixd>
#include <osg/Matrixf>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/Notify>
#include <osg/Point>
#include <osg/PositionAttitudeTransform>
#include <osg/PrimitiveSet>
#include <osg/Program>
#include <osg/Quat>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/Switch>
#include <osg/Texture>
#include <osg/TextureCubeMap>
#include <osg/TextureRectangle>
#include <osg/Texture1D>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/TriangleIndexFunctor>
#include <osg/Uniform>
#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Vec2d>
#include <osg/Vec2f>
#include <osg/Vec3d>
#include <osg/Vec3f>
#include <osg/Vec4d>
#include <osg/Vec4f>

#include <osgGA/TrackballManipulator>

#include <osgViewer/CompositeViewer>

#endif // __cplusplus
#endif // __INTELLISENSE__

#endif  // SURGSIM_PRECOMPILEDHEADER_H
