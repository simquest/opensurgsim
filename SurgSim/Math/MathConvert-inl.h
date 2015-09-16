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

#ifndef SURGSIM_MATH_MATHCONVERT_INL_H
#define SURGSIM_MATH_MATHCONVERT_INL_H

#include <string>

#include "SurgSim/Framework/Log.h"

namespace
{
const std::string rotationPropertyName = "Quaternion";
const std::string translationPropertyName = "Translation";
const std::string serializeLogger = "Serialization";
};

SURGSIM_DOUBLE_SPECIALIZATION
template <typename Type, int Rows, int MOpt>
YAML::Node YAML::convert<typename Eigen::Matrix<Type, Rows, 1, MOpt>>::encode(
	const typename Eigen::Matrix<Type, Rows, 1, MOpt>& rhs)
{
	Node node;
	node.SetStyle(YAML::FlowStyle);
	for (int i = 0; i < rhs.size(); ++i)
	{
		node.push_back(rhs[i]);
	}

	return node;
}

SURGSIM_DOUBLE_SPECIALIZATION
template <class Type, int Rows, int MOpt>
bool YAML::convert<typename Eigen::Matrix<Type, Rows, 1, MOpt>>::decode(
	const Node& node, typename Eigen::Matrix<Type, Rows, 1, MOpt>& rhs) //NOLINT
{
	if (! node.IsSequence() || node.size() != Rows)
	{
		return false;
	}

	for (unsigned i = 0; i < node.size(); ++i)
	{
		try
		{
			rhs[i] = node[i].as<Type>();
		}
		catch (YAML::RepresentationException)
		{
			rhs[i] = std::numeric_limits<Type>::quiet_NaN();

			auto logger = SurgSim::Framework::Logger::getLogger(serializeLogger);
			SURGSIM_LOG(logger, WARNING) << "Bad conversion: #NaN value";
		}
	}
	return true;
}

SURGSIM_DOUBLE_SPECIALIZATION
template <class Type, int Rows, int Cols, int MOpt>
YAML::Node YAML::convert<typename Eigen::Matrix<Type, Rows, Cols, MOpt>>::encode(
	const typename Eigen::Matrix<Type, Rows, Cols, MOpt>& rhs)
{
	YAML::Node node;
	node.SetStyle(YAML::FlowStyle);
	for (int row = 0; row < Rows; ++row)
	{
		YAML::Node rowNode;
		for (int col = 0; col < Cols; ++col)
		{
			rowNode.push_back(rhs.row(row)[col]);
		}
		node.push_back(rowNode);
	}
	return node;
}

SURGSIM_DOUBLE_SPECIALIZATION
template <class Type, int Rows, int Cols, int MOpt>
bool YAML::convert<typename Eigen::Matrix<Type, Rows, Cols, MOpt>>::decode(
			const Node& node,
			typename Eigen::Matrix<Type, Rows, Cols, MOpt>& rhs) //NOLINT
{
	if (! node.IsSequence() || node.size() != Rows)
	{
		return false;
	}

	for (size_t row = 0; row < node.size(); ++row)
	{
		YAML::Node rowNode = node[row];
		if (!rowNode.IsSequence() || node.size() != Cols)
		{
			return false;
		}
		for (size_t col = 0; col < rowNode.size(); ++col)
		{
			try
			{
				rhs.row(row)[col] = rowNode[col].as<Type>();
			}
			catch (YAML::RepresentationException)
			{
				rhs.row(row)[col] = std::numeric_limits<Type>::quiet_NaN();
				auto logger = SurgSim::Framework::Logger::getLogger(serializeLogger);
				SURGSIM_LOG(logger, WARNING) << "Bad conversion: #NaN value";
			}
		}
	}
	return true;
}

SURGSIM_DOUBLE_SPECIALIZATION
template <class Type, int QOpt>
YAML::Node YAML::convert<typename Eigen::Quaternion<Type, QOpt>>::encode(
	const typename Eigen::Quaternion<Type, QOpt>& rhs)
{
	return Node(convert<typename Eigen::Matrix<Type, 4, 1, QOpt>>::encode(rhs.coeffs()));
}

SURGSIM_DOUBLE_SPECIALIZATION
template <class Type, int QOpt>
bool YAML::convert<typename Eigen::Quaternion<Type, QOpt>>::decode(
	const Node& node,
	typename Eigen::Quaternion<Type, QOpt>& rhs) //NOLINT
{
	bool result = false;
	if (node.IsSequence() && node.size() == 4)
	{
		result = convert<typename Eigen::Matrix<Type, 4, 1, QOpt>>::decode(node, rhs.coeffs());
	}
	return result;
}

SURGSIM_DOUBLE_SPECIALIZATION
template <class Type, int Dim, int TMode, int TOptions>
YAML::Node YAML::convert<typename Eigen::Transform<Type, Dim, TMode, TOptions>>::encode(
	const typename Eigen::Transform<Type, Dim, TMode, TOptions>& rhs)
{
	typedef typename Eigen::Transform<Type, Dim, TMode, TOptions>::LinearMatrixType LinearMatrixType;
	LinearMatrixType linear(rhs.linear());
	Eigen::Quaternion<Type, TOptions> quaternion(linear);
	Eigen::Matrix<Type, Dim, 1, TOptions> translation(rhs.translation());

	Node node;
	node[rotationPropertyName] = quaternion;
	node[translationPropertyName] = translation;
	return node;
}

SURGSIM_DOUBLE_SPECIALIZATION
template <class Type, int Dim, int TMode, int TOptions>
bool YAML::convert<typename Eigen::Transform<Type, Dim, TMode, TOptions>>::decode(
			const Node& node,
			typename Eigen::Transform<Type, Dim, TMode, TOptions>& rhs) //NOLINT
{
	bool result = false;


	if (node.IsMap())
	{
		Eigen::Quaternion<Type, TOptions> rotation(Eigen::Quaternion<Type, TOptions>::Identity());
		Eigen::Matrix<Type, Dim, 1, TOptions> translation(Eigen::Matrix<Type, Dim, 1, TOptions>::Zero());
		if (node[rotationPropertyName].IsDefined())
		{
			rotation = node[rotationPropertyName].as<Eigen::Quaternion<Type, TOptions>>();
			result = true;
		}
		if (node[translationPropertyName].IsDefined())
		{
			translation = node[translationPropertyName].as<Eigen::Matrix<Type, Dim, 1, TOptions>>();
			result = true;
		}
		rhs.makeAffine();
		rhs.linear() = rotation.matrix();
		rhs.translation() = translation;
	}
	return result;
}

SURGSIM_DOUBLE_SPECIALIZATION
template <class Type>
YAML::Node YAML::convert<typename Eigen::AngleAxis<Type>>::encode(
	const typename Eigen::AngleAxis<Type>& rhs)
{
	YAML::Node node;
	node.SetStyle(YAML::FlowStyle);
	node.push_back(rhs.angle());
	node.push_back(rhs.axis()[0]);
	node.push_back(rhs.axis()[1]);
	node.push_back(rhs.axis()[2]);
	return node;
}

SURGSIM_DOUBLE_SPECIALIZATION
template <class Type>
bool YAML::convert<typename Eigen::AngleAxis<Type>>::decode(
	const Node& node,
	typename Eigen::AngleAxis<Type>& rhs) //NOLINT
{
	if (node.size() != 4)
	{
		return false;
	}
	try
	{
		rhs.angle() = node[0].as<Type>();
	}
	catch (YAML::RepresentationException)
	{
		rhs.angle() = std::numeric_limits<Type>::quiet_NaN();
		auto logger = SurgSim::Framework::Logger::getLogger(serializeLogger);
		SURGSIM_LOG(logger, WARNING) << "Bad conversion: #NaN value";
	}
	for (size_t i = 0; i < 3; ++i)
	{
		try
		{
			rhs.axis()[i] = node[i + 1].as<Type>();
		}
		catch (YAML::RepresentationException)
		{
			rhs.axis()[i] = std::numeric_limits<Type>::quiet_NaN();
			auto logger = SurgSim::Framework::Logger::getLogger(serializeLogger);
			SURGSIM_LOG(logger, WARNING) << "Bad conversion: #NaN value";
		}
	}
	return true;
}

#endif // SURGSIM_MATH_MATHCONVERT_INL_H
