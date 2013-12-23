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

#ifndef SURGSIM_SERIALIZE_MATHCONVERT_INL_H
#define SURGSIM_SERIALIZE_MATHCONVERT_INL_H

SURGSIM_EMPTY_TEMPLATE_SPECIALIZATION
template <typename Type, int Rows, int MOpt>
YAML::Node YAML::convert<typename Eigen::Matrix<Type,Rows,1,MOpt>>::encode(
	const typename Eigen::Matrix<Type,Rows,1,MOpt>& rhs)
{
	Node node;
	node.SetStyle(YAML::FlowStyle);
	for (int i = 0; i < rhs.size(); ++i)
	{
		node.push_back(rhs[i]);
	}

	return node;
}

SURGSIM_EMPTY_TEMPLATE_SPECIALIZATION
template <class Type, int Rows, int MOpt>
bool YAML::convert<typename Eigen::Matrix<Type,Rows,1,MOpt>>::decode(
	const Node& node,
	typename Eigen::Matrix<Type,Rows,1,MOpt>& rhs)
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
		catch(YAML::RepresentationException)
		{
			rhs[i] = std::numeric_limits<Type>::quiet_NaN();

			auto logger = SurgSim::Framework::Logger::getLogger(SurgSim::Serialize::serializeLogger);
			SURGSIM_LOG(logger, WARNING) << "Bad conversion: #NaN value";
		}
	}
	return true;
}

SURGSIM_EMPTY_TEMPLATE_SPECIALIZATION
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

SURGSIM_EMPTY_TEMPLATE_SPECIALIZATION
template <class Type, int Rows, int Cols, int MOpt>
bool YAML::convert<typename Eigen::Matrix<Type, Rows, Cols, MOpt>>::decode(
		const YAML::Node& node,
		typename Eigen::Matrix<Type, Rows, Cols, MOpt>& rhs)
{
	if (! node.IsSequence() || node.size() != Rows)
	{
		return false;
	}

	for (size_t row = 0; row < node.size(); ++row )
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
			catch(YAML::RepresentationException)
			{
				rhs.row(row)[col] = std::numeric_limits<Type>::quiet_NaN();
				auto logger = SurgSim::Framework::Logger::getLogger(SurgSim::Serialize::serializeLogger);
				SURGSIM_LOG(logger, WARNING) << "Bad conversion: #NaN value";
			}
		}
	}
	return true;
}

SURGSIM_EMPTY_TEMPLATE_SPECIALIZATION
template <class Type, int QOpt>
YAML::Node YAML::convert<Eigen::Quaternion<Type, QOpt>>::encode(const typename Eigen::Quaternion<Type, QOpt>& rhs)
{
	return Node(convert<Eigen::Matrix<Type,4,1,QOpt>>::encode(rhs.coeffs()));
}

SURGSIM_EMPTY_TEMPLATE_SPECIALIZATION
template <class Type, int QOpt>
bool YAML::convert<Eigen::Quaternion<Type, QOpt>>::decode(const Node& node, typename Eigen::Quaternion<Type, QOpt>& rhs)
{
	bool result = false;
	if (node.IsSequence() && node.size() == 4)
	{
		result = convert<Eigen::Matrix<Type,4,1,QOpt>>::decode(node, rhs.coeffs());
	}
	return result;
}

SURGSIM_EMPTY_TEMPLATE_SPECIALIZATION
template <class Type, int Dim, int TMode, int TOptions>
YAML::Node YAML::convert<Eigen::Transform<Type, Dim, TMode, TOptions>>::encode(
	const typename Eigen::Transform<Type, Dim, TMode, TOptions>& rhs)
{
	typedef typename Eigen::Transform<Type, Dim, TMode, TOptions>::MatrixType MatrixType;
	MatrixType temporary(rhs.matrix());
	return Node(convert<MatrixType>::encode(temporary));
}

SURGSIM_EMPTY_TEMPLATE_SPECIALIZATION
template <class Type, int Dim, int TMode, int TOptions>
bool YAML::convert<Eigen::Transform<Type, Dim, TMode, TOptions>>::decode(
	const Node& node,
	typename Eigen::Transform<Type, Dim, TMode, TOptions>& rhs)
{
	bool result = false;
	if (node.IsSequence())
	{
		typedef typename Eigen::Transform<Type, Dim, TMode, TOptions>::MatrixType MatrixType;
		MatrixType temporary;
		if (convert<MatrixType>::decode(node, temporary))
		{
			rhs.matrix() = temporary;
			result = true;
		}
	}
	return result;
}

#endif