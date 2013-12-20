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

template <class Type, int Rows, int MOpt>
YAML::Node YAML::convert<typename Eigen::Matrix<Type,Rows,1,MOpt>>::encode(const typename Eigen::Matrix<Type,Rows,1,MOpt>& rhs)
{
	Node node;
	node.SetStyle(YAML::FlowStyle);
	for (int i = 0; i < rhs.size(); ++i)
	{
		node.push_back(rhs[i]);
	}

	return node;
}

template <class Type, int Rows, int MOpt>
bool YAML::convert<typename Eigen::Matrix<Type,Rows,1,MOpt>>::decode(const Node& node, typename Eigen::Matrix<Type,Rows,1,MOpt>& rhs)
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
			rhs[i] = std::numeric_limits<SurgSim::Math::Vector4d::Scalar>::quiet_NaN();

			auto logger = SurgSim::Framework::Logger::getLogger(SurgSim::Serialize::serializeLogger);
			SURGSIM_LOG(logger, WARNING) << "Bad conversion: #NaN value";
		}
	}
	return true;
}

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
				rhs.row(row)[col] = std::numeric_limits<SurgSim::Math::Vector4d::Scalar>::quiet_NaN();
				auto logger = SurgSim::Framework::Logger::getLogger(SurgSim::Serialize::serializeLogger);
				SURGSIM_LOG(logger, WARNING) << "Bad conversion: #NaN value";
			}
		}
	}
	return true;
}