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

#ifndef SURGSIM_SERIALIZE_CONVERTER_INL_H
#define SURGSIM_SERIALIZE_CONVERTER_INL_H

namespace YAML
{
	/// Specialize of YAML::convert<> template vector3d class.
	template <>
	struct convert <SurgSim::Math::Vector3d> {
		static Node encode(const SurgSim::Math::Vector3d& rhs) {
			Node node;
			node.push_back(rhs[0]);
			node.push_back(rhs[1]);
			node.push_back(rhs[2]);
			return node;
		}

		static bool decode(const Node &node, SurgSim::Math::Vector3d& rhs) {
			if (! node.IsSequence() || node.size() != 3)
				return false;
			rhs[0] = node[0].as<double>();
			rhs[1] = node[1].as<double>();
			rhs[2] = node[2].as<double>();
			return true;
		}
	};


	/// Specialize of YAML::convert<> template vector4d class.
	template <>
	struct convert <SurgSim::Math::Vector4d> {
		static Node encode(const SurgSim::Math::Vector4d& rhs) {
			Node node;
			node.push_back(rhs[0]);
			node.push_back(rhs[1]);
			node.push_back(rhs[2]);
			node.push_back(rhs[3]);
			return node;
		}

		static bool decode(const Node& node, SurgSim::Math::Vector4d& rhs) {
			if (! node.IsSequence() || node.size() != 4 )
				return false;
			rhs[0] = node[0].as<double>();
			rhs[1] = node[1].as<double>();
			rhs[2] = node[2].as<double>();
			rhs[3] = node[3].as<double>();
			return true;
		}
	};

	/// Specialize of YAML::convert<> template quanterniond class.
	template <>
	struct convert <SurgSim::Math::Quaterniond> {
		static Node encode(const SurgSim::Math::Quaterniond& rhs) {

			Node node;
			node = convert<SurgSim::Math::Vector4d>::encode(rhs.coeffs());
			return node;
		}

		static bool decode(const Node& node, SurgSim::Math::Quaterniond& rhs) {

			SurgSim::Math::Vector4d coeffs;
			convert<SurgSim::Math::Vector4d>::decode(node, rhs.coeffs());
			return true;
		}
	};

	/// Specialize of YAML::convert<> template maxtrix44d class.
	template <>
	struct convert <SurgSim::Math::Matrix44d> {
		static Node encode(const SurgSim::Math::Matrix44d& rhs) {

			Node node;
			/// A row-major encoding
			for (auto row = 0; row < rhs.rows(); ++row)
				node.push_back(convert<SurgSim::Math::Vector4d>::encode(rhs.row(row)));
			return node;
		}

		static bool decode(const Node& node, SurgSim::Math::Matrix44d& rhs) {
			if (! node.IsSequence() || rhs.rows() != 4 || rhs.cols() != 4)
				return false;

			/// A row-major decoding
			for (auto row = 0; row < rhs.rows(); ++row)
			{
				SurgSim::Math::Vector4d vector4d;
				convert<SurgSim::Math::Vector4d>::decode(node[row], vector4d);
				rhs.row(row) = vector4d;
			}
			return true;
		}
	};

	/// Specialize of YAML::convert<> template RigidTransform class.
	template <>
	struct convert <SurgSim::Math::RigidTransform3d> {
		static Node encode(const SurgSim::Math::RigidTransform3d& rhs) {

			Node node;
			SurgSim::Math::Matrix44d mTransform = rhs.matrix();
			node = convert<SurgSim::Math::Matrix44d>::encode(mTransform);
			return node;
		}

		static bool decode(const Node& node, SurgSim::Math::RigidTransform3d& rhs) {

			SurgSim::Math::Matrix44d mTransform;
			convert<SurgSim::Math::Matrix44d>::decode(node, mTransform);
			rhs.matrix() = mTransform;

			return true;
		}
	};

}

#endif // SURGSIM_SERIALIZE_CONVERTER_INL_H