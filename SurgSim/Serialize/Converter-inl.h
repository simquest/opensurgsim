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
	/// Logger name for Serialization
	const std::string loggerName = "Serialization";

	/// Specialize of YAML::convert<> template vector3d class.
	template <>
	struct convert <SurgSim::Math::Vector3d>
	{
		static Node encode(const SurgSim::Math::Vector3d& rhs)
		{
			Node node;
			node.push_back(rhs[0]);
			node.push_back(rhs[1]);
			node.push_back(rhs[2]);
			return node;
		}

		static bool decode(const Node &node, SurgSim::Math::Vector3d& rhs)
		{
			if (! node.IsSequence() || node.size() != 3)
			{
				return false;
			}

			for (unsigned i = 0; i < node.size(); ++i)
			{
				try
				{
					rhs[i] = node[i].as<double>();
				}
				catch(YAML::RepresentationException)
				{
					rhs[i] = std::numeric_limits<SurgSim::Math::Vector3d::Scalar>::quiet_NaN();

					auto logger = SurgSim::Framework::Logger::getLogger(loggerName);
					SURGSIM_LOG(logger, WARNING) << "Bad conversion: #NaN value";
				}
			}
			return true;
		}
	};


	/// Specialize of YAML::convert<> template vector4d class.
	template <>
	struct convert <SurgSim::Math::Vector4d>
	{
		static Node encode(const SurgSim::Math::Vector4d& rhs)
		{
			Node node;
			node.push_back(rhs[0]);
			node.push_back(rhs[1]);
			node.push_back(rhs[2]);
			node.push_back(rhs[3]);
			return node;
		}

		static bool decode(const Node& node, SurgSim::Math::Vector4d& rhs) {
			if (! node.IsSequence() || node.size() != 4)
			{
				return false;
			}

			for (unsigned i = 0; i < node.size(); ++i)
			{
				try
				{
					rhs[i] = node[i].as<double>();
				}
				catch(YAML::RepresentationException)
				{
					rhs[i] = std::numeric_limits<SurgSim::Math::Vector4d::Scalar>::quiet_NaN();

					auto logger = SurgSim::Framework::Logger::getLogger(loggerName);
					SURGSIM_LOG(logger, WARNING) << "Bad conversion: #NaN value";
				}
			}
			return true;
		}
	};

	/// Specialize of YAML::convert<> template quanterniond class.
	template <>
	struct convert <SurgSim::Math::Quaterniond>
	{
		static Node encode(const SurgSim::Math::Quaterniond& rhs)
		{
			Node node;
			node = convert<SurgSim::Math::Vector4d>::encode(rhs.coeffs());
			return node;
		}

		static bool decode(const Node& node, SurgSim::Math::Quaterniond& rhs)
		{
			if (! node.IsSequence() || node.size() != 4)
			{
				return false;
			}

			SurgSim::Math::Vector4d coeffs;
			convert<SurgSim::Math::Vector4d>::decode(node, rhs.coeffs());
			return true;
		}
	};

	/// Specialize of YAML::convert<> template maxtrix33d class.
	template <>
	struct convert <SurgSim::Math::Matrix33d>
	{
		static Node encode(const SurgSim::Math::Matrix33d& rhs)
		{
			Node node;
			/// A row-major encoding
			for (auto row = 0; row < rhs.rows(); ++row)
				node.push_back(convert<SurgSim::Math::Vector3d>::encode(rhs.row(row)));
			return node;
		}

		static bool decode(const Node& node, SurgSim::Math::Matrix33d& rhs)
		{
			if (! node.IsSequence())
			{
				return false;
			}

			/// A row-major decoding
			SurgSim::Math::Vector3d vector3d = vector3d.setZero();
			for (auto row = 0; row < rhs.rows(); ++row)
			{
				convert<SurgSim::Math::Vector3d>::decode(node[row], vector3d);
				rhs.row(row) = vector3d;
			}
			return true;
		}
	};

	/// Specialize of YAML::convert<> template maxtrix44d class.
	template <>
	struct convert <SurgSim::Math::Matrix44d>
	{
		static Node encode(const SurgSim::Math::Matrix44d& rhs)
		{
			Node node;
			/// A row-major encoding
			for (auto row = 0; row < rhs.rows(); ++row)
				node.push_back(convert<SurgSim::Math::Vector4d>::encode(rhs.row(row)));
			return node;
		}

		static bool decode(const Node& node, SurgSim::Math::Matrix44d& rhs)
		{
			if (! node.IsSequence())
			{
				return false;
			}

			/// A row-major decoding
			SurgSim::Math::Vector4d vector4d = vector4d.setZero();
			for (auto row = 0; row < rhs.rows(); ++row)
			{
				convert<SurgSim::Math::Vector4d>::decode(node[row], vector4d);
				rhs.row(row) = vector4d;
			}
			return true;
		}
	};

	/// Specialize of YAML::convert<> template RigidTransform class.
	template <>
	struct convert <SurgSim::Math::RigidTransform3d>
	{
		static Node encode(const SurgSim::Math::RigidTransform3d& rhs)
		{
			Node node;
			SurgSim::Math::Matrix44d transform = rhs.matrix();
			node = convert<SurgSim::Math::Matrix44d>::encode(transform);
			return node;
		}

		static bool decode(const Node& node, SurgSim::Math::RigidTransform3d& rhs)
		{
			if (! node.IsSequence())
			{
				return false;
			}

			SurgSim::Math::Matrix44d transform;
			convert<SurgSim::Math::Matrix44d>::decode(node, transform);
			rhs.matrix() = transform;
			return true;
		}
	};


	/// Overload << for YAML::Emitter to support SurgSim::Math::Vector3d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Vector3d& rhs)
	{
		out << Flow;
		out << BeginSeq << rhs[0] << rhs[1] << rhs[2] << EndSeq;
		return out;
	}

	/// Overload << for YAML::Emitter to support SurgSim::Math::Vector4d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Vector4d& rhs)
	{
		out << Flow;
		out << BeginSeq << rhs[0] << rhs[1] << rhs[2] << rhs[3] << EndSeq;
		return out;
	}

	/// Overload << for YAML::Emitter to support SurgSim::Math::Quaterniond type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Quaterniond& rhs)
	{
		return (out << rhs.coeffs());
	}

	/// Overload << for YAML::Emitter to support SurgSim::Math::Matrix33d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Matrix33d& rhs)
	{
		out << BeginSeq;
		for (auto row = 0; row < rhs.rows(); ++row)
		{
			out << static_cast<SurgSim::Math::Vector3d>(rhs.row(row));
		}
		out << EndSeq;
		return out;
	}

	/// Overload << for YAML::Emitter to support SurgSim::Math::Matrix44d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Matrix44d& rhs)
	{
		out << BeginSeq;
		for (auto row = 0; row < rhs.rows(); ++row)
		{
			out << static_cast<SurgSim::Math::Vector4d>(rhs.row(row));
		}
		out << EndSeq;
		return out;
	}

	/// Overload << for YAML::Emitter to support SurgSim::Math::RigidTransform3d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::RigidTransform3d& rhs)
	{
		SurgSim::Math::Matrix44d transform = rhs.matrix();
		return (out << transform);
	}
}

#endif // SURGSIM_SERIALIZE_CONVERTER_INL_H