// This file is a part of the OpenSurgSim project.
// Copyright 2020, SimQuest Solutions Inc.
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

#include "DrawFunctions.h"

#include <typeinfo>

#include "imgui.h"

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/ComponentManager.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{ 
namespace EditDebug
{

// style values 
	float panelWidth = 600; // Width of the edit panel
	float poseNumWidth = 60.0; // Width of the number input for poses
	float leftColumnWidth = 200.0;

using DisplayFuncType = std::function<bool(Framework::Component*, const std::string&, const boost::any&)>;


bool labeledVec(const char* label, float width, SurgSim::Math::Vector3d* vec)
{
	ImGui::PushID(label);
	auto data = vec->data();
	bool result = false;
	ImGui::SetNextItemWidth(width);
	result = ImGui::InputDouble("##x", &data[0], 0.0, 0.0, "%.4f");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(width);
	result = ImGui::InputDouble("##y", &data[1], 0.0, 0.0, "%.4f") || result;
	ImGui::SameLine();
	ImGui::SetNextItemWidth(width);
	result = ImGui::InputDouble("##z", &data[2], 0.0, 0.0, "%.4f") || result;
	ImGui::PopID();
	return result;
}

bool typedDraw(SurgSim::Math::RigidTransform3d* pose)
{
	// Coming into this in the second column ... 
	ImGui::NextColumn();

	float width = poseNumWidth;

	ImGui::Text("Position");
	ImGui::NextColumn();
	bool changed = false;

	SurgSim::Math::Vector3d translation = pose->translation();
	changed = labeledVec("Position", width, &translation);
	ImGui::NextColumn();

	SurgSim::Math::Matrix33d rotation;
	SurgSim::Math::Matrix33d scale;

	pose->computeRotationScaling(&rotation, &scale);

	ImGui::Text("Rotation");
	ImGui::NextColumn();

	auto eulerRad = rotation.eulerAngles(0, 1, 2);
	auto rotationData = (eulerRad * (180 / M_PI)).eval();
	changed = labeledVec("Rotation", width, &rotationData) || changed;


	if (changed)
	{
		pose->translation() = translation;

		eulerRad = rotationData * (M_PI / 180);
		Eigen::AngleAxisd x(eulerRad(0), Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd y(eulerRad(1), Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd z(eulerRad(2), Eigen::Vector3d::UnitZ());
		pose->linear() = Eigen::Quaterniond(x * y * z).toRotationMatrix();
	}

	return changed;
}


bool typedDraw(int* v, int count = 1){	return ImGui::InputScalarN("", ImGuiDataType_S32,v,count);}
bool typedDraw(double* v, int count = 1) { return ImGui::InputScalarN("", ImGuiDataType_Double,v,count); }
bool typedDraw(bool* v) { return ImGui::Checkbox("", v); }
bool typedDraw(std::string* v) {
	ImGui::Text(v->c_str());
	return false;
}

bool typedDraw(SurgSim::Math::Vector3d* v)
{
	return typedDraw(v->data(), 3);
}

// Probably the vector API should allow to add and delete items 
template <class T>
bool typedDraw(std::vector<T>* v)
{
	return typedDraw(v->data(), v->size());
}

template <>
bool typedDraw(std::vector<std::string>* v)
{
	for (auto& s : *v)
	{
		typedDraw(&s);
	}
	return false;
}

template <class T, size_t Size>
bool typedDraw(std::array<T, Size>* v)
{
	return typedDraw(v->data(), Size);
}

// Templated function to convert the property into the correct type, this in turn then calls 
// the _overloaded_ typedDraw function.
template <class T>
bool convertAndSet(Framework::Component* component, const std::string& property, const boost::any& anyValue)
{
	auto value = boost::any_cast<T>(anyValue);
	auto changed = typedDraw(&value);
	if (changed)
	{
		component->setValue(property, static_cast<T>(value));
	}
	return changed;

}


// Handle editing and type conversion for a property
// Basically will convert each boost::any to the apropriate type and call the 
// correct typedDraw function
bool inputProperty(Framework::Component* component, const std::string& property)
{
	// We can customize the input for normal doubles and ints by adding parameters to the InputDouble/InputInt functions here
	static const std::unordered_map<const char*, DisplayFuncType> typeToImGui
	{
		{typeid(double).raw_name(), convertAndSet<double>},
		{typeid(int).raw_name(), convertAndSet<int>},
		{typeid(bool).raw_name(), convertAndSet<bool>},
		{typeid(Math::RigidTransform3d).raw_name(), convertAndSet<Math::RigidTransform3d>},
		{typeid(Math::Vector3d).raw_name(), convertAndSet<Math::Vector3d>},
		{typeid(std::array<int, 4>).raw_name(), convertAndSet<std::array<int, 4>>},
		{typeid(std::array<int, 2>).raw_name(), convertAndSet<std::array<int, 2>>},
		{typeid(std::vector<std::string>).raw_name(), convertAndSet<std::vector<std::string>>},
	};

	auto boostValue = component->getValue(property);
	bool handled = false;
	bool edited = false;

	const auto& type = boostValue.type();
	auto found = typeToImGui.find(type.raw_name());

	try
	{
		if (found != typeToImGui.cend())
		{
			handled = true;
			found->second(component, property, boostValue);
		}
	}
	catch (std::exception e)
	{

	}


	if (!handled)
	{
		ImGui::Text(type.name());
	}

	return edited;
}



void showElementEditor(Framework::SceneElement* element)
{
	bool active = element->isActive();
	if (ImGui::Checkbox("Active", &active)) element->setActive(active);

	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
	ImGui::Separator();
	for (const auto& component : element->getComponents())
	{
		showComponentEditor(component.get());
	}
	ImGui::Separator();
	ImGui::PopStyleVar();
}


// Draw a single property, this is mostly the layout and id handling
// the typeing and input field is done via inputProperty
void drawProperty(Framework::Component* component, const std::string& prop, int id)
{
	ImGui::PushID(id); 
	ImGui::AlignTextToFramePadding();
	ImGui::Text(prop.c_str());
	ImGui::NextColumn();
	inputProperty(component, prop);
	ImGui::NextColumn();
	ImGui::PopID();
}

void showComponentEditor(Framework::Component* component)
{
	auto properties = component->getProperties();
	std::vector<std::string> skip = { "IsLocalActive", "Pose", "LocalPose" };
	ImGui::PushID(component->getFullName().c_str()); 
	ImGui::AlignTextToFramePadding();

	convertAndSet<bool>(component, "IsLocalActive", component->getValue("IsLocalActive"));
	
	ImGui::SameLine();
	bool node_open = ImGui::TreeNodeEx(component->getName().c_str(),ImGuiTreeNodeFlags_CollapsingHeader);
	if (node_open)
	{
		ImGui::AlignTextToFramePadding();
		ImGui::Text(component->getClassName().c_str());
		ImGui::Columns(2);
		ImGui::SetColumnWidth(-1, leftColumnWidth);
		int id = 0;
		// Draw the fixed set of properties

		if (std::find(properties.cbegin(), properties.cend(), "Pose") != properties.cend()) drawProperty(component, "Pose", id++);
		if (std::find(properties.cbegin(), properties.cend(), "LocalPose") != properties.cend()) drawProperty(component, "LocalPose", id++);

		for (const auto& prop : properties)
		{
			if (std::find(skip.cbegin(), skip.cend(), prop) != skip.cend()) continue;
			drawProperty(component, prop, id++);
		}
		ImGui::Columns(1);
		//ImGui::TreePop(); no tree pop for "Collapsing Header"
	}
	ImGui::PopID();
}

// Set all but one manager to idle, this lets us pause the simulation but 
// keep running the graphics manager
void idleManagers(Framework::Runtime* runtime, bool idle, const std::string& excludes)
{
	auto managers = runtime->getManagers();
	for (auto& m : managers)
	{
		auto manager = m.lock();
		if (manager->getName() != excludes) manager->setIdle(idle);
	}
}


void showSceneEditor(Framework::Runtime* runtime, int width, int height)
{
	ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(panelWidth, static_cast<float>(height)), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSizeConstraints(ImVec2(0, static_cast<float>(height)), 
		ImVec2(static_cast<float>(width), static_cast<float>(height)));

	auto scene = runtime->getScene();

	static auto open = true;
	static auto paused = false;
	if (ImGui::Begin("Scene", &open, ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoMove))
	{
		if (!paused)
		{
			if (ImGui::Button("Pause"))
			{
				idleManagers(runtime, true, "Graphics Manager");
				paused = true;
			}
		}
		else
		{
			if (ImGui::Button("Resume")) 
			{
				idleManagers(runtime, false, "Graphics Manager");
				paused = false;
			};
		}

		// left
		static int selected = 0;
		Framework::SceneElement* selectedElement = scene->getSceneElements()[selected].get();
		ImGui::BeginChild("left pane", ImVec2(150, 0), true);
		int index = 0;
		for (const auto& element : scene->getSceneElements())
		{
			if (ImGui::Selectable(element->getName().c_str(), index++ == selected))
			{
				selectedElement = element.get();
				selected = index - 1;
			}
		}
		ImGui::EndChild();
		ImGui::SameLine();

		// right
		ImGui::BeginGroup();
		ImGui::BeginChild("item view", ImVec2(0, -ImGui::GetFrameHeightWithSpacing())); // Leave room for 1 line below us
		ImGui::Text(selectedElement->getName().c_str());
		ImGui::Separator();
		if (selectedElement) showElementEditor(selectedElement);
		ImGui::EndChild();
		ImGui::EndGroup();
	}
	ImGui::End();
}

}
}

