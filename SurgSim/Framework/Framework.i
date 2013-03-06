%module Framework

%include "attribute.i"
%include "std_shared_ptr.i"
%include "std_map.i"
%include "std_string.i"




%include "Scene.i"
%include "SceneElement.i"

using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;

%template(SceneElementMap) std::map< std::string,std::shared_ptr<SurgSim::Framework::SceneElement> >;
