%module Scene
%{

#include "SurgSim/Framework/Scene.h"

%}

%include <std_shared_ptr.i>
%shared_ptr(SurgSim::Framework::Scene)

%include "SurgSim/Framework/Scene.h"
