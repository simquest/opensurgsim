%module SceneElement

%{

#include <SurgSim/Framework/SceneElement.h>

%}


%include <std_shared_ptr.i>
%shared_ptr(SurgSim::Framework::SceneElement)

%include "SurgSim/Framework/SceneElement.h"
