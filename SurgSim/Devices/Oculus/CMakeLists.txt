find_package(OculusSdk REQUIRED)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${OCULUSSDK_INCLUDE_DIR}"
)

set(OCULUS_DEVICE_SOURCES
	OculusDevice.cpp
	OculusScaffold.cpp
)

set(OCULUS_DEVICE_HEADERS
	OculusDevice.h
	OculusScaffold.h
)

surgsim_add_library(
	OculusDevice
	"${OCULUS_DEVICE_SOURCES}"
	"${OCULUS_DEVICE_HEADERS}"
)

set(LIBS
	SurgSimDataStructures
	SurgSimFramework 
	SurgSimInput
	${OCULUSSDK_LIBRARIES}
)

target_link_libraries(OculusDevice ${LIBS})

if(BUILD_TESTING)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()

# Put OculusDevice into folder "Devices"
set_target_properties(OculusDevice PROPERTIES FOLDER "Devices")
