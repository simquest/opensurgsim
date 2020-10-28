macro (configure_dependency LIBRARY_NAME)

# Note HS-20200109 right now this installs dependencies into the root of the
# binary directory, this might not be the right behavior but at least would
# cause a clash between two applications trying to install the same dependency 

configure_file(${LIBRARY_NAME}-CMakeLists.txt.in ${CMAKE_BINARY_DIR}/${LIBRARY_NAME}-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} .
				RESULT_VARIABLE result
				WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${LIBRARY_NAME}-download)

if(result)
  message(FATAL_ERROR "CMake step for ${LIBRARY_NAME} failed: ${result}")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} --build .
				RESULT_VARIABLE result
				WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${LIBRARY_NAME}-download )

if(result)
  message(FATAL_ERROR "Build step for ${LIBRARY_NAME} failed: ${result}")
endif()

add_subdirectory(${CMAKE_BINARY_DIR}/${LIBRARY_NAME}-src
				 ${CMAKE_BINARY_DIR}/${LIBRARY_NAME}-build
				EXCLUDE_FROM_ALL)

endmacro()