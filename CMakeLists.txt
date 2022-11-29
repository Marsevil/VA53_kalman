cmake_minimum_required(VERSION 3.22)
project(kalman_filter)

# Looking for libs
find_package(OpenCV REQUIRED)

# Set sources & headers
set(SRCS
	src/main.cpp
	src/Detector.cpp
	src/DetectorDebugger.cpp
	src/KalmanFilterBuilder.cpp
	)

set(HDRS
	src/Utils.hpp
	src/IDetector.hpp
	src/Detector.hpp
	src/DetectorDebugger.hpp
	src/KalmanFilterBuilder.hpp
	)

# Degine options
option(DEBUG_DETECTOR "Show image at the preprocess step" OFF)

# Pass options
if(DEBUG_DETECTOR)
	add_compile_definitions(DEBUG_DETECTOR)
	message("DEBUG_DETECTOR enabled")
endif()

# Read conan deps
if (NOT EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
	message("Execute conan install")
	execute_process(
		COMMAND "conan" "install" ${CMAKE_SOURCE_DIR} "--build=missing"
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	)
	message("Conan install ends")
endif()
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

# Define targets
add_executable(${PROJECT_NAME} ${SRCS} ${HDRS})

# Link libs
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${CONAN_LIBS})

# Include datas
add_custom_command(
	TARGET ${PROJECT_NAME}
	POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E create_symlink
	${CMAKE_CURRENT_SOURCE_DIR}/Dataset $<TARGET_FILE_DIR:${PROJECT_NAME}>/Dataset
	)
