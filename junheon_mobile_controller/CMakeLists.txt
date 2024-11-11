cmake_minimum_required(VERSION 3.5)
project(junheon_mobile_controller)

# Required packages
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(eigenpy REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${eigenpy_INCLUDE_DIRS}
)

# Add the main library (DifferentialWheel and MecanumWheel)
add_library(${PROJECT_NAME} SHARED
    src/DifferentialWheel.cpp
    src/MecanumWheel.cpp
    src/CasterWheel.cpp
)

# Add the Python wrapper library
add_library(${PROJECT_NAME}_wrapper_cpp SHARED
    src/bindings.cpp
)

# Link libraries to main library and Python wrapper
target_link_libraries(${PROJECT_NAME} 
    Eigen3::Eigen
)

target_link_libraries(${PROJECT_NAME}_wrapper_cpp
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    Eigen3::Eigen
    Python3::Python
    eigenpy::eigenpy
)

# Set output properties for the Python module
set_target_properties(${PROJECT_NAME}_wrapper_cpp PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/jhmc
)

# Install the include headers
install(DIRECTORY include/ DESTINATION include/)

# Install the Python package
ament_python_install_package(jhmc)

# Install the wrapper library into the Python site-packages directory
install(TARGETS ${PROJECT_NAME}_wrapper_cpp
    LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/jhmc
)

# Export package dependencies for ROS 2
ament_export_dependencies(ament_cmake Eigen3 Boost eigenpy ament_cmake_python)

ament_package()
