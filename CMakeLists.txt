cmake_minimum_required(VERSION 3.12)
project(MeshSkeletonizationPlugin VERSION 0.1)

set(PLUGIN_Skeletonization_SRC_DIR src/)
set(HEADER_FILES
    ${PLUGIN_Skeletonization_SRC_DIR}/config.h.in
    ${PLUGIN_Skeletonization_SRC_DIR}/MeshSkeletonization.h
    ${PLUGIN_Skeletonization_SRC_DIR}/MeshSkeletonization.inl
    )
set(SOURCE_FILES
    ${PLUGIN_Skeletonization_SRC_DIR}/initMeshSkeletonizationPlugin.cpp
    ${PLUGIN_Skeletonization_SRC_DIR}/MeshSkeletonization.cpp
    )
set(README_FILES MeshSkeletonizationPlugin.txt)

# Dependencies
find_package(SofaFramework REQUIRED)
find_package(SofaSimulationCommon REQUIRED)
find_package(CGAL REQUIRED)
message(STATUS "CGAL VERSION = ${CGAL_VERSION}")

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_MeshSkeletonizationPlugin")
if(CGAL_VERSION VERSION_GREATER 4.9) # if CGAL >= 4.10
    target_compile_definitions(${PROJECT_NAME} PUBLIC "-DCGAL_MESH_3_VERBOSE=0")
endif()

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} PUBLIC SofaCore SofaSimulationCommon)
target_link_libraries(${PROJECT_NAME} PUBLIC CGAL::CGAL)



# Install rules for the library and the headers; CMake package configurations files
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "MeshSkeletonizationPlugin"
    RELOCATABLE "plugins"
    )


