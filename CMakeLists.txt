cmake_minimum_required(VERSION 3.12)
project(MeshSkeletonizationPlugin VERSION 0.1)

# Dependencies
find_package(Sofa.Config REQUIRED)
sofa_find_package(Sofa.Core REQUIRED)

sofa_find_package(CGAL REQUIRED)
message(STATUS "CGAL VERSION = ${CGAL_VERSION}")


set(PLUGIN_SKELETONIZATION_SRC_DIR src/MeshSkeletonizationPlugin)

set(HEADER_FILES
    ${PLUGIN_SKELETONIZATION_SRC_DIR}/config.h.in
    ${PLUGIN_SKELETONIZATION_SRC_DIR}/init.h
    ${PLUGIN_SKELETONIZATION_SRC_DIR}/MeshSkeletonization.h
    ${PLUGIN_SKELETONIZATION_SRC_DIR}/MeshSkeletonization.inl
)

set(SOURCE_FILES
    ${PLUGIN_SKELETONIZATION_SRC_DIR}/init.cpp
    ${PLUGIN_SKELETONIZATION_SRC_DIR}/MeshSkeletonization.cpp
)

set(README_FILES README.md)

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

if(CGAL_VERSION VERSION_GREATER 4.9) # if CGAL >= 4.10
    target_compile_definitions(${PROJECT_NAME} PUBLIC "-DCGAL_MESH_3_VERBOSE=0")
endif()

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Core)
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


