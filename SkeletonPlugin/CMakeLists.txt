cmake_minimum_required(VERSION 3.1)
project(SkeletonPlugin)

set(SkeletonPlugin_MAJOR_VERSION 0)
set(SkeletonPlugin_MINOR_VERSION 1)
set(SkeletonPlugin_VERSION ${SkeletonPlugin_MAJOR_VERSION}.${SkeletonPlugin_MINOR_VERSION})

set(HEADER_FILES
    MeshSkeletonization.h
    MeshSkeletonization.inl
)

set(SOURCE_FILES
    initSkeletonPlugin.cpp
    MeshSkeletonization.cpp
)

set(README_FILES SkeletonPlugin.txt)


### Dependencies


#find_package(SofaFramework REQUIRED)
find_package(Sofa.Helper REQUIRED)
find_package(Sofa.DefaultType REQUIRED)
find_package(Sofa.Core REQUIRED)
find_package(Sofa.Simulation.Core REQUIRED)
find_package(Sofa.Config REQUIRED)
find_package(Sofa.Type REQUIRED)
find_package(Sofa.GL REQUIRED)
find_package(image QUIET)
find_package(CGAL REQUIRED)

## CGAL dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(MPFR REQUIRED)
find_package(GMP REQUIRED)
find_package(Boost QUIET REQUIRED COMPONENTS system thread)

message(STATUS "CGAL VERSION ${CGAL_MAJOR_VERSION}.${CGAL_MINOR_VERSION}")
set(CGAL_VERSION ${CGAL_MAJOR_VERSION}.${CGAL_MINOR_VERSION})
if (CGAL_VERSION VERSION_GREATER 4.9) # if CGAL >= 4.10
    add_definitions(-DCGAL_MESH_3_VERBOSE=0)
endif()


if(WIN32)
    #add library dir as it is not added automatically on Windows (?)
    link_directories(${CGAL_LIBRARIES_DIR})
    # # Copy CGAL dlls
    # file(GLOB sharedObjects "${CGAL_INSTALL_PREFIX}/bin/*.dll")
    # foreach(sharedObject ${sharedObjects})
    #     file(COPY ${sharedObject} DESTINATION "${SOFA_BIN_DIR}")
    # endforeach()

    file(GLOB CGAL_DLLS "${CGAL_INSTALL_PREFIX}/bin/*.dll")
    # We suppose that the auxiliary DLLS for CGAL are always used from the CGAL package
    file(GLOB CGAL_AUX_DLLS "${GMP_INCLUDE_DIR}/../lib/*.dll")
    ## Copy DLLs to the build tree
    if(CMAKE_CONFIGURATION_TYPES) # Multi-config generator (MSVC)
        foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})
            file(COPY ${CGAL_DLLS} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CONFIG}")
            file(COPY ${CGAL_AUX_DLLS} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CONFIG}")
        endforeach()
    else()                      # Single-config generator (nmake)
        file(COPY ${CGAL_DLLS} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
        file(COPY ${CGAL_AUX_DLLS} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    endif()
    ## Install DLLs
    install(FILES ${DLLS} DESTINATION bin)
    install(FILES ${LIBS} DESTINATION lib)
endif()


add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} PUBLIC ${GMP_INCLUDE_DIR} ${MPFR_INCLUDE_DIR} ${CGAL_INCLUDE_DIRS})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${SkeletonPlugin_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SkeletonPlugin")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
target_link_libraries(${PROJECT_NAME} Sofa.Core Sofa.Simulation.Common ${Boost_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${CGAL_LIBRARY})


## Install rules for the library and the headers; CMake package configurations files
# sofa_create_package(SkeletonPlugin ${SkeletonPlugin_VERSION} SkeletonPlugin SkeletonPlugin)
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION "1.0"
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    EXAMPLE_INSTALL_DIR "examples"
    RELOCATABLE "plugins"
    )

## Install rules for the resources
install(DIRECTORY scenes/ DESTINATION share/sofa/plugins/${PROJECT_NAME})
