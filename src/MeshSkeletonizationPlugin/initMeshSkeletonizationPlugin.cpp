/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <MeshSkeletonizationPlugin/config.h>

namespace meshskeletonizationplugin
{

//Here are just several convenient functions to help users know what the plugin contains 

extern "C" {
    SOFA_MESHSKELETONIZATIONPLUGIN_API void initExternalModule();
    SOFA_MESHSKELETONIZATIONPLUGIN_API const char* getModuleName();
    SOFA_MESHSKELETONIZATIONPLUGIN_API const char* getModuleVersion();
    SOFA_MESHSKELETONIZATIONPLUGIN_API const char* getModuleLicense();
    SOFA_MESHSKELETONIZATIONPLUGIN_API const char* getModuleDescription();
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

const char* getModuleName()
{
    return sofa_tostring(SOFA_TARGET);
}

const char* getModuleVersion()
{
    return sofa_tostring(MESHSKELETONIZATIONPLUGIN_VERSION);
}

const char* getModuleLicense()
{
    return "GPL";
}


const char* getModuleDescription()
{
    return "Use CGAL to generate mesh skeleton";
}

}
