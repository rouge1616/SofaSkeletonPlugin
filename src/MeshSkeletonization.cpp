/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
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
/*
  * Author: Nazim Haouchine & Sidaty El Hadramy
 */
#define MESHSKELETONIZATION_CPP

#include <MeshSkeletonization.inl>

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

namespace meshskeletonization
{

using namespace sofa::defaulttype;

const int MeshSkeletonizationClass = sofa::core::RegisterObject("Ouput the skeleton of a triangulated mesh, from .off file or .obj (MeshLoader) using CGAL")
        .add< MeshSkeletonization<Vec3Types> >()
        ;

template class SOFA_MESHSKELETONIZATIONPLUGIN_API MeshSkeletonization<Vec3Types>;

} // namespace meshskeletonization
