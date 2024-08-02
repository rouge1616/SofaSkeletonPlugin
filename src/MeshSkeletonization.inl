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
 * CGALSkeltonization.cpp
 *
 * Last update: 15 of June 2021
 * Author: Nazim Haouchine & Sidaty El Hadramy
 */

#pragma once

#include <MeshSkeletonization.h>

#include <iostream>
#include <fstream>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

using namespace sofa::core::objectmodel;


namespace cgal {

    template <class DataTypes>
    MeshSkeletonization<DataTypes>::MeshSkeletonization()
        : m_inVertices(initData (&m_inVertices, "inputVertices", "List of vertices"))
        , m_inTriangles(initData(&m_inTriangles, "inputTriangles", "List of triangles"))
        , m_inFile(initData(&m_inFile, "inputFile", "File path"))
        , m_filename(initData (&m_filename, "filename", "Input mesh in .off format")) {
            

            addInput(&m_inVertices);
            addInput(&m_inTriangles);
            addInput(&m_inFile);
            addInput(&m_filename);
        }


    template <class DataTypes>
    void MeshSkeletonization<DataTypes>::init() {
        //Input
            if(m_inFile.getValue().empty()){
                msg_error() << "No input File to store the skeleton data, please set a inputFile path.";
                return;
            }           

            doUpdate();

    } // end init

    template<class DataTypes>
    void MeshSkeletonization<DataTypes>::reinit() {

    } // end reinit



    template <class DataTypes>
    void MeshSkeletonization<DataTypes>::geometryToPolyhedron(Polyhedron &s)
    {
        
        VecCoord inVertices = m_inVertices.getValue();
        SeqTriangles inTriangles = m_inTriangles.getValue();

        typedef Polyhedron::HalfedgeDS HalfedgeDS;
        typedef geometryToPolyhedronOp<HalfedgeDS> GTSO;

        GTSO gen(inVertices, inTriangles);
        s.delegate(gen);
    }

    template <class DataTypes>
    void MeshSkeletonization<DataTypes>::doUpdate() {

        Polyhedron tmesh;

        if(m_filename.getFullPath() != "") {
            const char* filename = m_filename.getFullPath().c_str();
            std::ifstream input(filename);
            input >> tmesh;
            std::cout << "Loading Polyhedron from file." << std::endl;
        } else {
            geometryToPolyhedron(tmesh);
            std::cout << "Loading Polyhedron from MeshObjLoader." << std::endl;
            std::cout << "Number of vertices of the input mesh: " << boost::num_vertices(tmesh) << std::endl;
        }

        if (!CGAL::is_triangle_mesh(tmesh)){
            std::cout << "Input geometry is not triangulated." << std::endl;	
            return;	
        }
        
        CGAL::extract_mean_curvature_flow_skeleton(tmesh, m_skeleton);
        std::cout << "Number of vertices of the output skeleton: " << boost::num_vertices(m_skeleton) << std::endl;
        std::cout << "Number of edges of the output skeleton: " << boost::num_edges(m_skeleton) << std::endl;

        std::ofstream OutputP(m_inFile.getValue(), std::ofstream::out | std::ofstream::trunc);
        Export_polylines extractor(m_skeleton, OutputP);
        CGAL::split_graph_into_polylines(m_skeleton, extractor);
        OutputP.close();

    } // end doUpdate

    template <class DataTypes>
    void MeshSkeletonization<DataTypes>::draw(const sofa::core::visual::VisualParams* vparams) {

            using Color = sofa::type::RGBAColor;
        	std::vector< type::Vector3 > dvec;

            for(Skeleton_edge e : CGAL::make_range(edges(m_skeleton))) {
                const Point& s = m_skeleton[source(e, m_skeleton)].point;
                const Point& t = m_skeleton[target(e, m_skeleton)].point;

                
                dvec.emplace_back(Coord(s[0], s[1], s[2]));
                dvec.emplace_back(Coord(t[0], t[1], t[2]));

        		vparams->drawTool()->drawLines( dvec, 40, Color::red());
                dvec.clear();
                
             } 
    }

} //cgal