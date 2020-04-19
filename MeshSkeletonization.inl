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
 * MeshSkeletonization.inl
 *
 * Created on: 2nd of Sep 2019
 * Author: Nazim Haouchine
 */

#ifndef CGALPLUGIN_MeshSkeletonization_INL
#define CGALPLUGIN_MeshSkeletonization_INL
#include "MeshSkeletonization.h"

#include <iostream>
#include <fstream>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

using namespace sofa::core::objectmodel;

namespace cgal
{

template <class DataTypes>
MeshSkeletonization<DataTypes>::MeshSkeletonization()
    : m_inVertices(initData (&m_inVertices, "inputVertices", "List of vertices"))
    , m_inTriangles(initData(&m_inTriangles, "inputTriangles", "List of triangles"))
    , m_outPoints(initData (&m_outPoints, "outputPoints", "List of points after skeletonization") )
    , m_outLines(initData (&m_outLines, "outputLines", "List of couple of points indices representing lines: (0 1) (1 2)") )
    , m_filename(initData (&m_filename, "filename", "Input mesh in .off format") )
{
}

template <class DataTypes>
MeshSkeletonization<DataTypes>::~MeshSkeletonization()
{
}

template <class DataTypes>
void MeshSkeletonization<DataTypes>::init()
{
	//Input
	addInput(&m_inVertices);
	addInput(&m_inTriangles);

	//Output
	addOutput(&m_outPoints);
	addOutput(&m_outLines);


	setDirtyValue();

	Polyhedron tmesh;

	if (m_filename.getFullPath() != "") 
	{
		const char* filename = m_filename.getFullPath().c_str();
		std::ifstream input(filename);
		input >> tmesh;
		std::cout << "Loading Polyhedron from file." << std::endl;		
	}
	else 
	{
		geometryToPolyhedron(tmesh);
		std::cout << "Loading Polyhedron from MeshObjLoader." << std::endl;
		std::cout << "Number of vertices of the input mesh: " << boost::num_vertices(tmesh) << std::endl;
	}


	if (!CGAL::is_triangle_mesh(tmesh))
	{
		std::cout << "Input geometry is not triangulated." << std::endl;		
	}

	
	CGAL::extract_mean_curvature_flow_skeleton(tmesh, m_skeleton);
	std::cout << "Number of vertices of the output skeleton: " << boost::num_vertices(m_skeleton) << std::endl;
	std::cout << "Number of edges of the output skeleton: " << boost::num_edges(m_skeleton) << std::endl;


	//Output skeleton points and lines
        helper::WriteAccessor< Data< VecCoord > > outPoints = m_outPoints;
	std::ofstream outputP("outputPoints.obj");

	for(Skeleton_vertex v : CGAL::make_range(vertices(m_skeleton)))
	{
		outPoints.push_back(Coord(m_skeleton[v].point[0], m_skeleton[v].point[1], m_skeleton[v].point[2]));
		//outputV << "v " << m_skeleton[v].point << std::endl;
		outputP << m_skeleton[v].point << " 0 0 0 1 ";
	}
	outputP.close();

        helper::WriteAccessor< Data< helper::vector<int> > > outLines = m_outLines;
	std::ofstream outputL("outputLines.obj");
	for(Skeleton_edge e : CGAL::make_range(edges(m_skeleton)))
	{
		outLines.push_back(source(e, m_skeleton));
		outLines.push_back(target(e, m_skeleton));
		outputL << source(e, m_skeleton) << " " << target(e, m_skeleton) << " ";

		//outputL << "l "<< source(e, m_skeleton)+1 << " " << target(e, m_skeleton)+1 << std::endl;

		//const Point& s = m_skeleton[source(e, m_skeleton)].point;
		//const Point& t = m_skeleton[target(e, m_skeleton)].point;
		//outputL << source(e, m_skeleton) << " " << m_skeleton[source(e, m_skeleton)].point << " | " << target(e, m_skeleton) << " " << m_skeleton[target(e, m_skeleton)].point << std::endl;
	}
	//std::sort(outLines.begin(), outLines.end());
	outputL.close();

	std::cout << "Skeleton stored in files outputPoints.obj and outputLines.obj in Sofa build root directory" << std::endl;		
	
	// TODO: Implement an engine to convert skeleton to Rigid3d type: (x,y,z, 0,0,0,1), that can be passed to BeamFEM

	reinit();
}

template <class DataTypes>
void MeshSkeletonization<DataTypes>::reinit()
{
    update();
}

template <class DataTypes>
void MeshSkeletonization<DataTypes>::doUpdate()
{

}


template <class DataTypes>
void MeshSkeletonization<DataTypes>::geometryToPolyhedron(Polyhedron &s)
{
    //helper::ReadAccessor< Data< VecCoord > > inVertices = m_inVertices;
    //helper::ReadAccessor< Data< SeqTriangles > > inTriangles = m_inTriangles;

    VecCoord inVertices = m_inVertices.getValue();
    SeqTriangles inTriangles = m_inTriangles.getValue();

    typedef Polyhedron::HalfedgeDS HalfedgeDS;
    typedef geometryToPolyhedronOp<DataTypes, HalfedgeDS> GTSO;

    GTSO gen(inVertices, inTriangles);
    s.delegate(gen);
}


template <class DataTypes>
void MeshSkeletonization<DataTypes>::draw(const sofa::core::visual::VisualParams* vparams)
{
	std::vector<Coord> dvec;

    	const sofa::defaulttype::Vec4f& colour = sofa::defaulttype::Vec4f(0.1,0.5,0.1,1);
	for(Skeleton_edge e : CGAL::make_range(edges(m_skeleton)))
  	{
		const Point& s = m_skeleton[source(e, m_skeleton)].point;
    		const Point& t = m_skeleton[target(e, m_skeleton)].point;
   		dvec.resize(2);
		dvec[0] = Coord(s[0], s[1], s[2]);
		dvec[1] = Coord(t[0], t[1], t[2]);
		vparams->drawTool()->drawLines( dvec, 40, colour);		
		dvec.clear();
		//vparams->drawTool()->drawLine( Coord(s[0], s[1], s[2]), Coord(t[0], t[1], t[2]), colour);		
	} 	


}




} //cgal

#endif //SkeletonPLUGIN_MeshSkeletonization_INL
