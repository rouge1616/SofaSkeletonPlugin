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
 * MeshSkeletonization.h
 *
 *  Created on: 2nd of June 2010
 *      Author: Olivier
 */

#ifndef SkeletonPlugin_MeshSkeletonization_H
#define SkeletonPlugin_MeshSkeletonization_H

#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/helper/gl/template.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/DataFileName.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>

// Adaptor for Polyhedron_3
#include <CGAL/Surface_mesh_simplification/HalfedgeGraph_Polyhedron_3.h>

// Skeletonization function
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/boost/graph/split_graph_into_polylines.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>

// Typedefs SOFA
typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
typedef sofa::core::topology::BaseMeshTopology::SeqEdges SeqEdges;
typedef sofa::core::topology::BaseMeshTopology::SeqTriangles SeqTriangles;


// Typedefs CGAL
typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
typedef Kernel::Point_3 Point;



typedef boost::graph_traits<Polyhedron>::vertex_descriptor vertex_descriptor;
typedef CGAL::Mean_curvature_flow_skeletonization<Polyhedron> Skeletonization;
typedef Skeletonization::Skeleton Skeleton;
typedef Skeleton::vertex_descriptor Skeleton_vertex;
typedef Skeleton::edge_descriptor Skeleton_edge;


using namespace sofa;
using namespace sofa::defaulttype;

namespace cgal
{

template <class DataTypes>
class MeshSkeletonization : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(MeshSkeletonization,DataTypes),sofa::core::DataEngine);

//        typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename Coord::value_type Real;
    typedef Vec<3,Real> Vec3;

public:
    MeshSkeletonization();
    virtual ~MeshSkeletonization();

    void init();
    void reinit();

    void doUpdate();
    void draw(const sofa::core::visual::VisualParams* vparams);

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const MeshSkeletonization<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }


    void geometryToPolyhedron(Polyhedron &s); //convert a set of vertices and triangles to CGAL Polyhedron


    //Inputs
    sofa::core::objectmodel::Data<VecCoord> m_inVertices; ///< List of vertices
    sofa::core::objectmodel::Data<SeqTriangles> m_inTriangles; ///< List of triangles

    // Outputs
    sofa::core::objectmodel::Data<VecCoord> m_outPoints;
    sofa::core::objectmodel::Data<helper::vector<int> > m_outLines;

    // Parameters
    sofa::core::objectmodel::DataFileName m_filename;    

    // Members	
    Skeleton m_skeleton;
};


template <class DataTypes, class HDS>
class geometryToPolyhedronOp :  public CGAL::Modifier_base<HDS>
{
public:

    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;

    typedef HDS Halfedge_data_structure;

private:

    VecCoord m_vertices;
    SeqTriangles m_triangles;


public:

    geometryToPolyhedronOp(const VecCoord &vertices, const SeqTriangles &triangles)
    //geometryToPolyhedronOp(helper::ReadAccessor vertices, helper::ReadAccessor triangles)
    {
        m_vertices = vertices;
        m_triangles = triangles;
    }

    void operator()( HDS& hds)
    {
        unsigned int numVertices = m_vertices.size();
        unsigned int numTriangles = m_triangles.size();

        CGAL::Polyhedron_incremental_builder_3<HalfedgeDS> builder(hds, true);
        builder.begin_surface(numVertices, numTriangles);

        for (unsigned int i = 0; i < numVertices; i++)
        {
            builder.add_vertex( Point( m_vertices[i][0], m_vertices[i][1], m_vertices[i][2] ));
        }

        for (unsigned int i = 0; i < numTriangles; i++ )
        {
            builder.begin_facet();
            for ( int j = 0; j < 3; j++ )
            {
                builder.add_vertex_to_facet( m_triangles[i][j] );
            }
            std::cout << std::endl;
            builder.end_facet();
        }

        if (builder.check_unconnected_vertices())
        {
            builder.remove_unconnected_vertices();
        }

        builder.end_surface();
    }

};


#if  !defined(SkeletonPlugin_SIMPLIFICATIONMESH_CPP)
extern template class SOFA_SkeletonPlugin_API MeshSkeletonization<defaulttype::Vec3Types>;
 
#endif

} //cgal

#endif /* SkeletonPlugin_MeshSkeletonization_H */
