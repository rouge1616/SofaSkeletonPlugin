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

#pragma once

#include <MeshSkeletonizationPlugin/config.h>

#include <sofa/type/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/DataFileName.h>

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/visual/VisualParams.h>


#include <CGAL/Modifier_base.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>

//// Skeletonization function
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/boost/graph/split_graph_into_polylines.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>

//// Typedefs CGAL
typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
typedef Kernel::Point_3 Point;

typedef CGAL::Mean_curvature_flow_skeletonization<Polyhedron> Skeletonization;
typedef Skeletonization::Skeleton Skeleton;
typedef Skeleton::vertex_descriptor Skeleton_vertex;
typedef Skeleton::edge_descriptor Skeleton_edge;

using namespace sofa;
using namespace sofa::defaulttype;

namespace meshskeletonization
{

template <class DataTypes>
class MeshSkeletonization : public sofa::core::DataEngine 
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(MeshSkeletonization,DataTypes),sofa::core::DataEngine);

    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename Coord::value_type Real;
    typedef type::Vec<3,Real> Vec3;

    // Typedefs SOFA Topology
    using Triangle = sofa::core::topology::BaseMeshTopology::Triangle;
    using SeqTriangles = sofa::core::topology::BaseMeshTopology::SeqTriangles;

    // Inputs 
    sofa::core::objectmodel::Data<VecCoord> m_inVertices; ///< List of vertices
    sofa::core::objectmodel::Data<SeqTriangles> m_inTriangles; ///< List of triangles
    sofa::core::objectmodel::DataFileName m_outFile; ///< File path to export skeleton
        
private:
    MeshSkeletonization();
    virtual ~MeshSkeletonization() = default;

    void init() override;
    void doUpdate() override;

    void draw(const sofa::core::visual::VisualParams* vparams) override;

    // convert a set of vertices and tringles to polyhedron 
    void geometryToPolyhedron(Polyhedron &s); 

    // Parameters
    sofa::core::objectmodel::DataFileName m_filename;   

    // Members	
    Skeleton m_skeleton;


    template <class HDS>
    class geometryToPolyhedronOp : public CGAL::Modifier_base<HDS> 
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
            geometryToPolyhedronOp(const VecCoord &vertices, const SeqTriangles &triangles) {
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
                    builder.end_facet();
                }

                if (builder.check_unconnected_vertices())
                {
                    builder.remove_unconnected_vertices();
                }

                builder.end_surface();
            }
    };


    struct Export_polylines
    {
        const Skeleton& skeleton;
        std::ofstream& OutputP;
        
        Export_polylines(const Skeleton& skeleton, std::ofstream& OutputP)
            : skeleton(skeleton), OutputP(OutputP) 
        { }

        void start_new_polyline() {
                    
        }

        void add_node(Skeleton_vertex v) {
            OutputP << skeleton[v].point[0] << " " << skeleton[v].point[1] << " " << skeleton[v].point[2] << "\n";
        }

        void end_polyline() {
            OutputP << "\n";
        }
    };

};


#if !defined(MESHSKELETONIZATION_CPP)
extern template class SOFA_MESHSKELETONIZATIONPLUGIN_API MeshSkeletonization<defaulttype::Vec3Types>;
#endif

} // namespace meshskeletonization
