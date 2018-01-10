/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef FAKEPOINTCLOUD_SAMPLER_HH
#define FAKEPOINTCLOUD_SAMPLER_HH

// VCG
#include <vcg/complex/complex.h>
#include <vcg/complex/allocate.h>
#include <vcg/complex/algorithms/point_sampling.h>
#include <vcg/complex/algorithms/update/position.h>

// yarp
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

class vertex;
class edge;
class face;

struct usedTypes : public vcg::UsedTypes<vcg::Use<vertex> ::AsVertexType,
					 vcg::Use<edge>   ::AsEdgeType,
					 vcg::Use<face>   ::AsFaceType> { };

class vertex : public vcg::Vertex<usedTypes,
				  vcg::vertex::Coord3f,
				  vcg::vertex::Normal3f,
				  vcg::vertex::BitFlags> {};

class face : public vcg::Face<usedTypes,
			      vcg::face::VertexRef,
			      vcg::face::Normal3f,
			      vcg::face::BitFlags> {};

class edge : public vcg::Edge<usedTypes> {};

class simpleTriMesh : public vcg::tri::TriMesh<std::vector<vertex>,
					       std::vector<face>,
					       std::vector<edge> > {};

class triMeshSampler : public MeshSampler<simpleTriMesh>
{
public:
    triMeshSampler(simpleTriMesh &m) : MeshSampler<simpleTriMesh>(m) {}
    bool qualitySampling;
};

typedef vcg::tri::SurfaceSampling<simpleTriMesh,
				  triMeshSampler> triMeshSurfSampler;
typedef vcg::Matrix44<simpleTriMesh::ScalarType> homogMatrix;

struct CloudItem
{
    yarp::sig::Vector point;
    yarp::sig::Vector normal;
    yarp::sig::PixelRgb rgb;

    CloudItem() : point(3, 0.0), normal(3, 0.0) { };
};

class FakePointCloudSampler
{    
public:
    /*
     * Constructor.
     */
    FakePointCloudSampler() : m_position(3, 0.0) { };
    
    /*
     * Load the model of the object as a triangular mesh
     * stored in a .OFF (Object File Format) file.
     * @param file_path the path of the file containing the model
     * @return true on success 
     */
    bool LoadObjectModel(const std::string &file_path);

    /*
     * Set the current pose of the object.
     * @param position yarp::sig::Vector containing the position
     * of the center of the object
     * @param attitude yarp::math::Quaternion containing the attitude
     * of the object
     */
    void SetPose(const yarp::sig::Vector &position,
		 const yarp::math::Quaternion &attitude);

    /*
     * Set the current origin of the observer.
     * @param origin yarp::sig::Vector containing the orign of the
     * observer looking at the object
     */
    void SetObserverOrigin(const yarp::sig::Vector &origin);

    /*
     * Transform the model using the current pose set.
     * @param mesh_out a simpleTriMesh containing the transformed model
     */
    void TransformModel(simpleTriMesh &transformed);

    /*
     * Sample a point cloud using Disk Poisson sampling
     * and taking into account the origin of the observer looking
     * at the object.
     * The function *tries* to return a point cloud with
     * a number of points equal to the argument n_points.
     *
     * @param n_points the desired number of points
     * @param cloud a std::vector<CloudItem> where each CloudItem
     * is filled with the position of the point and its normal
     */
    void SamplePointCloud(const int &n_points,
			  std::vector<CloudItem> &cloud);
private:
    // triangular mesh object
    simpleTriMesh m_mesh;

    // current position of the center of the object
    yarp::sig::Vector m_position;

    // current attitude of the object
    yarp::math::Quaternion m_attitude;

    // origin of the observer looking at the object
    yarp::sig::Vector m_observer;
};

#endif
