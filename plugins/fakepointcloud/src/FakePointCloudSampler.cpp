/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "FakePointCloudSampler.h"

// yarp
#include <yarp/os/all.h>
#include <yarp/math/Math.h>

// mesh IO
#include <wrap/io_trimesh/import_off.h>
#include <wrap/io_trimesh/export_off.h>

using namespace yarp::math;

// mesh IO
typedef vcg::tri::io::ImporterOFF<simpleTriMesh> simpleTriMeshImporter;
typedef simpleTriMeshImporter::OFFCodes OFFImportErrors;

// mesh copy
typedef vcg::tri::Append<simpleTriMesh, simpleTriMesh> copyTriMesh;

// VCG vector and matrices
typedef vcg::Matrix44<simpleTriMesh::ScalarType> vcgHomMatrix;
typedef vcg::Point3<simpleTriMesh::ScalarType> vcgVector;

// VCG transformation
typedef vcg::tri::UpdatePosition<simpleTriMesh> transformTriMesh;

// VCG face
typedef simpleTriMesh::FaceIterator FaceIterator;

// VCG vertex
typedef simpleTriMesh::VertexIterator  VertexIterator;

// VCG allocator
typedef vcg::tri::Allocator<simpleTriMesh> simpleTriMeshAllocator;

bool FakePointCloudSampler::LoadObjectModel(const std::string &file_path)
{
    int outcome;
    outcome = simpleTriMeshImporter::Open(m_mesh, file_path.c_str());

    if(outcome != OFFImportErrors::NoError) {	
	yError() << "FakePointCloudSampler::LoadObjectModel error: Error while importing .OFF file" << file_path;

	return false;
    }

    // update bounding box
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(m_mesh);

    // update face normals
    if(m_mesh.fn>0)
    	vcg::tri::UpdateNormal<simpleTriMesh>::PerFace(m_mesh);

    // update vertex normals
    if(m_mesh.vn>0)
	vcg::tri::UpdateNormal<simpleTriMesh>::PerVertex(m_mesh);        

    return true;
}
    
void FakePointCloudSampler::SetPose(const yarp::sig::Vector &position,
				    const yarp::math::Quaternion &attitude)
{
    m_position = position;
    m_attitude = attitude;
}

void FakePointCloudSampler::SetObserverOrigin(const yarp::sig::Vector &origin)
{
    m_observer = origin;
}

void FakePointCloudSampler::TransformModel(simpleTriMesh &transformed)
{
    // duplicate the original mesh
    copyTriMesh::Mesh(transformed, m_mesh);

    // update bounding box
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(transformed);

    // evaluate rototranslation matrix
    vcgHomMatrix Htransl;
    Htransl.SetTranslate(m_position[0],
			 m_position[1],
			 m_position[2]);

    // another matrix is required since SetRotateRad clear
    // the translational part
    vcgHomMatrix Hrot;

    // convert from quaternion to rotation matrix
    yarp::sig::Vector axis_angle = m_attitude.toAxisAngle();
    vcgVector axis(axis_angle[0], axis_angle[1], axis_angle[2]);
    Hrot.SetRotateRad(axis_angle[3], axis);

    // order of the matrix product is important here (do not change)
    vcgHomMatrix H = Htransl * Hrot;

    // apply rototranslation to the mesh    
    transformTriMesh::Matrix(transformed, H, true);
}

void FakePointCloudSampler::SamplePointCloud(const int &n_points,
					     PointCloud &cloud)
{
    // transform the model using the current pose set
    simpleTriMesh mesh_cp;
    TransformModel(mesh_cp);

    // perform back-face culling
    yarp::sig::Vector normal(3, 0.0);
    yarp::sig::Vector vertex0(3, 0.0);
    for (FaceIterator fi = mesh_cp.face.begin();
	 fi != mesh_cp.face.end();
	 fi++)
    {
	// extract face normal
	const auto n = fi->cN();
	normal[0] = n[0];
	normal[1] = n[1];
	normal[2] = n[2];

	// extract position of first vertex of the face
	const auto v = fi->cP(0);
	vertex0[0] = v[0];
	vertex0[1] = v[1];
	vertex0[2] = v[2];
	
	// eval vector from view point to first vertex
	yarp::sig::Vector diff = vertex0 - m_observer;
	
	// perform culling check
	if (yarp::math::dot(diff, normal) >= 0)
	    // remove face
	    simpleTriMeshAllocator::DeleteFace(mesh_cp, *fi);
    }
    // perform face garbage collection after culling
    simpleTriMeshAllocator::CompactFaceVector(mesh_cp);
    
    // perform Disk Poisson Sampling

    // since this functions returns only points visible
    // to the observer, whose origin is stored in m_observer,
    // a greater number of points, n_points_eff, is sampled
    int n_points_eff = n_points * 2 + 25;
    
    // some default parametrs as found in MeshLab
    int oversampling = 20;
    triMeshSurfSampler::PoissonDiskParam poiss_params;
    poiss_params.radiusVariance = 1;
    poiss_params.geodesicDistanceFlag=false;
    poiss_params.bestSampleChoiceFlag=true;
    poiss_params.bestSamplePoolSize=10;
    
    // estimate radius required to obtain disk poisson sampling
    // with the n_points_eff points
    simpleTriMesh::ScalarType radius;
    radius = triMeshSurfSampler::ComputePoissonDiskRadius(mesh_cp,
							  n_points_eff);

    // generate preliminar montecarlo sampling with uniform probability
    simpleTriMesh montecarlo_mesh;
    triMeshSampler mc_sampler(montecarlo_mesh);
    mc_sampler.qualitySampling=true;
    triMeshSurfSampler::Montecarlo(mesh_cp,
				   mc_sampler,
				   n_points_eff * oversampling);
    // copy the bounding box from the original mesh
    montecarlo_mesh.bbox = mesh_cp.bbox;

    // generate disk poisson samples by pruning the montecarlo cloud
    simpleTriMesh poiss_mesh;
    triMeshSampler dp_sampler(poiss_mesh);
    triMeshSurfSampler::PoissonDiskPruning(dp_sampler,
					   montecarlo_mesh,
					   radius,
					   poiss_params);
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(poiss_mesh);


    // clear the cloud
    cloud.clear();

    // store up to n_points vertices in the cloud	
    for (VertexIterator vi = poiss_mesh.vert.begin();
	 cloud.size() < n_points && vi != poiss_mesh.vert.end();
	 vi++)
    {
	// extract the point
	const auto p = vi->cP();
	yarp::sig::Vector point(3, 0.0);
	point[0] = p[0];
	point[1] = p[1];
	point[2] = p[2];

	// store it in the cloud
	PointCloudItem item;
	    
	item.x = point[0];
	item.y = point[1];
	item.z = point[2];
	    
	cloud.push_back(item);
    }
}
