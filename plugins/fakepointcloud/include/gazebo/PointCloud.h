/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef POINTCLOUD_H

#include <yarp/sig/Vector.h>

struct PointCloudItem
{
    double x;
    double y;
    double z;
};

struct RGBPointCloudItem
{
    double x;
    double y;
    double z;
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

typedef yarp::sig::VectorOf<PointCloudItem> PointCloud;
typedef yarp::sig::VectorOf<RGBPointCloudItem> RGBPointCloud;

#endif
