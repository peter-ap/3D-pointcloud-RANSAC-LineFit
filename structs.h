#include <Eigen/Dense>
#include <algorithm>

#ifndef STRUCT_FILE_H
#define STRUCT_FILE_H

struct point
{
    double x;
    double y;
    double z;
};

struct line
{
    int ID_parent;
    int ID_child;
    Eigen::Vector3d pos_vec;
    Eigen::Vector3d dir_vec;
    double points_error;
    point start;
    point end;

    double radiusStart;
    double radiusEnd;
};

#endif