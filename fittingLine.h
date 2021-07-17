#include <Eigen/Dense>
#include <algorithm>
#include <random>

using namespace std;
using namespace Eigen;

line line_model(vector<Vector3d> points)
{
    Vector3d mean;
    Vector3d slope;
    Vector3d intersect;
    double slope_xy_teller = 0;
    double slope_xy_noemer = 0;
    double slope_xz_teller = 0;
    double slope_xz_noemer = 0;
    double slope_yz_teller = 0;
    double slope_yz_noemer = 0;

    for (int i = 0; i < points.size(); i++)
    {
        mean += points.at(i);
    }
    mean = mean / points.size();
    // cout << mean[0] <<" " << mean[1] << " " << mean[2] << endl;

    //Calculate SLOPE
    for (int i = 0; i < points.size(); i++)
    {
        slope_xy_teller = slope_xy_teller + ((points.at(i)[0] - mean[0]) * (points.at(i)[1] - mean[1]));
        slope_xy_noemer = slope_xy_noemer + (pow((points.at(i)[0] - mean[0]), 2));

        slope_xz_teller = slope_xz_teller + ((points.at(i)[0] - mean[0]) * (points.at(i)[2] - mean[2]));
        slope_xz_noemer = slope_xz_noemer + (pow((points.at(i)[0] - mean[0]), 2));

        slope_yz_teller = slope_yz_teller + ((points.at(i)[1] - mean[1]) * (points.at(i)[2] - mean[2]));
        slope_yz_noemer = slope_yz_noemer + (pow((points.at(i)[1] - mean[1]), 2));
    }

    slope[0] = slope_xy_teller / slope_xy_noemer;
    slope[1] = slope_xz_teller / slope_xz_noemer;
    slope[2] = slope_yz_teller / slope_yz_noemer;

    intersect[0] = mean[1] - slope[0] * mean[0]; // b = y - xy*x
    intersect[1] = mean[2] - slope[1] * mean[0]; // b = z - xz*x
    intersect[2] = mean[2] - slope[2] * mean[1]; // b = z - yz*y

    line Line;
    Line.pos_vec = intersect;
    Line.dir_vec = slope;

    return Line;
}

double calculate_error(vector<Vector3d> points, line Line)
{
    //in xy plane
    double distance_xy = 0;
    double _slope = Line.dir_vec(0);
    double _yInt = Line.pos_vec(0);

    double xPoint = 0;
    double yPoint = 0;
    for (int i = 0; i < points.size(); i++)
    {
        xPoint = points.at(i)[0];
        yPoint = points.at(i)[1];
        distance_xy += abs(_slope * xPoint + -1 * yPoint + _yInt) / sqrt(pow(_slope, 2) + pow(-1, 2));
    }
    distance_xy = distance_xy/points.size();


    //in xz plane
    double distance_xz = 0;
     _slope = Line.dir_vec(1);
    double _zInt = Line.pos_vec(1);

     xPoint = 0;
    double zPoint = 0;
    for (int i = 0; i < points.size(); i++)
    {
        xPoint = points.at(i)[0];
        zPoint = points.at(i)[2];
        distance_xz += abs(_slope * xPoint + -1 * zPoint + _zInt) / sqrt(pow(_slope, 2) + pow(-1, 2));
    }
    distance_xz = distance_xz/points.size();


    //in yz plane
    double distance_yz = 0;
     _slope = Line.dir_vec(2);
     _zInt = Line.pos_vec(2);

     yPoint = 0;
     zPoint = 0;
    for (int i = 0; i < points.size(); i++)
    {
        yPoint = points.at(i)[1];
        zPoint = points.at(i)[2];
        distance_yz += abs(_slope * yPoint + -1 * zPoint + _zInt) / sqrt(pow(_slope, 2) + pow(-1, 2));
    }
    distance_yz = distance_yz/points.size();

    return sqrt(pow(distance_xy,2) + pow(distance_xz,2) + pow(distance_yz,2));
}

line calculate_line_segment(line Line, vector<Vector3d> points)
{
    vector<double> distances;
    double distance = 0;
    for(int i = 0; i < points.size(); i++)
    {
        distance = sqrt(pow(points.at(i)[0],2)+pow(points.at(i)[1],2)+pow(points.at(i)[2],2) );
        distances.push_back(distance);
    }

    int largestValueIndex = std::distance(distances.begin(),std::max_element(distances.begin(), distances.end()));
    int smallestValueIndex = std::distance(distances.begin(),std::min_element(distances.begin(), distances.end()));

    
    double z_start = Line.pos_vec(1) + Line.dir_vec(1)*points.at(smallestValueIndex)[0];
    double y_start = (z_start - Line.pos_vec(2))/Line.dir_vec(2);
    double x_start = (y_start - Line.pos_vec(0))/Line.dir_vec(0);

    double z_end = Line.pos_vec(1) + Line.dir_vec(1)*points.at(largestValueIndex)[0];
    double y_end = (z_end - Line.pos_vec(2))/Line.dir_vec(2);
    double x_end = (y_end - Line.pos_vec(0))/Line.dir_vec(0);



    Line.start.x = x_start;
    Line.start.y = y_start;
    Line.start.z = z_start;

    Line.end.x = x_end;
    Line.end.y = y_end;
    Line.end.z = z_end;
    
    return Line;
}