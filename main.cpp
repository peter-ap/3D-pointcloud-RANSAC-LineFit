#include "/usr/include/pcl-1.8/pcl/visualization/cloud_viewer.h"
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "structs.h"
#include "fittingLine.h"
#include <Eigen/Dense>
#include <algorithm>
#include <random>
#include <thread>

using namespace std;
using namespace Eigen;

void viewerOneOff(pcl::visualization::PCLVisualizer &viewer, line Line)
{
    viewer.setBackgroundColor(.0, .0, .0);

    pcl::PointXYZ lineCloudBegin;
    pcl::PointXYZ lineCloudEnd;
    lineCloudBegin.x = Line.start.x;
    lineCloudBegin.y = Line.start.y;
    lineCloudBegin.z = Line.start.z;

    lineCloudEnd.x = Line.end.x;
    lineCloudEnd.y = Line.end.y;
    lineCloudEnd.z = Line.end.z;
    viewer.addLine<pcl::PointXYZ>(lineCloudBegin, lineCloudEnd, "line");
}

void viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);
}


line calculate_model_and_error(vector<Vector3d> points, int sampleSize)
{

    vector<Vector3d> sampleSet;

    unsigned num = std::chrono::system_clock::now().time_since_epoch().count();
    auto rng = std::default_random_engine{num};
    std::shuffle(std::begin(points), std::end(points), rng);

    //Create sampleSet from shuffled dataset
    for (int i = 0; i < sampleSize; i++)
    {
        sampleSet.push_back(points[i]);
    }

    //Fit line model
    line Line = line_model(sampleSet);

    cout << Line.pos_vec(0) << " " << Line.pos_vec(1) << " " << Line.pos_vec(2) << " " << endl;
    cout << Line.dir_vec(0) << " " << Line.dir_vec(1) << " " << Line.dir_vec(2) << " " << endl;

    double error = calculate_error(points, Line);
    cout << "error = " << error << endl;

    Line.points_error = error;
    return Line;
}

line ransac(vector<Vector3d> points, int iterations, double setSize)
{
    int sampleSize = points.size() * setSize;
    line est_line;
    line best_line;
    
    best_line = calculate_model_and_error(points, sampleSize);

    for (int i = 0; i < iterations -1; i++)
    {
        est_line = calculate_model_and_error(points, sampleSize);

        if (est_line.points_error < best_line.points_error)
        {
            best_line = est_line;
        }
    }
    
    //print best values found
    cout << "intersect=("<< best_line.pos_vec(0) << " " << best_line.pos_vec(1) << " " << best_line.pos_vec(2) << ") " << endl;
    cout << "direction=(" <<best_line.dir_vec(0) << " " << best_line.dir_vec(1) << " " << best_line.dir_vec(2) << ") " << endl;
    cout << "error = " << best_line.points_error << endl;

    line Line = calculate_line_segment(best_line, points);

    return Line;
}

vector<Vector3d> reshape_data(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    vector<Vector3d> points;
    for (int i = 0; i <= cloud.size(); i++)
    {
        Vector3d point(cloud[i].x, cloud[i].y, cloud[i].z);
        points.push_back(point);
    }
    return points;
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    string file_name = "cylinder_3.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1) //* load the file
    {

        return (-1);
    }

    vector<Vector3d> points = reshape_data(*cloud);
    line Line = ransac(points, 100, .25);
    // Start the visualizer
    pcl::visualization::PCLVisualizer p("test_shapes");
    p.setBackgroundColor(0, 0, 0);
    p.addCoordinateSystem(.1, "first");

    p.addPointCloud(cloud);
    pcl::PointXYZ startPoint;
    pcl::PointXYZ endPoint;
    startPoint.x = Line.start.x;
    startPoint.y = Line.start.y;
    startPoint.z = Line.start.z;

    endPoint.x = Line.end.x;
    endPoint.y = Line.end.y;
    endPoint.z = Line.end.z;

    p.addLine(startPoint, endPoint, 1.0, 1.0, 0.0);
    p.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, "line");

    p.spin();
    return 0;
}
