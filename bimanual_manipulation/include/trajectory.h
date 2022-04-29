#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#define Stroke std::vector<geometry_msgs::Pose>

class Trajectory {
  public:
    DrawingInput(const std::string &, const std::string &, const char &, const std::string &, const geometry_msgs::Pose &);
    DrawingInput(const std::string &, const std::string &, const std::string &, const char &, const std::string &, const geometry_msgs::Pose &);

    std::vector<Stroke> strokes;
    std::vector<std::vector<Stroke>> strokes_by_range;

    // regarding input file
    std::string path;
    std::string file_name;
    char color;
    std::string file_extension;
    std::string file_name_full;

    // regarding drawing size
    std::vector<double> size; // width, height
    double target_size = 0.5; // target height
    double ratio;
    geometry_msgs::Pose drawing_pose;

    // regarding range splitting
    double max_range = 0.45;  // max range width
    std::vector<std::array<double, 2>> ranges;
    std::vector<double> diffs;

    // regarding wall file and data
    std::string wall_name;
    KDTree kdtree;


    // functions
    void setFileName(const std::string path, const std::string file_name, const char color, const std::string file_extension);
    void setTargetSize (const double target_size);
    void setDrawingSize (const double ratio);
    void setCanvasRange ();
    void splitByRange();

    void readDrawingFile ();

    void removeLongLine ();
    int detectRange(geometry_msgs::Pose p);
    double getDist(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
    void recenterDrawings();
    bool contains(std::vector<int> vec, const int elem);
    std::vector<std::string> split(const std::string input, const char delimiter);

    // wall related
    void setKDTree();
    void readTraj();
    std::tuple<double, point_t> getXAndQuaternion(point_t &pt);
    std::tuple<double, point_t> getXAndSurfaceNormal(point_t &pt);
    double getVectorSize(point_t &);
    point_t getCrossProduct(point_t &, point_t &);
    tf::Matrix3x3 getRotationMatrix(point_t &, double);
};