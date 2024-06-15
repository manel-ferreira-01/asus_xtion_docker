#ifndef POINTCLOUDVIZ_HPP
#define POINTCLOUDVIZ_HPP

#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include "tools.h"
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
class PointcloudViz
{
private:
    // Device
    std::vector<openni::Device*> deviceList;
    std::vector<openni::VideoStream*> depthStreamsList;
    std::vector<openni::VideoStream*> colorStreamsList;

    // Depth Buffer
    std::vector<openni::VideoFrameRef> depth_frames;
    std::vector<openni::VideoFrameRef> color_frames;

    std::vector<cv::Mat> colorMatList;

    // Point Cloud viewer and data
    pcl::visualization::PCLVisualizer::Ptr viewer;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcloudList;

    // thread
    boost::thread* viewer_thread;
    int viewer_launched_flag = 0;

public:
    // Constructor
    PointcloudViz();

    // Destructor
    ~PointcloudViz();

    // Processing
    void run();

private:
    // Initialize
    void initialize();

    // Initialize Device
    inline void initializeDevice();

    // Initialize Depth
    inline void initializeDepth();

    // Initialize Color
    inline void initializeColor();

    // Initialize Point Cloud
    inline void initializeViewer();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Depth
    inline void updateDepth();

    // Update Color
    inline void updateColor();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Point Cloud
    inline void drawPointCloud();

    // Show Data
    void show();

    // Show Point Cloud
    inline void showPointCloud();

    //show sensor data
    void showSensorData(const openni::Array< openni::VideoMode>& modesDepth);


};

#endif // POINTCLOUDVIZ_HPP
