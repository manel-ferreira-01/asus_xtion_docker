#include "pointcloudviz.h"


#define DISPLAY_POINTCLOUD true
#define DISPLAY_COLOR false

// Constructor
PointcloudViz::PointcloudViz()
{
    // Initialize
    initialize();
}

// Destructor
PointcloudViz::~PointcloudViz()
{

    //terminate viewer thread
    viewer_thread->interrupt();
    viewer_thread->join();
    

    cv::destroyAllWindows();

    openni::OpenNI::shutdown();


}

// Processing
void PointcloudViz::run()
{
    // Main Loop
    while( true ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        show();

        // Key Check
        const int32_t key = cv::waitKey( 10 );
        if( key == 'q' ){
            break;
        }

        if( key == 's' ){
            // Save a png of all images from colormatlist
            for (int i = 0; i < colorMatList.size(); i++)
            {
                //make the filename have its date and time
                time_t now = time(0);
                tm *ltm = localtime(&now);

                std::string filename = "color" + std::to_string(i) + "_" + std::to_string(1900 + ltm->tm_year) + 
                 "_" + std::to_string(1 + ltm->tm_mon) + "_" + std::to_string(ltm->tm_mday) + 
                  "_" + std::to_string(ltm->tm_hour) + "_" + std::to_string(ltm->tm_min) + 
                   "_" + std::to_string(ltm->tm_sec) + ".png";

                cv::imwrite(filename, colorMatList[i]);

            }

            // save pcloudList as various pcd files
            for (int i = 0; i < pcloudList.size(); i++)
            {
                //pcl::io::savePCDFileASCII("pointcloud" + std::to_string(i) + ".pcd", *pcloudList[i]);
                //std::cout << "Saved " << pcloudList[i]->points.size() << " data points to " << std::endl;            
            }

        }
    }
}


// Initialize
void PointcloudViz::initialize()
{
    cv::setUseOptimized( true );

    // Initialize OpenNI2
    OPENNI_CHECK( openni::OpenNI::initialize() );

    // Initialize Device
    initializeDevice();

    // Initialize Color
    initializeColor();

    // Initialize Depth
    initializeDepth();
    
    //initialize transforms
    initializeTransforms();


    // Initialize Point Cloud
    if (DISPLAY_POINTCLOUD) {
        viewer_thread = new boost::thread(boost::bind(&PointcloudViz::initializeViewer, this));
    }

    printf("innited all\n");

    depth_frames.resize(deviceList.size());
    color_frames.resize(deviceList.size());

}

// Initialize Device
inline void PointcloudViz::initializeDevice()
{   

    // check what devices are available
    openni::Array<openni::DeviceInfo> deviceListInfo;
    openni::OpenNI::enumerateDevices(&deviceListInfo);
    
    //print devices
    for (int i = 0; i < deviceListInfo.getSize(); i++)
    {
        printf("Device %i: %s\n", i, deviceListInfo[i].getName());
    }
    

    // Open all devices
    for (int i = 0; i < deviceListInfo.getSize(); i++)
    {
        openni::Device* device = new openni::Device;
        OPENNI_CHECK(device->open(deviceListInfo[i].getUri()));
        //OPENNI_CHECK(device->open(openni::ANY_DEVICE));
        deviceList.push_back(device);
    }

    //no device found and leave
    if (deviceList.size() == 0)
    {
        printf("No device found\n");
        exit(1);
    }

    printf("opened all devices\n");
}

// Initialize Depth
inline void PointcloudViz::initializeDepth()
{


    for (int i = 0; i < deviceList.size(); i++)
    {

        const openni::SensorInfo* sinfo = deviceList[i]->getSensorInfo(openni::SENSOR_DEPTH); 
        const openni::Array< openni::VideoMode>& modesDepth = sinfo->getSupportedVideoModes();

        // Create Stream
        openni::VideoStream* depth_stream = new openni::VideoStream;
        OPENNI_CHECK( depth_stream->create( *deviceList[i], openni::SENSOR_DEPTH ) );

        int mode;
        // Set Video Mode
        if (false){
            showSensorData(modesDepth);
            std::cout << "Select Depth Mode: ";
            std::cin >> mode;
        } else {

            // go through modes and select the 480p
            for (int j = 0; j < modesDepth.getSize(); j++)
            {
                if (modesDepth[j].getResolutionX() == 640 && modesDepth[j].getResolutionY() == 480 &&
                    modesDepth[j].getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_100_UM)
                {
                    mode = j;
                    break;
                }
            }

        }

        OPENNI_CHECK( depth_stream->setVideoMode( modesDepth[mode] ) );

        // Start Stream
        OPENNI_CHECK( depth_stream->start() );

        depthStreamsList.push_back(depth_stream);

    }
    
}

// Initialize Color
inline void PointcloudViz::initializeColor()
{

    for (int i = 0; i < deviceList.size(); i++)
    {

        const openni::SensorInfo* sinfo = deviceList[i]->getSensorInfo(openni::SENSOR_COLOR); 
        const openni::Array< openni::VideoMode>& modesColor = sinfo->getSupportedVideoModes();

        // Create Stream
        openni::VideoStream* color_stream = new openni::VideoStream;
        OPENNI_CHECK( color_stream->create( *deviceList[i], openni::SENSOR_COLOR ) );

        int mode;
        // Set Video Mode
        if (false){
            showSensorData(modesColor);
            std::cout << "Select color Mode: ";
            std::cin >> mode;
        } else {
            // go through modes and select the 480p
            for (int j = 0; j < modesColor.getSize(); j++)
            {
                if (modesColor[j].getResolutionX() == 640 && modesColor[j].getResolutionY() == 480 &&
                    modesColor[j].getPixelFormat() == openni::PIXEL_FORMAT_RGB888)
                {
                    mode = j;
                    break;
                }
            }
        }

        OPENNI_CHECK( color_stream->setVideoMode( modesColor[mode] ) );

        // Start Stream
        OPENNI_CHECK( color_stream->start() );

        colorStreamsList.push_back(color_stream);

    }

}

void PointcloudViz::initializeTransforms(){
    //i am using json lib from nlohmann
    
    // Read JSON file
    std::ifstream i("../config/pose_est.json");
    json j;
    i >> j;

    // Iterate through JSON object
    for (auto& item : j.items()) {
        // Get rotation matrix
        Eigen::Matrix3f R;
        const json& R_json = item.value()["R"];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R(i, j) = R_json[i][j];
            }
        }

        // Get translation vector
        Eigen::Vector3f t;
        const json& t_json = item.value()["t"];
        for (int i = 0; i < 3; i++) {
            t(i) = t_json[i];
        }

        // Create transformation matrix
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = R;
        transform.block<3, 1>(0, 3) = t;

        // Add transformation matrix to vector
        transforms.push_back(transform);
    }
}


// Initialize Point Cloud
inline void PointcloudViz::initializeViewer()
{
    
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addCoordinateSystem(10);
    viewer->initCameraParameters();

    viewer_launched_flag = 1;

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

    //close viewer
    viewer->close();

    //destroy this object pointcloudviz
    delete this;

}

// Update Data
void PointcloudViz::update()
{
    
    // Update Color
    updateColor();

    // Update Depth
    updateDepth();

}

// Update Color
inline void PointcloudViz::updateColor()
{

    for (int i = 0; i < colorStreamsList.size(); i++)
    {
        // Update Frame
        OPENNI_CHECK( colorStreamsList[i]->readFrame( &color_frames[i] ) );
    } 
    
}

// Update Depth
inline void PointcloudViz::updateDepth()
{
    for (int i = 0; i < depthStreamsList.size(); i++)
    {
        // Update Frame
        OPENNI_CHECK( depthStreamsList[i]->readFrame( &depth_frames[i] ) );
    }

}



// Draw Data
void PointcloudViz::draw()
{
    // Draw color
    drawColor();

    // Draw Point Cloud
    drawPointCloud();
}

// Draw Color
inline void PointcloudViz::drawColor()
{

    colorMatList.clear();
    for (int i = 0; i < color_frames.size(); i++)
    {
        // Create cv::Mat form Color Frame
        cv::Mat color_mat = cv::Mat( color_frames[i].getHeight() , color_frames[i].getWidth(), CV_8UC3,
         const_cast<void*>( color_frames[i].getData() ) );

        cv::cvtColor( color_mat, color_mat, cv::COLOR_RGB2BGR );

        colorMatList.push_back(color_mat);

        if (DISPLAY_COLOR){

            cv::namedWindow("Color " + std::to_string(i));
            cv::imshow("Color " + std::to_string(i), color_mat);

        }

    }


    
}

// Draw Point Cloud
inline void PointcloudViz::drawPointCloud()
{

    //printf("started drawing point cloud\n");
    pcloudList.clear();
    for (int i = 0; i < depth_frames.size(); i++)
    {    
    
        if( !depth_frames[i].isValid() ){
            printf("Invalid Depth Frame\n");
            break;
        }

        // Retrieve Depth
        const uint16_t* depth = static_cast<const uint16_t*>( depth_frames[i].getData() );
        const uint8_t* color = static_cast<const uint8_t*>( color_frames[i].getData() );


        // Create pcl::PointCloud<pcl::PointXYZRGB> object
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        

        // Resize the point cloud to match the depth frame size
        pcl_cloud->width = depth_frames[i].getWidth();
        pcl_cloud->height = depth_frames[i].getHeight();
        pcl_cloud->is_dense = false;
        pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);

        #pragma omp parallel for
        for (uint32_t y = 0; y < depth_frames[i].getHeight(); y++) {
            for (uint32_t x = 0; x < depth_frames[i].getWidth(); x++) {

            // Retrieve Depth
            const uint16_t z = depth[y * depth_frames[i].getWidth() + x];
            if (!z) {
                continue;
            }

            // Convert Depth to World
            float wx, wy, wz;
            OPENNI_CHECK(openni::CoordinateConverter::convertDepthToWorld(*depthStreamsList[i], static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), &wx, &wy, &wz));

            // Set Point XYZ coordinates
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].x = wx / 1000.0f;
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].y = wy / 1000.0f;
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].z = wz / 1000.0f;

            // Retrieve Color
            const uint8_t* color_ptr = &color[(y * depth_frames[i].getWidth() + x) * 3];
            
            // Set Point RGB color
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].r = color_ptr[2];
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].g = color_ptr[1];
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].b = color_ptr[0];
            
            }
        }

        // Add the pcl::PointCloud to the pcloudList
        pcloudList.push_back(pcl_cloud);
    }

    //printf("finished drawing point cloud\n");

}

// Show Data
void PointcloudViz::show()
{
    if (DISPLAY_POINTCLOUD)
    {
        // Show Point Cloud
        showPointCloud();
    }    

}

// Show Point Cloud
inline void PointcloudViz::showPointCloud()
{
    
    if (!viewer_launched_flag){
        return;
    }

    //printf("accessing pcloulist\n");
    // merge all point clouds in pcloudlist
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < pcloudList.size(); i++)
    {

        if (i == 0) {
            *merged_cloud = *pcloudList[i];
            continue;
        }

        // transofmr 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*pcloudList[i], *transformed_cloud, transforms[i - 1]);

        //merge the point cloud
        *merged_cloud += *transformed_cloud;

    }
    //printf("finished merging pcls");

    // Update Point Cloud
    if (!viewer->updatePointCloud(merged_cloud, "cloud"))
    {
        viewer->addPointCloud(merged_cloud, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    }

    //printf("accessed pcloulist\n");

}

void PointcloudViz::showSensorData(const openni::Array< openni::VideoMode>& modesDepth)
{
    std::cout << "Depth modes" << std::endl;
    for (int i = 0; i<modesDepth.getSize(); i++) {
        printf("%i: %ix%i, %i fps, %i format\n", i, modesDepth[i].getResolutionX(), modesDepth[i].getResolutionY(),
            modesDepth[i].getFps(), modesDepth[i].getPixelFormat()); //PIXEL_FORMAT_DEPTH_1_MM = 100, PIXEL_FORMAT_DEPTH_100_UM
    }

}