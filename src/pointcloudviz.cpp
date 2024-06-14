///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include "pointcloudviz.h"

// Constructor
PointcloudViz::PointcloudViz()
{
    // Initialize
    initialize();
}

// Destructor
PointcloudViz::~PointcloudViz()
{
    // Finalize
    finalize();
}

// Processing
void PointcloudViz::run()
{
    // Main Loop
    while( !viewers[0].wasStopped() ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        //show();

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
                pcl::io::savePCDFileASCII("pointcloud" + std::to_string(i) + ".pcd", *pcloudList[i]);
                std::cout << "Saved " << pcloudList[i]->points.size() << " data points to " << std::endl;            }

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

    // Initialize Point Cloud
    initializeViewer();

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

// Initialize Point Cloud
inline void PointcloudViz::initializeViewer()
{

    //set up a viewer for each stream
    for (int i = 0; i < deviceList.size(); i++)
    {
        // Create Window
        cv::viz::Viz3d viewer = cv::viz::Viz3d( "asus");
        viewer.setWindowSize( cv::Size(800,600));

        // Register Keyboard Callback Function
        viewer.registerKeyboardCallback( &keyboardCallback, this );

        viewers.push_back(viewer);
    }

    viewers[0].registerKeyboardCallback( &keyboardCallback, this );
}

// Keyboard Callback Function
void PointcloudViz::keyboardCallback( const cv::viz::KeyboardEvent& event, void* cookie )
{
    // Exit Viewer when Pressed ESC key
    if( (event.code == 'q' || event.code=='Q' || event.code==27) && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){

        // Retrieve Viewer
        cv::viz::Viz3d viewer = static_cast<PointcloudViz*>( cookie )->viewers[0]; // just callback in the first viewer

        // Close Viewer
        viewer.close();
    }

    // save screenshot
    if( (event.code == 's' || event.code=='S') && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
        // Retrieve Viewer
        cv::viz::Viz3d viewer = static_cast<PointcloudViz*>( cookie )->viewers[0];

        // Save Screenshot
        viewer.saveScreenshot("screenshot.png");
    }

    
};

// Finalize
void PointcloudViz::finalize()
{
    // Close Windows
    cv::destroyAllWindows();
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

    //show images from devices in two diffenrent windows
    for (int i = 0; i < color_frames.size(); i++)
    {
        cv::Mat image = cv::Mat( color_frames[i].getHeight() , color_frames[i].getWidth(), CV_8UC3,
         const_cast<void*>( color_frames[i].getData() ) );

        cv::cvtColor( image, image, cv::COLOR_RGB2BGR );

        cv::namedWindow("Color " + std::to_string(i));
        cv::imshow("Color " + std::to_string(i), image);
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

        colorMatList.push_back(color_mat);

    }
    
    
}

// Draw Point Cloud
inline void PointcloudViz::drawPointCloud()
{

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
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].x = wx;
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].y = wy;
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].z = wz;

            // Retrieve Color
            const uint8_t* color_ptr = &color[(y * depth_frames[i].getWidth() + x) * 3];
            
            // Set Point RGB color
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].r = color_ptr[0];
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].g = color_ptr[1];
            pcl_cloud->points[y * depth_frames[i].getWidth() + x].b = color_ptr[2];
            
            }
        }

        // Add the pcl::PointCloud to the pcloudList
        pcloudList.push_back(pcl_cloud);
    }
}

// Show Data
void PointcloudViz::show()
{
    // Show Point Cloud
    showPointCloud();
}

// Show Point Cloud
inline void PointcloudViz::showPointCloud()
{
    

    for (int i = 0; i < viewers.size(); i++)
    {

        if( pcloudList[i]->points.size() == 0 ){
            break;
        }

    }
    
}

void PointcloudViz::showSensorData(const openni::Array< openni::VideoMode>& modesDepth)
{
    std::cout << "Depth modes" << std::endl;
    for (int i = 0; i<modesDepth.getSize(); i++) {
        printf("%i: %ix%i, %i fps, %i format\n", i, modesDepth[i].getResolutionX(), modesDepth[i].getResolutionY(),
            modesDepth[i].getFps(), modesDepth[i].getPixelFormat()); //PIXEL_FORMAT_DEPTH_1_MM = 100, PIXEL_FORMAT_DEPTH_100_UM
    }

}