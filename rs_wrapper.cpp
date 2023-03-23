#include <ros/ros.h>
#include <iostream>
#include <string>
#include <map>
#include <librealsense2/rs.hpp>
#include <algorithm>
// #include <cv.h> # works for opencv3 or older version but not opencv4 
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher rgb_pub; 
ros::Publisher dpt_pub; 

bool device_with_streams(std::vector <rs2_stream> stream_requests, std::string& out_serial); 

// depth camera intrinsics 
bool gb_pub_pc = true; // false; // whether publish point cloud 
ros::Publisher pc_pub; 
float vcx, vcy, vfx, vfy; 
void convertToPC(sensor_msgs::ImagePtr rgb, sensor_msgs::ImagePtr dpt, const sensor_msgs::PointCloud2::Ptr& cloud_msg);

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char * argv[]) try
{
    // set up ros 
    ros::init(argc, argv, "pub_realsense_data");
    ros::NodeHandle nh; 
    ros::NodeHandle np("~"); 
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    bool b_pub_imu = true; 

    np.param("pub_point_cloud", gb_pub_pc, gb_pub_pc); 

    int q = 7; 
    rgb_pub = nh.advertise<sensor_msgs::Image>("/cam/color", q); 
    dpt_pub = nh.advertise<sensor_msgs::Image>("/cam/depth", q);

    if(gb_pub_pc)
        pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/rgbd_pc", q);

    // set up camera 
 	std::string serial;
    if (!device_with_streams({ RS2_STREAM_COLOR,RS2_STREAM_DEPTH }, serial))
        return EXIT_SUCCESS;
      // Create a pipeline to easily configure and start the camera
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_COLOR);
    pipe.start(cfg);

    // align to color 
    rs2::align align_to_color(RS2_STREAM_COLOR);

    unsigned int img_seq = 0; 
    while (ros::ok())
    {

       // Using the align object, we block the application until a frameset is available
        rs2::frameset frameset = pipe.wait_for_frames();

        // Align all frames to color viewport
        frameset = align_to_color.process(frameset);

        // With the aligned frameset we proceed as usual
        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();

        // Get the color frame's dimensions
        auto width = color.get_width();
        auto height = color.get_height();

        std::cout<<"color image size: "<<height<<" x "<<width<<std::endl; 

        // Get the depth frame's dimensions
        width = depth.get_width();
        height = depth.get_height();

 		float scale = depth.get_units(); 
        rs2_intrinsics rgb_intri = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); 
        vcx = rgb_intri.ppx; vcy = rgb_intri.ppy; vfx = rgb_intri.fx; vfy = rgb_intri.fy; 

        // std::cout<<"depth image size: "<<height<<" x "<<width<<" scale: "<<scale<<std::endl; 

        // Query the distance from the camera to the object in the center of the image
        // float dist_to_center = depth.get_distance(width / 2, height / 2);

           // generate rgb cv::Mat 
        cv::Mat rgb(height, width, CV_8UC3, (void*)(color.get_data()));
		cv::cvtColor(rgb, rgb, CV_BGR2RGB);

        cv::Mat dpt(height, width, CV_16UC1, (void*)(depth.get_data())); 

        // unsigned short pixel_v = dpt.at<unsigned short>(height/2, width/2);  

        // cv::imshow("depth", dpt); 
        // cv::waitKey(20);  

        // Print the distance
        // std::cout << "The camera is facing an object " << dist_to_center << " meters away \r"<<std::endl;
        // std::cout<<" The pixel value: "<<pixel_v<<" pixel_v*scale: "<<(float)(pixel_v*scale)<<std::endl;
    
        cv_bridge::CvImage rgb_msg, dpt_msg;
        rgb_msg.header.frame_id = "/camera"; 
        rgb_msg.header.stamp = ros::Time::now(); // fromSec(sample.visibleFrame.timestamp()); 
        rgb_msg.header.seq = ++img_seq; 
        rgb_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; 
        rgb_msg.image = rgb.clone(); 
        dpt_msg.header = rgb_msg.header; 
        dpt_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1; 
        dpt_msg.image = dpt.clone(); //dpt.clone(); 


        rgb_pub.publish(rgb_msg); 
        dpt_pub.publish(dpt_msg); 
        ros::spinOnce(); 

        if(gb_pub_pc)
        {
            sensor_msgs::PointCloud2::Ptr cloud_msg (new sensor_msgs::PointCloud2);
            convertToPC(rgb_msg.toImageMsg(), dpt_msg.toImageMsg(), cloud_msg); 
            pc_pub.publish(cloud_msg); 
            std::cout<<"pub_realsense_data: publish cloud_msg: width "<<cloud_msg->width<<std::endl; 
            ros::spinOnce(); 
            usleep(5000); 
        }
        std::cout<<"publish image "<<++img_seq<<std::endl; 
        usleep(1000*1000);
    }


    ros::shutdown();
    return EXIT_SUCCESS;

}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


void convertToPC(sensor_msgs::ImagePtr rgb, sensor_msgs::ImagePtr dpt, const sensor_msgs::PointCloud2::Ptr& cloud_msg)
{
    int pt_cnt = 0; 
    int index = 0; 
    unsigned short* pdpt = (unsigned short*)(&dpt->data[0]);
    for(int r = 0; r < dpt->height; r++)
        for(int c = 0; c<dpt->width; c++){

            index = r * dpt->width + c; 
            unsigned short d =  pdpt[index]; 

            if(!std::isnan(d) && d > 0.2)
                pt_cnt++; 
        }

    // cout <<"publish_struct_core_2: pt_cnt = "<<pt_cnt<<endl; 
     // Use depth image time stamp
    cloud_msg->header.stamp = rgb->header.stamp;
    cloud_msg->header.frame_id = "cloud_tf_frame"; //rgb->header.frame_id;
    cloud_msg->height = 1;
    cloud_msg->width  = pt_cnt;
    cloud_msg->is_dense = true;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    pcd_modifier.resize(pt_cnt);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");
    unsigned char* prgb = (unsigned char*)(&rgb->data[0]);
    int color_step = 3; 
    for(int r = 0; r < dpt->height; r++)
        for(int c = 0; c<dpt->width; c++){
            index = r * dpt->width + c; 
            unsigned short d =  pdpt[index]; 
            if(!std::isnan(d) && d> 0.2){
                float depth  = d * 0.0001; // to meters D405 scale is 0.0001
                float x = depth * (c - vcx) / vfx; 
                float y = depth * (r - vcy) / vfy; 
                std::size_t pix_color_offset =  r * rgb->width * color_step + c * color_step; 
                (*iter_x) = depth; (*iter_y) = -x; (*iter_z) = -y; 
                ++iter_x; ++iter_y; ++iter_z;
                (*iter_r) = prgb[pix_color_offset+2]; 
                (*iter_g) = prgb[pix_color_offset+1]; 
                (*iter_b) = prgb[pix_color_offset+0]; 
                (*iter_a) = 255;
                ++iter_r; ++iter_g; ++iter_b; ++iter_a;
            }
    }

}

// Find devices with specified streams
bool device_with_streams(std::vector <rs2_stream> stream_requests, std::string& out_serial)
{
    rs2::context ctx;
    auto devs = ctx.query_devices();
    std::vector <rs2_stream> unavailable_streams = stream_requests;
    for (auto dev : devs)
    {
        std::map<rs2_stream, bool> found_streams;
        for (auto& type : stream_requests)
        {
            found_streams[type] = false;
            for (auto& sensor : dev.query_sensors())
            {
                for (auto& profile : sensor.get_stream_profiles())
                {
                    if (profile.stream_type() == type)
                    {
                        found_streams[type] = true;
                        unavailable_streams.erase(std::remove(unavailable_streams.begin(), unavailable_streams.end(), type), unavailable_streams.end());
                        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
                            out_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    }
                }
            }
        }
        // Check if all streams are found in current device
        bool found_all_streams = true;
        for (auto& stream : found_streams)
        {
            if (!stream.second)
            {
                found_all_streams = false;
                break;
            }
        }
        if (found_all_streams)
            return true;
    }
    // After scanning all devices, not all requested streams were found
    for (auto& type : unavailable_streams)
    {
        switch (type)
        {
        case RS2_STREAM_POSE:
        case RS2_STREAM_FISHEYE:
            std::cerr << "Connect T26X and rerun the demo" << std::endl;
            break;
        case RS2_STREAM_DEPTH:
            std::cerr << "The demo requires Realsense camera with DEPTH sensor" << std::endl;
            break;
        case RS2_STREAM_COLOR:
            std::cerr << "The demo requires Realsense camera with RGB sensor" << std::endl;
            break;
        default:
            throw std::runtime_error("The requested stream: " + std::to_string(type) + ", for the demo is not supported by connected devices!"); // stream type
        }
    }
    return false;
}
