#include <chrono>
#include <memory>
#include <string>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <vector>

#include "ArducamTOFCamera.hpp"

using namespace std::chrono_literals;
using namespace Arducam;

ArducamTOFCamera tof;

class TOFPublisher : public rclcpp::Node
{
public:
    TOFPublisher() : Node("arducam"), pointsize_(43200)
    {
        pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud>();
        pc2_msg_->points.resize(pointsize_);
        pc2_msg_->channels.resize(1);
        pc2_msg_->channels[0].name = "intensities";
        pc2_msg_->channels[0].values.resize(pointsize_);
        depth_msg_ = std::make_shared<std_msgs::msg::Float32MultiArray>();
        depth_msg_->layout.dim.resize(2);
        depth_msg_->layout.dim[0].label = "height";
        depth_msg_->layout.dim[0].size = 180;
        depth_msg_->layout.dim[0].stride = 43200;
        depth_msg_->layout.dim[1].label = "width";
        depth_msg_->layout.dim[1].size = 240;
        depth_msg_->layout.dim[1].stride = 240;

        ///-------
        depth_img_msg_ = std::make_shared<sensor_msgs::msg::Image>() ;
        depth_img_msg_->height = 180;
        depth_img_msg_->width = 240;
        depth_img_msg_->is_bigendian = 0;
        depth_img_msg_->encoding = "32FC1";
        depth_img_msg_->step = depth_img_msg_->width * sizeof(float);

        std::string url  = "file:///home/orin/ardu_tof_bridge_ws/src/arducam/calibration/ArducamToF.yaml";
        cim_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "ArducamToF", url);
        cam_info_msg_ = std::make_shared<sensor_msgs::msg::CameraInfo>(cim_->getCameraInfo());
        printf("after cam info msg and url\n");

        ///-----DELETE \/\/ LATER------

        // std::string flir_url  = "file:///home/orin/air_dmg_assesment_ws/src/flir_boson_usb/example_calibrations/Boson640.yaml";
        // cim2_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "Boson640", flir_url);
        // cam_info_msg2_ = std::make_shared<sensor_msgs::msg::CameraInfo>(cim2_->getCameraInfo());
        // publisher_cam_info2_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("flir_boson/camera_info", 2);
        ///-----DELETE /\/\ LATER ------



        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("arducam_tof/point_cloud", 10);
        publisher_depth_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("arducam_tof/depth_frame", 10);
        
        ///-------
        publisher_depth_img_ = this->create_publisher<sensor_msgs::msg::Image>("arducam_tof/image_raw", 10);
        publisher_cam_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("arducam_tof/camera_info", 2);
        ///-------


        timer_ = this->create_wall_timer(
            50ms, std::bind(&TOFPublisher::update, this));
    }

private:
    void generateSensorPointCloud()
    {
        printf("generate sensor\n");
        ArducamFrameBuffer *frame;
        do
        {
            frame = tof.requestFrame(200);
        } while (frame == nullptr);
        depth_frame.clear();
        float *depth_ptr = (float *)frame->getData(FrameType::DEPTH_FRAME);
        float *amplitude_ptr = (float *)frame->getData(FrameType::AMPLITUDE_FRAME);
        unsigned long int pos = 0;
        for (int row_idx = 0; row_idx < 180; row_idx++)
            for (int col_idx = 0; col_idx < 240; col_idx++, pos++)
            {
                if (amplitude_ptr[pos] > 30)
                {
                    float zz = depth_ptr[pos]; 
                    pc2_msg_->points[pos].x = (((120 - col_idx)) / fx) * zz;
                    pc2_msg_->points[pos].y = ((90 - row_idx) / fy) * zz;
                    pc2_msg_->points[pos].z = zz;
                    pc2_msg_->channels[0].values[pos] = depth_ptr[pos];
                    depth_frame.push_back(depth_ptr[pos]);
                }
                else
                {
                    pc2_msg_->points[pos].x = 0;
                    pc2_msg_->points[pos].y = 0;
                    pc2_msg_->points[pos].z = 0;
                    pc2_msg_->channels[0].values[pos] = 0;
                    depth_frame.push_back(0);
                }
            }
        tof.releaseFrame(frame);
        pc2_msg_->header.frame_id = frame_id_;
        depth_msg_->data = depth_frame;

        // Check sizes
        //std::cout << "depth_frame size: " << depth_frame.size() << std::endl;
        //std::cout << "depth_img_msg_ data size before resize: " << depth_img_msg_->data.size() << std::endl;

        ///-------
        depth_img_msg_->data.resize(depth_frame.size() * sizeof(float));
        depth_img_msg_->header.frame_id = frame_id_;

        // Check new size
        //std::cout << "depth_img_msg_ data size after resize: " << depth_img_msg_->data.size() << std::endl;


        memcpy(depth_img_msg_->data.data(), depth_frame.data(), depth_frame.size() * sizeof(float));
        ///-------
    }
    void update()
    {
        printf("updates?\n");
        generateSensorPointCloud();
        pc2_msg_->header.stamp = now();

        ///-------
        depth_img_msg_->header.stamp = pc2_msg_->header.stamp;
        cam_info_msg_->header.stamp = depth_img_msg_->header.stamp;
        cam_info_msg_->header.frame_id = frame_id_;
        ///-------

        /// DELETE \/\/ LATER -------
        // cam_info_msg2_->header.stamp = depth_img_msg_->header.stamp;
        // cam_info_msg2_->header.frame_id = "boson_camera";
        // publisher_cam_info2_->publish(*cam_info_msg2_);
        /// DELETE /\/\ LATER-------


        publisher_->publish(*pc2_msg_);
        publisher_depth_->publish(*depth_msg_);

        ///-------
        publisher_depth_img_->publish(*depth_img_msg_);
        publisher_cam_info_->publish(*cam_info_msg_);
        ///-------
    }
    std::string frame_id_ = "ardu_tof_frame";
    std::vector<float> depth_frame;
    sensor_msgs::msg::PointCloud::SharedPtr pc2_msg_;
    std_msgs::msg::Float32MultiArray::SharedPtr depth_msg_;

    ///-------
    std::shared_ptr<camera_info_manager::CameraInfoManager> cim_;


    sensor_msgs::msg::Image::SharedPtr depth_img_msg_;
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg_;

    // std::shared_ptr<camera_info_manager::CameraInfoManager> cim2_; //DELETE LATER
    // sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg2_; //DELETE LATER
    ///-------

    size_t pointsize_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_depth_;
    
    ///-------
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_img_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_cam_info_;

    //rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_cam_info2_; //DELETE LATER
    ///-------


    float fx = 240 / (2 * tan(0.5 * M_PI * 64.3 / 180));
    float fy = 180 / (2 * tan(0.5 * M_PI * 50.4 / 180));
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if (tof.open(Connection::CSI))
    {
        printf("initialize fail\n");
        exit(-1);
    }

    if (tof.start())
    {
        printf("start fail\n");
        exit(-1);
    }
    tof.setControl(CameraCtrl::RANGE,4);

    printf("pointcloud publisher start\n");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    printf("before spin...\n");
    rclcpp::spin(std::make_shared<TOFPublisher>());
    printf("after spin...\n");
    rclcpp::shutdown();
    tof.stop();
    tof.close();
    return 0;
}
