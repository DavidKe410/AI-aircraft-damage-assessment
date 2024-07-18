#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>


class DmgFilterLocal : public rclcpp::Node
{
public:
    DmgFilterLocal() : Node("dmg_filter_local")
    {
        // Create subscribers
        flir_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/flir_boson/camera_info", 3, std::bind(&DmgFilterLocal::flirInfoCallback, this, std::placeholders::_1));
        arducam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/arducam_tof/camera_info", 3, std::bind(&DmgFilterLocal::arducamInfoCallback, this, std::placeholders::_1));
        arducam_tof_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/arducam_tof/image_rect", 5, std::bind(&DmgFilterLocal::depthCallback, this, std::placeholders::_1));
        coords_string_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/coords", 5, std::bind(&DmgFilterLocal::coordsCallback, this, std::placeholders::_1));

        // Create publisher
        // TODO: rename and make sure it matches expected topic/message-type output
        labeled_point_pub_ = this->create_publisher<std_msgs::msg::String>("/registered/labeled_point", 10);


        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Create a timer to periodically look up a transform
        tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DmgFilterLocal::tfCallback, this));
    }

private:
    void flirInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){flir_camera_info_ = msg;}
    void arducamInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){arducam_camera_info_ = msg;}
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg){arducam_tof_image_ = msg;}
    void tfCallback(){
        try{
            transform_stamped_ = tf_buffer_->lookupTransform("boson_camera", "ardu_tof_frame", tf2::TimePointZero);}
        catch (tf2::TransformException &ex){
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());}
    }

    static std::string toString(const Eigen::Vector3d& vec){
        std::stringstream ss;
        ss << vec;
        return ss.str();
    }

    // Function to back-project an IR pixel into a 3D ray in the IR camera frame
    Eigen::Vector3d backProjectIRPixel(int u_ir, int v_ir, const sensor_msgs::msg::CameraInfo& ir_info) {
        float fx_ir = ir_info.k[0];
        float fy_ir = ir_info.k[4];
        float cx_ir = ir_info.k[2];
        float cy_ir = ir_info.k[5];

        float x = (u_ir - cx_ir) / fx_ir;
        float y = (v_ir - cy_ir) / fy_ir;

        return Eigen::Vector3d(x, y, 1.0); // This is the 3D ray direction in the IR camera frame
    }

        // Function to find corresponding depth pixel and retrieve 3D coordinates
    Eigen::Vector3d irPixelTo3DPoint(int u_ir, int v_ir, const sensor_msgs::msg::CameraInfo& ir_info,
        const sensor_msgs::msg::CameraInfo& depth_info, const Eigen::Matrix3d& R, const Eigen::Vector3d& T,
        const sensor_msgs::msg::Image& depth_image) {
        
        // Step 1: Back-project IR pixel into a 3D ray in the IR camera frame
        Eigen::Vector3d ray_ir = backProjectIRPixel(u_ir, v_ir, ir_info);

        // Step 2: Transform the 3D ray to the depth camera frame
        Eigen::Vector3d ray_depth = R.inverse() * (ray_ir - T);

        // Step 3: Find corresponding depth pixel in the depth image
        float fx_d = depth_info.k[0];
        float fy_d = depth_info.k[4];
        float cx_d = depth_info.k[2];
        float cy_d = depth_info.k[5];

        float u_depth = fx_d * ray_depth.x() / ray_depth.z() + cx_d;
        float v_depth = fy_d * ray_depth.y() / ray_depth.z() + cy_d;

        int u_depth_int = static_cast<int>(u_depth);
        int v_depth_int = static_cast<int>(v_depth);

        RCLCPP_INFO(this->get_logger(), "U Depth int pixel: '%d'", u_depth_int);
        RCLCPP_INFO(this->get_logger(), "V Depth int pixel: '%d'", v_depth_int);

        // Don't need to static cast below but just don't want to see warnings
        if (u_depth_int < 0 || u_depth_int >= static_cast<int>(depth_image.width) || v_depth_int < 0 || v_depth_int >= static_cast<int>(depth_image.height)) {
            RCLCPP_INFO(this->get_logger(), "Depth pixel out of bounds, tossing coordinates");
            Eigen::Vector3d null_point(-1, -1, -1);
            return null_point;
        }else{

            // Step 4: Retrieve depth value and compute 3D coordinates
            int index = v_depth_int * depth_image.step + u_depth_int * sizeof(float);
            float depth_value = *(reinterpret_cast<const float*>(&depth_image.data[index]));

            Eigen::Vector3d point_depth_camera(
                (u_depth - cx_d) * depth_value / fx_d,
                (v_depth - cy_d) * depth_value / fy_d,
                depth_value
            );

            return point_depth_camera;
        }
    }

//TODO Check the math, test the actual program with inputs, remove the flir boson camera info from arducam? launch file for the ros1 stuff

    void coordsCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Received String: '%s'", msg->data.c_str());

        std::stringstream ss;
        ss << msg->data;

        std::vector<std::string> splitString;

        while(ss.good()){
            std::string substr;
            getline( ss, substr, ',' );
            splitString.push_back( substr );
        }

        int ir_pixel_u = std::stoi(splitString[0]); // IR pixel coordinates
        int ir_pixel_v = std::stoi(splitString[1]);

        // Use Eigen to perform some transformation or computation
        Eigen::Isometry3d eigen_transform = tf2::transformToEigen(transform_stamped_);
        
        // Extract rotation matrix and translation vector
        Eigen::Matrix3d rotation_mat = eigen_transform.rotation(); //move this into function irPixelTo3DPoint?
        Eigen::Vector3d translation_vec = eigen_transform.translation();

        Eigen::Vector3d point_3d = irPixelTo3DPoint(ir_pixel_u, ir_pixel_v, *flir_camera_info_, *arducam_camera_info_, rotation_mat, translation_vec, *arducam_tof_image_);

        // Prepare the message to be published
        auto message = std_msgs::msg::String();
        message.data = "Processed string: " + msg->data + " with Eigen result: \n" + DmgFilterLocal::toString(point_3d);
        
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        labeled_point_pub_->publish(message);
    }

    sensor_msgs::msg::CameraInfo::SharedPtr flir_camera_info_;
    sensor_msgs::msg::CameraInfo::SharedPtr arducam_camera_info_;
    sensor_msgs::msg::Image::SharedPtr arducam_tof_image_;
    //std::shared_ptr<std_msgs::msg::String> labeled_point_msg_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr flir_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr arducam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr arducam_tof_image_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr coords_string_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr labeled_point_pub_;

    // Transformation Content
    geometry_msgs::msg::TransformStamped transform_stamped_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    //
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DmgFilterLocal>());
  rclcpp::shutdown();
  return 0;
}