#ifndef __IMAGE_SPLITTER_H
#define __IMAGE_SPLITTER_H

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>

class ImageSplitter : public rclcpp::Node
{
public:
    ImageSplitter();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg);
    void timer_callback();
    bool convert_ros_image_to_cv_image(const sensor_msgs::msg::Image& ros_image, cv::Mat cv_image);
    double get_time();
    void save_image(cv::Mat& image, std::string save_dir_path, double time_stamp);
    std::string padding_0(int num, int digit);

    double hz;
    std::string save_dir_path;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    bool convert_all_msg;
    cv::Mat cv_image;
    double start_time;
    double save_time;
};

#endif // __IMAGE_SPLITTER_H