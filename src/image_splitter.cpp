#include "image_splitter/image_splitter.h"

ImageSplitter::ImageSplitter() : Node("image_splitter"), convert_all_msg(false)
{
    hz = this->declare_parameter("hz", 10.0);
    save_dir_path = this->declare_parameter("save_dir_path", "/your/save/dir");
    if(save_dir_path.back() != '/') save_dir_path += "/";

    image_sub = this->create_subscription<sensor_msgs::msg::Image>("image", 1, std::bind(&ImageSplitter::image_callback, this, std::placeholders::_1));

    if(hz == 0.0) convert_all_msg = true;
    start_time = get_time();

    std::cout << "=== image_splitter ===" << std::endl;
    std::cout << "hz: " << hz << std::endl;
    std::cout << "save_dir_path: " << save_dir_path << std::endl;
    std::cout << "image topic: " << image_sub->get_topic_name() << std::endl;

    if(!opendir(save_dir_path.c_str()))
    {
        if(!mkdir_recursion(save_dir_path))
        {
            std::cerr << "Cannot mkdir_recursion: " << save_dir_path << std::endl;
            return;
        }
    }
    if(!convert_all_msg)
    {
       std::chrono::microseconds ms{(int)((1.0 / hz) * 1000)};
       this->create_wall_timer(ms, std::bind(&ImageSplitter::timer_callback, this));
    }
}

bool ImageSplitter::mkdir_recursion(const std::string path)
{
    std::string new_dir_path = "/";
    for(size_t i=1; i<path.size(); i++)
    {
        new_dir_path += path.at(i);
        if(path.at(i) == '/' && !opendir(new_dir_path.c_str()))
        {
            if(mkdir(new_dir_path.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP) != 0)
            {
                std::cerr << "Cannot mkdir: " << new_dir_path << std::endl;
                return false;
            }
            std::cout << "mkdir: " << new_dir_path << std::endl;
        }
    }
    return true;
}

void ImageSplitter::image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
    if(!convert_ros_image_to_cv_image(*image_msg, cv_image)) return;
    save_time = get_time();

    if(convert_all_msg)
    {
        save_image(cv_image, save_dir_path, save_time - start_time);
    }
}

void ImageSplitter::timer_callback()
{
    save_image(cv_image, save_dir_path, save_time - start_time);
}

double ImageSplitter::get_time()
{
    return this->now().seconds() + 1e-9 * this->now().nanoseconds();
}

bool ImageSplitter::convert_ros_image_to_cv_image(const sensor_msgs::msg::Image& ros_image, cv::Mat& cv_image)
{
    save_time = get_time();
    cv_bridge::CvImageConstPtr cv_image_ptr;
    try
    {
        cv_image_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }

    cv::Mat image(cv_image_ptr->image.rows, cv_image_ptr->image.cols, cv_image_ptr->image.type());
    image = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8)->image;
    cv_image = image.clone();
    // std::cout << "catch image" << std::endl;
    return true;
}

void ImageSplitter::save_image(cv::Mat& image, std::string dir_path, double time_stamp)
{
    static int count = 0;
    if(image.empty())
    {
        std::cerr << "image is empty" << std::endl;
        return;
    }

    static const int padding_digit = 4;
    std::string save_name = dir_path + "image" + padding_0(count, padding_digit) + "_" + std::to_string(time_stamp) + ".jpg";

    cv::imwrite(save_name, image);
    std::cout << "save image: " << save_name << std::endl;
    count++;
    image.release();
}

std::string ImageSplitter::padding_0(int num, int digit)
{
    std::ostringstream oss;
    oss << std::setw(digit) << std::setfill('0') << num;
    std::string str(oss.str());

    // std::cout << "padding 0 : " << num << " to " << str << std::endl;
    return str;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSplitter>());
    rclcpp::shutdown();
    return 0;
}