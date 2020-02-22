#include "image_splitter/image_splitter.h"

ImageSplitter::ImageSplitter() : Node("image_splitter"), convert_all_msg(false)
{
    hz = this->declare_parameter("hz", 10.0);
    save_dir_path = this->declare_parameter("save_dir_path", "/your/save/dir");

    image_sub = this->create_subscription<sensor_msgs::msg::Image>("/image", 1, std::bind(&ImageSplitter::image_callback, this, std::placeholders::_1));

    if(hz == 0.0) convert_all_msg = true;
    start_time = get_time();

    std::cout << "=== image_splitter ===" << std::endl;
    std::cout << "hz: " << hz << std::endl;
    std::cout << "save_dir_path: " << save_dir_path << std::endl;
}

void ImageSplitter::image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
    if(!convert_ros_image_to_cv_image(*image_msg, cv_image)) return;

    if(convert_all_msg)
    {
        save_image(cv_image, save_dir_path, save_time - start_time);
    }
}

double ImageSplitter::get_time()
{
    return this->now().seconds + 1e-9 * this->now().nanoseconds;
}

bool ImageSplitter::convert_ros_image_to_cv_image(const sensor_msgs::msg::Image& ros_image, cv::Mat cv_image)
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
    cv_image = image;
    return true;
}

void ImageSplitter::save_image(cv::Mat& image, std::string dir_path, double time_stamp)
{
    if(dir_path.back() != '/') dir_path += "/";
    if(!opendir(dir_path.c_str())) return;
}
