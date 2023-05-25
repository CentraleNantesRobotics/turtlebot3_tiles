#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;


class ProcessingNode : public rclcpp::Node
{
public:
  ProcessingNode() : Node("tile_detector")
  {
    im_sub = image_transport::create_subscription(this, "/image_raw", [&](const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {img = cv_bridge::toCvCopy(msg, "bgr8")->image;}, "compressed");

    process_timer = create_wall_timer(50ms, [&](){process();});
  }


  void process()
  {
    if(img.cols * img.rows == 0)
      return;



    cv::imshow("Image", img);
    cv::waitKey(1);
  }



private:

  image_transport::Subscriber im_sub;
  cv::Mat img;
  rclcpp::TimerBase::SharedPtr process_timer;
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcessingNode>());
  rclcpp::shutdown();
}
