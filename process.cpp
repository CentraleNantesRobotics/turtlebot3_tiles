#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <turtlebot3_tiles/ransac_lines.h>
#include <turtlebot3_tiles/pnp.h>
#include <turtlebot3_tiles/mbt.h>

using namespace std::chrono_literals;

constexpr auto useMBT{true};

inline auto createKernel(int size)
{
  return cv::getStructuringElement(cv::MORPH_CROSS, {2*size+1, 2*size+1}, {size,size});
}

class ProcessingNode : public rclcpp::Node
{
public:
  ProcessingNode() : Node("tile_detector")
  {
    im_sub = image_transport::create_subscription(this, "/image_raw", [&](const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {img = cv_bridge::toCvCopy(msg, "bgr8")->image;}, "compressed", rmw_qos_profile_sensor_data);


    if(useMBT)
    {
      tracker = std::make_unique<MBT>();
      process_timer = create_wall_timer(500ms, [&](){processMBT();});
    }
    else
    {
      process_timer = create_wall_timer(500ms, [&](){process();});

      cv::namedWindow("Edges", cv::WINDOW_AUTOSIZE );
      cv::createTrackbar("blurr: ","Edges", &blurr, 50);
      cv::createTrackbar("min: ","Edges", &min_thr, 600);
      cv::createTrackbar("max: ","Edges", &max_thr, 600);

      cv::namedWindow("Skeleton", cv::WINDOW_AUTOSIZE );
      cv::createTrackbar("dilate: ","Skeleton", &dilate, 10);

      cv::namedWindow("Lines", cv::WINDOW_AUTOSIZE );
      cv::createTrackbar("thr: ","Lines", &hough_thr, 1000);
    }
  }


  void process()
  {
    cv::waitKey(1);
    if(img.cols * img.rows == 0)
      return;

    //std::cout << " ---- new image -----\n";

    static cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    //tracker.track(gray);
    //return;

    if(!Line::yOffset)
    {
      bottom.y = bottom.height = Line::yOffset = img.rows/2;
      bottom.x = 0;
      bottom.width = img.cols;
    }

    auto tiles = gray(bottom);

    static cv::Mat filtered;
    cv::GaussianBlur(tiles, filtered, {5,5}, blurr);

    static cv::Mat edges;
    cv::Canny(filtered, edges, min_thr, max_thr, 5);

    // get skeleton
    static cv::Mat dilated, eroded;
    cv::dilate(edges, dilated, createKernel(dilate));
    cv::ximgproc::thinning(dilated, eroded, cv::ximgproc::THINNING_GUOHALL);

    cv::imshow("Skeleton", eroded);
    cv::imshow("Edges", edges);

    static std::vector<Line> lines;
    static cv::Mat im_lines;
    im_lines = img.clone();

    lines = detectHough(eroded, hough_thr);
    //lines = detectRansac(eroded, hough_thr);

    removeSimilar(lines);

    if(lines.size() < 4)
      return;

    auto mid = Line::partition(lines);

    if(mid == lines.end())
      return;

    Line::sort(lines.begin(), mid);
    Line::sort(mid, lines.end());
    //std::sort(lines.begin(), mid, Line::compareOffset);
    //std::sort(mid, lines.end(), Line::compareOffset);

    const Lines vert{lines.begin(), std::min(mid, lines.begin()+4)};
    const Lines hor{mid, std::min(mid+4, lines.end())};

    displayLines(vert, im_lines, true);
    displayLines(hor, im_lines, false);

    if(vert.size() && hor.size())
    {
      uint x{0};
      for(auto &h: hor)
      {
        uint y{0};
        for(auto &v: vert)
        {
          //cv::circle(im_lines, , 5, cv::Scalar(0,0,r), -1);
          if(const auto p{h.intersect(v)}; p.has_value())
            cv::putText(im_lines, "(" + std::to_string(x) + "," + std::to_string(y) + ")",
                        p.value(), 1, 1, {}, 2, cv::LINE_AA);
          y++;
        }
        x++;
      }
    }

    const auto pose = cam.computePose(hor, vert);

    if(pose.has_value())
    {
      cam.showPose(im_lines, pose.value());

      std::cout << "T: " << pose->inverse().T.t() << std::endl;
    }

    cv::imshow("Lines", im_lines);
  }

  void processMBT()
  {
    if(img.cols * img.rows == 0)
      return;

    tracker->track(img);
  }


private:

  Camera cam;
  int min_thr{400};
  int max_thr{600};
  int blurr{10};
  int hough_thr{180};
  int dilate{2};
  cv::Rect bottom;

  std::unique_ptr<MBT> tracker;

  image_transport::Subscriber im_sub;
  cv::Mat img;
  rclcpp::TimerBase::SharedPtr process_timer;

  static void displayLines(const Lines &lines, cv::Mat &im, bool b)
  {
    if(lines.empty())
      return;
    auto step{255/lines.size()};
    uint r{0};

    Line::detail(lines.begin(), lines.end(), b ? "vert" : "hor");

    for(auto &line: lines)
    {
      if(b)
        line.display(im, cv::Scalar(255, 0, r));
      else
        line.display(im, cv::Scalar(0, 255, r));
      r = (r + step) % 255;
    }
    std::cout << std::endl;
  }
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcessingNode>());
  rclcpp::shutdown();
}
