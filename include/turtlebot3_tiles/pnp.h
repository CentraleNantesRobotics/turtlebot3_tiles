#ifndef PNP_H
#define PNP_H

#include <turtlebot3_tiles/ransac_lines.h>
#include <opencv2/calib3d.hpp>
#include <turtlebot3_tiles/calibration.h>

struct Pose
{
  cv::Mat T, R;

  inline Pose inverse() const
  {
    cv::Mat M;
    cv::Rodrigues(R, M);
    return {-M.t()*T, -R};
  }

};

class Camera
{
public:

  explicit Camera(double size = 0.1)
  {
    this->size = size;    

    K = (cv::Mat1d(3, 3) << camera::px, 0, camera::u0, 0, camera::py, camera::v0, 0, 0, 1);
    D = cv::Mat1d(camera::dist());

    frame.reserve(4);
    frame.emplace_back(0,0,0);
    frame.emplace_back(size/2,0,0);
    frame.emplace_back(0,size/2,0);
    frame.emplace_back(0,0,size/2);
  }

  std::optional<Pose> computePose(const Lines &hor, const Lines &vert) const
  {
    if(hor.size() < 3 || vert.size() < 3)
      return std::nullopt;

    const auto n{hor.size()*vert.size()};

    Points p2d;
    std::vector<cv::Point3d> p3d;
    p2d.reserve(n);
    p3d.reserve(n);

    double x{};
    for(auto &h: hor)
    {
      double y{};
      for(auto &v: vert)
      {

        if(const auto p{h.intersect(v)}; p.has_value())
        {
          // 2D point
          p2d.push_back(p.value());

          // corresponding 3D point
          p3d.emplace_back(x,y,0.);
        }

        y += size;
      }
      x += size;
    }

    if(p2d.size() < 8)
      return std::nullopt;

    cv::Mat R, T;
    cv::solvePnPRansac(p3d,p2d,K,D,R,T);
    cv::solvePnPRefineVVS(p3d, p2d, K, D, R, T);

    return {{T,R}};
  }


  void showPose(cv::Mat &im, const Pose &pose) const
  {

    // project frame points
    std::vector<cv::Point2d> points;
    cv::projectPoints(frame, pose.R, pose.T, K, D, points);

    cv::arrowedLine(im, points[0], points[1], {0,0,255}, 3);
    cv::arrowedLine(im, points[0], points[2], {0,255,0}, 3);
    cv::arrowedLine(im, points[0], points[3], {255,0,0}, 3);

  }


private:
  cv::Mat1d K;
  cv::Mat1d D;

  std::vector<cv::Point3d> frame;

  double size = 0.1;
};




#endif // PNP_H
