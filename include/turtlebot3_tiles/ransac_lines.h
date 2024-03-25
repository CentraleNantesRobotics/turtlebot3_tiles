#ifndef RANSAC_LINES_H
#define RANSAC_LINES_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <optional>
#include <random>

using Point = cv::Point2d;
using Points = std::vector<Point>;
using Indices = std::vector<uint>;

struct LineDist
{
  double d;
  uint i1, i2;
  inline bool operator<(const LineDist &other) const
  {
    return d < other.d;
  }
};

struct Line
{
  using LineIt = std::vector<Line>::iterator;
  using LineCstIt = std::vector<Line>::const_iterator;

  double a, b, c;
  inline static int yOffset{};
  Line(const cv::Vec2f &rt = {})
    : a(cos(rt[1])), b(sin(rt[1])), c(-rt[0]) {}

  inline void normalize()
  {
    auto scale{1.f/sqrt(a*a + b*b)};
    if(c < 0)
      scale *= -1;
    a *= scale;
    b *= scale;
    c *= scale;
  }

  void fit(const Points &points, const Indices &idx)
  {
    float x{}, y{};
    float xx{}, xy{}, yy{};
    for(auto i: idx)
    {
      const auto &p{points[i]};
      x += p.x;
      y += p.y;
      xx += p.x*p.x;
      yy += p.y*p.y;
      xy += p.x*p.y;
    }

    const auto xMean{x/idx.size()};
    const auto yMean{y/idx.size()};
    const auto xDenom{xx - x*xMean};
    const auto yDenom{yy - y*yMean};

    if(fabs(xDenom) > fabs(yDenom))
    {
      // y = m.x + v
      b = -1;
      a = (xy - x*yMean) / xDenom;
      c = yMean - a*xMean;
    }
    else
    {
      // x = m.y + v
      a = -1;
      b = (xy - y*xMean) / yDenom;
      c = xMean - b*yMean;
    }
    normalize();
  }

  Indices fit(const Points &points, uint i1, uint i2, double max_dist)
  {
    // initial fit
    // a.x + b.y + c = 0
    a = points[i1].y - points[i2].y;
    b = points[i2].x - points[i1].x;
    c = points[i1].x*points[i2].y - points[i2].x*points[i1].y;
    normalize();

    // loop until max inliers
    Indices idx, best;

    while(true)
    {
      idx.clear();
      for(uint p = points.size(); p != 0; --p)
      {
        if(distTo(points[p-1]) < max_dist)
          idx.push_back(p-1);
      }
      if(best.size() >= idx.size())
        break;
      fit(points, idx);
      best = idx;
    }
    return best;
  }

  inline float angleDist(const Line &other) const
  {
    return fabs(a*other.b - b*other.a);
  }

  inline float angle() const
  {
    return atan2(b, a);
  }

  inline static bool compareOffset(const Line &l1, const Line &l2)
  {
    return l1.c > l2.c;
  }
  inline static bool compareAngle(const Line &l1, const Line &l2)
  {
    return l1.angle() > l2.angle();
  }

  inline bool operator==(const Line &other) const
  {
    if(angleDist(other) > 1e-1)
      return false;
    return fabs(c - other.c) < 30;
  }

  inline float distTo(const Point &p) const
  {
    return fabs(a*p.x + b*p.y + c);
  }

  void display(cv::Mat &im, cv::Scalar color = {0,0,255}) const
  {
    cv::Point pt1, pt2;
    const auto x0{-a*c}, y0{-b*c + yOffset};
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
    cv::line(im, pt1, pt2, color, 2, cv::LINE_AA);
  }

  inline std::optional<Point> intersect(const Line &other) const
  {
    const auto z{a*other.b - b*other.a};
    if(fabs(z) < 1e-3)
      return{};
    return {{(b*other.c - other.b*c)/z,
            (other.a*c - a*other.c)/z + yOffset}};
  }

  inline static void detail(LineCstIt first, LineCstIt last, std::string name)
  {
    std::cout << name << ": ";
    for(auto line = first; line != last; ++line)
      std::cout << '(' << line->angle() << ", " << line->c << ") ";
    std::cout << std::endl;
  }

  inline static void sort(LineIt first, LineIt last)
  {
    const auto angle{first->angle()};
    if(std::all_of(first+1, last, [angle](const auto &line){return line.angle()*angle > 0;}))
      std::sort(first, last, Line::compareOffset);
    else
      std::sort(first, last, Line::compareAngle);
  }


  inline static LineIt partition(std::vector<Line> &lines)
  {
    const auto ori{lines};

    std::vector<LineDist> distances;
    distances.reserve(lines.size()*(lines.size()-1)/2);
    for(uint i1{0}; i1 < lines.size()-1; ++i1)
    {
      for(uint i2{i1+1}; i2 < lines.size(); ++i2)
        distances.push_back({lines[i1].angleDist(lines[i2]), i1, i2});
    }

    std::sort(distances.begin(), distances.end());

    // all similar lines
    if(distances.back().d < 0.2)
      return lines.end();

    // pick hor / vert
    auto ih = distances.back().i1;
    auto iv = distances.back().i2;

    if(fabs(lines[iv].angle()) < fabs(lines[ih].angle()))
      std::swap(iv, ih);

    if(lines.size() == 2)
    {
      lines = {ori[iv], ori[ih]};
      return lines.begin()+1;
    }

    // consume all indices
    std::vector<uint> done{ih, iv};
    // insertion points
    //uint vert{1}, hor(lines.size()-2);
    auto vert{lines.begin()+1};
    auto hor{lines.end()-2};

  //  while(done.size() < lines.size())
    {
    }

    // nearest neighboor partitionning, put vertical at beginning
    auto cur{vert+1};

    while(cur < hor)
    {
      std::cout << "vert: " << std::distance(lines.begin(), vert+1)
                << " / hor: " << std::distance(hor, lines.end()) << std::endl;

      detail(lines.begin(), vert+1, "vert");
      detail(hor, lines.end(), "hor");
      std::cout << "Examining " << cur->angle() << std::endl;

      const auto cmp = [cur](const Line &h1, const Line &h2)
      {
        return cur->angleDist(h1) < cur->angleDist(h2);
      };

      // get min distance to vert
      const auto v_dist = std::min_element(lines.begin(), vert+1, cmp)->angleDist(*cur);

      // get min distance to hor
      const auto h_dist = std::min_element(hor, lines.end(), cmp)->angleDist(*cur);

      std::cout << "  distances: to vert = " << v_dist << ", to hor = " << h_dist << std::endl;

      if(v_dist < h_dist)
        vert++;
      else
        std::iter_swap(cur, --hor);
      cur++;
    }
    return hor;
  }
};

using Lines = std::vector<Line>;

Indices fit(const Points &points, uint min_inliers)
{
  if(points.size() < min_inliers)
    return {};

  constexpr double max_dist{2.};

  constexpr int s = 2;//number of points required by the model
  const float r{((float)min_inliers)/points.size()};
  const float e{1-r};  //percentage of outliers
  const float p{r*r};//chance of hitting a valid pair
  const auto N = 400; //(uint)ceilf(log(1-p)/log(1 - pow(1-e, s)));//number of independent trials

  static Indices idx;
  static std::default_random_engine dev;
  std::uniform_int_distribution<uint> choice(0, points.size()-1);

  Line line;
  for (int i = 0; i < N; i++)
  {
    // pick two points and fit
    const auto i1{choice(dev)};
    const auto i2{choice(dev)};

    idx = line.fit(points, i1, i2, max_dist);

    if(idx.size() >= min_inliers)
      return idx;
  }
  return {};
}

void removeSimilar(std::vector<Line> &lines)
{
  if(lines.empty())
    return;

  auto cur{lines.begin()};
  auto last{lines.end()};

  while(cur < last)
  {
    last = std::remove(cur+1, last, *cur);
    cur++;
  }

  std::cout << "Kept " << std::distance(lines.begin(), last) << " out of " << lines.size() << std::endl;
  lines.erase(last, lines.end());
}


std::vector<Line> detectRansac(const cv::Mat &edges, uint min_inliers)
{
  std::vector<Line> lines;

  // extract list of points
  Points points;
  for(uint x = 0; x < edges.cols; ++x)
  {
    for(uint y = 0; y < edges.rows; ++y)
    {
      if(edges.at<uchar>(y,x))
        points.push_back(Point(x,y));
    }
  }

  while(true)
  {
    const auto idx{fit(points, min_inliers)};
    if(idx.size() <= min_inliers)
      break;

    // we got one, register it
    auto &line{lines.emplace_back()};
    //line.fit(points, idx);
    line.fit(points, idx);
    //return lines;

    // remove these points
    auto valid{points.size()-1};
    for(auto &p: idx)
    {
      std::swap(points[p], points[valid]);
      valid--;
    }
    points.resize(points.size()-idx.size());
  }

  return lines;
}


std::vector<Line> detectHough(const cv::Mat &edges, uint hough_thr)
{
  static std::vector<cv::Vec2f> rt;
  cv::HoughLines(edges, rt, 0.5, 0.05, hough_thr);

  std::vector<Line> lines;
  lines.reserve(rt.size());
  std::copy(rt.begin(), rt.end(), std::back_inserter(lines));

  return lines;
}




#endif // RANSAC_LINES_H
