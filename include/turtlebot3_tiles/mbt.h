#ifndef MBT_H
#define MBT_H

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <turtlebot3_tiles/calibration.h>

#include <visp/vpMbGenericTracker.h>
#include <visp/vpDisplayX.h>
#include <geometry_msgs/msg/pose.hpp>

struct MBT
{
  explicit MBT()
  {
    cam.initProjWithKannalaBrandtDistortion(camera::px, camera::py, camera::u0, camera::v0,
                                            camera::dist());

    me.setMaskSize(15);
    me.setMaskNumber(360);
    me.setRange(20);
    me.setThreshold(100);
    me.setMu1(0.2);
    me.setMu2(0.8);
    me.setSampleStep(4);

    const auto model = ament_index_cpp::get_package_share_directory("turtlebot3_tiles")
        + "/model/tiles.cao";
    tracker.loadModel(model);

    tracker.setCameraParameters(cam);
    tracker.setMovingEdge(me);
    tracker.setDisplayFeatures(true);
    tracker.setProjectionErrorComputation(true);
    tracker.setAngleAppear(vpMath::rad(10));
    tracker.setAngleDisappear(vpMath::rad(20));
    tracker.setNearClippingDistance(0.01);
    tracker.setFarClippingDistance(100.0);
    tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);

    cMo.buildFrom(-0.02, 0.32, 0.496, 1.23278, 1.23278, -1.38949);

    std::cout << "Tracking " << tracker.getNbPolygon() << " polygons" << std::endl;
  }

  void reset(const geometry_msgs::msg::Pose &pose)
  {
    const auto &t{pose.position};
    cMo.insert(vpTranslationVector{t.x, t.y, t.z});
    const auto &qm{pose.orientation};
    cMo.insert(vpQuaternionVector{qm.x, qm.y, qm.z, qm.w});

    const auto se3{vpPoseVector(cMo)};
    std::cout << "Reset to ";
    for(int i = 0; i < 6; ++i)
      std::cout << se3[i] << ", ";
    std::cout << std::endl;
  }

  void track(const cv::Mat &im)
  {
    vpImageConvert::convert(im, I);
    vpDisplay::display(I);

    static auto first{true};
    if(first)
    {
      display.init (I, 100, 100,"Pose" );
      first = false;
      tracker.initFromPose(I, cMo);
    }


    //tracker.track(I);
    //tracker.getPose(cMo);

    //tracker.getNbPoints()

    std::vector<vpImagePoint> points;
    for(uint pol = 0; pol < tracker.getNbPolygon(); ++pol)
    {
      const auto poly = tracker.getPolygon(pol);
      for(uint p = 0; p < 4; ++p)
      {
        auto point{poly->getPoint(p)};
        point.track(cMo);
        auto &uv{points.emplace_back()};
        vpMeterPixelConversion::convertPointWithDistortion(cam, point.get_x(), point.get_y(), uv);
        vpDisplay::displayCircle(I, uv, 5, vpColor::cyan, true);
      }
    }

    tracker.display(I, cMo, cam, vpColor::red, 3);
    display.displayFrame(I, cMo, cam, 0.1);    



    std::cout << "Displaying tiles" << std::endl;

    //vpDisplay::display ( I );
    vpDisplay::flush ( I );


  }

  vpMbEdgeTracker tracker;
  vpMe me;
  vpCameraParameters cam;
  vpHomogeneousMatrix cMo;
  vpImage<vpRGBa> I;
  vpDisplayX display;


};

#endif // MBT_H
