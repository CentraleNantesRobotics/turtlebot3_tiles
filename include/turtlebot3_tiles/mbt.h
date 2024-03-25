#ifndef MBT_H
#define MBT_H

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <turtlebot3_tiles/calibration.h>

#include <visp/vpMbGenericTracker.h>
#include <visp/vpDisplayX.h>

struct MBT
{
  explicit MBT()
  {
    cam.initProjWithKannalaBrandtDistortion(camera::px, camera::py, camera::u0, camera::v0,
                                            camera::dist());

    me.setMaskSize(5);
    me.setMaskNumber(180);
    me.setRange(7);
    me.setThreshold(5000);
    me.setMu1(0.5);
    me.setMu2(0.5);
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
    tracker.setNearClippingDistance(0.1);
    tracker.setFarClippingDistance(100.0);
    tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);

    cMo.buildFrom(0.01, 0.17, 0.5, 1.6, 0., 0.1);

    std::cout << "Tracking " << tracker.getNbPolygon() << " polygons" << std::endl;
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
      //tracker.initFromPose(I, cMo);

    }


    tracker.track(I);
    tracker.getPose(cMo);

    tracker.display(I, cMo, cam, vpColor::red, 3);
    display.displayFrame(I, cMo, cam, 0.1);




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
