#ifndef MESH_BASED_MAPPING_FILE_IO_H_
#define MESH_BASED_MAPPING_FILE_IO_H_

#include <opencv2/highgui/highgui.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/kinematics/FrameTypedefs.hpp> //TODO replace the mappointvector struc
#define MIN_CAM_DISTANCE 0.05

namespace mesh_based_mapping {


	
/**

*/

void saveCSV(std::string filename, std::string timestamp, std::string img_path,
             std::string mesh_path, okvis::kinematics::Transformation &T_WCRef) {
  Eigen::Vector3d p_WS_W = T_WCRef.r();
  Eigen::Quaterniond q_WS = T_WCRef.q();

  std::fstream csvFile(filename, std::fstream::app | std::fstream::out);

  csvFile << timestamp << "," << img_path << "," << mesh_path << "," <<
          std::scientific
          << std::setprecision(18) << p_WS_W[0] << "," << p_WS_W[1] << ","
          << p_WS_W[2] << "," << q_WS.x() << "," << q_WS.y() << ","
          << q_WS.z() << "," << q_WS.w() << std::endl;
  csvFile.close();
}


/**

*/
void projectLandmarks(const double focalU, const double focalV,
                      const double centerU, const double centerV,
                      const double dimU, const double dimV,
                      const double minQuality,
                      okvis::kinematics::Transformation &T_WCRef,
                      const okvis::MapPointVector &matchedLandmarks,
                      VecPoint3f &filteredLandmarks,
                      std::vector<GEOM_FADE2D::Point2> &landmarks2D) {

  okvis::kinematics::Transformation T_CRefW =  T_WCRef.inverse();

  for (unsigned int i = 0 ; i < matchedLandmarks.size() ; i++) {

    if (matchedLandmarks[i].point(3) < 1.0e-8) {
      continue;
    }

    if (matchedLandmarks[i].quality < minQuality) {
      continue;
    }

    Eigen::Vector4d pt_CRef = T_CRefW * matchedLandmarks[i].point;



    pt_CRef(0) = pt_CRef(0) / pt_CRef(3);
    pt_CRef(1) = pt_CRef(1) / pt_CRef(3);
    pt_CRef(2) = pt_CRef(2) / pt_CRef(3);


    if (pt_CRef(2) > MIN_CAM_DISTANCE) {
      double x = ((pt_CRef(0) / pt_CRef(2)) * focalU) + centerU;
      double y = ((pt_CRef(1) / pt_CRef(2)) * focalV) + centerV;

      if (x <= 0 || y <= 0 || x >= dimU || y >= dimV) {
        continue;
      }

      filteredLandmarks.push_back(Eigen::Vector3f(pt_CRef(0), pt_CRef(1),
                                  pt_CRef(2)));
      landmarks2D.push_back(GEOM_FADE2D::Point2(x, y));
      landmarks2D.back().setCustomIndex(filteredLandmarks.size() - 1);
    }
  }
}



void buildMeshDepthMap(const double focalU,
                       const double focalV,
                       const double centerU,
                       const double centerV,
                       const double dimU,
                       const double dimV,
                       const double minQuality,
                       const okvis::MapPointVector &matchedLandmarks,
                       okvis::kinematics::Transformation &T_WCRef,
                       cv::Mat &image,
                       uint64_t timestampNsec,
                       cv::Mat &resultDepthMap,
                       const double laplaceAlpha = 0.1,
                       const unsigned int smoothingIteration = 3,
                       const double maxDelta = 0.2) {

  std::vector<GEOM_FADE2D::Point2> points2D;
  VecPoint3f points3D;

  projectLandmarks(focalU, focalV, centerU, centerV,
                   dimU, dimV, minQuality, T_WCRef,
                   matchedLandmarks, points3D, points2D);

  GEOM_FADE2D::Fade_2D dt;
  dt.insert(points2D);
  std::vector<GEOM_FADE2D::Triangle2 *> vAllTriangles;
  dt.getTrianglePointers(vAllTriangles);

  std::vector<bool> blacklist(vAllTriangles.size(), false);

#ifdef MM_DEBUG
  saveObj("meshProj_before.obj", points3D, points2D, vAllTriangles, blacklist);
#endif


  filteringAndSmoothing(points3D, vAllTriangles, blacklist, laplaceAlpha,
                        smoothingIteration, maxDelta);


#ifdef MM_DEBUG
  saveObj("meshProj_after.obj", points3D, points2D, vAllTriangles, blacklist);
#endif

  //clean the result map
  resultDepthMap.setTo(cv::Vec2f(0, 0));

  //write on the result map
  for (uint i = 0; i < vAllTriangles.size() ; i++) {
    if (blacklist[i]) {
      continue;
    }

    GEOM_FADE2D::Triangle2 *itri = vAllTriangles[i];
    std::vector<cv::Point> tpts(3);
    std::vector<float> zs(3);

    tpts[0]    = cv::Point(itri->getCorner(0)->x(), itri->getCorner(0)->y());
    zs[0]      = (points3D[itri->getCorner(0)->getCustomIndex()])(2);

    tpts[1]    = cv::Point(itri->getCorner(1)->x(), itri->getCorner(1)->y());
    zs[1]      = points3D[itri->getCorner(1)->getCustomIndex()](2);

    tpts[2]    = cv::Point(itri->getCorner(2)->x(), itri->getCorner(2)->y());
    zs[2]      = points3D[itri->getCorner(2)->getCustomIndex()](2);

    rasterTriangle(tpts, zs, itri, resultDepthMap);
  }

#ifdef MM_DEBUG

  for (int i = 0; i < 2; i++) {
    cv::Mat labels;
    cv::extractChannel(resultDepthMap, labels, i);
    // writeCSV("meshProj_nn_" + std::to_string(i) + ".csv", labels);
  }

#endif
  ///3DV stuff
  ///
  VecPoint3f point3D_world_frame;

  for (size_t i = 0; i < points3D.size(); i++) {
    Eigen::Vector4d pt_cam;
    pt_cam(0) = points3D[i](0);
    pt_cam(1) = points3D[i](1);
    pt_cam(2) = points3D[i](2);
    pt_cam(3) = 1.0;
    Eigen::Vector4d pt_world = T_WCRef * pt_cam;
    point3D_world_frame.push_back(Eigen::Vector3f(pt_world(0) / pt_world(3),
                                  pt_world(1) / pt_world(3), pt_world(2) / pt_world(3)));

  }

  //TODO this is wrong, because this should be the undistorted image, but I am using the distorted one.
  cv::Mat img2;
  image.convertTo(img2, CV_32FC1, 1.0 / 255.0);
  cv::insertChannel(img2, resultDepthMap, 1);


  std::string timestamp_string;
  std::stringstream timestamp_ss;

  timestamp_ss << timestampNsec;
  timestamp_string = timestamp_ss.str();

  std::string folder_path =
    "/tmp/okvis_dataset/";//need to create this folder and add inside the folders data/meshes/ and data/images/

  std::string csv_path  = "data.csv";
  std::string mesh_path = "meshes/" + timestamp_string + ".obj";
  std::string img_path  = "images/" + timestamp_string + ".png";

  saveCSV(folder_path + csv_path, timestamp_string, img_path, mesh_path, T_WCRef);
  saveObj(folder_path + "data/" + mesh_path, point3D_world_frame, points2D,
          vAllTriangles, blacklist);
  cv::imwrite(folder_path + "data/" + img_path, image);

  return;
}
}


#endif