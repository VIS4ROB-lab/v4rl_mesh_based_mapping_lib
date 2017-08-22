
#include<mesh_based_mapping/mesh_based_mapping.hpp>
#include<mesh_based_mapping/utils/file_io.hpp>
#include<mesh_based_mapping/utils/triangle_rasterization.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <okvis_features_dataset.hpp>
#include <optimization_window_dataset.hpp>

#include "okvis_data_reader.hpp"


const double kMinQualityLandmark = 0.000001;
const double kMinDistanceFromCamera = 0.005;//meters


void ConvertSelectOkvisLandmarks(const FeatureLandmarkFrameData &frame_data,
                                 mesh_based_mapping::VecPoint2f &keypoints,
                                 mesh_based_mapping::VecPoint3f &landmarks) {
  landmarks.clear();
  keypoints.clear();

  for (int i = 0; i < frame_data.keypoints.size(); i++) {
    if (frame_data.landmarks.count(frame_data.landmarks_id[i])) {
      const OkvisLandmark &okvis_landmark = frame_data.landmarks.at(
                                              frame_data.landmarks_id[i]);

//      if ((okvis_landmark.quality > kMinQualityLandmark) &&
//          okvis_landmark.pos[2] > kMinDistanceFromCamera) {
      landmarks.push_back(okvis_landmark.pos.cast<float>());
      keypoints.push_back(Eigen::Vector2f(frame_data.keypoints[i].pt.x,
                                          frame_data.keypoints[i].pt.y));
//      }
    }
  }


}

int main(int, char **) {

  std::string testDatasetFeatures  =
    "/home/lucas/data/icra18/cab-ground/okvis_data/brisk_keypoints/data.csv";
  std::string testDatasetOptWindow =
    "/home/lucas/data/icra18/cab-ground/okvis_data/optimization_window/data.csv";

  OkvisDataReader refrOkvisDataReader ;

  if (!refrOkvisDataReader.Init(testDatasetFeatures, testDatasetOptWindow)) {
    std::cerr << "could not find the files";
    return 1;
  }

  mesh_based_mapping::VecPoint3f in_landmarks_3d;//in camera frame
  mesh_based_mapping::VecPoint2f in_landmarks_2d;//in image coords
  const mesh_based_mapping::VecPoint3f *out_landmarks_3d;
  const mesh_based_mapping::VecTriangle *out_triangles;
  const mesh_based_mapping::VecPoint2f *out_landmarks_2d;

  mesh_based_mapping::MeshMapper mapper(0.3, 10);

  FeatureLandmarkFrameData okvis_data;
  cv::Mat result_map = cv::Mat::zeros(cv::Size(752,480), CV_32FC1);
  cv::Mat c_result_map = cv::Mat::zeros(cv::Size(752,480), CV_8UC3);
  cv::Mat g_result_map = cv::Mat::zeros(cv::Size(752,480), CV_8UC1);

#if 1

  while (refrOkvisDataReader.GetNextSample(okvis_data)) {

    ConvertSelectOkvisLandmarks(okvis_data, in_landmarks_2d, in_landmarks_3d);

    if (in_landmarks_2d.size() < 30) {
      continue;
    }

    mapper.SetPoints(in_landmarks_2d, in_landmarks_3d);
    // mapper.SetPoints(470,470,376,247,752,480, in_landmarks_3d);
    mapper.ComputeMesh();
    mapper.GetMesh(out_landmarks_3d, out_landmarks_2d, out_triangles);
//    std::cout << count++ << " " << okvis_data.timestamp << " " <<
//              in_landmarks_3d.size() << " " << out_triangles->size() << std::endl;



    //if (best < out_triangles->size()) {

//    mesh_based_mapping::saveObj("/tmp/mesh_before8.obj", in_landmarks_3d,
//                                *out_triangles);//
//    mesh_based_mapping::saveObj("/tmp/mesh_after8.obj", *out_landmarks_3d,
//                              *out_triangles);



    mesh_based_mapping::RasterMesh(*out_landmarks_2d, *out_landmarks_3d,
                                   *out_triangles, result_map);

    double min;
    double max;
    cv::minMaxIdx(result_map, &min, &max);

    result_map.convertTo(g_result_map,CV_8UC1, 255 / (max-min), -min);

    cv::applyColorMap(g_result_map,c_result_map,cv::COLORMAP_PARULA);
    cv::imshow("depth map", c_result_map);

    if ((char)27 == cv::waitKey(100)) {
      break;
    }

  }

#else

  refrOkvisDataReader.GetSample(uint64_t(1456317951390953500), okvis_data);
  ConvertSelectOkvisLandmarks(okvis_data, in_landmarks_2d, in_landmarks_3d);
//  Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[",
//                           "]");

//  std::cout << okvis_data.t_ws.getPosition().format(HeavyFmt) <<  " - " <<
//            okvis_data.t_ws.getRotation().vector().format(HeavyFmt) <<  " - " <<
//            okvis_data.t_ws.getEigenQuaternion().coeffs().format(HeavyFmt);

//    std::cout << count++ <<  " " << okvis_data.landmarks.size() <<  " " <<
//              okvis_data.keypoints.size() << " - " << in_landmarks_2d.size() << " " <<
//              in_landmarks_3d.size() << std::endl;


  mapper.SetPoints(in_landmarks_2d, in_landmarks_3d);
  // mapper.SetPoints(470,470,376,247,752,480, in_landmarks_3d);
  mapper.ComputeMesh();
  mapper.GetMesh(out_landmarks_3d, out_landmarks_2d, out_triangles);
  std::cout << okvis_data.timestamp << " " <<
            in_landmarks_3d.size() << " " << out_triangles->size() << std::endl;

  mesh_based_mapping::saveObj("/tmp/mesh_before4.obj", in_landmarks_3d,
                              *out_triangles);
  mesh_based_mapping::saveObj("/tmp/mesh_after4.obj", *out_landmarks_3d,
                              *out_triangles);

#endif



  return 0;
}
