
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include<mesh_based_mapping/mesh_based_mapping.hpp>
#include<mesh_based_mapping/utils/file_io.hpp>
#include<mesh_based_mapping/utils/triangle_rasterization.hpp>

#include <okvis_features_dataset.hpp>
#include <optimization_window_dataset.hpp>
#include <string>
#include "okvis_data_reader.hpp"

const double kMinQualityLandmark = 0.000001;
const double kMinDistanceFromCamera = 0.005;//meters


//void imagesc(const std::string &winname, cv::Mat image, double min = 0,
//             double max = 0,int colormap = cv::COLORMAP_JET) {
//  cv::Mat g_result_map, c_result_map;

//  if (min == max) {
//    cv::minMaxIdx(image, &min, &max);

//  }

//  double scale = 255.0 / std::abs(max - min);
//  std::cout << winname << " min:" << min << " max:" << max << std::endl;
//  image.convertTo(g_result_map, CV_8UC1, scale, -scale * min);

//  cv::applyColorMap(g_result_map, c_result_map, colormap);
//  cv::imshow(winname, c_result_map);
//}


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

//void ReadCsvDepth(uint64 timestamp, cv::Mat &image) {
//  const std::string folder = "/home/lucas/tmp/lagout_slow/vi_dataset/depth_cam0/";
//  std::string filename = folder + std::to_string(timestamp) +
//                         ".yml"; //"/home/lucas/tmp/lagout_slow/blender_result_2017-08-28-21-03-27/depth/19230000001.yml";
//  cv::FileStorage fs2(filename, cv::FileStorage::READ);
//  fs2["one_matrix"] >> image;

//  if (image.rows) {
//    imagesc("gt map", image, 4, 9,cv::COLORMAP_BONE);
//  }
//}

int main(int, char **)
{
    std::cout << "OKVIS Dataset Mesh Generator" << std::endl;

    std::string datasetFeatures  = "/home/fabiola/datasets/visensor_simulator/lagout_45/okvis_dataset/brisk_keypoints/data.csv";
    std::string datasetOptWindow = "/home/fabiola/datasets/visensor_simulator/lagout_45/okvis_dataset/optimization_window/data.csv";
    std::string meshesFolder = "/home/fabiola/datasets/visensor_simulator/meshes_lagout_45/";

    OkvisDataReader okvisDataReader;
    if (!okvisDataReader.Init(datasetFeatures, datasetOptWindow))
    {
        std::cerr << "could not find the files";
        return 1;
    }

  mesh_based_mapping::VecPoint3f in_landmarks_3d;//in camera frame
  mesh_based_mapping::VecPoint2f in_landmarks_2d;//in image coords
  const mesh_based_mapping::VecPoint3f *out_landmarks_3d;
  const mesh_based_mapping::VecTriangle *out_triangles;
  const mesh_based_mapping::VecPoint2f *out_landmarks_2d;

  mesh_based_mapping::MeshMapper mapper(0.3, 4, 0.3);

  FeatureLandmarkFrameData okvis_data;
  cv::Mat result_map = cv::Mat::zeros(cv::Size(752, 480), CV_32FC1);
  //cv::Mat gt_map;


#if 1

    while (okvisDataReader.GetNextSample(okvis_data))
    {
//        std::cout << okvis_data.timestamp << std::endl;
//        continue;

        ConvertSelectOkvisLandmarks(okvis_data, in_landmarks_2d, in_landmarks_3d);

        if (in_landmarks_2d.size() < 30)
            continue;

        //mapper.SetPoints(in_landmarks_2d, in_landmarks_3d);
        if (mapper.SetPoints(455, 455, 376.5, 240.5, 752, 480, in_landmarks_3d) > 5)
        {
            mapper.ComputeMesh();
            mapper.GetMesh(out_landmarks_3d, out_landmarks_2d, out_triangles);

          //    std::cout << count++ << " " << okvis_data.timestamp << " " <<
          //              in_landmarks_3d.size() << " " << out_triangles->size() << std::endl;



          //if (best < out_triangles->size()) {

          //    mesh_based_mapping::saveObj("/tmp/mesh_before8.obj", in_landmarks_3d,
          //                                *out_triangles);//
          //    mesh_based_mapping::saveObj("/tmp/mesh_after8.obj", *out_landmarks_3d,
          //                              *out_triangles);



          mesh_based_mapping::RasterMesh( *out_landmarks_2d, *out_landmarks_3d, *out_triangles, result_map );

          std::string meshFilename = meshesFolder;
          meshFilename += boost::lexical_cast<std::string>(okvis_data.timestamp);
          meshFilename += ".yml";

          cv::FileStorage fs(meshFilename, cv::FileStorage::WRITE);
          //fs.open(meshFilename, cv::FileStorage::WRITE);
          fs << "mesh" << result_map;


          //cv::Mat gt_map_float;
          //gt_map.convertTo(gt_map_float, CV_32FC1);
          //cv::Mat diff = result_map - gt_map_float;
          //imagesc("triang", result_map, 4, 9,cv::COLORMAP_BONE);
          //imagesc("diff", diff, -1, 1);
          //imagesc("abs-diff", cv::abs(diff), 0, 1,cv::COLORMAP_AUTUMN);
        }
        else
        {
            std::cout << "not enough points in the field of view" << std::endl;
        }

        if ((char)27 == cv::waitKey(100))
        {
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
