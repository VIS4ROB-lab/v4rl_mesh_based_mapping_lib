#include "okvis_data_reader.hpp"
#include <opencv2/opencv.hpp>

OkvisDataReader::OkvisDataReader() {
}

OkvisDataReader::~OkvisDataReader() {

}

bool OkvisDataReader::GetNextSample(FeatureLandmarkFrameData &data) {

    uint64_t timestamp;

  do {
    if (next_sample_id_ >= timestamps_.size()) {
      return false;
    }

    timestamp = timestamps_[next_sample_id_];
    next_sample_id_++;
  } while (!GetSample(timestamp, data));

  return true;
}

bool OkvisDataReader::GetSample(uint64_t timestamp,
                                FeatureLandmarkFrameData &data) {

  OkvisFeaturesEntry okvisFeaturesEntry;

  if (!features_dataset_.getOkvisFeaturesEntry(timestamp, okvisFeaturesEntry)) {
    return false;
  }

  if (!features_dataset_.readKeypointsFile(okvisFeaturesEntry, data.keypoints,
      data.landmarks_id)) {
    return false;
  }

  OptimizationWindowEntry optEntry;

  if (!opt_window_dataset_.getOptimizationWindowEntry(timestamp, optEntry)) {
    return false;
  }

  if (!opt_window_dataset_.readLandmarksFile(optEntry, data.landmarks)) {
    return false;
  }

  data.timestamp = timestamp;

  data.t_ws = optEntry.t_ws;

  kindr::minimal::QuatTransformation T_WC;
  T_WC = data.t_ws * t_sc;
  kindr::minimal::QuatTransformation T_CW = T_WC.inverse();

  //std::cout << T_CW << std::endl;

  //convert from world to camera frame
  for (auto it = data.landmarks.begin(); it != data.landmarks.end(); ++it) {
    // std::cout << 0.25<<"," << it->second.pos[0] <<","<<it->second.pos[1] <<","<<it->second.pos[2] << std::endl;
    it->second.pos = T_CW.transform(it->second.pos);
    // std::cout << 1.0<<"," << it->second.pos[0] <<","<<it->second.pos[1] <<","<<it->second.pos[2] << std::endl;
  }
  return true;
}

bool OkvisDataReader::Init(std::string features_dataset_filename,
                           std::string opt_window_dataset_filename)   {
  next_sample_id_ = 0;

  if (features_dataset_.load(features_dataset_filename) == 0) {
    return false;
  }

  if (opt_window_dataset_.load(opt_window_dataset_filename) == 0) {
    return false;
  }

  Eigen::Matrix4d T_SC_Eigen; //TODO(@weblucas) read from file
 //   T_SC_Eigen = Eigen::MatrixXd::Identity(4,4);
//  T_SC_Eigen << 0.9997754002442455   , -0.021161371053313276,
//             -0.0011599317232830833, -0.03742703361193287  ,
//             0.021167568626536626 ,  0.999760145165724   ,  0.00562015806284527  ,
//             0.0059817697251900205,
//             0.0010407232579056848, -0.005643448711071745,  0.9999835340553093   ,
//             0.0005720705107382817,
//             0.0,                    0.0,                   0.0,                    1.0;

      T_SC_Eigen << 0, 0, 1, 0,
              -1, 0, 0, 0,
              0,-1, 0, 0,
              0, 0, 0, 1;



  t_sc = kindr::minimal::QuatTransformation(T_SC_Eigen);

  InitTimestamps();
  return true;
}

void OkvisDataReader::InitTimestamps() {
  for (auto it = features_dataset_.index.begin();
       it != features_dataset_.index.end(); ++it) {
    timestamps_.push_back(it->first);
  }
}
