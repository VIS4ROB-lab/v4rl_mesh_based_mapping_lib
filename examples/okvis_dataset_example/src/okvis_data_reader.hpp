#ifndef OKVIS_DATA_READER_MESH_MAPPING_H_
#define OKVIS_DATA_READER_MESH_MAPPING_H_

#include <okvis_features_dataset.hpp>
#include <optimization_window_dataset.hpp>
#include <string>

struct FeatureLandmarkFrameData
{
    uint64_t timestamp;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<uint64_t> landmarks_id;
    OkvisLandmarkMap landmarks;
    kindr::minimal::QuatTransformation t_ws;
};

class OkvisDataReader
{
public:
    OkvisDataReader();
    ~OkvisDataReader();

    bool GetNextSample( FeatureLandmarkFrameData& data );
    bool GetSample(uint64_t timestamp, FeatureLandmarkFrameData& data );
    bool Init(std::string features_dataset_filename, std::string opt_window_dataset_filename );

private:
    void InitTimestamps();
    OkvisFeaturesDataset features_dataset_;
    OptimizationWindowDataset opt_window_dataset_;
    uint64_t next_sample_id_;
    std::vector<uint64_t> timestamps_;
    kindr::minimal::QuatTransformation t_sc; //TODO(@weblucas) read from file
};

#endif
