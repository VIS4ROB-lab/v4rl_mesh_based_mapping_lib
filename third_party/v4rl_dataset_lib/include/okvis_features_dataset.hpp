#ifndef OKVIS_FEATURES_DATASET_HPP_
#define OKVIS_FEATURES_DATASET_HPP_

#include <glog/logging.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scoped_array.hpp>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

struct OkvisFeaturesEntry
{
    std::string timestamp_string;
    std::string keypoints_filename;
    std::string descriptors_filename;
};

class OkvisFeaturesDataset
{
public:
    std::vector<OkvisFeaturesEntry> data;
    std::map<uint64_t,int> index;

    bool getOkvisFeaturesEntry( uint64_t timestamp, OkvisFeaturesEntry & result ) const
    {
        std::map<uint64_t,int>::const_iterator it = index.find( timestamp );

        if(it != index.end())
        {
            result = data.at( it->second );
            return true;
        }
        return false;
    }

    size_t load( std::string filename )
    {
        datasetPath_ = filename;
        datasetPath_ = datasetPath_.parent_path() ;

        data.clear();

        std::ifstream file( filename );

        if (!file.is_open())
            return 0;

        std::string line;
        while(std::getline(file,line))
        {
           std::vector<std::string> strs;
            boost::split(strs,line,boost::is_any_of(","));

            if(strs.size() != 3)
            {
                std::cerr << "Format problem line #" << line << "#";
                continue;
            }

            OkvisFeaturesEntry current_entry;
            current_entry.timestamp_string = strs[0];
            current_entry.keypoints_filename = strs[1];
            current_entry.descriptors_filename = strs[2];
            data.push_back( current_entry );

            uint64_t timestamp = boost::lexical_cast<uint64_t>(current_entry.timestamp_string);
            index[timestamp] = data.size()-1;
        }

        return data.size();
    }

    bool readKeypointsFile( OkvisFeaturesEntry entry, std::vector<cv::KeyPoint>& keypoints, std::vector<uint64_t>& landmarksId )
    {
        return readKeypointsFile( (datasetPath_ / entry.keypoints_filename).string(), keypoints, landmarksId );
    }

//    void readDescriptorsFile( OkvisFeaturesEntry entry, cv::Mat& descriptors )
//    {
//        readDescriptorsFile( (datasetPath_ / entry.descriptors_filename).string(), descriptors );
//    }

private:

    boost::filesystem::path datasetPath_;

//    void readDescriptorsFile( std::string filename, cv::Mat& descriptors )
//    {
//        cv::FileStorage fs( filename, cv::FileStorage::READ );
//        cv::FileNode descriptorsFileNode = fs["descriptors"];
//        cv::read( descriptorsFileNode, descriptors );
//        fs.release();
//    }

    bool readKeypointsFile( std::string filename, std::vector<cv::KeyPoint>& keypoints, std::vector<uint64_t>& landmarksId )
    {
        keypoints.clear();
        landmarksId.clear();
        std::fstream f( filename, std::ios::in );
        if(!f.is_open())
        {
            std::cout << "####### Error on read the file-" << filename << std::endl;
            return false;
        }

        std::string line;
        while (!f.eof())
        {
            getline(f, line);
            if (!f.eof() && !line.empty())
            {
                uint64_t landmarkId = 0;
                cv::KeyPoint keypoint;
                sscanf(line.c_str(), "%lu, %f, %f, %f, %f, %f, %d, %d", &landmarkId, &keypoint.pt.x, &keypoint.pt.y,
                         &keypoint.size, &keypoint.response, &keypoint.angle, &keypoint.octave, &keypoint.class_id);
                keypoints.push_back( keypoint );
                landmarksId.push_back( landmarkId );
            }
        }

        f.close();
        return true;
    }
};

#endif /* OKVIS_FEATURES_DATASET_HPP_ */
