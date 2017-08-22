#ifndef OPTIMIZATION_WINDOW_DATASET_HPP__
#define OPTIMIZATION_WINDOW_DATASET_HPP__

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scoped_array.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <map>
#include <kindr/minimal/quat-transformation.h>

using namespace std;

struct OptimizationWindowEntry
{
    std::string timestamp_string;
    kindr::minimal::QuatTransformation t_ws;
    std::string landmarks_filename;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct OkvisLandmark
{
    //uint64_t id;
    Eigen::Vector3d pos;
    double quality;
    double distance;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::vector<OkvisLandmark, Eigen::aligned_allocator<OkvisLandmark> > OkvisLandmarkVector;
typedef std::map<uint64_t, OkvisLandmark, std::less<uint64_t>, Eigen::aligned_allocator<OkvisLandmark> > OkvisLandmarkMap;


class OptimizationWindowDataset
{
public:

    std::vector<OptimizationWindowEntry,Eigen::aligned_allocator<OptimizationWindowEntry>> data;
    std::map<uint64_t,int> index;

    bool getOptimizationWindowEntry( uint64_t timestamp, OptimizationWindowEntry & result ) const
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

            if(strs.size() != 12)
            {
                LOG(ERROR) << "Format problem line #" << line << "#";
                continue;
            }

            OptimizationWindowEntry current_entry;
            current_entry.timestamp_string   = strs[0];

            Eigen::Vector3d p_WS(boost::lexical_cast<double>(strs[1]),
                                 boost::lexical_cast<double>(strs[2]),
                                 boost::lexical_cast<double>(strs[3]));

            Eigen::Quaterniond q_WS;
            q_WS.x() = boost::lexical_cast<double>(strs[4]);
            q_WS.y() = boost::lexical_cast<double>(strs[5]);
            q_WS.z() = boost::lexical_cast<double>(strs[6]);
            q_WS.w() = boost::lexical_cast<double>(strs[7]);

            current_entry.t_ws = kindr::minimal::QuatTransformation(p_WS,q_WS);

            current_entry.landmarks_filename = strs[9];
            data.push_back( current_entry );


            uint64_t timestamp = boost::lexical_cast<uint64_t>(current_entry.timestamp_string);
            index[timestamp] = data.size()-1;
        }

        return data.size();
    }

    bool readLandmarksFile( OptimizationWindowEntry entry, OkvisLandmarkMap& landmarks )
    {
        return readLandmarksFile( (datasetPath_ / entry.landmarks_filename).string(), landmarks );
    }

    void readLandmarksPositions( OptimizationWindowEntry entry, std::map<uint64_t,std::vector<double> >& landmarks )
    {
        readLandmarksPositions( (datasetPath_ / entry.landmarks_filename).string(), landmarks );
    }

private:

    boost::filesystem::path datasetPath_;

    bool readLandmarksFile( std::string filename, OkvisLandmarkMap& landmarks )
    {
        landmarks.clear();

        fstream f( filename, ios::in );
        if(! f.is_open() )
            return false;

        std::string line;
        while (!f.eof())
        {
            getline(f, line);
            if (!f.eof() && !line.empty())
            {
                OkvisLandmark landmark;
                uint64_t landmarkId;
                double x, y, z;
                sscanf( line.c_str(), "%lu, %lf, %lf, %lf, %lf, %lf", &landmarkId, &x, &y, &z, &landmark.quality, &landmark.distance );
                landmark.pos = Eigen::Vector3d(x,y,z);
                landmarks.insert( std::pair<uint64_t,OkvisLandmark>(landmarkId,landmark) );
            }
        }

        f.close();
        return true;
    }

    void readLandmarksPositions( std::string filename, std::map<uint64_t,std::vector<double> >& landmarks )
    {
        fstream f( filename, ios::in );
        assert( f.is_open() );

        std::string line;
        while (!f.eof())
        {
            getline(f, line);
            if (!f.eof() && !line.empty())
            {
                uint64_t landmarkId;
                double x, y, z;
                sscanf( line.c_str(), "%lu, %lf, %lf, %lf, %*f, %*f", &landmarkId, &x, &y, &z );

                std::vector<double> landmarkPosition;
                landmarkPosition.push_back( x );
                landmarkPosition.push_back( y );
                landmarkPosition.push_back( z );

                landmarks.insert( std::pair<uint64_t,std::vector<double> >(landmarkId,landmarkPosition) );
            }
        }

        f.close();
    }

};

#endif
