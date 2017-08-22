#ifndef INCLUDE_ASL_DATASET_HELPER_H_
#define INCLUDE_ASL_DATASET_HELPER_H_


#include <map>
#include <string>
#include <fstream>
#include <functional>
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>

//TODO change the variable style to lalala_lelelele_lilili
class ASLData{
public:
    ASLData(){}
    virtual ~ASLData(){}
    std::string datasetUniqueID_; //TODO add some kind of unique id
};

class CameraData : public ASLData
{
private:
    struct CameraDataMeasurement
    {
        unsigned long long int timestamp;
        std::string filename;//relative to the folder data
    };

public:
    const CameraDataMeasurement& get(size_t i) const
    {
        return measurement_.at(i);
    }

    size_t n() const
    {
        return measurement_.size();
    }

    const std::string getImagePathRoot() const
    {
        return imagePathRoot_.string();
    }

    const std::string getImagePath(size_t i) const
    {
        return (imagePathRoot_ / measurement_[i].filename).string();
    }

    const std::vector<CameraDataMeasurement>& getMeasurement()
    {
        return measurement_;
    }


private:
    std::vector<CameraDataMeasurement> measurement_;
    boost::filesystem::path dataSourceRoot_;
    boost::filesystem::path imagePathRoot_;

    CameraData() {}
    friend class ASLDatasetHelper;
    static ASLData* Create(std::string datasetFolder)
    {
        boost::filesystem::path dPath(datasetFolder);
        dPath /= "data.csv";
        std::ifstream file ( dPath.string() );
        if(file.is_open())
        {
            CameraData* dataSource = new CameraData();
            std::string line;
            std::getline(file,line);//drop first line
            while(std::getline(file,line))
            {
                CameraDataMeasurement temp;
                line = line.substr( 0, line.find_first_of("#") );
                char buffer[256];
                if(sscanf(line.c_str(),"%llu,%s", &(temp.timestamp), buffer)==2)
                {
                    temp.filename = buffer;
                    dataSource->measurement_.push_back(temp);
                }
            }
            dataSource->dataSourceRoot_ = dPath.parent_path();
            dataSource->imagePathRoot_ = dPath.parent_path();            
            return dataSource;
        }
        return nullptr;
    }


};


class ImuData : public ASLData
{
private:
    struct ImuDataMeasurement
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        unsigned long long int timestamp;
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
    };

public:
    const ImuDataMeasurement& get(size_t i) const
    {
        return measurement_.at(i);
    }

    size_t n() const
    {
        return measurement_.size();
    }

    const std::vector<ImuDataMeasurement>& getMeasurement()
    {
        return measurement_;
    }

private:
    std::vector<ImuDataMeasurement> measurement_;

    ImuData() {}
    friend class ASLDatasetHelper;
    static ASLData* Create(std::string datasetFolder)
    {
        boost::filesystem::path dPath(datasetFolder);
        dPath /= "data.csv";
        std::ifstream file ( dPath.string() );
        if(file.is_open())
        {
            ImuData* dataSource = new ImuData();
            std::string line;
            std::getline(file,line);//drop first line
            while(std::getline(file,line))
            {
                ImuDataMeasurement temp;
                line = line.substr( 0, line.find_first_of("#") );
                double acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z;
                sscanf(line.c_str(),"%llu,%lf,%lf,%lf,%lf,%lf,%lf", &(temp.timestamp), &gyro_x, &gyro_y, &gyro_z, &acc_x, &acc_y, &acc_z);
                temp.acc << acc_x,acc_y,acc_z;
                temp.gyro << gyro_x,gyro_y,gyro_z;
                dataSource->measurement_.push_back(temp);
            }
            return dataSource;
        }
        return nullptr;
    }


};

class non_copyable
{
protected:
    non_copyable() = default;
    ~non_copyable() = default;

    non_copyable(non_copyable const &) = delete;
    void operator=(non_copyable const &x) = delete;
};


class ASLDatasetHelper : public non_copyable
{
public:
    ASLDatasetHelper() {
        factories["imu"] = &ImuData::Create;
        factories["camera"] = &CameraData::Create;
    }

    void addDatasetPath(){}

    std::string getDataSourceType(const std::string filename)
    {
        YAML::Node config = YAML::LoadFile(filename);
        return config["sensor_type"].as<std::string>();
    }



    bool loadDataSource( const std::string id,const  std::string remapedId="",const  std::string datasetPath = "")
    {

        if(datasetPath.empty())
        {
            return false;
        }
        std::string finalId = id;
        if(!remapedId.empty())
        {
            finalId = remapedId;
        }

        boost::filesystem::path dPath(datasetPath);
        dPath /= id;
        dPath /= "sensor.yaml";
        if(boost::filesystem::exists(dPath))
        {
            std::string sensorType = getDataSourceType(dPath.string());
            if(factories.count(sensorType)>0)
            {
                ASLData* dataSource = factories[sensorType](dPath.parent_path().string()); //TODO add test in case of unknow sensor type
                if(dataSource != nullptr)
                {
                    data_[finalId] = dataSource;
                    return true;
                }
            }
        }
        return false;
    }

    template<typename T>
    const T* getData(const std::string id) const
    {
        auto search = data_.find(id);
        if(search !=data_.end())
        {
            ASLData* dataSource = search->second;
            return dynamic_cast<const T*>( dataSource );
        }

        return nullptr;
    }

    std::map<std::string,ASLData*> data_;
    std::vector<std::string> paths;
    std::map<std::string,std::function<ASLData*(std::string)>> factories;
};




#endif //INCLUDE_ASL_DATASET_HELPER_H_
