
#include <asl_dataset_helper.h>

int main(void)
{

    ASLDatasetHelper dataHelper;
    dataHelper.loadDataSource("imu0","","/home/lucas/data/bags/outputdir");
    dataHelper.loadDataSource("cam0","","/home/lucas/data/bags/outputdir");


    const ImuData* imu0 = dataHelper.getData<ImuData>("imu0");
    for(int i=0 ; i < 2 ; i++)
    {
        std::cout << imu0->get(i).timestamp << " " << imu0->get(i).acc << " " << imu0->get(i).gyro << std::endl;
    }

    const CameraData* cam0 = dataHelper.getData<CameraData>("cam0");
    for(int i=0 ; i < 2 ; i++)
    {
        std::cout << cam0->get(i).timestamp  << " " << cam0->getImagePathRoot() << " " << cam0->get(i).filename << std::endl;
    }

    return 0;
}
