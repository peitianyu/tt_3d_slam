#include "common/tt_test.h"
#include "common/tt_log.h"
#include "filter/simulation/read_data.h"
#include "filter/simulation/sensor_interface.h"
#include "filter/imu_gps_fuse/imu_gps_localizer.h"
#include <fstream>

class ImuGpsFuse
{
public:
    ImuGpsFuse(const std::string& data_path) : data_path_(data_path){
        simulation::RegisterGpsCallback(GpsProcess);
        simulation::RegisterImuCallback(ImuProcess);
    }

    void Run(){
        std::ofstream log_file;
        log_file.open("../../log/imu_gps_fuse.log");

        while(1){
            simulation::ReadData(data_path_);
            LOG_FILE(log_file, imu_gps_localizer_->GetFuseState().position.transpose()) << std::endl;
        }

        log_file.close();
    }
private:
    static void GpsProcess(const front_end::Gps& gps)
    {
        imu_gps_localizer_->GpsUpdate(gps);
    }

    static void ImuProcess(const front_end::Imu &imu)
    {
        if(!imu_gps_localizer_->ImuPredict(imu))
            return;
    }
private:
    std::string data_path_;
    static std::unique_ptr<ImuGpsLocalizer> imu_gps_localizer_;
};

std::unique_ptr<ImuGpsLocalizer> ImuGpsFuse::imu_gps_localizer_ = 
                std::unique_ptr<ImuGpsLocalizer>(new ImuGpsLocalizer(Eigen::Vector3d::Zero(), 
                front_end::ImuPreintegrator::Option(1e-2, 1e-4, 1e-6, 1e-8, Eigen::Vector3d(0., 0., -9.81007))));




// JUST_RUN_TEST(imu_gps_location, test) 
TEST(imu_gps_location, test) 
{
    std::string data_path = "../../src/test/data/imu_gps_data.txt";
    static ImuGpsFuse imu_gps_fuse(data_path);
    imu_gps_fuse.Run();
}