#include<iostream>
#include"common/tt_test.h"
#include"common/tt_log.h"
#include"common/tt_assert.h"
#include"common/tt_ini_parse.h"
#include"common/tt_tic_toc.h"
#include"common/tt_sleep.h"

#include"types/pose3d.h"
#include"types/point3d.h"
#include"types/rot3d.h"

#include"grid_map/grid_map_base.h"

#include"test_singleton.h"

TEST(common, log)
{
    LOG_INFO("---------->This is a info log") << std::endl;
    LOG_DEBUG("This is a debug log") << std::endl;
    LOG_WARN("This is a warn log") << std::endl;
    LOG_ERROR("This is a error log") << std::endl;

    LOG_DEBUG("This is a debug log", 1, 2, 3) << std::endl;
}

TEST(common, singleton)
{
    test::TestSingleton::GetInstance()->Print();
    LOG_DEBUG(test::TestSingleton::GetInstance()->m_str) << std::endl;
}

TEST(common, ini_parse)
{
    common::IniParse ini_parse;
    if (!ini_parse.ReadIni("../../src/test/config/param.ini"))
    {
        LOG_ERROR("Cant Read ini, please check ini path!") << std::endl;
        return;
    }

    std::string sensor_name = "";
    double gravity = 0.0;
    int use = 0;
    int num = 0;
    std::string extrinsic_rot = "";
    ini_parse.GetValue("ImuParam", "sensor_name", sensor_name);
    ini_parse.GetValue("ImuParam", "gravity", gravity);
    ini_parse.GetValue("ImuParam", "use", use);
    ini_parse.GetValue("ImuParam", "num", num);
    ini_parse.GetValue("ImuParam", "extrinsic_rot", extrinsic_rot);

    ASSERT_EQ(sensor_name, "laser");
    ASSERT_EQ(gravity, 9.80511);
    ASSERT_EQ(use, 1);
    ASSERT_EQ(num, 2000);
    ASSERT_EQ(extrinsic_rot, "1,0,0,0,1,0,0,0,-1");

    Eigen::Matrix3f extrinsic_rot_mat = [&](std::string& str)
    {
        std::replace(str.begin(), str.end(), ',', ' ');
        std::istringstream iss(str);
        Eigen::Matrix3f mat;
        iss >> mat(0, 0) >> mat(0, 1) >> mat(0, 2)
            >> mat(1, 0) >> mat(1, 1) >> mat(1, 2)
            >> mat(2, 0) >> mat(2, 1) >> mat(2, 2);
        return mat;
    }(extrinsic_rot);
    LOG_DEBUG("extrinsic_rot_mat: \n", extrinsic_rot_mat) << std::endl;
}

TEST(common, sleep_tictoc)
{
    common::TicToc tic_toc;
    tic_toc.Tic();
    common::Sleep(1.0); // sleep 1s
    ASSERT_TRUE(tic_toc.Toc() < 1.0 + 1e-3);
}

TEST(common, assert)
{
    tt_assert(1 == 1);
    tt_assert(1.2 == 1.2);
}

TEST(types, pose)
{
    types::Pose3D pose(Eigen::Vector3d(1, 2, 3), Eigen::Quaterniond(1, 2, 3, 4));
    LOG_DEBUG("construct from quaternion: ", pose) << std::endl;

    Eigen::Matrix<double, 6, 1> pose_vec = Eigen::Matrix<double, 6, 1>::Zero();
    pose_vec << pose.Point() , pose.Rot().ToEuler();
    types::Pose3D pose1(pose_vec);
    LOG_DEBUG("construct from vector: ", pose1) << std::endl;

    Eigen::Matrix3d rot = pose.Rot().ToMatrix();
    types::Pose3D pose2(Eigen::Vector3d(1, 2, 3), rot);
    LOG_DEBUG("construct from matrix: ", pose2) << std::endl;

    types::Pose3D pose5(pose.ToMatrix());
    LOG_DEBUG("construct from matrix: ", pose5) << std::endl;

    LOG_DEBUG("to_matrx: \n", pose.ToMatrix()) << std::endl;

    types::Pose3D d_pose(Eigen::Vector3d(1, 2, 3), Eigen::Quaterniond(1, 2, 3, 4));
    types::Pose3D pose3 = pose.TransformAdd(d_pose);

    types::Pose3D pose4 = pose3.TransformAdd(d_pose.TransformFrom(types::Pose3D()));
    LOG_DEBUG("transform: ", pose4) << std::endl;
}

using namespace grid_map;
using namespace types;

std::vector<types::Point3D> BuildRandomPoints()
{
    std::vector<types::Point3D> points;
    for(int i = 0; i < 100; ++i)
    {
        Point3D point;
        point.x() = double(rand() % 1000)/100.0;
        point.y() = double(rand() % 1000)/100.0;
        point.z() = double(rand() % 1000)/100.0;
        // point.transpose() << std::endl;
        points.push_back(point);
    }

    return points;
}

TEST(grid_map, grid_map_base)
{
    grid_map::GridMapBase grid_map;
    std::vector<Point3D> points = BuildRandomPoints();
    Pose3D cur_pose = Pose3D();
    grid_map.UpdateByScan(cur_pose, points);
    LOG_DEBUG("--------grid_map.GetData().size(): ", grid_map.GetData().size()) << std::endl;

    LOG_DEBUG("--------grid_map.GetMapLimit()")<< std::endl;
    for(auto limit : grid_map.GetMapLimit())
        LOG_DEBUG(limit.index.transpose()) << std::endl;

    LOG_DEBUG("--------grid_map.IsValid()") << std::endl;
    LOG_DEBUG(grid_map.IsValid(grid_map::Index3D(Eigen::Vector3i(10,10,10)))) << std::endl;
    LOG_DEBUG(grid_map.IsValid(grid_map::Index3D(Eigen::Vector3i(1000,1000,1000)))) << std::endl;

    LOG_DEBUG("--------grid_map.GetCellProb()") << std::endl;
    LOG_DEBUG(grid_map.GetCellProb(grid_map::Index3D(Eigen::Vector3i(10,10,10)))) << std::endl;
    LOG_DEBUG(grid_map.GetCellProb(grid_map::Index3D(Point3D(6.01, 0.97, 9.02)))) << std::endl;

    LOG_DEBUG("--------grid_map.Clear()") << std::endl;
    grid_map.Clear();
    LOG_DEBUG("grid_map.GetData().size(): ", grid_map.GetData().size()) << std::endl;
}



int main()
{
    return RunAllTests();
}