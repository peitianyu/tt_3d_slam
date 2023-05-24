#include "common/tt_test.h"
#include "point_cloud/scan_match/scan2scan/ndt_match.h"
#include "viz/tt_visual.h"
#include "load_pcd.h"

#include <random>

void TestOnce(const std::vector<types::Point3D>& ref_ps, front_end::point_cloud::NdtMatch& ndt_match)
{
    // std::random_device rd;
    // std::mt19937_64 e(rd());
    // std::uniform_real_distribution<double> u(-0.3, 0.3);
    // types::Pose3D cur_pose(types::Point3D(u(e), u(e), u(e)), types::Rot3D(u(e), u(e), u(e)));
    types::Pose3D cur_pose(types::Point3D(0.1, 0.1, 0.1), types::Rot3D(0.1, 0.1, 0.1));

    std::vector<types::Point3D> cur_ps;
    for (const auto &p : ref_ps){ cur_ps.push_back(cur_pose.TransformAdd(p));}

    ndt_match.Match(ref_ps, cur_ps, types::Pose3D());

    std::vector<types::Point3D> ref_grid_points = ndt_match.GetGridPoints();

    front_end::point_cloud::ScanMatchResult result = ndt_match.GetResult();
    std::cout << "result: " << result.robot_pose << " true_pose: " << cur_pose << std::endl;
    std::cout << "result: " << result.error << " " << result.is_converged <<  " " << result.score << std::endl;

    std::vector<types::Point3D> result_ps;
    for (const auto &p : cur_ps)
        result_ps.push_back(result.robot_pose.TransformAdd(p));
    std::cout << "result_ps size: " << result_ps.size() << std::endl;

    // 可视化
    viz::Visual::GetInstance()->ShowPointCloud("ref", ref_ps, viz::COLOR_RED);
    viz::Visual::GetInstance()->ShowPointCloud("cur", ref_grid_points, viz::COLOR_BLUE);
    viz::Visual::GetInstance()->ShowPointCloud("result", result_ps, viz::COLOR_GREEN);

    viz::Visual::GetInstance()->Show();

    ASSERT_TRUE(result.is_converged);
}

JUST_RUN_TEST(ndt_match, test1)
TEST(ndt_match, test1)
{
    front_end::point_cloud::NdtMatch::Option option(100, 1e-5, 2.0, 200, 1.0);
    front_end::point_cloud::NdtMatch ndt_match(option);

    std::vector<types::Point3D> ref_ps = load_pcd("/mnt/d/file_ws/Learning/slam/tt_3d_slam/front_end/src/test/data/rabbit3.pcd");
    
    TestOnce(ref_ps, ndt_match);

    // for (int i = 0; i < 5; ++i)
    //     TestOnce(ref_ps, ndt_match);
}

const uint PointToGrid( const Point3D &point, front_end::point_cloud::NdtMatch::Option option)
{
    int x = static_cast<int>(point(0) / option.resolution) + option.voxel_size/2;
    int y = static_cast<int>(point(1) / option.resolution) + option.voxel_size/2;
    int z = static_cast<int>(point(2) / option.resolution) + option.voxel_size/2;

    assert(x >= 0 && x < option.voxel_size);
    assert(y >= 0 && y < option.voxel_size);
    assert(z >= 0 && z < option.voxel_size);

    return x + y * option.voxel_size + z * option.voxel_size * option.voxel_size;
}

// JUST_RUN_TEST(ndt_match, test2)
TEST(ndt_match, test2)
{
    Point3D p1(-4.4838514, 6.3016939, -3.7409768);
    Point3D p2(-5.1461086, 6.7603765, -3.7641163);
    Point3D p3(-4.9079995, 6.5820603, -3.8900011);
    Point3D p4(-4.6174178, 6.797513, -3.7502959);

    front_end::point_cloud::NdtMatch::Option option;
    
    std::cout << PointToGrid(p1, option) << std::endl;
    std::cout << PointToGrid(p2, option) << std::endl;
    std::cout << PointToGrid(p3, option) << std::endl;
    std::cout << PointToGrid(p4, option) << std::endl;
}

// JUST_RUN_TEST(ndt_match, test3)
TEST(ndt_match, test3)
{
    // 测试cov matrix sqrt
    Eigen::Matrix3d cov;
    cov << 0.1, 0.0, 0.0,
           0.0, 0.2, 0.0,
           0.0, 0.0, 0.3;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    Eigen::Matrix3d cov_sqrt = eig.operatorSqrt();

    std::cout << cov << std::endl;
    std::cout << cov_sqrt << std::endl;
}