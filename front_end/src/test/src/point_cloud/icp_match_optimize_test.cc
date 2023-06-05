#include "common/tt_test.h"
#include "point_cloud/scan_match/scan2scan/icp_match_optimize.h"
#include "viz/tt_visual.h"
#include "load_pcd.h"


// JUST_RUN_TEST(icp_match_optimize, test)
TEST(icp_match_optimize, test)
{
    front_end::point_cloud::IcpMatchOptimize::Option option(100, 1e-5, 2.0);
    front_end::point_cloud::IcpMatchOptimize icp_match_optimize(option);

    std::vector<types::Point3D> ref_ps = load_pcd("../../src/test/data/rabbit3.pcd");
    
    types::Pose3D cur_pose(types::Point3D(0.1, 0.1, 0.1), types::Rot3D(0.1, 0.1, 0.1));
    std::vector<types::Point3D> cur_ps;
    for (const auto &p : ref_ps){ cur_ps.push_back(cur_pose.TransformAdd(p));}

    icp_match_optimize.Match(ref_ps, cur_ps, types::Pose3D());

    front_end::point_cloud::ScanMatchResult result = icp_match_optimize.GetResult();
    std::cout << "result: " << result.robot_pose << " true_pose: " << cur_pose << std::endl;
    std::cout << "result: " << result.error << " " << result.is_converged << " " << result.score << std::endl;

    std::vector<types::Point3D> result_ps;
    for (const auto &p : cur_ps)
        result_ps.push_back(result.robot_pose.TransformAdd(p));

    // 可视化
    viz::Visual::GetInstance()->ShowPointCloud("ref", ref_ps, viz::COLOR_RED);
    // viz::Visual::GetInstance()->ShowPointCloud("cur", cur_ps, viz::COLOR_BLUE);
    viz::Visual::GetInstance()->ShowPointCloud("result", result_ps, viz::COLOR_GREEN);

    viz::Visual::GetInstance()->Show();
}

#include <random>

void TestOnce(const std::vector<types::Point3D>& ref_ps, front_end::point_cloud::IcpMatchOptimize& icp_match_optimize)
{
    std::random_device rd;
    std::mt19937_64 e(rd());
    std::uniform_real_distribution<double> u(-0.3, 0.3);
    types::Pose3D cur_pose(types::Point3D(u(e), u(e), u(e)), types::Rot3D(u(e), u(e), u(e)));

    std::vector<types::Point3D> cur_ps;
    for (const auto &p : ref_ps){ cur_ps.push_back(cur_pose.TransformAdd(p));}

    icp_match_optimize.Match(ref_ps, cur_ps, types::Pose3D());

    front_end::point_cloud::ScanMatchResult result = icp_match_optimize.GetResult();
    std::cout << "result: " << result.robot_pose << " true_pose: " << cur_pose << std::endl;
    std::cout << "result: " << result.error << " " << result.is_converged <<  " " << result.score << std::endl;

    ASSERT_TRUE(result.is_converged);
}

// JUST_RUN_TEST(icp_match_optimize, test1)
TEST(icp_match_optimize, test1)
{
    front_end::point_cloud::IcpMatchOptimize::Option option(100, 1e-7, 10);
    front_end::point_cloud::IcpMatchOptimize icp_match_optimize(option);

    std::vector<types::Point3D> ref_ps = load_pcd("../../src/test/data/rabbit3.pcd");
    
    for (int i = 0; i < 5; ++i)
        TestOnce(ref_ps, icp_match_optimize);
}