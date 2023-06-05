## ICP配准

### 程序步骤

```txt
1. 根据target_point构造kd_tree
2. 变换需要配准的点云到参考坐标系
3. 实现最小二乘
```

### 程序设计

```c++
        bool EstimateTransformationOnce(const icp_kdtree &ref_kdtree, const std::vector<Point3D> &cur_ps)
        {
            std::vector<Point3D> world_ps = TransformPointCloud(cur_ps, m_result.robot_pose);

            Eigen::Matrix3d d_R = m_result.robot_pose.Rot().ToMatrix();
            uint good_point_cnt = 0;
            float score = 0.0;

            Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
            for (uint i = 0; i < world_ps.size(); i++)
            {
                Point3D nearest_p;
                if (!FindNearestPoint(ref_kdtree, world_ps[i], nearest_p))
                    continue;

                Eigen::Vector3d error = world_ps[i] - nearest_p;
                Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
                jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                jacobian.block<3, 3>(0, 3) = -d_R * common::Hat(cur_ps[i]);

                H += jacobian.transpose() * jacobian;
                b += jacobian.transpose() * error;

                ++good_point_cnt;
                score += error.norm();
            }

            if (H.determinant() == 0)
                return false;

            Eigen::Matrix<double, 6, 1> delta_x = -H.inverse() * b;

            m_result.is_converged = std::fabs(delta_x.norm() - m_result.error) < m_option.max_error;
            m_result.error = delta_x.norm();
            m_result.score = score / good_point_cnt;
            m_result.robot_pose = m_result.robot_pose.TransformAdd(Pose3D(delta_x));

            std::cout << "score: " << m_result.score << " error: " << m_result.error << " good_point_cnt: " << good_point_cnt << std::endl;

            return m_result.is_converged;
        }
```

### 测试程序

```c++
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
```
