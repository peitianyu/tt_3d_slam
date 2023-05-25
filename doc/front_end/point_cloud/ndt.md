## NDT配准

### 设计思路

```txt
1. 实现参考icp配准
2. find_neib_point: 采用GridCell实现
3. 采用马氏距离, 实现加权最小二乘
```

### 程序步骤

```txt
1. 根据option设定栅格大小
2. 计算网络的概率分布模型
3. 变换需要配准的点云到参考坐标系
4. 根据马氏距离, 实现最小二乘
```

### 程序实现

```c++
    bool EstimateTransformationOnce(const std::vector<Point3D> &cur_ps)
    {
        std::vector<Point3D> world_ps = TransformPointCloud(cur_ps, m_result.robot_pose);
        Eigen::Matrix3d d_R = m_result.robot_pose.Rot().ToMatrix();
        uint good_point_cnt = 0;
        float score = 0.0;

        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
        for (uint i = 0; i < world_ps.size(); i++)
        {
            uint index = PointToGrid(world_ps[i]);
            if(index >= m_option.max_grid_size() || m_grid_cells[index].points.size() < 5)
                continue;

            Eigen::Vector3d error = world_ps[i] - m_grid_cells[index].mean;
            Eigen::Matrix3d inv_cov = m_grid_cells[index].covarince.inverse();

            Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
            jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            jacobian.block<3, 3>(0, 3) = -d_R * common::Hat(cur_ps[i]);

            H += jacobian.transpose() * inv_cov * jacobian;
            b += jacobian.transpose() * inv_cov * error;

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

### 程序测试

```c++
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
```
