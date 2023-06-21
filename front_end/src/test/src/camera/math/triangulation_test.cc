#include "common/tt_test.h"
#include "camera/feature/feature_match.h"
#include "camera/math/triangulation.h"
#include "camera/math/epipolar_constraint.h"
#include <fstream>
#include <sstream>
#include <iostream>

JUST_RUN_TEST(triangulation, test) 
TEST(triangulation, test) 
{
    std::string img_path = "../../src/test/data/feature/";
    cv::Mat img1 = cv::imread(img_path + "1.png");
    cv::Mat img2 = cv::imread(img_path + "2.png");

    front_end::camera::FeatureMatch feature_match;

    std::vector<cv::KeyPoint> kp1;
    std::vector<cv::KeyPoint> kp2;
    std::vector<cv::DMatch> matches; // 并打印
    feature_match.Match(img1, img2, kp1, kp2, matches);

    std::cout << "matches size: " << matches.size() << std::endl;

    front_end::camera::EpipolarConstraint epipolar_constraint;

    cv::Mat R, t;
    epipolar_constraint.PoseEstimation(kp1, kp2, matches, R, t);
    std::cout << "R: \n" << R << std::endl;
    std::cout << "t: \n" << t << std::endl;

    std::vector<cv::Point3d> points;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    front_end::camera::math::triangulation(kp1, kp2, R, t, K, points);

    std::cout << "points size: " << points.size() << std::endl;
}