#include "common/tt_test.h"
#include "camera/feature/feature_match.h"
#include "camera/math/epipolar_constraint.h"
#include <fstream>
#include <sstream>
#include <iostream>

JUST_RUN_TEST(epipolar_constraint, test) 
TEST(epipolar_constraint, test) 
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
}