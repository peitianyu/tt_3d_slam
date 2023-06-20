#include "common/tt_test.h"
#include "camera/feature/feature_match.h"
#include <fstream>
#include <sstream>
#include <iostream>

// JUST_RUN_TEST(feature_match, test) 
TEST(feature_match, test) 
{
    std::string img_path = "../../src/test/data/feature/";
    cv::Mat img1 = cv::imread(img_path + "1.png");
    cv::Mat img2 = cv::imread(img_path + "2.png");

    front_end::camera::FeatureMatch feature_match;

    std::vector<cv::KeyPoint> kp1;
    std::vector<cv::KeyPoint> kp2;
    std::vector<cv::DMatch> matches; // 并打印
    feature_match.Match(img1, img2, kp1, kp2, matches);

    // 可视化
    cv::Mat img_match;
    cv::drawMatches(img1, kp1, img2, kp2, matches, img_match);
    cv::imshow("match", img_match);
    cv::waitKey(0);

    std::cout << "matches size: " << matches.size() << std::endl;
}