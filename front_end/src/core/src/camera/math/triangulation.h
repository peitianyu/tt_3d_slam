#ifndef __TRIANGULATION_H__
#define __TRIANGULATION_H__

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace front_end{
namespace camera{
namespace math{

void triangulation(const std::vector<cv::KeyPoint>& key_pts1, 
                    const std::vector<cv::KeyPoint>& key_pts2,
                    const cv::Mat& R, const cv::Mat& t, const cv::Mat& K, 
                    std::vector<cv::Point3d>& points)
{
    cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0, 
        0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

    std::vector<cv::Point2f> pts1, pts2;
    for(uint i = 0; i < key_pts1.size(); i++) {
        pts1.push_back(key_pts1[i].pt);
        pts2.push_back(key_pts2[i].pt);
    }

    cv::Mat pts4d;
    cv::triangulatePoints(T1, T2, pts1, pts2, pts4d);

    for(int i = 0; i < pts4d.cols; i++) {
        cv::Mat x = pts4d.col(i);
        x /= x.at<float>(3, 0);
        cv::Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        points.push_back(p);
    }
}

} // namespace math
} // namespace camera
} // namespace front_end

#endif // __TRIANGULATION_H__