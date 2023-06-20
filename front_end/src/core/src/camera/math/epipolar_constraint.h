#ifndef __EPIPOLAR_CONSTRAINT_H__
#define __EPIPOLAR_CONSTRAINT_H__

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace front_end{
namespace camera{
class EpipolarConstraint
{
public:
    struct Param
    {
        double cx;
        double cy;
        double fx;
        double fy;
        double focal_length;

        Param(double cx_ = 325.1, double cy_ = 249.7, double fx_ = 520.9, double fy_ = 521.0, double focal_length_ = 521.0)
            : cx(cx_), cy(cy_), fx(fx_), fy(fy_), focal_length(focal_length_) {}
        
        cv::Mat K() const
        {
            cv::Mat k = cv::Mat::eye(3, 3, CV_64F);
            k.at<double>(0, 0) = fx;
            k.at<double>(1, 1) = fy;
            k.at<double>(0, 2) = cx;
            k.at<double>(1, 2) = cy;
            return k;
        }
    };

    EpipolarConstraint(const Param& param = Param())
        : m_param(param) {}

    void PoseEstimation(const std::vector<cv::KeyPoint>& key_pts1, 
                        const std::vector<cv::KeyPoint>& key_pts2, 
                        const std::vector<cv::DMatch>& matches, 
                        cv::Mat& R, cv::Mat& t)
    {
        cv::Mat K = m_param.K();
        
        // 1. 将匹配点转化为std::vector<cv::Point2f>形式
        std::vector<cv::Point2f> pts1, pts2;
        for(uint i = 0; i < matches.size(); i++) {
            pts1.push_back(key_pts1[matches[i].queryIdx].pt);
            pts2.push_back(key_pts2[matches[i].trainIdx].pt);
        }

        // 2. 计算基础矩阵F
        cv::Mat fundamental_matrix = cv::findFundamentalMat(pts1, pts2, cv::FM_8POINT);

        // 3. 计算本质矩阵E
        cv::Mat essential_matrix = cv::findEssentialMat(pts1, pts2, m_param.focal_length, cv::Point2d(m_param.cx, m_param.cy), cv::RANSAC);

        // 4. 计算单应矩阵H
        cv::Mat homography_matrix = cv::findHomography(pts1, pts2, cv::RANSAC, 3);

        // 5. 从本质矩阵E中恢复旋转矩阵R和平移向量t
        cv::recoverPose(essential_matrix, pts1, pts2, R, t, m_param.focal_length, cv::Point2d(m_param.cx, m_param.cy));
    }
private:
    Param m_param;
};

} // namespace camera
} // namespace front_end

#endif // __EPIPOLAR_CONSTRAINT_H__