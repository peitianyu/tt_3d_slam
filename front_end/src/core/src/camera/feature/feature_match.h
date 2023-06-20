#ifndef __FEATURE_MATCH_H__
#define __FEATURE_MATCH_H__

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace front_end{
namespace camera{

class FeatureMatch
{
public:
    struct Param
    {
        double ratio;
        double min_dist;

        Param(double ratio_ = 2.0, double min_dist_ = 30.0)
            : ratio(ratio_), min_dist(min_dist_) {}
    };

    FeatureMatch(const Param& param = Param())
        : m_param(param)
    {
        m_detector = cv::ORB::create();
        m_descriptor = cv::ORB::create();
    }

    void Match(const cv::Mat& img1, const cv::Mat& img2, 
                std::vector<cv::KeyPoint>& kp1,
                std::vector<cv::KeyPoint>& kp2,
                std::vector<cv::DMatch>& matches)
    {
        cv::Mat des1, des2;
        m_detector->detect(img1, kp1);          // 检测Oriented FAST角点位置
        m_detector->detect(img2, kp2);
        m_descriptor->compute(img1, kp1, des1); // 根据角点位置计算BRIEF描述子
        m_descriptor->compute(img2, kp2, des2);

        cv::BFMatcher matcher(cv::NORM_HAMMING); // 用汉明距离作为描述子之间的距离
        std::vector<cv::DMatch> match;
        matcher.match(des1, des2, match);

        double min_dist = 10000, max_dist = 0;  // 找出所有匹配之间的最小距离和最大距离
        for(int i = 0; i < des1.rows; i++)
        {
            double dist = match[i].distance;
            if(dist < min_dist) min_dist = dist;
            if(dist > max_dist) max_dist = dist;
        }

        for(int i = 0; i < des1.rows; i++)     // 当描述子之间的距离大于两倍的最小距离时，即认为匹配有误
        {
            if(match[i].distance <= std::max(m_param.ratio * min_dist, m_param.min_dist))
                matches.push_back(match[i]);
        }
    }
private:
    Param m_param;
    cv::Ptr<cv::ORB> m_detector;
    cv::Ptr<cv::ORB> m_descriptor;
};

} // namespace camera
} // namespace front_end

#endif // __FEATURE_MATCH_H__