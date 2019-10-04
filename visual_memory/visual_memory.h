
#include "opencv2/features2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>
#include <utility> //std::pair
#include <Eigen/Geometry>

using namespace cv::xfeatures2d;
/*
 * Author: Luiz Felipe Maciel Correia
 * y9luizufrn@gmail.com
 */

class VisualMemory
{
protected:
  std::vector<cv::Mat> trainDescriptorsSet_;

  cv::Ptr<cv::DescriptorMatcher> matcher_;

  //static bool sort_assistant(std::pair<int, int> a, std::pair<int, int> b);
  //this is useful for visual memory experiments
  int frame_idx_;

  Eigen::Matrix4f pose_;

public:
  //Default constructor
  VisualMemory();
  //Constructor if you desire dont store your trainDescriptorSet in  cv::Ptr<cv::DescriptorMatcher> matcher_;
  VisualMemory(std::vector<cv::Mat> trainDescriptorsSet);
  // Constructor if you wnat to store your descriptors in   cv::Ptr<cv::DescriptorMatcher> matcher_; (recommended)
  VisualMemory(cv::Ptr<cv::DescriptorMatcher> matcher);
  //create a VisualMemory storing the descriptors in         std::vector<cv::Mat> trainDescriptorsSet_; (not recommended)
  void create(std::vector<cv::Mat> trainDescriptorsSet_);
  //add descriptors in   cv::Ptr<cv::DescriptorMatcher> matcher_;
  void add(cv::Mat trainDescriptors);
  void add(cv::Mat trainDescriptors, int frame_idx, Eigen::Matrix4f pose);

  //search a descriptor of a query image in one of two structures  std::vector<cv::Mat> trainDescriptorsSet_ or         cv::Ptr<cv::DescriptorMatcher> matcher_;

  std::vector<int> searchDescriptor(cv::Mat queryDescriptor, int n);

  std::vector<int> searchDescriptor(cv::Mat, int n, cv::Ptr<cv::DescriptorMatcher> matcher);

  std::vector<int> searchImage(cv::Mat img);
};