#ifndef DEBUG_IMAGES_H
#define DEBUG_IMAGES_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

namespace road_object_detection {

/**
 * contains all debug images to debug road watcher
 */
class DebugImages {
 public:
  DebugImages() = default;

  cv::Mat* getCameraImage();
  void setCameraImage(const cv::Mat& image);

  cv::Mat* getBirdsviewPatch(unsigned int cluster_id);
  std::vector<cv::Mat>* getBirdsviewPatches();
  void addBirdsviewPatch(unsigned int cluster_id, const cv::Mat& image);

  cv::Mat* getCannyPatch(unsigned int cluster_id);
  std::vector<cv::Mat>* getCannyPatches();
  void addCannyPatch(unsigned int cluster_id, const cv::Mat& image);

  cv::Mat* getImagePatch(unsigned int cluster_id);
  std::vector<cv::Mat>* getImagePatches();
  void addImagePatch(unsigned int cluster_id, const cv::Mat& image);

  /**
   * resets all arrays etc.
   */
  void clear();

 private:
  cv::Mat camera_image_;
  std::vector<unsigned int> birdsview_patch_ids_;
  std::vector<cv::Mat> birdsview_patches_;
  std::vector<unsigned int> canny_patch_ids_;
  std::vector<cv::Mat> canny_patches_;
  std::vector<unsigned int> image_patch_ids_;
  std::vector<cv::Mat> image_patches_;
};


}  // namespace road_object_detection

#endif  // DEBUG_IMAGES_H
