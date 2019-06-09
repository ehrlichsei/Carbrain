#include "debug_images.h"

namespace road_object_detection {

cv::Mat* DebugImages::getCameraImage() { return &camera_image_; }

void DebugImages::setCameraImage(const cv::Mat& image) {
  // convert image from MONO8 to BGR
  cv::cvtColor(image, camera_image_, CV_GRAY2BGR);
}

cv::Mat* DebugImages::getBirdsviewPatch(unsigned int cluster_id) {
  for (unsigned int i = 0; i < birdsview_patch_ids_.size(); i++) {
    if (birdsview_patch_ids_[i] == cluster_id) {
      return &(birdsview_patches_.at(i));
    }
  }
  ROS_ERROR("could not find birdsview patch corresponds to cluster_id %u", cluster_id);
  return &(birdsview_patches_.at(0));
}

std::vector<cv::Mat>* DebugImages::getBirdsviewPatches() {
  return &birdsview_patches_;
}

void DebugImages::addBirdsviewPatch(unsigned int cluster_id, const cv::Mat& image) {
  cv::Mat birdsview_patch;
  // convert image from MONO8 to BGR
  cv::cvtColor(image, birdsview_patch, CV_GRAY2BGR);
  birdsview_patch_ids_.push_back(cluster_id);
  birdsview_patches_.push_back(birdsview_patch);
}

cv::Mat* DebugImages::getCannyPatch(unsigned int cluster_id) {
  for (unsigned int i = 0; i < canny_patch_ids_.size(); i++) {
    if (canny_patch_ids_[i] == cluster_id) {
      return &(canny_patches_.at(i));
    }
  }
  ROS_ERROR("could not find canny patch corresponds to cluster_id %u", cluster_id);
  return &(canny_patches_.at(0));
}

std::vector<cv::Mat>* DebugImages::getCannyPatches() { return &canny_patches_; }

void DebugImages::addCannyPatch(unsigned int cluster_id, const cv::Mat& image) {
  cv::Mat canny_patch;
  // convert image from MONO8 to BGR
  cv::cvtColor(image, canny_patch, CV_GRAY2BGR);
  canny_patch_ids_.push_back(cluster_id);
  canny_patches_.push_back(canny_patch);
}


cv::Mat* DebugImages::getImagePatch(unsigned int cluster_id) {
  for (unsigned int i = 0; i < image_patch_ids_.size(); i++) {
    if (image_patch_ids_[i] == cluster_id) {
      return &(image_patches_.at(i));
    }
  }
  ROS_ERROR("could not find image patch corresponds to cluster_id %u", cluster_id);
  return &(image_patches_.at(0));
}

std::vector<cv::Mat>* DebugImages::getImagePatches() { return &image_patches_; }

void DebugImages::addImagePatch(unsigned int cluster_id, const cv::Mat& image) {
  cv::Mat image_patch;
  // convert image from MONO8 to BGR
  cv::cvtColor(image, image_patch, CV_GRAY2BGR);
  image_patch_ids_.push_back(cluster_id);
  image_patches_.push_back(image_patch);
}


void DebugImages::clear() {
  birdsview_patch_ids_.clear();
  birdsview_patches_.clear();
  canny_patch_ids_.clear();
  canny_patches_.clear();
  image_patch_ids_.clear();
  image_patches_.clear();
}


} //namespace road_object_detection
