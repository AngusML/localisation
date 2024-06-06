/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */

#include <iostream>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "aruco_ros/aruco_ros_utils.hpp"
#include "aruco_msgs/msg/marker_array.hpp"

#if __has_include("cv_bridge/cv_bridge.hpp")
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rcpputils/asserts.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"








////////////////////////////////////////////////

/*migrated from marker.h*/
// #ifndef _Aruco_Marker_H
// #define _Aruco_Marker_H

// #include "aruco_export.h"

#include <opencv2/core/core.hpp>

// #include <cstdint>
#include <iostream>
#include <vector>


/*migrated from marker.cpp*/
// #include "marker.h"

// / \todo set this definition in the cmake code
// #define _USE_MATH_DEFINES

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdio>
#include <math.h>
// #include "cameraparameters.h"
// #include "ippe.h"
////////////////////////////////////////////////


























using namespace std::chrono_literals;

class ArucoMarkerPublisher : public rclcpp::Node
{
private:







  /*A ADDITIONS*/
  std::vector<std::vector<double>> markerTrans_;







  rclcpp::Node::SharedPtr subNode;
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  aruco::CameraParameters camParam_;
  std::vector<aruco::Marker> markers_;

  // node params
  bool useRectifiedImages_;
  std::string marker_frame_;
  std::string camera_frame_;
  std::string reference_frame_;
  double marker_size_;

  // ROS pub-sub
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;
  rclcpp::Publisher<aruco_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr marker_list_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<aruco_msgs::msg::MarkerArray> marker_msg_;
  cv::Mat inImage_;
  bool useCamInfo_;
  std_msgs::msg::UInt32MultiArray marker_list_msg_;

public:
  ArucoMarkerPublisher()
  : Node("marker_publisher"), useCamInfo_(true)
  {
  }
  //A ADDITIONS - MODIFIED SETUP FUNCITON
  bool setup()
  {
        //RCLCPP_INFO(this->get_logger(), "IT RUNG this FUNCITON setup");

    subNode = this->create_sub_node(this->get_name());
    // Declare node parameters
    this->declare_parameter<double>("marker_size", 0.05);
    this->declare_parameter<std::string>("reference_frame", "");
    this->declare_parameter<std::string>("camera_frame", "");
    this->declare_parameter<bool>("image_is_rectified", true);
    this->declare_parameter<bool>("use_camera_info", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    image_sub_ = it_->subscribe("/camera/color/image_raw", 1, &ArucoMarkerPublisher::image_callback, this);

    this->get_parameter_or<bool>("use_camera_info", useCamInfo_, true);
    if (useCamInfo_) {
      //RCLCPP_INFO(this->get_logger(), "Waiting for the camera info...");
      sensor_msgs::msg::CameraInfo camera_info;
      rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(
        camera_info,
        shared_from_this(), "/camera/color/camera_info");
      //RCLCPP_INFO(this->get_logger(), "Successfully obtained the camera info!");


      this->get_parameter_or<double>("marker_size", marker_size_, 0.05);
      this->get_parameter_or<bool>("image_is_rectified", useRectifiedImages_, true);
      this->get_parameter_or<std::string>("reference_frame", reference_frame_, "");
      this->get_parameter_or<std::string>("camera_frame", camera_frame_, "");
      camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(camera_info, useRectifiedImages_);
      rcpputils::assert_true(
        !(camera_frame_.empty() && !reference_frame_.empty()),
        "Either the camera frame is empty and also reference frame is empty..");
      if (reference_frame_.empty()) {
        reference_frame_ = camera_frame_;
      }
    } else {
      camParam_ = aruco::CameraParameters();
    }

    image_pub_ = it_->advertise(std::string("/aruco_single/result"), 1);
    debug_pub_ = it_->advertise(this->get_name() + std::string("/debug"), 1);
    marker_pub_ = subNode->create_publisher<aruco_msgs::msg::MarkerArray>("markers", 100);
    marker_list_pub_ =
      subNode->create_publisher<std_msgs::msg::UInt32MultiArray>("markers_list", 10);

    marker_msg_ = aruco_msgs::msg::MarkerArray::Ptr(new aruco_msgs::msg::MarkerArray());
    marker_msg_->header.frame_id = reference_frame_;
    //RCLCPP_INFO(this->get_logger(), "Successfully setup the marker publisher!");

    return true;
  }










/*A ADDITIONS*/

  // returns the SE3 (4x4) transform matrix
// cv::Mat getTransformMatrix() const
// {
//   cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
//   cv::Mat rot = T.rowRange(0, 3).colRange(0, 3);
//   cv::Rodrigues(Rvec, rot);
//   for (int i = 0; i < 3; i++)
//     T.at<float>(i, 3) = Tvec.ptr<float>(0)[i];
//   return T;
// }

  // returns translation values from the SE3 (4x4) transform matrix
std::vector<double> getTranslation(const cv::Mat &transformMatrix) {
  // Check if the matrix has the correct size (4x4)
  if (transformMatrix.rows != 4 || transformMatrix.cols != 4) {
    throw std::invalid_argument("Input matrix must be a 4x4 homogeneous transformation matrix");
  }

  // Extract the top-right 3x1 sub-matrix
  std::vector<double> translationM(3);
  translationM[0] = transformMatrix.at<double>(0, 3);
  translationM[1] = transformMatrix.at<double>(1, 3);
  translationM[2] = transformMatrix.at<double>(2, 3);

  std::cout << "ran this" << std::endl;

  return translationM;
}
















  bool getTransform(
    const std::string & refFrame, const std::string & childFrame,
    geometry_msgs::msg::TransformStamped & transform)
  {
    //RCLCPP_INFO(this->get_logger(), "DOESN'T IT RUNG HTINSG FUNCITON gettransform");

    std::string errMsg;

    if (!tf_buffer_->canTransform(
        refFrame, childFrame, tf2::TimePointZero,
        tf2::durationFromSec(0.5), &errMsg))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get pose from TF : " << errMsg.c_str());
      return false;
    } else {
      try {
        transform = tf_buffer_->lookupTransform(
          refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(
            0.5));
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
        return false;
      }
    }
    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
      //RCLCPP_INFO(this->get_logger(), "IT RUNG HTINSG FUNCITON imagecasllback CONTINUOUSOLYYUYY FOREVER UNTIL I: ctrl + c          then for some reason int main() runs once.....");
    bool publishMarkers = marker_pub_->get_subscription_count() > 0;
    bool publishMarkersList = marker_list_pub_->get_subscription_count() > 0;
    bool publishImage = image_pub_.getNumSubscribers() > 0;
    bool publishDebug = debug_pub_.getNumSubscribers() > 0;

    if (!publishMarkers && !publishMarkersList && !publishImage && !publishDebug) {
      //RCLCPP_INFO(this->get_logger(), " [2] Executing some code... ");
      return;
    }

    builtin_interfaces::msg::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try {
      //RCLCPP_INFO(this->get_logger(), " [4] Publishing MarkerArray with %zu markers", marker_msg_->markers.size());
      cv_ptr = cv_bridge::toCvCopy(*msg.get(), sensor_msgs::image_encodings::RGB8);
      inImage_ = cv_ptr->image;

      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

      // marker array publish
      if (publishMarkers) {
        //RCLCPP_INFO(this->get_logger(), " [6] Calling function: getTransform...");
        marker_msg_->markers.clear();
        marker_msg_->markers.resize(markers_.size());
        marker_msg_->header.stamp = curr_stamp;

        for (std::size_t i = 0; i < markers_.size(); ++i) {
          //RCLCPP_INFO(this->get_logger(), " [8] Processing marker %zu (ID: %d)", i, markers_[i].id);
          aruco_msgs::msg::Marker & marker_i = marker_msg_->markers.at(i);
          marker_i.header.stamp = curr_stamp;
          marker_i.id = markers_.at(i).id;
          marker_i.confidence = 1.0;
        }

        // if there is camera info let's do 3D stuff
        if (useCamInfo_) {
          //RCLCPP_INFO(this->get_logger(), "[10] Exiting function: image_callback");
          // get the current transform from the camera frame to output ref frame
          tf2::Stamped<tf2::Transform> cameraToReference;
          cameraToReference.setIdentity();

          if (reference_frame_ != camera_frame_) {
            //RCLCPP_INFO(this->get_logger(), " [2] Executing some code... ");
            geometry_msgs::msg::TransformStamped transform;
            getTransform(reference_frame_, camera_frame_, transform);
            tf2::fromMsg(transform, cameraToReference);
            // RCLCPP_INFO_STREAM(this->get_logger(), "transform i think is: " << /*MUSTBEAVALUE*/);
          }

          // now find the transform for each detected marker
          for (std::size_t i = 0; i < markers_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), " [2] Executing some code... ");
            aruco_msgs::msg::Marker & marker_i = marker_msg_->markers.at(i);
            tf2::Transform transform = aruco_ros::arucoMarker2Tf2(markers_[i]);
            transform = static_cast<tf2::Transform>(cameraToReference) * transform;
            tf2::toMsg(transform, marker_i.pose.pose);
            RCLCPP_INFO_STREAM(this->get_logger(), "I think position is: " << "x = " << marker_i.pose.pose.position.x << "  y = " << marker_i.pose.pose.position.y << "z = " << marker_i.pose.pose.position.z);


            marker_i.header.frame_id = reference_frame_;
          }
          RCLCPP_INFO(this->get_logger(), " [2] Executing some code... ");
        }

  RCLCPP_INFO(this->get_logger(), " EXITS MASSSIVE LOOOOOOOOOOOOOOOOOOOOOOPPPPP");

        /*DOESN'T PRINT ANYTHIGN*/
        // publish marker array
        if (marker_msg_->markers.size() > 0) {
          //RCLCPP_INFO(this->get_logger(), " ENTERS THIS LAST IF LOOP ");
          // //RCLCPP_INFO(this->get_logger(), "before");
                    // RCLCPP_INFO_STREAM(this->get_logger(),"I think position is: " << "x = " << marker_msg_->markers.pose.pose.position.x << "   y = " << marker_msg_->markers.pose.pose.position.y << "   z = " << marker_msg_->markers.pose.pose.position.z);
          // RCLCPP_INFO_STREAM(this->get_logger(), "I think position is: " << "x = " << marker_i.pose.pose.position.x << "  y = " << marker_i.pose.pose.position.y << "z = " << marker_i.pose.pose.position.z);
          marker_pub_->publish(*marker_msg_);
          // //RCLCPP_INFO(this->get_logger(), "after");
          // RCLCPP_INFO_STREAM(this->get_logger(), "I think position is: " << "x = " << marker_i.pose.pose.position.x << "  y = " << marker_i.pose.pose.position.y << "z = " << marker_i.pose.pose.position.z);
          // RCLCPP_INFO_STREAM(this->get_logger(),"I think position is: " << "x = " << marker_msg_->pose.pose.position.x << "   y = " << marker_msg_->markers.pose.pose.position.y << "   z = " << marker_msg_->markers.pose.pose.position.z);
        }
          //RCLCPP_INFO(this->get_logger(), " exxxxxxxxxxxxxxxxxiiiiiiiiiits THIS LAST IF LOOP ");
      }

      if (publishMarkersList) {
                  //RCLCPP_INFO(this->get_logger(), " ENTERS THIS LASTLASTLASTLAST IF LOOP ");

        marker_list_msg_.data.resize(markers_.size());
        for (std::size_t i = 0; i < markers_.size(); ++i) {
                          //RCLCPP_INFO(this->get_logger(), " ENTERS THIS LASTLASTLASTLAST foooooooorrr LOOP ");

          marker_list_msg_.data[i] = markers_[i].id;
        }
          RCLCPP_INFO(this->get_logger(), " exxxxxxxxxxxxxxxxxiiiiiiiiiits THIS LASTLASTLASTLASTLAST IF LOOP ");

        marker_list_pub_->publish(marker_list_msg_);
      }
          RCLCPP_INFO(this->get_logger(), " exxxxxxxxxxxxxxxxxiiiiiiiiiits THIS LAST IF LOOP ");

      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers_.size(); ++i) {
                                  RCLCPP_INFO(this->get_logger(), "2222222 ENTERS THIS LASTLASTLASTLAST foooooooorrr LOOP ");

        markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
      }
                                  //RCLCPP_INFO(this->get_logger(), "2222222 exits THIS LASTLASTLASTLAST foooooooorrr LOOP ");
      // draw a 3D cube in each marker if there is 3D info
      if (camParam_.isValid() && marker_size_ > 0) {
                                          //RCLCPP_INFO(this->get_logger(), "2222222 ENTERS THIS LASTLASTLASTLAST if LOOP ");

        for (std::size_t i = 0; i < markers_.size(); ++i) {
                                                          //RCLCPP_INFO(this->get_logger(), "12345i2 enterssdfsdfs THIS LASTLASTLASTLAST for LOOP ");

          aruco::CvDrawingUtils::draw3dAxis(inImage_, markers_[i], camParam_);
          

            /*A ADDITIONS*/


                      // markerTrans_.at(i) = getTranslation(markers_[i].getTransformMatrix())[0];
            // Assuming getTranslation returns a vector of size 3 (x, y, z)



            // std::vector<double> translations = getTranslation(markers_[i].getTransformMatrix());
            // // std::copy(translations.begin(), translations.end(), markerTrans_.begin() + i);
            // for (size_t j = 0; j < translations.size(); ++j) {
            //   markerTrans_[i + j] = translations[j];
            //   RCLCPP_INFO_STREAM(this->get_logger(), "translation x y z is:" << translations.at(0));//  << translations[j][1]  << translations[j][2]);
              
            // }












          /*OBTAINED THE POSITION VALUES OF THE MARKERS*/


            // Assuming getTranslation returns a vector of size 3 (x, y, z)
            markerTrans_.push_back(getTranslation(markers_[i].getTransformMatrix()));
              RCLCPP_INFO_STREAM(this->get_logger(), "translation x y z is:" << markerTrans_.at(i).at(0)<< markerTrans_.at(i).at(1)<< markerTrans_.at(i).at(2));























        }

                                                  RCLCPP_INFO(this->get_logger(), "e12345i212345i212345i212345i2xits for but still in if LOOPPPPPPPPPPPPPPPPPPPPP ");

      }
                                          //RCLCPP_INFO(this->get_logger(), "2222222 exits THIS LASTLASTLASTLAST if LOOP ");

      // publish input image with markers drawn on it
      if (publishImage) {
                                                        //RCLCPP_INFO(this->get_logger(), "33333 enters THIS LASTLASTLASTLAST if LOOP ");

        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_;
        image_pub_.publish(out_msg.toImageMsg());
      }
                                          //RCLCPP_INFO(this->get_logger(), "333 exits THIS LASTLASTLASTLAST if LOOP ");

      // publish image after internal image processing
      if (publishDebug) {
                                                        //RCLCPP_INFO(this->get_logger(), "4444 enters THIS LASTLASTLASTLAST if LOOP ");

        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_.getThresholdedImage();
        debug_pub_.publish(debug_msg.toImageMsg());
      }
                                                //RCLCPP_INFO(this->get_logger(), "4444 exits THIS LASTLASTLASTLAST if LOOP ");

    }catch (cv_bridge::Exception & e) {
                                                          //RCLCPP_INFO(this->get_logger(), "ENTERED CATCH ");
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
                                                              //RCLCPP_INFO(this->get_logger(), "EXITED CATCH ");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ArucoMarkerPublisher> marker_pub = std::make_shared<ArucoMarkerPublisher>();
  marker_pub->setup();
  rclcpp::spin(marker_pub);
  std::cout << "does this main even ruang; sikfja;" << std::endl; // RCLCPP_INFO_STREAM(this->get_logger(), "DOESN'T IT RUSNG HTINSG FUNCITON main");
  std::cout << "correction,. actually runs once;" << std::endl; // RCLCPP_INFO_STREAM(this->get_logger(), "DOESN'T IT RUSNG HTINSG FUNCITON main");
  rclcpp::shutdown();
}
