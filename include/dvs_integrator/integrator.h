#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dvs_msgs/EventArray.h>
#include <opencv2/core/core.hpp>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dvs_integrator/dvs_integratorConfig.h>

namespace dvs_integrator
{

class Integrator {
public:
  Integrator(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~Integrator();

private:
  ros::NodeHandle nh_;

  // Callback functions
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);

  // Subscribers
  ros::Subscriber event_sub_;

  // Publishers
  image_transport::Publisher image_pub_;
  image_transport::Publisher time_map_pub_;
  cv::Mat state_time_map_;
  cv::Mat state_image_;
  void publishState(double t_last_);

  enum ConvolutionMask
  {
    IDENTITY, BLUR, SOBELX, SOBELY, LAPLACIAN
  } conv_mask_idx_;

  // Dynamic reconfigure
  void reconfigureCallback(dvs_integrator::dvs_integratorConfig &config, uint32_t level);
  boost::shared_ptr<dynamic_reconfigure::Server<dvs_integrator::dvs_integratorConfig> > server_;
  dynamic_reconfigure::Server<dvs_integrator::dvs_integratorConfig>::CallbackType dynamic_reconfigure_callback_;

  double alpha_cutoff_; // alpha
  int convolution_mask_;
  double c_pos_; // contrast threshold for positive events
  double c_neg_; // contrast threshold for netagite events

  void minMaxLocRobust(const cv::Mat& image, double& min, double& max,
                       const double& percentage_pixels_to_discard);
};

} // namespace
