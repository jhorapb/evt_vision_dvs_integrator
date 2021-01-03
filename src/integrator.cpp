#include "dvs_integrator/integrator.h"
#include <cv_bridge/cv_bridge.h>
#include <math.h>

namespace dvs_integrator {

Integrator::Integrator(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{
  // Get parameters of display method
  std::string display_method_str;
  nh_private.param<double>("cut_off", alpha_cutoff_, 5.);

  // Set up subscribers and publishers
  event_sub_ = nh_.subscribe("events", 1, &Integrator::eventsCallback, this);

  image_transport::ImageTransport it_(nh_);
  // We define the publishers to advertise the time map and image
  time_map_pub_ = it_.advertise("time_map", 1);
  image_pub_ = it_.advertise("image_out", 1);

  // Dynamic reconfigure                  
  dynamic_reconfigure_callback_ = boost::bind(&Integrator::reconfigureCallback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<dvs_integrator::dvs_integratorConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);

  c_pos_ = 0.1;
  c_neg_ = 0.1; // Later we may use different contrast sensitivities for positive and negative events
}


Integrator::~Integrator()
{
  // close the publishers
  time_map_pub_.shutdown();
  image_pub_.shutdown();
}


void Integrator::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // Need to update the state (time_map and brightness image) if there are no subscribers on the output image

  std::cout << "alpha_cutoff_ = " << alpha_cutoff_ << std::endl;
  const double t_first_ = msg->events[0].ts.toSec();
  const double t_last_ = msg->events[msg->events.size()-1].ts.toSec();
    
  if (!(state_time_map_.rows == msg->height && state_time_map_.cols == msg->width))
  {
    // Allocate memory for time map and for image out
    state_time_map_ = cv::Mat(msg->height, msg->width, CV_64FC1, cv::Scalar(t_first_));
    state_image_ = cv::Mat::zeros(msg->height, msg->width, CV_64FC1);
  }

  // Process events in the message msg, one by one (in a loop)

  double threshold_pol;
  for (const dvs_msgs::Event& ev : msg->events)
  {
    if (ev.polarity > 0)
      threshold_pol = c_pos_;
    else
      threshold_pol = -c_neg_;

    double beta = exp(-alpha_cutoff_ * (ev.ts.toSec() - state_time_map_.at<double>(ev.y, ev.x)));
    state_image_.at<double>(ev.y, ev.x) = beta * state_image_.at<double>(ev.y, ev.x) + threshold_pol;
    state_time_map_.at<double>(ev.y, ev.x) = ev.ts.toSec();
  }

  // Exit if there are no subscribers
  if (image_pub_.getNumSubscribers() + time_map_pub_.getNumSubscribers() > 0)
  {
    publishState(t_last_);
  }
  else
  {
    return;
  }

}


void Integrator::publishState(double t_last_)
{
  // Just publish the current state (time map and image)

  // initialize, including header
  cv_bridge::CvImage cv_image, cv_image_time;
  cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = "mono8";
  cv_image_time.header.stamp = ros::Time::now();
  cv_image_time.encoding = "mono8";

  std::cout << "t_last_ = " << t_last_ << std::endl;

  // Maybe use minMaxLocRobust
  double min_time, max_time, min_img, max_img;
  minMaxLocRobust(state_time_map_, min_time, max_time, 1.0);
  minMaxLocRobust(state_image_, min_img, max_img, 1.0);

  const double diff_time = max_time - min_time;
  const double diff_img = max_img - min_img;

  // Scaling pixel values to range [0, 255]
  const double time_scale = 255.0 / diff_time;
  const double img_scale = 255.0 / diff_img;
  cv::Mat image_time_norm = time_scale * (state_time_map_ - min_time);
  cv::Mat image_norm = img_scale * (state_image_ - min_img);

  if (diff_time != 0)
  {
    image_time_norm.convertTo(cv_image_time.image, CV_8U);
  }
  
  if (diff_img != 0)
  {
    image_norm.convertTo(cv_image.image, CV_8U);
  }

  // publish
  time_map_pub_.publish(cv_image_time.toImageMsg());
  image_pub_.publish(cv_image.toImageMsg());
}


void Integrator::reconfigureCallback(dvs_integrator::dvs_integratorConfig &config, uint32_t level)
{
  alpha_cutoff_ = config.Cutoff_frequency;
  convolution_mask_ = config.Convolution_mask;
}

void Integrator::minMaxLocRobust(const cv::Mat& image, double& rmin, double& rmax,
                                 const double& percentage_pixels_to_discard)
{
  cv::Mat image_as_row = image.reshape(0,1);
  cv::Mat image_as_row_sorted;
  cv::sort(image_as_row, image_as_row_sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
  image_as_row_sorted.convertTo(image_as_row_sorted, CV_64FC1);
  const int single_row_idx_min = (0.5*percentage_pixels_to_discard/100.)*image.total();
  const int single_row_idx_max = (1 - 0.5*percentage_pixels_to_discard/100.)*image.total();
  rmin = image_as_row_sorted.at<double>(single_row_idx_min);
  rmax = image_as_row_sorted.at<double>(single_row_idx_max);
}

} // namespace
