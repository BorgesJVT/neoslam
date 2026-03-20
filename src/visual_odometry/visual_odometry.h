#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class VisualOdometry
{
public:

  VisualOdometry(int vtrans_image_x_min, int vtrans_image_x_max,
                 int vtrans_image_y_min, int vtrans_image_y_max,
                 int vrot_image_x_min, int vrot_image_x_max,
                 int vrot_image_y_min, int vrot_image_y_max,
                 double camera_fov_deg, double camera_hz,
                 double vtrans_scaling, double vtrans_max);

  void on_image(const unsigned char * data, bool greyscale, unsigned int image_width, unsigned int image_height, double *vtrans_ms, double *vrot_rads);

  template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
      ar & IMAGE_HEIGHT;
      ar & IMAGE_WIDTH;

      ar & VTRANS_IMAGE_X_MIN;
      ar & VTRANS_IMAGE_X_MAX;
      ar & VTRANS_IMAGE_Y_MIN;
      ar & VTRANS_IMAGE_Y_MAX;

      ar & VROT_IMAGE_X_MIN;
      ar & VROT_IMAGE_X_MAX;
      ar & VROT_IMAGE_Y_MIN;
      ar & VROT_IMAGE_Y_MAX;

      ar & CAMERA_FOV_DEG;
      ar & CAMERA_HZ;

      ar & VTRANS_SCALING;
      ar & VTRANS_MAX;

      ar & vtrans_profile;
      ar & vrot_profile;

      ar & vtrans_prev_profile;
      ar & vrot_prev_profile;

      ar & first;
    }

private:

  VisualOdometry() {;}

  int IMAGE_HEIGHT;
  int IMAGE_WIDTH;

  int VTRANS_IMAGE_X_MIN;
  int VTRANS_IMAGE_X_MAX;
  int VTRANS_IMAGE_Y_MIN;
  int VTRANS_IMAGE_Y_MAX;

  int VROT_IMAGE_X_MIN;
  int VROT_IMAGE_X_MAX;
  int VROT_IMAGE_Y_MIN;
  int VROT_IMAGE_Y_MAX;

  double CAMERA_FOV_DEG;
  double CAMERA_HZ;

  double VTRANS_SCALING;
  double VTRANS_MAX;

  std::vector<double> vtrans_profile;
  std::vector<double> vrot_profile;

  std::vector<double> vtrans_prev_profile;
  std::vector<double> vrot_prev_profile;

  bool first;

  void visual_odo(double *data, unsigned short width, double *olddata, double *vtrans_ms, double *vrot_rads);

  void convert_view_to_view_template(double *current_view, const unsigned char *view_rgb, bool grayscale, int X_RANGE_MIN, int X_RANGE_MAX, int Y_RANGE_MIN, int Y_RANGE_MAX);

};
