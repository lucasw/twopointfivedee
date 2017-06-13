/**
  Copyright 2017 Lucas Walter
  GNU GPL v 3.0
*/

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>


class Labyrinth
{
  ros::NodeHandle nh_;
  cv::Mat wall_texture_;
  cv::Mat floor_map_;
  cv::Mat output_;

  uint16_t x_;
  uint16_t y_;
  // only 256 ways to aim
  uint16_t dir_;
  uint16_t fov_;
public:
  // TODO(lucasw) later have image subscribers for the texture and map
  Labyrinth(const std::string floor_name, const std::string wall_name) :
    x_(128),
    y_(128),
    dir_(16384),
    fov_(24000)
  {
    wall_texture_ = cv::imread(wall_name);
    if (wall_texture_.empty())
    {
      ROS_ERROR_STREAM("couldn't load " << wall_name);
      return;
    }
    floor_map_ = cv::imread(floor_name, CV_LOAD_IMAGE_COLOR);
    if (floor_map_.empty())
    {
      ROS_ERROR_STREAM("couldn't load " << floor_name);
      return;
    }
    output_ = cv::Mat(cv::Size(320, 240), CV_8UC3, cv::Scalar::all(0));

    ROS_INFO_STREAM("entering the labyrinth");
    while (ros::ok())
    {
      draw();
      ros::Duration(0.05).sleep();
    }
  }

  ~Labyrinth()
  {

  }

  bool draw()
  {
    output_ = cv::Scalar(0);
    for (size_t i = 0; i < output_.cols; ++i)
    {
      // uint8_t dir = dir_ - 
    }

    cv::imshow("output", floor_map_);  // output_);
    const int key = cv::waitKey(10);
    if (key == 'w')
      y_ -= 1;
    else if (key == 's')
      y_ += 1;
    else if (key == 'a')
      x_ -= 1;
    else if (key == 'd')
      x_ += 1;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lucaswlabyrinth");
  Labyrinth labyrinth("data/floor.png", "data/wall.png");
  // ros::spin();
}
