/**
  Copyright 2017 Lucas Walter
  GNU GPL v 3.0
*/

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class Labyrinth
{
  ros::NodeHandle nh_;
  cv::Mat wall_texture_;
  cv::Mat floor_map_;
  cv::Mat output_;

  float x_;
  float y_;
  float dir_;
  float angle_per_column_;
  float forty_five_deg_;
public:
  // TODO(lucasw) later have image subscribers for the texture and map
  Labyrinth(const std::string floor_name, const std::string wall_name) :
    x_(128),
    y_(128),
    dir_(0.0),
    angle_per_column_(60 / 180.0 * M_PI / 320.0),
    forty_five_deg_(45.0 / 180.0 * M_PI)
  {
    wall_texture_ = cv::imread(wall_name, CV_LOAD_IMAGE_COLOR);
    if (wall_texture_.empty())
    {
      ROS_ERROR_STREAM("couldn't load " << wall_name);
      return;
    }
    floor_map_ = cv::imread(floor_name, CV_LOAD_IMAGE_GRAYSCALE);
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
      // ros::Duration(0.05).sleep();
    }
  }

  ~Labyrinth()
  {

  }

  // return the first map grid location that the ray intersects
  bool getIntersection(float x, float y,
      const float dx, const float dy, float& dist, float& texture_x)
  {
    // x += 0.5;
    // y += 0.5;
    dist = 0;
    const float inc_dist = std::sqrt(dx * dx + dy * dy);

    const int max_range = 328;
    for (size_t i = 0; i < max_range; ++i)
    {
      if (size_t(y + dy) >= floor_map_.rows)
        return false;
      if (size_t(x + dx) >= floor_map_.cols)
        return false;
      // test the grid location (y and x are actually the center of a grid cell)
      if (floor_map_.at<uint8_t>(y + dy, x + dx) > 0)
      {
        float fr = 0.0;
        const size_t x1 = size_t(x);
        const size_t x2 = size_t(x + dx);
        const size_t y1 = size_t(y);
        const size_t y2 = size_t(y + dy);
        if ((x1 != x2) && (y1 == y2))
        {
          fr = std::abs((float(x2) - x) / dx);
          texture_x = y + dy * fr - y1;
        }
        else if ((x1 == x2) && (y1 != y2))
        {
          fr = std::abs((float(y2) - y) / dy);
          texture_x = x + dx * fr - x1;
        }
        else
        {
          // crossed a boundary in both x and y

        }
        // x += x * fr;
        // y += y * fr;
        dist += inc_dist * fr;
        // if (std::abs(dx) > std::abs(dy))
        return true;
      }
      // TODO(lucasw) this approach is going to miss some
      // intersections near edges
      x += dx;
      y += dy;
      dist += inc_dist;
    }

    return false;
  }

  size_t getApparentHeight(const float dist)
  {
    return output_.rows / dist * 4.0;
  }

  /////////////////////////////////////////////////////////////
  bool draw()
  {
    const float scale = 4.0;
    cv::Mat map_bw, map;
    cv::resize(floor_map_, map_bw, cv::Size(), scale, scale, cv::INTER_NEAREST);
    cv::cvtColor(map_bw, map, CV_GRAY2RGB);
    cv::circle(map, cv::Point(x_ * scale, y_ * scale), 3, cv::Scalar(255, 0, 0));

    output_ = cv::Scalar(0);
    const size_t half_width = output_.cols / 2;
    const size_t half_height = output_.rows / 2;
    float cur_dir = dir_ - half_width * angle_per_column_;
    for (size_t i = 0; i < output_.cols; ++i)
    {
      cur_dir += angle_per_column_;
/**

       16384
            8192
       \ | /
32768 -    - 0
       / | \

       49152
*/

      // TODO(lucasw) get rid of floats later
      float angle = cur_dir;  // float(cur_dir) / float(1 << 16) * 2.0 * M_PI;
      if (angle > M_PI)
        angle -= 2.0 * M_PI;
      else if (angle < -M_PI)
        angle += 2.0 * M_PI;
      float dx = cos(angle);
      float dy = sin(angle);

      /*
      if ((cur_dir < forty_five_deg_) || (cur_dir > forty_five_deg_ * 7) ||
          ((cur_dir > forty_five_deg_ * 3) && (cur_dir < forty_five_deg_ * 5)))
      {
        dy = 1.0 / dx * dy;
        dx = 1.0;
      }
      else
      {
        dx = 1.0 / dy * dx;
        dy = 1.0;
      }
      */

      // draw a 'sky'
      cv::Scalar sky_color((M_PI + angle) / M_PI * 128.0, 0, 200);
      cv::line(output_,
          cv::Point(i, 0),
          cv::Point(i, output_.rows / 2),
          sky_color);

      float dist;
      float texture_x;
      if (getIntersection(x_, y_, dx, dy, dist, texture_x))
      {
        size_t height = getApparentHeight(dist);
        cv::line(output_,
            cv::Point(i, half_height - height / 2),
            cv::Point(i, half_height + height / 2),
            cv::Scalar(256 - dist * 4, 256 - dist, texture_x));
      }

      // draw the boundaries of where the camera can see
      if ((i == 0) || (i == output_.cols - 1))
      {
        ROS_INFO_STREAM(sky_color);
        cv::line(map,
            cv::Point(x_ * scale, y_ * scale),
            cv::Point(x_ * scale + dx * scale * 100, y_ * scale + dy * scale * 100),
            sky_color);
      }
    }

    cv::imshow("output", output_);

    const float angle = dir_;  // float(dir_) / float(1 << 16) * 2.0 * M_PI;
    const float dx = cos(angle);
    const float dy = sin(angle);

    // show the direction the player is facing
    cv::line(map,
        cv::Point(x_ * scale, y_ * scale),
        cv::Point(x_ * scale + dx * scale * 20, y_ * scale + dy * scale * 20),
        cv::Scalar(255, 0, 0));

    cv::imshow("map", map);

    const int key = cv::waitKey(50);
    if (key == 'w')
    {
      x_ += dx;
      y_ += dy;
    }
    else if (key == 's')
    {
      x_ -= dx;
      y_ -= dy;
    }
    else if (key == 'a')
    {
      x_ += dy;
      y_ -= dx;
    }
    else if (key == 'd')
    {
      x_ -= dy;
      y_ += dx;
    }
    else if (key == 'q')
      dir_ -= angle_per_column_ * 4;
    else if (key == 'e')
      dir_ += angle_per_column_ * 4;

    if (dir_ < 0)
      dir_ += 2.0 * M_PI;
    else if (dir_ > 2.0 * M_PI)
      dir_ -= 2.0 * M_PI;
    ROS_DEBUG_STREAM(x_ << " " << y_ << " " << angle * 180 / M_PI);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lucaswlabyrinth");
  Labyrinth labyrinth("data/floor.png", "data/wall.png");
  // ros::spin();
}
