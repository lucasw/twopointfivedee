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
  bool getIntersection(const float sx, const float sy,
      const float dx, const float dy, float& dist, float& texture_x)
  {
    float x = sx;
    float y = sy;
    dist = 0;

    // TODO(lucasw) redefine in terms of max map size
    const float EPSILON = 0.0001;

    const int max_range = 328;
    for (size_t i = 0; i < max_range; ++i)
    {
      // TODO(lucasw) get the intersection with a right or left hand
      // vertical grid line and the horizontal up or down grid line,
      // then find closest one.

      float edx = 0;
      float edy = 0;
      if (dx > EPSILON)
      {
        // right hand vertical line intersection
        edx = float(size_t(x)) + 1.0 - x + EPSILON;
        edy = edx / dx * dy;
      }
      else if (dx < -EPSILON)
      {
        edx = x - float(size_t(x)) - EPSILON;
        edy = edx / dx * dy;
      }
      float ndx = 0;
      float ndy = 0;
      if (dy > EPSILON)
      {
        // right hand vertical line intersection
        ndy = float(size_t(y)) + 1.0 - y + EPSILON;
        ndx = ndy / dy * dx;
      }
      else if (dy < -EPSILON)
      {
        ndy = y - float(size_t(y)) - EPSILON;
        ndx = ndy / dy * dx;
      }

      float dx2, dy2;
      // which coord to use for the wall texture
      bool use_x_not_y;
      // is edx the nearest intersection (or ndx has no intersection at all)
      if ((edx != 0) && ((std::abs(edx) <= std::abs(ndx)) ||
          (ndx == 0)))
      {
        dx2 = edx;
        dy2 = edy;
        use_x_not_y = false;
      }
      else if ((ndy != 0) && ((std::abs(ndy) <= std::abs(edy)) ||
          (edy == 0)))
      {
        dx2 = ndx;
        dy2 = ndy;
        use_x_not_y = true;
      }
      else
      {
        ROS_ERROR_STREAM("bad intersection " << x << " " << y << ", " << dx << " " << dy);
        ROS_ERROR_STREAM("e " << edx << " " << edy << ", n " << ndx << " " << ndy);
        return false;
      }

      // TODO(lucasw) will this gaurantee x + dx2 or y is going to be
      // in right grid cell, or could it fall a little short?
      x += dx2;
      y += dy2;
      if (size_t(y) >= floor_map_.rows)
        return false;
      if (size_t(x) >= floor_map_.cols)
        return false;

      // test the grid location (y and x are actually the center of a grid cell)
      if (floor_map_.at<uint8_t>(y, x) > 0)
      {
        dist = std::sqrt((y - sy) * (y - sy) + (x - sx) * (x - sx));
        if (use_x_not_y)
          texture_x = x - size_t(x);
        else
          texture_x = y - size_t(y);
        return true;
      }
      // TODO(lucasw) this approach is going to miss some
      // intersections near edges
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
            cv::Scalar(256 - dist * 4, 256 - dist, 255.0 * texture_x));
      }

      // draw the boundaries of where the camera can see
      if ((i == 0) || (i == output_.cols - 1))
      {
        // ROS_INFO_STREAM(sky_color);
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
