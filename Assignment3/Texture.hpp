//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBiliner(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = v * height;

        float u0 = (int)u_img;
        float u1 = (int)u_img + 1;
        float v0 = (int)v_img;
        float v1 = (int)v_img + 1;

        float s = u_img - u0;           //ratio
        float t = v_img - v0;   

        auto color00 = getColor(u0 / width, v0 / height);
        auto color01 = getColor(u0 / width, v1 / height);
        auto color10 = getColor(u1 / width, v0 / height);
        auto color11 = getColor(u1 / width, v1 / height);

        auto color = (color00 * (1.0 - s) + color01 * s) * (1.0 - t) + (color10 * (1.0 - s) + color11 * s) * t;   //Biliner

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
