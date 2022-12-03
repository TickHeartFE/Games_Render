/*
 * @Author: GuinGuinSzu guinguinboys@gmail.com
 * @Date: 2022-11-28 23:38:36
 * @LastEditors: GuinGuinSzu guinguinboys@gmail.com
 * @LastEditTime: 2022-12-03 14:00:24
 * @FilePath: \Assignment1\Triangle.cpp
 * @Description:
 *
 * Copyright (c) 2022 by GuinGuinSzu guinguinboys@gmail.com, All Rights Reserved.
 */

#include "Triangle.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>

Triangle::Triangle() {
    v[0] << 0, 0, 0;
    v[1] << 0, 0, 0;
    v[2] << 0, 0, 0;

    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

void Triangle::setVertex(int ind, Eigen::Vector3f ver) { v[ind] = ver; }

void Triangle::setNormal(int ind, Vector3f n) { normal[ind] = n; }

void Triangle::setColor(int ind, float r, float g, float b) {
    // 进行一个颜色剪枝
    // 不符合rgb的颜色直接被剪枝掉
    if((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) || (b > 255.)) {
        // throw std::runtime_error("Invalid color values");
        throw std::runtime_error("Invalid color values");
    }

    // color[ind] = Vector3f((float)r / 255., (float)g / 255., (float)b / 255.);
    // then set the color
    // set the vextex color
    // color[ind] = Vector3f((float)r / 255.f, (float)g / 255.f, (float)b / 255.f);

    color[ind] = Vector3f((float)r / 255.f, (float)g / 255.f, (float)b / 255.f);

    return;
}

void Triangle::setTexCoord(int ind, float s, float t) {
    tex_coords[ind] = Vector2f(s, t);
}

std::array<Vector4f, 3> Triangle::toVector4() const {
    std::array<Vector4f, 3> res;
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) {
        return Vector4f(vec.x(), vec.y(), vec.z(), 1.f);
    });
    return res;
}
