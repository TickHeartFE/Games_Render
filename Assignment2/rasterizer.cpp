// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions) {
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices) {
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols) {
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v) {
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // 使用叉乘来解决点是否在三角形中
    // Eigen::Vector3f p0p1(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 1.f);
    // Eigen::Vector3f p1p2(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 1.f);
    // Eigen::Vector3f p2p0(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 1.f);

    // // 构造p0p p1p p2p
    // Eigen::Vector3f p0p(x - _v[0].x(), y - _v[0].y(), 1.f);
    // Eigen::Vector3f p1p(x - _v[1].x(), y - _v[1].y(), 1.f);
    // Eigen::Vector3f p2p(x - _v[2].x(), y - _v[2].y(), 1.f);

    // return (p0p1.cross(p0p).z() > 0 && p1p2.cross(p1p).z() > 0 && p2p0.cross(p2p).z() > 0) || (p0p1.cross(p0p).z() < 0 && p1p2.cross(p1p).z() < 0 && p2p0.cross(p2p).z() < 0);

    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // Eigen::Vector3f p0p1(_v[0].x() - _v[1].x(), _v[0].y() - _v[1].y(), 1.0f);
    // Eigen::Vector3f p1p2(_v[1].x() - _v[2].x(), _v[1].y() - _v[2].y(), 1.0f);
    // Eigen::Vector3f p2p0(_v[2].x() - _v[0].x(), _v[2].y() - _v[0].y(), 1.0f);

    // Eigen::Vector3f p0p(_v[0].x() - x, _v[0].y() - y, 1.0f);
    // Eigen::Vector3f p1p(_v[1].x() - x, _v[1].y() - y, 1.0f);
    // Eigen::Vector3f p2p(_v[2].x() - x, _v[2].y() - y, 1.0f);

    // if(p0p1.cross(p0p).z() > 0.f) {
    //     return p1p2.cross(p1p).z() > 0.f && p2p0.cross(p2p).z() > 0.f;
    // } else {
    //     return p1p2.cross(p1p).z() < 0.f && p2p0.cross(p2p).z() < 0.f;
    // }


    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // Vector3f Q = { x,y,0 };

    // Vector3f p0p1 = _v[1] - _v[0];
    // Vector3f p0Q = Q - _v[0];

    // Vector3f p1p2 = _v[2] - _v[1];
    // Vector3f p1Q = Q - _v[1];

    // Vector3f p2p0 = _v[0] - _v[2];
    // Vector3f p2Q = Q - _v[2];

    // //类定义里面已经定义是逆时针，所以只用考虑同正情况。
    // return p0p1.cross(p0Q).z() > 0 && p1p2.cross(p1Q).z() > 0 && p2p0.cross(p2Q).z() > 0;

    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // 检查某个点是否在三角形里
    Eigen::Vector3f p0p1(_v[0].x() - _v[1].x(), _v[0].y() - _v[1].y(), 1.0f);
    Eigen::Vector3f p1p2(_v[1].x() - _v[2].x(), _v[1].y() - _v[2].y(), 1.0f);
    Eigen::Vector3f p2p0(_v[2].x() - _v[0].x(), _v[2].y() - _v[0].y(), 1.0f);

    Eigen::Vector3f p0p(_v[0].x() - x, _v[0].y() - y, 1.0f);
    Eigen::Vector3f p1p(_v[1].x() - x, _v[1].y() - y, 1.0f);
    Eigen::Vector3f p2p(_v[2].x() - x, _v[2].y() - y, 1.0f);

    if(p0p1.cross(p0p).z() > 0.f) {
        return p1p2.cross(p1p).z() > 0.f && p2p0.cross(p2p).z() > 0.f;
    } else {
        return p1p2.cross(p1p).z() < 0.f && p2p0.cross(p2p).z() < 0.f;
    }



}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v) {
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return { c1,c2,c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type) {
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for(auto& i : ind) {
        Triangle t;
        Eigen::Vector4f v[] = {
            // mvp * to_vec4(buf[i[0]], 1.0f),
            // mvp * to_vec4(buf[i[1]], 1.0f),
            // mvp * to_vec4(buf[i[2]], 1.0f
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)
        };

        // Homogeneous division
        // 透视除法
        for(auto& vec : v) {
            vec /= vec.w();
        }

        // 视口转换 
        // Viewport transformation
        // 在这里进行一个视口坐标系的变换
        for(auto& vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for(int i = 0; i < 3; ++i) {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }


        // auto col_x = col[i[0]];
        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);

        // 光栅化后在这里draw
        if(bIsSSAA) {
            for(int x = 0; x < width; x++) {
                for(int y = 0; y < height; y++) {
                    Eigen::Vector3f color(0, 0, 0);
                    for(int i = 0; i < 4; i++) {
                        color += frame_buf_2xSSAA[get_index(x, y)][i];
                    }
                    color /= 4;
                    set_pixel(Eigen::Vector3f(x, y, 1.0f), color);
                }

            }
        }

    }
    // std::cout << "hello" << std::endl;
}

// Screen space rasterization
// 屏幕空间下的光栅化算法
void rst::rasterizer::rasterize_triangle(const Triangle& t) {


    auto v = t.toVector4();

    std::vector<float> x_arry{ v[0].x(), v[1].x(), v[2].x() };
    std::vector<float> y_arry{ v[0].y(), v[1].y(), v[2].y() };
    std::sort(x_arry.begin(), x_arry.end());
    std::sort(y_arry.begin(), y_arry.end());
    int x_min = floor(x_arry[0]), x_max = ceil(x_arry[2]),
        y_min = floor(y_arry[0]), y_max = ceil(y_arry[2]);

    for(int x = x_min; x < x_max; x++) {
        for(int y = y_min; y < y_max; y++) {
            if(bIsSSAA) {
                Eigen::Vector3f point(x, y, 1.0);
                // child pixel 划分四个子像素
                int inside_count = 0;
                int update_depth = 0;
                int index = 0;
                for(int i = 0.25; i < 1; i += 0.5) {
                    for(int j = 0.25; j < 1; j += 0.5) {
                        if(insideTriangle(x + i, y + j, t.v)) {
                            auto [alpha, beta, gamma] = computeBarycentric2D(x + i, y + j, t.v);
                            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;
                            if(z_interpolated < depth_buf_2xSSAA[get_index(x, y)][index]) {
                                point << x + i, y + j, 1.0;
                                // 更新两个buffer
                                frame_buf_2xSSAA[get_index(x, y)][index] = t.getColor();
                                depth_buf_2xSSAA[get_index(x, y)][index] = z_interpolated;
                                inside_count++;
                                update_depth += z_interpolated;
                            }

                        }
                        index++;
                    }
                }

            } else {
                if(insideTriangle(x + 0.5f, y + 0.5f, t.v)) {
                    auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if(z_interpolated < depth_buf[get_index(x, y)]) {
                        Eigen::Vector3f point(x, y, 1.0f);
                        set_pixel(point, t.getColor());
                        depth_buf[get_index(x, y)] = z_interpolated;
                    }
                }
            }


        }
    }

}



void rst::rasterizer::set_model(const Eigen::Matrix4f& m) {
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v) {
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p) {
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
    if((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
        for(int i = 0; i < frame_buf_2xSSAA.size(); i++) {
            frame_buf_2xSSAA[i].resize(4);
            std::fill(frame_buf_2xSSAA[i].begin(), frame_buf_2xSSAA[i].end(), Eigen::Vector3f{ 0, 0, 0 });
        }
    }
    if((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        for(int i = 0; i < depth_buf_2xSSAA.size(); i++) {
            depth_buf_2xSSAA[i].resize(4);
            std::fill(depth_buf_2xSSAA[i].begin(), depth_buf_2xSSAA[i].end(), std::numeric_limits<float>::infinity());
        }
    }
}

rst::rasterizer::rasterizer(int w, int h): width(w), height(h) {
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_2xSSAA.resize(w * h);
    depth_buf_2xSSAA.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y) {
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color) {
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

// clang-format on