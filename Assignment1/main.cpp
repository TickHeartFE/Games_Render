#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

// 最后得到的exe存储再bulid文件夹中
// bulid to get the Rasterizer.exe中

// Excutable exe Rasterizer.exe

// PI 
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 需要做的就是这个model_matrix
// 获取模型矩阵
Eigen::Matrix4f get_model_matrix(float rotation_angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // 首先角度转为弧度制
    rotation_angle = rotation_angle / 180 * MY_PI;
    // Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotation;

    rotation << cos(rotation_angle), -sin(rotation_angle), 0, 0,
        sin(rotation_angle), cos(rotation_angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    model = rotation * model;


    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // 在这里构造对应的透视投影矩阵

    eye_fov = eye_fov / 180 * MY_PI;
    Eigen::Matrix4f aspect_fovY;

    float ty = -1.0f / tan(eye_fov / 2.0f);
    aspect_fovY << (ty / aspect_ratio), 0, 0, 0,
        0, ty, 0, 0,
        0, 0, (zNear + zFar) / (zNear - zFar), (-2 * zNear * zFar) / (zNear - zFar),
        0, 0, 1, 0;

    projection = aspect_fovY * projection;


    return projection;
}

// 新定义的函数,获取对应的旋转量
// 绕任意过原点的轴
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle) {
    // 首先依旧是弧度制和角度制的相互转换
    angle = angle / 180 * MY_PI;
    Eigen::Matrix4f any_rotation = Eigen::Matrix4f::Zero();

    any_rotation(3, 3) = 1;

    Eigen::Vector3f normal_axis = axis.normalized();
    Eigen::Matrix3f mult_factor;

    // 构造一个mult_factor矩阵
    mult_factor << 0, -normal_axis.z(), normal_axis.y(),
        normal_axis.z(), 0, -normal_axis.x(),
        -normal_axis.y(), normal_axis.x(), 0;

    mult_factor = cos(angle) * Eigen::Matrix3f::Identity()
        + (1 - cos(angle)) * normal_axis * normal_axis.transpose()
        + sin(angle) * mult_factor;

    any_rotation.block(0, 0, 2, 2) = mult_factor.block(0, 0, 2, 2);

    return any_rotation;

}


// main函数逻辑
int main(int argc, const char** argv) {
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    // 如果命令行参数大于3，就处理对应的参数
    // set command_line = true
    // Then argv[2] -> angle, argv[3] -> file name
    if(argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if(argc == 4) {
            filename = std::string(argv[3]);
        } else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = { 0, 0, 5 };

    std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

    std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    // 用key维护angle
    int key = 0;
    // 记录frame_count
    int frame_count = 0;

    if(command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Eigen::Vector3f(-1, 1, 0), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);


        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Eigen::Vector3f(-1, 1, 0), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        // draw完之后, 使用OpenCV把一个点向量，转化为一张图片
        // OpenCV -> Get the Picture
        // to get the image by the OpenCV
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);


        // show the image here
        // to show the gameplayability
        cv::imshow("image", image);
        key = cv::waitKey(10);

        // Test Code Here
        // std::cout << "This is guin Test" << std::endl;
        std::cout << "frame count: " << frame_count++ << '\n';

        if(key == 'a') {
            angle += 10;
        } else if(key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
