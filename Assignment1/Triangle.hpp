//
// Created by LEI XU on 4/11/19.
//

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <eigen3/Eigen/Eigen>

using namespace Eigen;


// 三角形的类别
// Triangle for this
// Triangle
class Triangle
{
public:
  // the original coordinates of the triangle, v0, v1, v2 in counter clockwise order
  // 三角形的原始的坐标,v0,v1,v2按照逆时针的顺序
  Vector3f v[3];
  
  /*Per vertex values*/
  // 三角形顶点的属性 
  // 颜色
  Vector3f color[3];      // color at each vertex;
  // uv贴图
  Vector2f tex_coords[3]; // texture u,v
  // 顶点法线
  Vector3f normal[3];     // normal vector for each vertex

  Vector3f normal[3];      // nomal vector for each vertex

  // Texture *tex; *tex to make the Texture Mapping
  Triangle();

  // Texture *tex; *tex to make the Texture Mapping

  // get the original coordinates
  // 获取三个点abc的值
  Eigen::Vector3f a() const { return v[0]; }
  Eigen::Vector3f b() const { return v[1]; }
  Eigen::Vector3f c() const { return v[2]; }

  void setVertex(int ind, Vector3f ver); /*set i-th vertex coordinates */
  void setNormal(int ind, Vector3f n);   /*set i-th vertex normal vector*/
  void setColor(int ind, float r, float g, float b); /*set i-th vertex color*/
  void setTexCoord(int ind, float s,
                    float t); /*set i-th vertex texture coordinate*/
  std::array<Vector4f, 3> toVector4() const;
};

#endif // RASTERIZER_TRIANGLE_H
