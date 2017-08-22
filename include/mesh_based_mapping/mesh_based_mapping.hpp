/*********************************************************************************
 *  Mesh Mapping- Code reference for the paper Lucas and Chli - IROS 2016
 *  Copyright (c) 2017, Vision for Robotics Lab / ETH Zurich
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Created on: Jan 1, 2017
 *      Author: Lucas Teixeira (lteixeira@mavt.ethz.ch)
 *
 *
 *********************************************************************************/


#ifndef MESH_BASED_MAPPING_MESH_MAPPING_H_
#define MESH_BASED_MAPPING_MESH_MAPPING_H_

#include <iomanip>
#include <memory>
#include <Eigen/Core>

namespace GEOM_FADE2D {
class Point2;
class Triangle2;
class Fade_2D;
}

namespace mesh_based_mapping {

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >
VecPoint3f;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >
VecPoint2f;
typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> >
VecTriangle;

class MeshMapper {
 public:
  MeshMapper(double laplace_alpha = 0.1,
             unsigned int smoothing_iteration = 3,
             double max_delta = 20);

  ~MeshMapper();


  void SetPoints(double focal_u, double focal_v,
                 double center_u, double center_v,
                 uint dim_u, uint dim_v, const VecPoint3f &in_landmarks_3d);

  void SetPoints(const VecPoint2f &in_landmarks_2d,
                 const VecPoint3f &in_landmarks_3d);

  void ComputeMesh();

  bool GetFilteredLandmarks(const VecPoint3f *&out_landmarks_3d);

  bool GetMesh(const VecPoint3f *&out_landmarks_3d,
               const VecPoint2f *&out_landmarks_2d,
               const VecTriangle
               * &out_triangles); //on the landmarks vector could include unused landmaks by the triangles.(noise)

  void Clear();


  void SaveObj(std::string filepath, uint dim_u=0, uint dim_v=0);

 private:

  void convert2dLandmarks(const VecPoint2f &in_landmarks_2d);

  void ProjectLandmarks(const double &focalU, const double &focalV,
                        const double &centerU, const double &centerV, const double &dimU,
                        const double &dimV, const mesh_based_mapping::VecPoint3f &landmarks);

  void filteringAndSmoothing(mesh_based_mapping::VecPoint3f &points3d,
                             const std::vector<GEOM_FADE2D::Triangle2 *> &triangles,
                             std::vector<bool> &blacklist,
                             double laplaceAlpha, unsigned int smoothingIteration,
                             double maxDelta);

  double laplace_alpha_;
  unsigned int smoothing_iteration_;
  double max_delta_;

  std::vector<GEOM_FADE2D::Point2> *landmarks_2d_;
  GEOM_FADE2D::Fade_2D *fade_; //stores most of the information
  std::vector<GEOM_FADE2D::Triangle2 *> triangles_;//pointers to fade_ structure

  VecPoint3f landmarks_3d_;
  VecPoint3f filtered_landmarks_3d_output_;
  std::vector<bool> triangle_blacklist_;

  VecTriangle triangles_output_;
  VecPoint2f landmarks_2d_output_;

};

}


#endif
