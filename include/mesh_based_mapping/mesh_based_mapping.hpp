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
#include <Eigen/Core>
#include <memory>

namespace GEOM_FADE2D {
class Point2;
class Triangle2;
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

  void Clear();

  void SetPoints(double focal_u, double focal_v,
                 double center_u, double center_v,
                 uint dim_u, uint dim_v, const VecPoint3f &in_landmarks_3d);

  void SetPoints(const VecPoint2f &in_landmarks_2d,
                 const VecPoint3f &in_landmarks_3d);

  void ComputeMesh() {
    // ComputeMeshInPlace(landmarks_2d_.get(), landmarks_3d_, triangles_);
  }

  bool GetFilteredLandmarks(const VecPoint3f *&out_landmarks_3d) const;

  bool GetMesh(const VecPoint3f *&out_landmarks_3d,
               const VecPoint2f *&out_landmarks_2d,
               const VecTriangle *&out_triangles);

//  void ComputeMeshInPlace(const VecPoint2f &in_landmarks_2d,
//                          VecPoint3f &in_out_landmarks_3d, VecTriangle &out_triangles);


 private:

  double laplace_alpha_;
  unsigned int smoothing_iteration_;
  double max_delta_;

  std::vector<GEOM_FADE2D::Point2> *landmarks_2d_;
  std::vector<GEOM_FADE2D::Triangle2 *> triangles_;

  VecPoint3f landmarks_3d_;

  VecTriangle triangles_output_;
  VecPoint2f landmarks_2d_output_;

  void ComputeMeshInPlace(const std::vector<GEOM_FADE2D::Point2> *in_landmarks_2d,
                          VecPoint3f &in_out_landmarks_3d, VecTriangle &out_triangles) {
    ;
  }


  void ProjectLandmarks(const double &focalU, const double &focalV,
                        const double &centerU, const double &centerV,
                        const double &dimU, const double &dimV,
                        const VecPoint3f &landmarks,
                        std::vector<GEOM_FADE2D::Point2> &landmarks2D);

//  /**

//  */
//  void filteringAndSmoothing(VecPoint3f &points3d,
//                             std::vector<GEOM_FADE2D::Triangle2 *> triangles,
//                             std::vector<bool> &blacklist,
//                             const double laplaceAlpha = 0.1,
//                             const unsigned int smoothingIteration = 3,
//                             const double maxDelta = 20);


//  void buildMeshDepthMap(const double &focalU,
//                         const double &focalV,
//                         const double &centerU,
//                         const double &centerV,
//                         const double &dimU,
//                         const double &dimV,
//                         VecPoint3f &in_out_landmarks_3d,
//                         VecTriangle &output_triangles,
//                         const double laplaceAlpha = 0.1,
//                         const unsigned int smoothingIteration = 3,
//                         const double maxDelta = 0.2);

};





}


#endif
