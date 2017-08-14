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
#include <opencv2/core/core.hpp>

#include <Fade_2D.h>


namespace mesh_based_mapping {

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
    VecPoint3f;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
    VecPoint2f;
typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>
    VecTriangle;

class MeshMapper {
 public:
  MeshMapper(double laplace_alpha = 0.1,
             unsigned int smoothing_iteration = 3,
             double max_delta = 20): laplace_alpha_(laplace_alpha),
    smoothing_iteration_(smoothing_iteration), max_delta_(max_delta) {}

  void Clear() {
    landmarks_2d_.clear();
    landmarks_3d_.clear();
    triangles_.clear();
  }

  void SetPoints(double focal_u, double focal_v,
                 double center_u, double center_v,
                 uint dim_u, uint dim_v, const VecPoint3f &in_landmarks_3d) {
    landmarks_3d_ = in_landmarks_3d;

    ProjectLandmarks(focal_u, focal_v, center_u, center_v, dim_u, dim_v,
                     landmarks_3d_, landmarks_2d_);
  }

  void SetPoints(const VecPoint2f &in_landmarks_2d,
                 const VecPoint3f &in_landmarks_3d) {
    landmarks_3d_ = in_landmarks_3d;
    landmarks_2d_ = in_landmarks_2d;
  }

  void ComputeMesh() {
    ComputeMeshInPlace(landmarks_2d_, landmarks_3d_, triangles_);
  }

  bool GetMesh(const VecPoint3f *&out_landmarks_3d,
               const VecTriangle *&out_triangles) const {
    if (triangles_.size() == 0) {
      return false;
    }

    out_landmarks_3d = &landmarks_3d_;
    out_triangles = &triangles_;

    return true;
  }

  bool GetMesh(const VecPoint3f *&out_landmarks_3d,
               const VecPoint2f *&out_landmarks_2d,
               const VecTriangle *&out_triangles) {
    if (triangles_.size() == 0) {
      return false;
    }

    out_landmarks_3d = &landmarks_3d_;
    out_triangles = &triangles_;

    return true;
  }

  void ComputeMeshInPlace(const VecPoint2f &in_landmarks_2d,
                          VecPoint3f &in_out_landmarks_3d, VecTriangle &out_triangles);


 private:
  double laplace_alpha_;
  unsigned int smoothing_iteration_;
  double max_delta_;

  VecPoint3f landmarks_3d_;
  std::vector<GEOM_FADE2D::Point2> landmarks_2d_; //TODO(@weblucas) use pImpl tecnique to eliminate this
  VecTriangle triangles_;


  void ProjectLandmarks(const double &focalU, const double &focalV,
                        const double &centerU, const double &centerV,
                        const double &dimU, const double &dimV,
                        const VecPoint3f &landmarks,
                        std::vector<GEOM_FADE2D::Point2> &landmarks2D) {


    for (unsigned int i = 0 ; i < landmarks.size() ; i++) {
      Eigen::Vector3f &pt_CRef = landmarks[i];

      double x = ((pt_CRef(0) / pt_CRef(2)) * focalU) + centerU;
      double y = ((pt_CRef(1) / pt_CRef(2)) * focalV) + centerV;

      if (x <= 0 || y <= 0 || x >= dimU || y >= dimV) {
        continue;
      }

      landmarks2D.push_back(GEOM_FADE2D::Point2(x, y));
      landmarks2D.back().setCustomIndex(i);
    }
  }

  /**

  */
  void filteringAndSmoothing(VecPoint3f &points3d,
                             std::vector<GEOM_FADE2D::Triangle2 *> triangles,
                             std::vector<bool> &blacklist,
                             const double laplaceAlpha = 0.1,
                             const unsigned int smoothingIteration = 3,
                             const double maxDelta = 20) {



    if (triangles.size() < 2) {
      return;
    }

    double mean = 0;
    double std = 0;

    std::vector<double> maxEdge(triangles.size());
    std::vector<bool> borderPoint(points3d.size(), false);
    std::vector<bool> borderTriangle(triangles.size(), false);

    //TODO put back the border detection.

    //remove the big edges in 2D
    for (unsigned int i = 0; i < triangles.size();
         i++) {
      GEOM_FADE2D::Triangle2 *t = triangles[i];
      maxEdge[i] = cv::max(t->getSquaredEdgeLength(0),
                           cv::max(t->getSquaredEdgeLength(1), t->getSquaredEdgeLength(2)));
      mean += maxEdge[i] ;
    }

    mean /=  triangles.size();

    for (unsigned int i = 0; i < triangles.size(); i++) {
      double delta = maxEdge[i] - mean;
      std += delta * delta;
    }

    std = sqrt(std / triangles.size());
    double threshold = std;

    for (unsigned int i = 0; i < triangles.size(); i++) {
      GEOM_FADE2D::Triangle2 *t = triangles[i];
      double diff = maxEdge[i] - mean;

      if ((std::abs(diff) > threshold) || (borderTriangle[i] && diff > 0.6 * std)
          || (borderTriangle[i] &&
              (std::abs((points3d[t->getCorner(0)->getCustomIndex()] - points3d[t->getCorner(
                           1)->getCustomIndex()]).norm())) > 0.3)
          || (borderTriangle[i] &&
              (std::abs((points3d[t->getCorner(1)->getCustomIndex()] - points3d[t->getCorner(
                           2)->getCustomIndex()]).norm())) > 0.3)
          || (borderTriangle[i] &&
              (std::abs((points3d[t->getCorner(2)->getCustomIndex()] - points3d[t->getCorner(
                           0)->getCustomIndex()]).norm())) > 0.3)) {
        blacklist[i] = true;
      }
    }

    //smooth
    std::vector<std::set<unsigned int>> adjTable(points3d.size());
    VecPoint3f points3dNew(points3d.size());

    for (unsigned int i = 0; i < triangles.size(); i++) { //build adjcense table
      if (blacklist[i]) {
        continue;
      }

      for (unsigned int j = 0; j < 3 ; j++) {
        GEOM_FADE2D::Point2 *ptA = triangles[i]->getCorner(j);
        GEOM_FADE2D::Point2 *ptB = triangles[i]->getCorner((j + 1) % 3);
        adjTable[ptA->getCustomIndex()].insert(ptB->getCustomIndex());
        adjTable[ptB->getCustomIndex()].insert(ptA->getCustomIndex());
      }
    }



    for (unsigned int k = 0; k < smoothingIteration ; k++) {
      for (unsigned int i = 0; i < adjTable.size(); i++) {
        std::set<unsigned int> &l = adjTable[i];
        Eigen::Vector3f meanPt(0, 0, 0);

        if (!borderPoint[i] && l.size() > 0) {
          for (std::set<unsigned int>::iterator it = l.begin(); it != l.end(); ++it) {
            meanPt += points3d[*it];
          }

          meanPt /= l.size();

          Eigen::Vector3f delta = meanPt - points3d[i];

          if (delta.norm() < maxDelta) {
            points3dNew[i] = (laplaceAlpha * (meanPt - points3d[i])) +
                             points3d[i]; // smoothing
          } else {
            if (points3d[i][2] != 0) {
              points3dNew[i] =  points3d[i] * (meanPt[2] /
                                               points3d[i][2]);// replaces point3D Z by meanPt Z
            } else {
              points3dNew[i] =
                meanPt;//replace the point by the meanPt instead of to blacklist it for better visualization.
            }
          }
        } else {
          //do not move point in the border
          points3dNew[i] =  points3d[i];
        }
      }

      points3d = points3dNew;
    }
  }

};




void buildMeshDepthMap(const double &focalU,
                       const double &focalV,
                       const double &centerU,
                       const double &centerV,
                       const double &dimU,
                       const double &dimV,
                       VecPoint3f &in_out_landmarks_3d,
                       VecTriangle &output_triangles,
                       const double laplaceAlpha = 0.1,
                       const unsigned int smoothingIteration = 3,
                       const double maxDelta = 0.2) {
  if (in_out_landmarks_3d.size() < 5) {
    return;
  }

  std::vector<GEOM_FADE2D::Point2> points2D;
  projectLandmarks(focalU, focalV, centerU, centerV, dimU, dimV,
                   in_out_landmarks_3d, points2D);

  GEOM_FADE2D::Fade_2D dt;
  dt.insert(points2D);
  std::vector<GEOM_FADE2D::Triangle2 *> vAllTriangles;
  dt.getTrianglePointers(vAllTriangles);

  std::vector<bool> blacklist(vAllTriangles.size(), false);

  filteringAndSmoothing(in_out_landmarks_3d, vAllTriangles, blacklist,
                        laplaceAlpha,
                        smoothingIteration, maxDelta);

  for (uint i = 0; i < vAllTriangles.size() ; i++) {
    if (blacklist[i]) {
      continue;
    }

    GEOM_FADE2D::Triangle2 *itri = vAllTriangles[i];

    Eigen::Vector3i triangle(itri->getCorner(0)->getCustomIndex(),
                             itri->getCorner(1)->getCustomIndex(),
                             itri->getCorner(2)->getCustomIndex());

    output_triangles.push_back(triangle);

  }


}

}


#endif
