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
#include <Fade_2D.h>
#include <mesh_based_mapping/mesh_based_mapping.hpp>

mesh_based_mapping::MeshMapper::MeshMapper(double laplace_alpha,
    unsigned int smoothing_iteration,
    double max_delta): laplace_alpha_(laplace_alpha),
  smoothing_iteration_(smoothing_iteration), max_delta_(max_delta) {
  landmarks_2d_ = new std::vector<GEOM_FADE2D::Point2>();
  fade_ = new GEOM_FADE2D::Fade_2D(500);
}

mesh_based_mapping::MeshMapper::~MeshMapper() {
  delete fade_;
  delete landmarks_2d_;
}

void mesh_based_mapping::MeshMapper::Clear() {
  delete fade_;
  fade_ = new GEOM_FADE2D::Fade_2D(500);

  landmarks_2d_->clear();
  triangles_.clear();
  landmarks_3d_.clear();

  landmarks_2d_output_.clear();
  triangles_output_.clear();
  filtered_landmarks_3d_output_.clear();
}

void mesh_based_mapping::MeshMapper::SaveObj(std::string filepath, uint dim_u,
    uint dim_v) {
  std::ofstream ofs;
  ofs.open(filepath, std::ofstream::out);


  for (unsigned int i = 0; i < landmarks_3d_.size(); i++) {
    Eigen::Vector3f &hPoint = landmarks_3d_[i];
    ofs << "v "  << hPoint[0] << " " << hPoint[1] << " " << hPoint[2]  << std::endl;
  }

  if (dim_u != 0 && dim_v != 0) {
    for (unsigned int i = 0; i < landmarks_2d_->size(); i++) {
      GEOM_FADE2D::Point2 &hPoint = landmarks_2d_->at(i);
      ofs << "vt "  << hPoint.x() / (double)dim_u << " " << hPoint.y() /
          (double)dim_v << std::endl;
    }
  }

  for (unsigned int i = 0; i < triangles_.size(); i++) {
    if (triangle_blacklist_[i]) {
      continue;
    }

    GEOM_FADE2D::Triangle2 *t = triangles_[i];

    if (dim_u != 0 && dim_v != 0) {
      ofs << "f "  << t->getCorner(2)->getCustomIndex() + 1 << "/" << t->getCorner(
            2)->getCustomIndex() + 1 << " ";
      ofs << t->getCorner(1)->getCustomIndex() + 1 << "/" << t->getCorner(
            1)->getCustomIndex() + 1 << " ";
      ofs << t->getCorner(0)->getCustomIndex() + 1 << "/" << t->getCorner(
            0)->getCustomIndex() + 1 << std::endl;
    } else {

      ofs << "f "  << t->getCorner(2)->getCustomIndex() + 1 << " "
          << t->getCorner(1)->getCustomIndex() + 1 << " "
          << t->getCorner(0)->getCustomIndex() + 1 << std::endl;
    }
  }

  ofs.close();
}

int mesh_based_mapping::MeshMapper::SetPoints(double focal_u, double focal_v,
    double center_u, double center_v, uint dim_u, uint dim_v,
    const mesh_based_mapping::VecPoint3f &in_landmarks_3d) {

  Clear();
  ProjectLandmarks(focal_u, focal_v, center_u, center_v, dim_u, dim_v,
                   in_landmarks_3d);
  return landmarks_2d_->size();
}

int mesh_based_mapping::MeshMapper::SetPoints(const
    mesh_based_mapping::VecPoint2f &in_landmarks_2d,
    const mesh_based_mapping::VecPoint3f &in_landmarks_3d) {

  Clear();
  assert(in_landmarks_2d.size() == in_landmarks_3d.size());

  landmarks_3d_ = in_landmarks_3d;
  convert2dLandmarks(in_landmarks_2d);

  return landmarks_2d_->size();
}

bool mesh_based_mapping::MeshMapper::ComputeMesh() {

  if (landmarks_2d_->size() < 5) {
    return false;
  }

  fade_->insert(*landmarks_2d_);


  fade_->getTrianglePointers(triangles_);
  //fade_->writeObj("/tmp/file.obj");

  triangle_blacklist_.resize(triangles_.size(), false);

  filteringAndSmoothing(landmarks_3d_, triangles_, triangle_blacklist_,
                        laplace_alpha_, smoothing_iteration_, max_delta_);

}

bool mesh_based_mapping::MeshMapper::GetFilteredLandmarks(
  const mesh_based_mapping::VecPoint3f *&out_landmarks_3d) {
  if (triangles_.size() == 0) {
    return false;
  }

  std::vector<bool> landmarks_3d_blacklist(landmarks_3d_.size(), true);

  if (filtered_landmarks_3d_output_.size() == 0) {

    for (size_t i = 0; i < triangles_.size(); i++) {
      if (triangle_blacklist_[i]) {
        continue;
      }

      GEOM_FADE2D::Triangle2 *itri = triangles_[i];

      triangles_output_.push_back(Eigen::Vector3i(itri->getCorner(
                                    0)->getCustomIndex(),
                                  itri->getCorner(1)->getCustomIndex(),
                                  itri->getCorner(2)->getCustomIndex()));
      landmarks_3d_blacklist.at(itri->getCorner(0)->getCustomIndex()) = false;
      landmarks_3d_blacklist.at(itri->getCorner(1)->getCustomIndex()) = false;
      landmarks_3d_blacklist.at(itri->getCorner(2)->getCustomIndex()) = false;
    }

    for (size_t i = 0; i < landmarks_3d_.size(); i++) {
      if (landmarks_3d_blacklist[i]) {
        continue;
      }

      filtered_landmarks_3d_output_.push_back(landmarks_3d_[i]);
    }
  }


  out_landmarks_3d = &landmarks_3d_;

}


bool mesh_based_mapping::MeshMapper::GetMesh(const
    mesh_based_mapping::VecPoint3f *&out_landmarks_3d,
    const mesh_based_mapping::VecPoint2f *&out_landmarks_2d,
    const mesh_based_mapping::VecTriangle *&out_triangles) {
  if (triangles_.size() == 0) {
    return false;
  }

  if (triangles_output_.size() == 0 || landmarks_2d_output_.size() == 0) {
    for (size_t i = 0; i < landmarks_2d_->size(); i++) {
      landmarks_2d_output_.push_back(Eigen::Vector2f(landmarks_2d_->at(i).x(),
                                     landmarks_2d_->at(i).y()));
    }

    for (size_t i = 0; i < triangles_.size(); i++) {
      if (triangle_blacklist_[i]) {
        continue;
      }

      GEOM_FADE2D::Triangle2 *itri = triangles_[i];

      triangles_output_.push_back(Eigen::Vector3i(itri->getCorner(
                                    0)->getCustomIndex(),
                                  itri->getCorner(1)->getCustomIndex(),
                                  itri->getCorner(2)->getCustomIndex()));

    }
  }

  out_landmarks_2d = &landmarks_2d_output_;
  out_landmarks_3d = &landmarks_3d_;
  out_triangles = &triangles_output_;

  return true;
}


void mesh_based_mapping::MeshMapper::convert2dLandmarks(
  const mesh_based_mapping::VecPoint2f &in_landmarks_2d) {
  for (size_t i = 0; i < in_landmarks_2d.size(); i++) {
    landmarks_2d_->push_back(GEOM_FADE2D::Point2(in_landmarks_2d[i](0),
                             in_landmarks_2d[i](1)));
    landmarks_2d_->back().setCustomIndex(i);
  }
}


void mesh_based_mapping::MeshMapper::ProjectLandmarks(const double &focalU,
    const double &focalV, const double &centerU, const double &centerV,
    const double &dimU, const double &dimV,
    const mesh_based_mapping::VecPoint3f &landmarks) {


  for (unsigned int i = 0 ; i < landmarks.size() ; i++) {
    const Eigen::Vector3f &pt_CRef = landmarks[i];

    double x = ((pt_CRef(0) / pt_CRef(2)) * focalU) + centerU;
    double y = ((pt_CRef(1) / pt_CRef(2)) * focalV) + centerV;

    if (x <= 0 || y <= 0 || x >= dimU || y >= dimV) {
      continue;
    }

    GEOM_FADE2D::Point2 pt = GEOM_FADE2D::Point2(x, y);
    pt.setCustomIndex(landmarks_2d_->size());
    landmarks_2d_->push_back(pt);
    landmarks_3d_.push_back(pt_CRef);
  }
}

void mesh_based_mapping::MeshMapper::filteringAndSmoothing(
  mesh_based_mapping::VecPoint3f &points3d,
  const std::vector<GEOM_FADE2D::Triangle2 *> &triangles,
  std::vector<bool> &blacklist,
  double laplaceAlpha, unsigned int smoothingIteration,
  double maxDelta) {



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
    maxEdge[i] = std::max(t->getSquaredEdgeLength(0),
                          std::max(t->getSquaredEdgeLength(1), t->getSquaredEdgeLength(2)));
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
 //     blacklist[i] = true;
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
