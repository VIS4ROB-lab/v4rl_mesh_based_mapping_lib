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
}

mesh_based_mapping::MeshMapper::~MeshMapper() {
    delete landmarks_2d_;
}

void mesh_based_mapping::MeshMapper::Clear() {
    landmarks_2d_->clear();

    triangles_.clear();//TODO is this a leak?
    landmarks_3d_.clear();
    landmarks_2d_output_.clear();
    triangles_output_.clear();
}

void mesh_based_mapping::MeshMapper::SetPoints(double focal_u, double focal_v,
                                               double center_u, double center_v, uint dim_u, uint dim_v,
                                               const mesh_based_mapping::VecPoint3f &in_landmarks_3d) {

    Clear();
    landmarks_3d_ = in_landmarks_3d;

    ProjectLandmarks(focal_u, focal_v, center_u, center_v, dim_u, dim_v,
                     landmarks_3d_);
}

void mesh_based_mapping::MeshMapper::SetPoints(const
                                               mesh_based_mapping::VecPoint2f &in_landmarks_2d,
                                               const mesh_based_mapping::VecPoint3f &in_landmarks_3d) {

    Clear();
    landmarks_3d_ = in_landmarks_3d;
    convert2dLandmarks(in_landmarks_2d);
}

bool mesh_based_mapping::MeshMapper::GetFilteredLandmarks(
        const mesh_based_mapping::VecPoint3f *&out_landmarks_3d) const {
    out_landmarks_3d = &landmarks_3d_;
}

bool mesh_based_mapping::MeshMapper::GetMesh(const mesh_based_mapping::VecPoint3f *&out_landmarks_3d, const mesh_based_mapping::VecPoint2f *&out_landmarks_2d, const mesh_based_mapping::VecTriangle *&out_triangles) {
    if (triangles_.size() == 0) {
        return false;
    }

    if (triangles_output_.size() == 0 || landmarks_2d_output_.size() == 0) {

    }

    out_landmarks_2d = &landmarks_2d_output_;
    out_landmarks_3d = &landmarks_3d_;
    out_triangles = &triangles_output_;

    return true;
}


void mesh_based_mapping::MeshMapper::convert2dLandmarks(const mesh_based_mapping::VecPoint2f &in_landmarks_2d)
{
    for(size_t i=0;i<in_landmarks_2d.size();i++)
    {
        landmarks_2d_->push_back(GEOM_FADE2D::Point2(in_landmarks_2d[i](0), in_landmarks_2d[i](1)));
                                 landmarks_2d_->back().setCustomIndex(i);
    }
}

//bool mesh_based_mapping::MeshMapper::GetMesh(mesh_based_mapping::VecPoint3f const *  &out_landmarks_3d,
//    VecTriangle const * &out_triangles) const {
//  if (triangles_.size() == 0) {
//    return false;
//  }

//  out_landmarks_3d = &landmarks_3d_;
//  out_triangles = &triangles_;

//  return true;
//}



//void mesh_based_mapping::MeshMapper::ComputeMeshInPlace(
//  const mesh_based_mapping::VecPoint2f &in_landmarks_2d,
//  mesh_based_mapping::VecPoint3f &in_out_landmarks_3d,
//  mesh_based_mapping::VecTriangle &out_triangles) {

//}

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

        landmarks_2d_->push_back(GEOM_FADE2D::Point2(x, y));
        landmarks_2d_->back().setCustomIndex(i);
    }
}

//void mesh_based_mapping::MeshMapper::filteringAndSmoothing(mesh_based_mapping::VecPoint3f &points3d, std::vector<GEOM_FADE2D::Triangle2 *> triangles, std::vector<bool> &blacklist, const double laplaceAlpha, const unsigned int smoothingIteration, const double maxDelta) {



//    if (triangles.size() < 2) {
//        return;
//    }

//    double mean = 0;
//    double std = 0;

//    std::vector<double> maxEdge(triangles.size());
//    std::vector<bool> borderPoint(points3d.size(), false);
//    std::vector<bool> borderTriangle(triangles.size(), false);

//    //TODO put back the border detection.

//    //remove the big edges in 2D
//    for (unsigned int i = 0; i < triangles.size();
//         i++) {
//        GEOM_FADE2D::Triangle2 *t = triangles[i];
//        maxEdge[i] = cv::max(t->getSquaredEdgeLength(0),
//                             cv::max(t->getSquaredEdgeLength(1), t->getSquaredEdgeLength(2)));
//        mean += maxEdge[i] ;
//    }

//    mean /=  triangles.size();

//    for (unsigned int i = 0; i < triangles.size(); i++) {
//        double delta = maxEdge[i] - mean;
//        std += delta * delta;
//    }

//    std = sqrt(std / triangles.size());
//    double threshold = std;

//    for (unsigned int i = 0; i < triangles.size(); i++) {
//        GEOM_FADE2D::Triangle2 *t = triangles[i];
//        double diff = maxEdge[i] - mean;

//        if ((std::abs(diff) > threshold) || (borderTriangle[i] && diff > 0.6 * std)
//                || (borderTriangle[i] &&
//                    (std::abs((points3d[t->getCorner(0)->getCustomIndex()] - points3d[t->getCorner(
//                                   1)->getCustomIndex()]).norm())) > 0.3)
//                || (borderTriangle[i] &&
//                    (std::abs((points3d[t->getCorner(1)->getCustomIndex()] - points3d[t->getCorner(
//                                   2)->getCustomIndex()]).norm())) > 0.3)
//                || (borderTriangle[i] &&
//                    (std::abs((points3d[t->getCorner(2)->getCustomIndex()] - points3d[t->getCorner(
//                                   0)->getCustomIndex()]).norm())) > 0.3)) {
//            blacklist[i] = true;
//        }
//    }

//    //smooth
//    std::vector<std::set<unsigned int>> adjTable(points3d.size());
//    VecPoint3f points3dNew(points3d.size());

//    for (unsigned int i = 0; i < triangles.size(); i++) { //build adjcense table
//        if (blacklist[i]) {
//            continue;
//        }

//        for (unsigned int j = 0; j < 3 ; j++) {
//            GEOM_FADE2D::Point2 *ptA = triangles[i]->getCorner(j);
//            GEOM_FADE2D::Point2 *ptB = triangles[i]->getCorner((j + 1) % 3);
//            adjTable[ptA->getCustomIndex()].insert(ptB->getCustomIndex());
//            adjTable[ptB->getCustomIndex()].insert(ptA->getCustomIndex());
//        }
//    }



//    for (unsigned int k = 0; k < smoothingIteration ; k++) {
//        for (unsigned int i = 0; i < adjTable.size(); i++) {
//            std::set<unsigned int> &l = adjTable[i];
//            Eigen::Vector3f meanPt(0, 0, 0);

//            if (!borderPoint[i] && l.size() > 0) {
//                for (std::set<unsigned int>::iterator it = l.begin(); it != l.end(); ++it) {
//                    meanPt += points3d[*it];
//                }

//                meanPt /= l.size();

//                Eigen::Vector3f delta = meanPt - points3d[i];

//                if (delta.norm() < maxDelta) {
//                    points3dNew[i] = (laplaceAlpha * (meanPt - points3d[i])) +
//                            points3d[i]; // smoothing
//                } else {
//                    if (points3d[i][2] != 0) {
//                        points3dNew[i] =  points3d[i] * (meanPt[2] /
//                                points3d[i][2]);// replaces point3D Z by meanPt Z
//                    } else {
//                        points3dNew[i] =
//                                meanPt;//replace the point by the meanPt instead of to blacklist it for better visualization.
//                    }
//                }
//            } else {
//                //do not move point in the border
//                points3dNew[i] =  points3d[i];
//            }
//        }

//        points3d = points3dNew;
//    }
//}

//void mesh_based_mapping::MeshMapper::buildMeshDepthMap(const double &focalU, const double &focalV, const double &centerU, const double &centerV, const double &dimU, const double &dimV, mesh_based_mapping::VecPoint3f &in_out_landmarks_3d, mesh_based_mapping::VecTriangle &output_triangles, const double laplaceAlpha, const unsigned int smoothingIteration, const double maxDelta) {
//    if (in_out_landmarks_3d.size() < 5) {
//        return;
//    }

//    std::vector<GEOM_FADE2D::Point2> points2D;
//    projectLandmarks(focalU, focalV, centerU, centerV, dimU, dimV,
//                     in_out_landmarks_3d, points2D);

//    GEOM_FADE2D::Fade_2D dt;
//    dt.insert(points2D);
//    std::vector<GEOM_FADE2D::Triangle2 *> vAllTriangles;
//    dt.getTrianglePointers(vAllTriangles);

//    std::vector<bool> blacklist(vAllTriangles.size(), false);

//    filteringAndSmoothing(in_out_landmarks_3d, vAllTriangles, blacklist,
//                          laplaceAlpha,
//                          smoothingIteration, maxDelta);

//    for (uint i = 0; i < vAllTriangles.size() ; i++) {
//        if (blacklist[i]) {
//            continue;
//        }

//        GEOM_FADE2D::Triangle2 *itri = vAllTriangles[i];

//        Eigen::Vector3i triangle(itri->getCorner(0)->getCustomIndex(),
//                                 itri->getCorner(1)->getCustomIndex(),
//                                 itri->getCorner(2)->getCustomIndex());

//        output_triangles.push_back(triangle);

//    }


//}
