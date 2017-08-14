
#include<mesh_based_mapping/mesh_based_mapping.hpp>
#include<mesh_based_mapping/utils/file_io.hpp>

const double pi = std::acos(-1);

void buildHemiSphere(double ro_step, double phi_step,
                     double radius, double central_hole,
                     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
                     &landmarks_3d) {

  float x, y, z;

  for (double ro = central_hole; ro < pi / 2.0; ro += ro_step) {
    z = radius * std::cos(ro);

    for (double phi = -pi; phi < pi; phi += phi_step) {
      x = radius * std::sin(ro) * std::cos(phi);
      y = radius * std::sin(ro) * std::sin(phi);
      landmarks_3d.push_back(Eigen::Vector3f(x, y, z));
    }
  }
}

int main(int, char **) {


  const double focalU = 450;
  const double focalV = 450;
  const double centerU = 400.5;
  const double centerV = 400.5;
  const double dimU = 800;
  const double dimV = 800;
  const double laplaceAlpha = 0.1;
  const unsigned int smoothingIteration = 3;
  const double maxDelta = 0.2;
  mesh_based_mapping::VecPoint3f landmarks_3d;//in camera frame

  buildHemiSphere(0.01, 0.1, 10, 0, landmarks_3d);

  mesh_based_mapping::VecPoint3f landmarks_3d_filtered = landmarks_3d;

  mesh_based_mapping::VecTriangle triangles;
  mesh_based_mapping::buildMeshDepthMap(focalU, focalV, centerU, centerV, dimU,
                                        dimV, landmarks_3d_filtered, triangles, laplaceAlpha, smoothingIteration,
                                        maxDelta);

  mesh_based_mapping::saveObj("/tmp/mesh_before.obj", landmarks_3d, triangles);

  mesh_based_mapping::saveObj("/tmp/mesh_after.obj", landmarks_3d_filtered,
                              triangles);

  return 0;
}
