#include "icp_2d.h"

#include <iostream>
#include <string>
#include <vector>

#include "utils.h"

double ceres_solve_icp_2d(const double* init_guess_yaw_t,
                          const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src,
                          const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr tar,
                          double* final_guess_yaw_t);

int main() {
  // generate two frame point cloud
  double true_yaw_t[kOptValNum] = {45. * kDegree2Rad, 5., 8.};
  Eigen::Matrix4d true_transform = Eigen::Matrix4d::Identity();
  generate_affine_matrix(true_yaw_t, &true_transform);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_pc(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  generate_point_cloud_2d(src_pc);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tar_pc(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  transform_point_cloud(true_transform, src_pc, tar_pc);

  // take a look at the initial two point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_pc(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  double init_guess_yaw_t[kOptValNum] = {(45-5.) * kDegree2Rad, 5. - 2,
                                         8. - 3};
  Eigen::Matrix4d init_transform = Eigen::Matrix4d::Identity();
  generate_affine_matrix(init_guess_yaw_t, &init_transform);
  transform_point_cloud(init_transform, src_pc, merge_pc);
	set_point_cloud_color(merge_pc, 0, 255, 0);
  *merge_pc += (*tar_pc);
  std::string title = "init point cloud";
  show_point_cloud(merge_pc, title);

	// Ceres find the transform
  double final_guess_yaw_t[kOptValNum] = {0.};
  ceres_solve_icp_2d(init_guess_yaw_t, src_pc, tar_pc, final_guess_yaw_t);
  cout << "final guess: " << final_guess_yaw_t[0] / kDegree2Rad << " "
       << final_guess_yaw_t[1] << " " << final_guess_yaw_t[2] << endl;

	// take a look at the final two point cloud.
  Eigen::Matrix4d final_transform = Eigen::Matrix4d::Identity();
  generate_affine_matrix(final_guess_yaw_t, &final_transform);
  transform_point_cloud(final_transform, src_pc, merge_pc);
	set_point_cloud_color(merge_pc, 0, 0, 255);
  *merge_pc += (*tar_pc);
  title = "final point cloud";
  show_point_cloud(merge_pc, title);

  return 0;
}

double ceres_solve_icp_2d(const double* init_guess_yaw_t,
                          const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src,
                          const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr tar,
                          double* final_guess_yaw_t) {
	
	double each_step_yaw_xy[3] = {0};

  // updated transform in each step
  Eigen::Matrix4d final_guess_T = Eigen::Matrix4d::Identity();
  generate_affine_matrix(init_guess_yaw_t, &final_guess_T);

  int max_ite = 0;
  while (max_ite < 20) {
    ++max_ite;

    // ICP find correspondences.
    std::vector<Corres2d> corres;
    find_point_correspondences(final_guess_T, src, tar, &corres);

    /******** Ceres Solver *************/
    // build problem
    ceres::Problem problem;
    for (const auto& cor : corres) {
      Eigen::Vector2d src;
      Eigen::Vector2d tar;
      src << cor.x1, cor.y1;
      tar << cor.x2, cor.y2;
//#define AUTO_DIFF
#ifdef AUTO_DIFF
      ceres::CostFunction* cost_function = ICP2DResiaual::Create(src, tar);
      problem.AddResidualBlock(cost_function, nullptr, each_step_yaw_xy);
			// problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), each_step_yaw_xy);
#else
			ICP2DResiaual* cost_function = new ICP2DResiaual(src, tar);
			problem.AddResidualBlock(cost_function, nullptr, each_step_yaw_xy);
#endif
    }
    //problem.AddParameterBlock(each_step_yaw_xy, 1, NormalizeAngle );

    // solve problem
    ceres::Solver::Options options;
    // options.minimizer_progress_to_stdout = true; //true;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.num_threads = 6;
    options.max_num_iterations = 500;
    ceres::Solver::Summary summary;
    options.function_tolerance = 1e-20;
    options.parameter_tolerance = 1e-20;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.min_line_search_step_size = 1e-3;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;
    /******** Ceres Solver *************/

    // Update Transform
    Eigen::Matrix4d guess_T = Eigen::Matrix4d::Identity();
    generate_affine_matrix(each_step_yaw_xy, &guess_T);
    final_guess_T = guess_T * final_guess_T;

    // if loop can break?
    bool can_break = true;
    for (int i = 0; i < kOptValNum; ++i) {
      if (each_step_yaw_xy[i] > KConverge[i]){
				can_break = false;
			}
			each_step_yaw_xy[i] = 0.; 
    }
		if(can_break){
			break;
		}

  }
  final_guess_yaw_t[0] = acos(final_guess_T(0, 0));
  final_guess_yaw_t[1] = final_guess_T(0, 3);
  final_guess_yaw_t[2] = final_guess_T(1, 3);
}