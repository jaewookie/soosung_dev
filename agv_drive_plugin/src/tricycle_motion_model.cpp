// #include <cmath>
// #include "nav2_amcl/angleutils.hpp"
// #include "agv_drive_plugin/tricycle_motion_model.hpp"
// #include "nav2_amcl/pf/pf_pdf.hpp"

// namespace nav2_amcl
// {

// void TricycleMotionModel::initialize(
//   double alpha1, double alpha2, double alpha3, double alpha4,
//   double wheelbase)
// {
//   alpha1_ = alpha1;
//   alpha2_ = alpha2;
//   alpha3_ = alpha3;
//   alpha4_ = alpha4;
//   wheelbase_ = wheelbase;
// }

// void TricycleMotionModel::odometryUpdate(
//   pf_t * pf, const pf_vector_t & pose,
//   const pf_vector_t & delta)
// {
//   // Compute the new sample poses
//   pf_sample_set_t * set;
//   set = pf->sets + pf->current_set;
//   pf_vector_t old_pose = pf_vector_sub(pose, delta);

//   // Implement sample_motion_odometry for tricycle model
//   double delta_rot, delta_trans, delta_steer;
//   double delta_rot_hat, delta_trans_hat, delta_steer_hat;

//   // Extract motion parameters
//   delta_rot = angleutils::angle_diff(delta.v[2], old_pose.v[2]);
//   delta_trans = sqrt(delta.v[0] * delta.v[0] + delta.v[1] * delta.v[1]);
//   delta_steer = atan2(delta_trans * sin(delta_rot), wheelbase_);

//   for (int i = 0; i < set->sample_count; i++) {
//     pf_sample_t * sample = set->samples + i;

//     // Add noise to the motion model
//     delta_rot_hat = delta_rot + pf_ran_gaussian(sqrt(alpha1_ * delta_rot * delta_rot + alpha2_ * delta_trans * delta_trans));
//     delta_trans_hat = delta_trans + pf_ran_gaussian(sqrt(alpha3_ * delta_trans * delta_trans + alpha4_ * delta_rot * delta_rot));
//     delta_steer_hat = delta_steer + pf_ran_gaussian(sqrt(alpha1_ * delta_steer * delta_steer + alpha2_ * delta_trans * delta_trans));

//     // Compute the new sample pose
//     sample->pose.v[0] += delta_trans_hat * cos(sample->pose.v[2] + delta_rot_hat);
//     sample->pose.v[1] += delta_trans_hat * sin(sample->pose.v[2] + delta_rot_hat);
//     sample->pose.v[2] += delta_rot_hat;
//   }
// }

// }  // namespace nav2_amcl

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(nav2_amcl::TricycleMotionModel, nav2_amcl::MotionModel)
