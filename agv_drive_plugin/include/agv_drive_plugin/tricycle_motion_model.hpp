#ifndef TRICYCLE_MOTION_MODEL_HPP
#define TRICYCLE_MOTION_MODEL_HPP

#include "nav2_amcl/motion_model/motion_model.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"

namespace nav2_amcl
{

class TricycleMotionModel : public MotionModel
{
public:
  void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4,
    double wheelbase);

  void odometryUpdate(
    pf_t * pf, const pf_vector_t & pose,
    const pf_vector_t & delta) override;

private:
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
  double wheelbase_;
};

}  // namespace nav2_amcl

#endif  // TRICYCLE_MOTION_MODEL_HPP
