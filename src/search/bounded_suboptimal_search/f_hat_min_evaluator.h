#ifndef BOUNDED_SUBOPTIMAL_SEARCH_F_HAT_MIN_EVALUATOR_H
#define BOUNDED_SUBOPTIMAL_SEARCH_F_HAT_MIN_EVALUATOR_H

#include "../floating_point_evaluator/floating_point_evaluator.h"

namespace bounded_suboptimal_search {
class FHatMinEvaluator : public floating_point_evaluator::FloatingPointEvaluator {
protected:
	double f_hat_min;
	double d_hat;

	const bool track_variance;
	double average_f_hat_min;
	double f_hat_min_M2_sum;
	int count;

	auto compute_value(EvaluationContext &) -> double override { return f_hat_min; }

public:
	FHatMinEvaluator(const options::Options &opts, bool track_variance);
	~FHatMinEvaluator() override = default;

	auto get_f_hat_min() const -> double { return f_hat_min; }
	auto get_d_hat() const -> double { return d_hat; }

	auto get_variance() const -> double;

	void update(double f_hat_min, double d_hat);
};
} // namespace bounded_suboptimal_search

#endif
