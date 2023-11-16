#ifndef BOUNDED_SUBOPTIMAL_SEARCH_SUBOPTIMALITY_BOUND_ASSUMPTIONS_NANCY_EVALUATOR_H
#define BOUNDED_SUBOPTIMAL_SEARCH_SUBOPTIMALITY_BOUND_ASSUMPTIONS_NANCY_EVALUATOR_H

#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "../heuristic_error/heuristic_error.h"
#include "f_hat_min_evaluator.h"

class Evaluator;

namespace bounded_suboptimal_search {
class NancyAssumptionsSBSEvaluator : public floating_point_evaluator::FloatingPointEvaluator {
public:
	enum class CostBoundVarianceMethod { HEURISTIC_ERROR, F_MIN_VARIANCE, F_MIN_VARIANCE_TIMES_DISTANCE, ZERO, ZERO_IMPROVED };

protected:
	const double suboptimality_factor;
	const std::shared_ptr<Evaluator> f_evaluator;
	const std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> f_hat_evaluator;
	const std::shared_ptr<FHatMinEvaluator> f_hat_min_evaluator;
	const std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> d_hat_evaluator;
	const std::shared_ptr<heuristic_error::HeuristicError> heuristic_error;
	const bool use_online_variance;
	const bool admissible_h;

	const CostBoundVarianceMethod cost_bound_variance_method;

	static auto cumulative_distribution(double x) -> double;

	auto compute_value(EvaluationContext &eval_context) -> double override;

public:
	explicit NancyAssumptionsSBSEvaluator(const options::Options &opts);
	~NancyAssumptionsSBSEvaluator() override = default;
};
} // namespace bounded_suboptimal_search

#endif
