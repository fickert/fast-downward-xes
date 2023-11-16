#ifndef BOUNDED_COST_SEARCH_COST_BOUND_ASSUMPTIONS_NANCY_EVALUATOR_H
#define BOUNDED_COST_SEARCH_COST_BOUND_ASSUMPTIONS_NANCY_EVALUATOR_H

#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "../heuristic_error/heuristic_error.h"

class Evaluator;

namespace bounded_cost_search {
class NancyAssumptionsCBSEvaluator : public floating_point_evaluator::FloatingPointEvaluator {
protected:
	const int cost_bound;
	const std::shared_ptr<Evaluator> f_evaluator;
	const std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> f_hat_evaluator;
	const std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> d_hat_evaluator;
	const std::shared_ptr<heuristic_error::HeuristicError> heuristic_error;

	static auto cumulative_distribution(double x) -> double;

	auto compute_value(EvaluationContext &eval_context) -> double override;

public:
	explicit NancyAssumptionsCBSEvaluator(const options::Options &opts);
	~NancyAssumptionsCBSEvaluator() override = default;
};
} // namespace bounded_cost_search

#endif
