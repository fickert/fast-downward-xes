#ifndef BOUNDED_COST_SEARCH_LINEAR_RELATIVE_ERROR_POTENTIAL_EVALUATOR_H
#define BOUNDED_COST_SEARCH_LINEAR_RELATIVE_ERROR_POTENTIAL_EVALUATOR_H

#include "../floating_point_evaluator/floating_point_evaluator.h"

namespace bounded_cost_search {
class LinearRelativeErrorPotentialEvaluator : public floating_point_evaluator::FloatingPointEvaluator {
protected:
	const int bound;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> heuristic;

	auto compute_value(EvaluationContext &eval_context) -> double override;

public:
	LinearRelativeErrorPotentialEvaluator(const options::Options &opts);

};
} // namespace bounded_cost_search

#endif
