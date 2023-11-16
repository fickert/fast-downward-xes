#ifndef FLOATING_POINT_EVALUATOR_WEIGHTED_EVALUATOR_H
#define FLOATING_POINT_EVALUATOR_WEIGHTED_EVALUATOR_H

#include "floating_point_evaluator.h"

namespace floating_point_evaluator {
class WeightedEvaluator : public FloatingPointEvaluator {
	std::shared_ptr<FloatingPointEvaluator> evaluator;
	const double weight;

protected:
	auto compute_value(EvaluationContext &eval_context) -> double override;

public:
	explicit WeightedEvaluator(const options::Options &opts);
	WeightedEvaluator(std::shared_ptr<FloatingPointEvaluator> evaluator, double weight);
};
} // namespace bounded_cost_search

#endif
