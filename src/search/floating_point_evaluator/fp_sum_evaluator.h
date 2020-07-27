#ifndef FLOATING_POINT_EVALUATOR_SUM_EVALUATOR_H
#define FLOATING_POINT_EVALUATOR_SUM_EVALUATOR_H

#include "floating_point_evaluator.h"

namespace floating_point_evaluator {
class SumEvaluator : public FloatingPointEvaluator {
	std::vector<std::shared_ptr<FloatingPointEvaluator>> subevaluators;

protected:
	auto compute_value(EvaluationContext &eval_context) -> double override;

public:
	explicit SumEvaluator(const options::Options &opts);
	SumEvaluator(std::vector<std::shared_ptr<FloatingPointEvaluator>> subevaluators);
};
} // namespace bounded_cost_search

#endif
