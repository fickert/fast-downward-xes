#ifndef FLOATING_POINT_EVALUATOR_DIVISION_EVALUATOR_H
#define FLOATING_POINT_EVALUATOR_DIVISION_EVALUATOR_H

#include "floating_point_evaluator.h"

namespace floating_point_evaluator {
class DivisionEvaluator : public FloatingPointEvaluator {
	std::shared_ptr<FloatingPointEvaluator> dividend;
	std::shared_ptr<FloatingPointEvaluator> divisor;

protected:
	auto compute_value(EvaluationContext &eval_context) -> double override;

public:
	explicit DivisionEvaluator(const options::Options &opts);
	DivisionEvaluator(std::shared_ptr<FloatingPointEvaluator> dividend, std::shared_ptr<FloatingPointEvaluator> divisor);
};
} // namespace bounded_cost_search

#endif
