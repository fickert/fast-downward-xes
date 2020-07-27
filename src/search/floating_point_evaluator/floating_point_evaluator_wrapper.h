#ifndef FLOATING_POINT_EVALUATOR_FLOATING_POINT_EVALUATOR_WRAPPER_H
#define FLOATING_POINT_EVALUATOR_FLOATING_POINT_EVALUATOR_WRAPPER_H

#include "floating_point_evaluator.h"

class Evaluator;

namespace floating_point_evaluator {
class FloatingPointEvaluatorWrapper : public FloatingPointEvaluator {
	std::shared_ptr<Evaluator> evaluator;

	auto compute_value(EvaluationContext &eval_context) -> double override;

public:
	explicit FloatingPointEvaluatorWrapper(const options::Options &opts);
	explicit FloatingPointEvaluatorWrapper(const std::shared_ptr<Evaluator> &evaluator);
};
} // namespace floating_point_evaluator

#endif
