#ifndef HEURISTIC_ERROR_DEBIASED_DISTANCE_H
#define HEURISTIC_ERROR_DEBIASED_DISTANCE_H

#include "../evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "heuristic_error.h"

namespace heuristic_error {
class DebiasedDistance : public floating_point_evaluator::FloatingPointEvaluator {
	std::shared_ptr<Evaluator> distance;
	std::shared_ptr<HeuristicError> heuristic_error;

	auto compute_value(EvaluationContext &eval_context) -> double override;

public:
	DebiasedDistance(const options::Options &opts);
	~DebiasedDistance() override = default;
};
} // namespace heuristic_error

#endif
