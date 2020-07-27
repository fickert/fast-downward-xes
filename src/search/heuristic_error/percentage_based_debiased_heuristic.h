#ifndef HEURISTIC_ERROR_PERCENTAGE_BASED_DEBIASED_HEURISTIC_H
#define HEURISTIC_ERROR_PERCENTAGE_BASED_DEBIASED_HEURISTIC_H

#include "../evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "heuristic_error.h"

namespace heuristic_error {
class PercentageBasedDebiasedHeuristic : public floating_point_evaluator::FloatingPointEvaluator {
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<HeuristicError> heuristic_error;

	auto compute_value(EvaluationContext &eval_context) -> double override;

public:
	PercentageBasedDebiasedHeuristic(const options::Options &opts);
	~PercentageBasedDebiasedHeuristic() override = default;
};
} // namespace heuristic_error

#endif
