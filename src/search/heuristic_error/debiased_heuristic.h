#ifndef HEURISTIC_ERROR_DEBIASED_HEURISTIC_H
#define HEURISTIC_ERROR_DEBIASED_HEURISTIC_H

#include "../evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "heuristic_error.h"

namespace heuristic_error {
class DebiasedHeuristic : public floating_point_evaluator::FloatingPointEvaluator {
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<FloatingPointEvaluator> distance;
	std::shared_ptr<HeuristicError> heuristic_error;
	const bool admissible_h;

	auto compute_value(EvaluationContext &eval_context) -> double override;

public:
	DebiasedHeuristic(const options::Options &opts);
	~DebiasedHeuristic() override = default;
};
} // namespace heuristic_error

#endif
