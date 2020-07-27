#ifndef HEURISTIC_ERROR_PERCENTAGE_BASED_HEURISTIC_ERROR_H
#define HEURISTIC_ERROR_PERCENTAGE_BASED_HEURISTIC_ERROR_H

#include "one_step_error.h"

namespace heuristic_error {
class PercentageBasedHeuristicError : public OneStepError {
	// This class implements a percentage-based heuristic error by comparing the heuristic value with that of the best successor.
protected:
	const int warm_start_samples;
	const double warm_start_value;
	int best_successor_op_cost;

	void add_successor(StateID state_id, int op_cost, int evaluator_value) override;
	auto compute_error(const GlobalState &state, const GlobalState &best_successor) const -> double override;

public:
	PercentageBasedHeuristicError(const options::Options &opts);

	void notify_initial_state() override;
	void notify_initial_state(EvaluationContext &eval_context, int cost_bound, int distance) override;
};
} // namespace heuristic_error

#endif
