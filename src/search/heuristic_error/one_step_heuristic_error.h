#ifndef HEURISTIC_ERROR_ONE_STEP_HEURISTIC_ERROR_H
#define HEURISTIC_ERROR_ONE_STEP_HEURISTIC_ERROR_H

#include "one_step_error.h"

namespace heuristic_error {
class OneStepHeuristicError : public OneStepError {
	// This class implements the global one-step heuristic error estimation (Thayer, Dionne, and Ruml, 2011).
protected:
	const int warm_start_samples;
	const double warm_start_value;
	int best_successor_op_cost;

	void add_successor(StateID state_id, int op_cost, int evaluator_value) override;
	auto compute_error(const GlobalState &state, const GlobalState &best_successor) const -> double override;

public:
	OneStepHeuristicError(const options::Options &opts);

	void notify_initial_state() override;
	void notify_initial_state(EvaluationContext &eval_context, int cost_bound, int distance) override;
};
} // namespace heuristic_error

#endif
