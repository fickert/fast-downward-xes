#ifndef HEURISTIC_ERROR_ONE_STEP_DISTANCE_ERROR_H
#define HEURISTIC_ERROR_ONE_STEP_DISTANCE_ERROR_H

#include "one_step_error.h"

namespace heuristic_error {
class OneStepDistanceError : public OneStepError {
	// This class implements the one-step distance error estimation (Thayer, Dionne, and Ruml, 2011).
protected:
	void add_successor(StateID state_id, int op_cost, int evaluator_value) override;
	auto compute_error(const GlobalState &state, const GlobalState &best_successor) const -> double override;

public:
	OneStepDistanceError(const options::Options &opts);
};
} // namespace heuristic_error

#endif
