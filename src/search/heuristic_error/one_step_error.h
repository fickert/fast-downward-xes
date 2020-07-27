#ifndef HEURISTIC_ERROR_ONE_STEP_ERROR_H
#define HEURISTIC_ERROR_ONE_STEP_ERROR_H

#include "../state_id.h"
#include "heuristic_error.h"

namespace heuristic_error {
class OneStepError : public HeuristicError {
	// This is the base class for one-step error estimation models (Thayer, Dionne, and Ruml, 2011).
protected:
	const StateRegistry *state_registry;

	double average_heuristic_error;
	double heuristic_errors_M2_sum;
	int count;

	StateID current_state_id;
	StateID best_successor_id;
	int best_successor_value;

	static constexpr auto NO_VALUE = -1;

	virtual void add_successor(StateID state_id, int op_cost, int evaluator_value) = 0;
	virtual auto compute_error(const GlobalState &state, const GlobalState &best_successor) const -> double = 0;

public:
	OneStepError(const options::Options &opts);

	void initialize(const StateRegistry &state_registry) override { this->state_registry = &state_registry; }

	void set_expanding_state(const GlobalState &state) override;
	void add_successor(const SearchNode &successor_node, int op_cost) override;
	void update_error() override;

	auto get_average_heuristic_error() const -> double override { return average_heuristic_error; }
	auto get_heuristic_error_variance() const -> double override;
};
} // namespace heuristic_error

#endif
