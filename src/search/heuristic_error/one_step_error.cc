#include "one_step_error.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../search_space.h"

namespace heuristic_error {
OneStepError::OneStepError(const options::Options &opts)
	: HeuristicError(opts),
	  state_registry(nullptr),
	  average_heuristic_error(0),
	  heuristic_errors_M2_sum(0),
	  count(0),
	  current_state_id(StateID::no_state),
	  best_successor_id(StateID::no_state),
	  best_successor_value(NO_VALUE) {}

void OneStepError::set_expanding_state(const GlobalState &state) {
	current_state_id = state.get_id();
	best_successor_value = NO_VALUE;
}

void OneStepError::add_successor(const SearchNode &successor_node, int op_cost) {
	assert(successor_node.is_open() || successor_node.is_closed());
	auto eval_context = EvaluationContext(successor_node.get_state(), successor_node.get_g(), true, nullptr);
	if (eval_context.is_evaluator_value_infinite(evaluator.get()))
		return;
	const auto value = eval_context.get_evaluator_value(evaluator.get());
	add_successor(successor_node.get_state_id(), op_cost, value);
}

void OneStepError::update_error() {
	if (best_successor_value == NO_VALUE)
		return;
	const auto state = state_registry->lookup_state(current_state_id);
	const auto successor_state = state_registry->lookup_state(best_successor_id);
	assert(evaluator->is_estimate_cached(state));
	assert(evaluator->is_estimate_cached(successor_state));
	const auto error = compute_error(state, successor_state);
	// see https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
	++count;
	const auto delta = error - average_heuristic_error;
	average_heuristic_error += delta / count;
	const auto delta2 = error - average_heuristic_error;
	heuristic_errors_M2_sum += std::abs(delta * delta2);
}

auto OneStepError::get_heuristic_error_variance() const -> double {
	assert(count > 0);
	assert(heuristic_errors_M2_sum >= 0);
	return count == 1 ? 0. : heuristic_errors_M2_sum / (count - 1);
}
} // namespace heuristic_error
