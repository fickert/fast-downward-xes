#include "one_step_heuristic_error.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace heuristic_error {
OneStepHeuristicError::OneStepHeuristicError(const options::Options &opts)
	: OneStepError(opts),
	  warm_start_samples(opts.get<int>("warm_start_samples")),
	  warm_start_value(opts.get<double>("warm_start_value")),
	  best_successor_op_cost(0) {}

void OneStepHeuristicError::notify_initial_state() {
	if (warm_start_samples == 0)
		return;
	count = warm_start_samples;
	average_heuristic_error = warm_start_value;
}

void OneStepHeuristicError::notify_initial_state(EvaluationContext &eval_context, int cost_bound, int distance) {
	if (warm_start_samples == 0)
		return;
	if (eval_context.is_evaluator_value_infinite(evaluator.get()) || distance == 0 || distance == EvaluationResult::INFTY)
		return;
	count = warm_start_samples;
	const auto initial_value = eval_context.get_evaluator_value(evaluator.get());
	average_heuristic_error = (static_cast<double>(cost_bound) - initial_value) / distance;
}

void OneStepHeuristicError::add_successor(StateID state_id, int op_cost, int evaluator_value) {
	const auto f_value = evaluator_value + op_cost;
	if (best_successor_value == NO_VALUE || f_value < best_successor_value) {
		best_successor_value = f_value;
		best_successor_op_cost = op_cost;
		best_successor_id = state_id;
	}
}

auto OneStepHeuristicError::compute_error(const GlobalState &state, const GlobalState &best_successor) const -> double {
	return evaluator->get_cached_estimate(best_successor) + best_successor_op_cost - evaluator->get_cached_estimate(state);
}

static auto _parse(OptionParser &parser) -> std::shared_ptr<HeuristicError> {
	parser.add_option<int>("warm_start_samples",
	                       "Initialize the error such that extreme values are avoided. "
	                       "If the search passes a bound to the heuristic error model, use it to make the initial heuristic value equal to the bound, "
	                       "otherwise initialize the error with the given number of samples.",
	                       "0", Bounds("0", "infinity"));
	parser.add_option<double>("warm_start_value", "Value to warm-start the heuristic error with.", "1");
	HeuristicError::add_options_to_parser(parser);
	Options opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<OneStepHeuristicError>(opts);
}

static Plugin<HeuristicError> _plugin("one_step_heuristic_error", _parse);
} // namespace heuristic_error
