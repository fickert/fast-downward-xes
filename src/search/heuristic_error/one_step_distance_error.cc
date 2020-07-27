#include "one_step_distance_error.h"

#include <algorithm>

#include "../evaluator.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace heuristic_error {
OneStepDistanceError::OneStepDistanceError(const options::Options &opts) : OneStepError(opts) {
	const auto warm_start_samples = opts.get<int>("warm_start_samples");
	if (warm_start_samples != 0) {
		count = warm_start_samples;
		average_heuristic_error = opts.get<double>("warm_start_value");
	}
}

void OneStepDistanceError::add_successor(StateID state_id, int, int evaluator_value) {
	if (best_successor_value == NO_VALUE || evaluator_value < best_successor_value) {
		best_successor_value = evaluator_value;
		best_successor_id = state_id;
	}
}

auto OneStepDistanceError::compute_error(const GlobalState &state, const GlobalState &best_successor) const -> double {
	return std::clamp(evaluator->get_cached_estimate(best_successor) + 1 - evaluator->get_cached_estimate(state), 0, 1);
}

static auto _parse(OptionParser &parser) -> std::shared_ptr<HeuristicError> {
	HeuristicError::add_options_to_parser(parser);
	parser.add_option<int>("warm_start_samples", "Number of samples to warm-start the distance error with.", "0", Bounds("0", "infinity"));
	parser.add_option<double>("warm_start_value", "Value to warm-start the distance error with.", ".2", Bounds("0", "1"));
	Options opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<OneStepDistanceError>(opts);
}

static Plugin<HeuristicError> _plugin("one_step_distance_error", _parse);
} // namespace heuristic_error
