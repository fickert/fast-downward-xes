#include "floating_point_evaluator.h"

#include <memory>

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace floating_point_evaluator {
auto FloatingPointEvaluator::get_default_options() -> options::Options {
	auto opts = options::Options();
	opts.set("cache_estimates", false);
	return opts;
}

FloatingPointEvaluator::FloatingPointEvaluator(const options::Options &opts)
	: cache(std::numeric_limits<double>::quiet_NaN()), cache_evaluator_values(opts.get<bool>("cache_estimates")) {}

auto FloatingPointEvaluator::compute_result(EvaluationContext &eval_context) -> double {
	const auto &global_state = eval_context.get_state();
	if (cache_evaluator_values && is_estimate_cached(global_state))
		return get_cached_estimate(global_state);
	const auto value = compute_value(eval_context);
	if (cache_evaluator_values)
		cache[global_state] = value;
	return value;
}

void FloatingPointEvaluator::add_options_to_parser(options::OptionParser &parser) {
	// NOTE: the cache is disabled by default because most floating point evaluators are lightweight wrappers
	parser.add_option<bool>("cache_estimates", "cache heuristic estimates", "false");
}

static PluginTypePlugin<FloatingPointEvaluator> _type_plugin("FloatingPointEvaluator", "An estimator that computes a floating point value for a given state.",
                                                             "fp_evaluator", "fpeval");
} // namespace floating_point_evaluator
