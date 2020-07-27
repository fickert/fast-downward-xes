#include "debiased_distance.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace heuristic_error {
DebiasedDistance::DebiasedDistance(const Options &opts)
	: FloatingPointEvaluator(opts),
	  distance(opts.get<std::shared_ptr<Evaluator>>("d")),
	  heuristic_error(opts.get<std::shared_ptr<HeuristicError>>("error")) {}

auto DebiasedDistance::compute_value(EvaluationContext &eval_context) -> double {
	if (eval_context.is_evaluator_value_infinite(distance.get()))
		return DEAD_END;
	return eval_context.get_evaluator_value(distance.get()) / (1 - heuristic_error->get_average_heuristic_error());
}

static auto _parse(OptionParser &parser) -> std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> {
	parser.add_option<std::shared_ptr<Evaluator>>("d", "distance");
	parser.add_option<std::shared_ptr<HeuristicError>>("error", "heuristic error model to use");
	floating_point_evaluator::FloatingPointEvaluator::add_options_to_parser(parser);
	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<DebiasedDistance>(opts);
}

static Plugin<floating_point_evaluator::FloatingPointEvaluator> _plugin("debiased_d", _parse);
} // namespace heuristic_error
