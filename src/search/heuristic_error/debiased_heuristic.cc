#include "debiased_heuristic.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace heuristic_error {
DebiasedHeuristic::DebiasedHeuristic(const Options &opts)
	: FloatingPointEvaluator(opts),
	  heuristic(opts.get<std::shared_ptr<Evaluator>>("h")),
	  distance(opts.get<std::shared_ptr<FloatingPointEvaluator>>("d")),
	  heuristic_error(opts.get<std::shared_ptr<HeuristicError>>("error")) {}

auto DebiasedHeuristic::compute_value(EvaluationContext &eval_context) -> double {
	auto distance_value = distance->compute_result(eval_context);
	if (eval_context.is_evaluator_value_infinite(heuristic.get()) || is_dead_end(distance_value))
		return DEAD_END;
	const auto h = eval_context.get_evaluator_value(heuristic.get());
	// if a negative heuristic error cause the debiased value to become negative bump it up to zero
	return std::max(0., h + distance_value * heuristic_error->get_average_heuristic_error());
}

static auto _parse(OptionParser &parser) -> std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> {
	parser.add_option<std::shared_ptr<Evaluator>>("h", "heuristic");
	parser.add_option<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("d", "(debiased) distance");
	parser.add_option<std::shared_ptr<HeuristicError>>("error", "heuristic error model to use");
	floating_point_evaluator::FloatingPointEvaluator::add_options_to_parser(parser);
	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<DebiasedHeuristic>(opts);
}

static Plugin<floating_point_evaluator::FloatingPointEvaluator> _plugin("debiased_h", _parse);
} // namespace heuristic_error
