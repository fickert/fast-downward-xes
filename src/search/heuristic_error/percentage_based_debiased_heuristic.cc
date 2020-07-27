#include "percentage_based_debiased_heuristic.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace heuristic_error {
PercentageBasedDebiasedHeuristic::PercentageBasedDebiasedHeuristic(const Options &opts)
	: FloatingPointEvaluator(opts),
	  heuristic(opts.get<std::shared_ptr<Evaluator>>("h")),
	  heuristic_error(opts.get<std::shared_ptr<HeuristicError>>("error")) {}

auto PercentageBasedDebiasedHeuristic::compute_value(EvaluationContext &eval_context) -> double {
	if (eval_context.is_evaluator_value_infinite(heuristic.get()))
		return DEAD_END;
	const auto h = eval_context.get_evaluator_value(heuristic.get());
	return h * heuristic_error->get_average_heuristic_error();
}

static auto _parse(OptionParser &parser) -> std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> {
	parser.add_option<std::shared_ptr<Evaluator>>("h", "heuristic");
	parser.add_option<std::shared_ptr<HeuristicError>>("error", "heuristic error model to use");
	floating_point_evaluator::FloatingPointEvaluator::add_options_to_parser(parser);
	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<PercentageBasedDebiasedHeuristic>(opts);
}

static Plugin<floating_point_evaluator::FloatingPointEvaluator> _plugin("percentage_based_debiased_h", _parse);
} // namespace heuristic_error
