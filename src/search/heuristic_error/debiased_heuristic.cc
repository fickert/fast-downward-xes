#include "debiased_heuristic.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace heuristic_error {
DebiasedHeuristic::DebiasedHeuristic(const Options &opts)
	: FloatingPointEvaluator(opts),
	  heuristic(opts.get<std::shared_ptr<Evaluator>>("h")),
	  distance(opts.get<std::shared_ptr<FloatingPointEvaluator>>("d")),
	  heuristic_error(opts.get<std::shared_ptr<HeuristicError>>("error")),
	  admissible_h(opts.get<bool>("admissible_h")) {}

auto DebiasedHeuristic::compute_value(EvaluationContext &eval_context) -> double {
	auto distance_value = distance->compute_result(eval_context);
	if (eval_context.is_evaluator_value_infinite(heuristic.get()) || is_dead_end(distance_value))
		return DEAD_END;
	const auto h = eval_context.get_evaluator_value(heuristic.get());
	// if a negative heuristic error causes the debiased value to become negative bump it up to zero
	// if the heuristic is admissible ensure that the resulting value is not reduced below h
	return std::max(admissible_h ? h : 0., h + distance_value * heuristic_error->get_average_heuristic_error());
}

static auto _parse(OptionParser &parser) -> std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> {
	parser.add_option<std::shared_ptr<Evaluator>>("h", "heuristic");
	parser.add_option<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("d", "(debiased) distance");
	parser.add_option<std::shared_ptr<HeuristicError>>("error", "heuristic error model to use");
	parser.add_option<bool>("admissible_h", "Indicate that the heuristic is admissible; the resulting debiased value will not be lower than h.", "false");
	floating_point_evaluator::FloatingPointEvaluator::add_options_to_parser(parser);
	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<DebiasedHeuristic>(opts);
}

static Plugin<floating_point_evaluator::FloatingPointEvaluator> _plugin("debiased_h", _parse);
} // namespace heuristic_error
