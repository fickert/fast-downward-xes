#include "fp_weighted_evaluator.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace floating_point_evaluator {
WeightedEvaluator::WeightedEvaluator(const options::Options &opts)
	: FloatingPointEvaluator(opts),
	  evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("evaluator")),
	  weight(opts.get<double>("weight")) {}

WeightedEvaluator::WeightedEvaluator(std::shared_ptr<FloatingPointEvaluator> evaluator, double weight)
	: FloatingPointEvaluator(get_default_options()), evaluator(evaluator), weight(weight) {}

auto WeightedEvaluator::compute_value(EvaluationContext &eval_context) -> double {
	const auto evaluator_value = evaluator->compute_result(eval_context);
	if (is_dead_end(evaluator_value))
		return DEAD_END;
	return weight * evaluator_value;
}

static std::shared_ptr<FloatingPointEvaluator> _parse(options::OptionParser &parser) {
	parser.add_option<std::shared_ptr<FloatingPointEvaluator>>("evaluator", "evaluator");
	parser.add_option<double>("weight", "weight");
	FloatingPointEvaluator::add_options_to_parser(parser);
	const auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<WeightedEvaluator>(opts);
}

static Plugin<FloatingPointEvaluator> _plugin("fp_weight", _parse);
} // namespace floating_point_evaluator
