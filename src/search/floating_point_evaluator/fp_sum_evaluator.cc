#include "fp_sum_evaluator.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace floating_point_evaluator {
SumEvaluator::SumEvaluator(const options::Options &opts)
	: FloatingPointEvaluator(opts), subevaluators(opts.get_list<std::shared_ptr<FloatingPointEvaluator>>("evals")) {}

SumEvaluator::SumEvaluator(std::vector<std::shared_ptr<FloatingPointEvaluator>> subevaluators)
	: FloatingPointEvaluator(get_default_options()), subevaluators(subevaluators) {}

auto SumEvaluator::compute_value(EvaluationContext &eval_context) -> double {
	auto sum = 0.;
	for (const auto &subevaluator : subevaluators) {
		const auto subevaluator_value = subevaluator->compute_result(eval_context);
		if (is_dead_end(subevaluator_value))
			return DEAD_END;
		sum += subevaluator_value;
	}
	return sum;
}

static std::shared_ptr<FloatingPointEvaluator> _parse(options::OptionParser &parser) {
	parser.add_list_option<std::shared_ptr<FloatingPointEvaluator>>("evals", "at least one evaluator");
	FloatingPointEvaluator::add_options_to_parser(parser);

	const auto opts = parser.parse();
	opts.verify_list_non_empty<std::shared_ptr<Evaluator>>("evals");
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<SumEvaluator>(opts);
}

static Plugin<FloatingPointEvaluator> _plugin("fp_sum", _parse);
} // namespace floating_point_evaluator
