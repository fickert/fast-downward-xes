#include "floating_point_evaluator_wrapper.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace floating_point_evaluator {
FloatingPointEvaluatorWrapper::FloatingPointEvaluatorWrapper(const options::Options &opts)
	: FloatingPointEvaluator(opts), evaluator(opts.get<std::shared_ptr<Evaluator>>("eval")) {}

FloatingPointEvaluatorWrapper::FloatingPointEvaluatorWrapper(const std::shared_ptr<Evaluator> &evaluator)
	: FloatingPointEvaluator(get_default_options()), evaluator(evaluator) {}

auto FloatingPointEvaluatorWrapper::compute_value(EvaluationContext &eval_context) -> double {
	if (eval_context.is_evaluator_value_infinite(evaluator.get()))
		return DEAD_END;
	assert(!std::isinf(static_cast<double>(eval_context.get_evaluator_value(evaluator.get()))));
	return static_cast<double>(eval_context.get_evaluator_value(evaluator.get()));
}

static std::shared_ptr<FloatingPointEvaluator> _parse(OptionParser &parser) {
	parser.add_option<std::shared_ptr<Evaluator>>("eval", "evaluator");
	FloatingPointEvaluator::add_options_to_parser(parser);
	const auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<FloatingPointEvaluatorWrapper>(opts);
}

static Plugin<FloatingPointEvaluator> _plugin("fp_wrapper", _parse);
} // namespace floating_point_evaluator
