#include "fp_division_evaluator.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace floating_point_evaluator {
DivisionEvaluator::DivisionEvaluator(const options::Options &opts)
	: FloatingPointEvaluator(opts),
	  dividend(opts.get<std::shared_ptr<FloatingPointEvaluator>>("dividend")),
	  divisor(opts.get<std::shared_ptr<FloatingPointEvaluator>>("divisor")) {}

DivisionEvaluator::DivisionEvaluator(std::shared_ptr<FloatingPointEvaluator> dividend, std::shared_ptr<FloatingPointEvaluator> divisor)
	: FloatingPointEvaluator(get_default_options()), dividend(dividend), divisor(divisor) {}

auto DivisionEvaluator::compute_value(EvaluationContext &eval_context) -> double {
	const auto divisor_value = divisor->compute_result(eval_context);
	if (is_dead_end(divisor_value))
		return DEAD_END;
	if (divisor_value == 0.)
		return std::numeric_limits<double>::infinity();
	const auto dividend_value = dividend->compute_result(eval_context);
	if (is_dead_end(dividend_value))
		return DEAD_END;
	return dividend_value / divisor_value;
}

static std::shared_ptr<FloatingPointEvaluator> _parse(options::OptionParser &parser) {
	parser.add_option<std::shared_ptr<FloatingPointEvaluator>>("dividend", "dividend of the division");
	parser.add_option<std::shared_ptr<FloatingPointEvaluator>>("divisor", "divisor of the division");
	FloatingPointEvaluator::add_options_to_parser(parser);
	const auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<DivisionEvaluator>(opts);
}

static Plugin<FloatingPointEvaluator> _plugin("div", _parse);
} // namespace floating_point_evaluator
