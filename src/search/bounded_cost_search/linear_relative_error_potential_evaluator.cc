#include "linear_relative_error_potential_evaluator.h"

#include "../evaluation_context.h"
#include "../option_parser.h"

namespace bounded_cost_search {
LinearRelativeErrorPotentialEvaluator::LinearRelativeErrorPotentialEvaluator(const options::Options &opts)
	: FloatingPointEvaluator(opts),
	  bound(opts.get<int>("bound")),
	  heuristic(opts.get<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("heuristic")) {}

auto LinearRelativeErrorPotentialEvaluator::compute_value(EvaluationContext &eval_context) -> double {
	auto value = heuristic->compute_result(eval_context);
	if (is_dead_end(value) || std::isinf(value) || value == 0)
		return value;
	const auto g = eval_context.get_g_value();
	return value / (1. - (static_cast<double>(g) / bound));
}
} // namespace bounded_cost_search
