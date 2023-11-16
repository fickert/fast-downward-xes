#include "cost_bound_assumptions_nancy_evaluator.h"

#include <cmath>

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace bounded_cost_search {
auto NancyAssumptionsCBSEvaluator::cumulative_distribution(double x) -> double {
	return (1 + std::erf(x / std::sqrt(2.))) / 2.;
}

NancyAssumptionsCBSEvaluator::NancyAssumptionsCBSEvaluator(const options::Options &opts)
	: floating_point_evaluator::FloatingPointEvaluator(opts),
	  cost_bound(opts.get<int>("cost_bound")),
	  f_evaluator(opts.get<std::shared_ptr<Evaluator>>("f")),
	  f_hat_evaluator(opts.get<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("f_hat")),
	  d_hat_evaluator(opts.get<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("d_hat", nullptr)),
	  heuristic_error(opts.get<std::shared_ptr<heuristic_error::HeuristicError>>("heuristic_error", nullptr)) {}

auto NancyAssumptionsCBSEvaluator::compute_value(EvaluationContext &eval_context) -> double {
	if (eval_context.is_evaluator_value_infinite(f_evaluator.get()))
		return DEAD_END;
	const auto f = eval_context.get_evaluator_value(f_evaluator.get());
	if (f == eval_context.get_g_value())
		// The heuristic has evaluated to zero. If the search prunes states with g > cost_bound, the following case distinction would be unnecessary.
		// NOTE: This may break with 0-cost actions and an inadmissible heuristic than can give estimates > 0 for states that do have a 0-cost path to the goal.
		return f <= cost_bound ? 1. : 0.;
	// NOTE: If f is admissible, then we can truncate the gaussian at f instead of g.
	// NOTE: The lower bound has probability 0, so maybe we need to decrease it by one (or some epsilon), at least in the presence of 0-cost actions and/or a heuristic that is not goal-aware.
	const auto lower_bound = eval_context.get_g_value();
	// const auto lower_bound = eval_context.get_g_value() - 1;
	const auto f_hat = f_hat_evaluator->compute_result(eval_context);
	assert(!std::isnan(f_hat));
	assert(!is_dead_end(f_hat));
	if (f == f_hat)
		// Returning absolute values is really dangerous here since the estimate may not be accurate.
		// This should essentially never happen if the error is initialized to a non-zero value.
		return f <= cost_bound ? 1. : 0.;
	if (f_hat == std::numeric_limits<double>::infinity())
		// One case where this happens is if the distance error is 1, making the debiased distance (and also the debiased heuristic) infinite.
		return 0.;
	// compute the cumulative distribution function for the one-sided truncated gaussian centered around f-hat with variance (abs(f-hat - f) / 2)^2, see
	// https://en.wikipedia.org/wiki/Truncated_normal_distribution
	// https://people.sc.fsu.edu/~jburkardt/presentations/truncated_normal.pdf
	const auto mean = static_cast<double>(f_hat);
	assert(!heuristic_error || d_hat_evaluator);
	const auto standard_deviation = heuristic_error ? std::sqrt(heuristic_error->get_heuristic_error_variance() * d_hat_evaluator->compute_result(eval_context))
	                                                : std::abs(f_hat - f) / 2;
	assert(standard_deviation >= 0.);
	if (standard_deviation == 0.)
		return mean <= cost_bound + .5 ? 1. : 0.;
	const auto cdf_xi = cumulative_distribution((cost_bound + .5 - mean) / standard_deviation);
	const auto cdf_alpha = cumulative_distribution((lower_bound - mean) / standard_deviation);
#ifndef NDEBUG
	const auto is_valid_probility = [](double p) {
		return p >= 0. && p <= 1.;
	};
	std::cout << "g=" << eval_context.get_g_value() << ", f=" << f << ", fhat=" << f_hat << ", p=" << (cdf_xi - cdf_alpha) / (1 - cdf_alpha) << std::endl;
	assert(is_valid_probility(cdf_xi));
	assert(is_valid_probility(cdf_alpha));
	assert(is_valid_probility((cdf_xi - cdf_alpha) / (1 - cdf_alpha)));
#endif
	return (cdf_xi - cdf_alpha) / (1 - cdf_alpha);
}

static auto _parse(OptionParser &parser) -> std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> {
	floating_point_evaluator::FloatingPointEvaluator::add_options_to_parser(parser);
	parser.add_option<int>("cost_bound", "cost bound", "", options::Bounds("0", "infinity"));
	parser.add_option<std::shared_ptr<Evaluator>>("f", "f evaluator");
	parser.add_option<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("f_hat", "f-hat evaluator");
	parser.add_option<std::shared_ptr<heuristic_error::HeuristicError>>("heuristic_error", "heuristic error observer from which to read the variance");
	Options opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<NancyAssumptionsCBSEvaluator>(opts);
}

static Plugin<floating_point_evaluator::FloatingPointEvaluator> _plugin("cbs_nancy_assumptions", _parse);
} // namespace bounded_cost_search
