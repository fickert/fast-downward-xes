#include "suboptimality_bound_assumptions_nancy_evaluator.h"

#include <cmath>

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../suboptimal_search/util.h"

namespace bounded_suboptimal_search {
auto NancyAssumptionsSBSEvaluator::cumulative_distribution(double x) -> double {
	return (1 + std::erf(x / std::sqrt(2.))) / 2.;
}

NancyAssumptionsSBSEvaluator::NancyAssumptionsSBSEvaluator(const options::Options &opts)
	: floating_point_evaluator::FloatingPointEvaluator(opts),
	  suboptimality_factor(opts.get<double>("suboptimality_factor")),
	  f_evaluator(opts.get<std::shared_ptr<Evaluator>>("f")),
	  f_hat_evaluator(opts.get<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("f_hat")),
	  f_hat_min_evaluator(std::static_pointer_cast<FHatMinEvaluator>(opts.get<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("f_hat_min"))),
	  d_hat_evaluator(opts.get<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("d_hat")),
	  heuristic_error(opts.get<std::shared_ptr<heuristic_error::HeuristicError>>("heuristic_error")),
	  use_online_variance(opts.get<bool>("use_online_variance")),
	  admissible_h(opts.get<bool>("admissible_h")),
	  cost_bound_variance_method(opts.get<CostBoundVarianceMethod>("cost_bound_variance_method")) {}

auto NancyAssumptionsSBSEvaluator::compute_value(EvaluationContext &eval_context) -> double {
	if (eval_context.is_evaluator_value_infinite(f_evaluator.get()))
		return DEAD_END;

	const auto f = eval_context.get_evaluator_value(f_evaluator.get());
	const auto f_hat = f_hat_evaluator->compute_result(eval_context);
	assert(!FloatingPointEvaluator::is_dead_end(f_hat));
	const auto f_hat_min = f_hat_min_evaluator->get_f_hat_min();

	if (std::isinf(f_hat_min) || std::isinf(f_hat))
		// the heuristic error is likely not yet initialized in this case
		return 0.;

	// belief distribution about the solution cost (f-hat)
	const auto solution_cost_mean = static_cast<double>(f_hat);
	assert(!FloatingPointEvaluator::is_dead_end(d_hat_evaluator->compute_result(eval_context)));
	const auto solution_cost_stddev = use_online_variance
	                                        ? std::sqrt(heuristic_error->get_heuristic_error_variance() * d_hat_evaluator->compute_result(eval_context))
	                                        : std::abs(f_hat - f) / 2;
	assert(solution_cost_stddev >= 0.);

	// belief distribution about the cost bound
	const auto cost_bound_mean = suboptimality_factor * f_hat_min;
	auto cost_bound_stddev = 0.;
	switch (cost_bound_variance_method) {
	case CostBoundVarianceMethod::HEURISTIC_ERROR:
		cost_bound_stddev = std::sqrt(heuristic_error->get_heuristic_error_variance() * f_hat_min_evaluator->get_d_hat());
		break;
	case CostBoundVarianceMethod::F_MIN_VARIANCE:
		cost_bound_stddev = std::sqrt(f_hat_min_evaluator->get_variance());
		break;
	case CostBoundVarianceMethod::F_MIN_VARIANCE_TIMES_DISTANCE:
		cost_bound_stddev = std::sqrt(f_hat_min_evaluator->get_variance() * f_hat_min_evaluator->get_d_hat());
		break;
	case CostBoundVarianceMethod::ZERO:
		cost_bound_stddev = 0.;
		break;
	case CostBoundVarianceMethod::ZERO_IMPROVED: {
		if (solution_cost_stddev == 0.)
			return solution_cost_mean <= cost_bound_mean ? 1. : 0.;
		const auto lower_bound = admissible_h ? f : eval_context.get_g_value();
		const auto cdf_xi = cumulative_distribution((cost_bound_mean - solution_cost_mean) / solution_cost_stddev);
		const auto cdf_alpha = cumulative_distribution((lower_bound - solution_cost_mean) / solution_cost_stddev);
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
	default:
		utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
	assert(cost_bound_stddev >= 0.);


	// distribution describing the difference between the two distributions
	const auto mean = cost_bound_mean - solution_cost_mean;
	const auto stddev = std::sqrt(cost_bound_stddev * cost_bound_stddev + solution_cost_stddev * solution_cost_stddev);
	assert(stddev >= 0.);

	if (stddev == 0.)
		return mean >= 0. ? 1. : 0.;

	// the probability that the solution will be within the bound is the probability mass above zero
	const auto p_geq_zero = 1 - cumulative_distribution(-mean / stddev);
	assert(p_geq_zero >= 0. && p_geq_zero <= 1.);
	return p_geq_zero;
}

static auto _parse(OptionParser &parser) -> std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> {
	floating_point_evaluator::FloatingPointEvaluator::add_options_to_parser(parser);
	parser.add_option<int>("cost_bound", "cost bound", "", options::Bounds("0", "infinity"));
	parser.add_option<std::shared_ptr<Evaluator>>("f", "f evaluator");
	parser.add_option<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("f_hat", "f-hat evaluator");
	parser.add_option<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("f_hat_min", "f-hat-min evaluator");
	parser.add_option<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("d_hat", "d-hat evaluator");
	parser.add_option<std::shared_ptr<heuristic_error::HeuristicError>>("heuristic_error", "heuristic error observer from which to read the variance");
	parser.add_enum_option<NancyAssumptionsSBSEvaluator::CostBoundVarianceMethod>("cost_bound_variance_method", {"HEURISTIC_ERROR", "F_MIN_VARIANCE"},
	                                                                              "how to model the variance of the cost bound", "F_MIN_VARIANCE");
	suboptimal_search::add_online_variance_option(parser);
	Options opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<NancyAssumptionsSBSEvaluator>(opts);
}

static Plugin<floating_point_evaluator::FloatingPointEvaluator> _plugin("sbs_nancy_assumptions", _parse);
} // namespace bounded_suboptimal_search
