#include "../evaluators/g_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator_wrapper.h"
#include "../floating_point_evaluator/fp_sum_evaluator.h"
#include "../heuristic_error/debiased_distance.h"
#include "../heuristic_error/debiased_heuristic.h"
#include "../heuristic_error/one_step_distance_error.h"
#include "../heuristic_error/one_step_heuristic_error.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../suboptimal_search/util.h"
#include "eager_bounded_cost_search.h"
#include "linear_relative_error_potential_evaluator.h"

using namespace floating_point_evaluator;
using namespace suboptimal_search;

namespace bounded_cost_search {
static auto create_potential_search(options::Options &opts, const std::shared_ptr<FloatingPointEvaluator> &heuristic) -> std::shared_ptr<SearchEngine> {
	auto potential_opts = options::Options();
	potential_opts.set("bound", opts.get<int>("bound"));
	potential_opts.set("heuristic", heuristic);
	potential_opts.set("cache_estimates", false);
	const auto potential_evaluator = std::make_shared<LinearRelativeErrorPotentialEvaluator>(potential_opts);

	if (opts.get<bool>("enable_tie_breaking")) {
		auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
		auto f_evaluator = std::make_shared<floating_point_evaluator::SumEvaluator>(std::vector<std::shared_ptr<FloatingPointEvaluator>>{
				std::make_shared<floating_point_evaluator::FloatingPointEvaluatorWrapper>(g_evaluator), heuristic});
		auto last_tie_breaker =
				opts.contains("distance")
						? std::make_shared<floating_point_evaluator::FloatingPointEvaluatorWrapper>(opts.get<std::shared_ptr<Evaluator>>("distance"))
						: heuristic;
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {potential_evaluator, f_evaluator, last_tie_breaker});
		return std::make_shared<EagerBoundedCostSearch<3>>(opts);
	} else {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {potential_evaluator});
		return std::make_shared<EagerBoundedCostSearch<1>>(opts);
	}
}

static std::shared_ptr<SearchEngine> _parse_pts(OptionParser &parser) {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	add_f_then_h_tie_breaking_option(parser);
	add_options_to_parser(parser);

	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;

	auto fp_wrapper_opts = options::Options();
	fp_wrapper_opts.set("eval", opts.get<std::shared_ptr<Evaluator>>("heuristic"));
	fp_wrapper_opts.set("cache_estimates", false);
	const auto fp_heuristic = std::make_shared<FloatingPointEvaluatorWrapper>(fp_wrapper_opts);
	return create_potential_search(opts, fp_heuristic);
}

static std::shared_ptr<SearchEngine> _parse_pts_hat(OptionParser &parser) {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	add_bounded_cost_warm_start_options(parser);
	add_percentage_based_error_option(parser);
	add_f_hat_then_d_tie_breaking_option(parser);
	add_options_to_parser(parser);

	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;

	auto distance_error_opts = options::Options();
	distance_error_opts.set("eval", opts.get<std::shared_ptr<Evaluator>>("distance"));
	distance_error_opts.set("warm_start_samples", opts.get<int>("warm_start_samples"));
	distance_error_opts.set("warm_start_value", opts.get<double>("warm_start_value_distance"));
	auto distance_error = std::make_shared<heuristic_error::OneStepDistanceError>(distance_error_opts);

	auto heuristic_error = get_heuristic_error(opts.get<bool>("percentage_based_error"), opts.get<std::shared_ptr<Evaluator>>("heuristic"),
	                                           opts.get<int>("warm_start_samples"), opts.get<double>("warm_start_value_heuristic"));

	auto debiased_distance_opts = options::Options();
	debiased_distance_opts.set("cache_estimates", false);
	debiased_distance_opts.set("d", opts.get<std::shared_ptr<Evaluator>>("distance"));
	debiased_distance_opts.set<std::shared_ptr<heuristic_error::HeuristicError>>("error", distance_error);
	auto debiased_distance = std::make_shared<heuristic_error::DebiasedDistance>(debiased_distance_opts);

	auto debiased_heuristic = get_debiased_heuristic(opts.get<bool>("percentage_based_error"), opts.get<std::shared_ptr<Evaluator>>("heuristic"),
	                                                 debiased_distance, heuristic_error);

	auto heuristic_error_observers = opts.get_list<std::shared_ptr<heuristic_error::HeuristicError>>("error");
	heuristic_error_observers.push_back(distance_error);
	heuristic_error_observers.push_back(heuristic_error);
	opts.set("error", std::move(heuristic_error_observers));

	return create_potential_search(opts, debiased_heuristic);
}

static options::Plugin<SearchEngine> _plugin_bees("pts", _parse_pts);
static options::Plugin<SearchEngine> _plugin_beeps("pts_hat", _parse_pts_hat);
} // namespace bounded_cost_search
