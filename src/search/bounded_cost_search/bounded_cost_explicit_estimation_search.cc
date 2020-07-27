#include "bounded_cost_explicit_estimation_search.h"

#include <cassert>
#include <memory>

#include "../algorithms/ordered_set.h"
#include "../evaluation_context.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator_wrapper.h"
#include "../floating_point_evaluator/fp_sum_evaluator.h"
#include "../floating_point_open_list/floating_point_open_list.h"
#include "../heuristic_error/debiased_distance.h"
#include "../heuristic_error/debiased_heuristic.h"
#include "../heuristic_error/heuristic_error.h"
#include "../heuristic_error/one_step_distance_error.h"
#include "../heuristic_error/one_step_heuristic_error.h"
#include "../option_parser.h"
#include "../options/plugin.h"
#include "cost_bound_assumptions_nancy_evaluator.h"
#include "linear_relative_error_potential_evaluator.h"
#include "util.h"

using namespace floating_point_evaluator;
using namespace floating_point_open_list;

namespace bounded_cost_search {
template <std::size_t N>
const typename BoundedCostExplicitEstimationSearch<N>::OpenListCreationFunction BoundedCostExplicitEstimationSearch<N>::create_focal_open_list = []() {
	return std::make_unique<FocalOpenList<N, StateID>>();
};

template <std::size_t N>
BoundedCostExplicitEstimationSearch<N>::BoundedCostExplicitEstimationSearch(const Options &opts)
	: EagerBoundedCostSearch<N>(opts, create_focal_open_list),
	  f_hat_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("f_hat_evaluator")),
	  d_hat_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator")) {}

template <std::size_t N>
void BoundedCostExplicitEstimationSearch<N>::insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) {
	EagerBoundedCostSearch<N>::insert(eval_context, evaluator_values, state_id, preferred);
	assert(!eval_context.get_calculate_preferred());
	const auto f_hat_value = f_hat_evaluator->compute_result(eval_context);
	if (f_hat_value <= EagerBoundedCostSearch<N>::bound) {
		const auto d_hat_value = d_hat_evaluator->compute_result(eval_context);
		EagerBoundedCostSearch<N>::open_list->push_focal(d_hat_value, state_id, preferred);
	}
}

static auto _parse_bees(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	add_warm_start_options(parser);
	add_percentage_based_error_option(parser);
	add_d_tie_breaking_option(parser);
	bounded_cost_search::add_options_to_parser(parser);

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

	auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	auto f_evaluator = std::make_shared<sum_evaluator::SumEvaluator>(
			std::vector<std::shared_ptr<Evaluator>>{g_evaluator, opts.get<std::shared_ptr<Evaluator>>("heuristic")});
	auto f_hat_evaluator = std::make_shared<SumEvaluator>(
			std::vector<std::shared_ptr<FloatingPointEvaluator>>{std::make_shared<FloatingPointEvaluatorWrapper>(g_evaluator), debiased_heuristic});

	auto fp_f_opts = options::Options();
	fp_f_opts.set("cache_estimates", false);
	fp_f_opts.set<std::shared_ptr<Evaluator>>("eval", f_evaluator);
	const auto fp_f_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(fp_f_opts);

	auto heuristic_error_observers = opts.get_list<std::shared_ptr<heuristic_error::HeuristicError>>("error");
	heuristic_error_observers.push_back(distance_error);
	heuristic_error_observers.push_back(heuristic_error);
	opts.set("error", std::move(heuristic_error_observers));

	opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat_evaluator", f_hat_evaluator);
	opts.set<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator", debiased_distance);

	if (opts.get<bool>("enable_tie_breaking")) {
		auto d_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(opts.get<std::shared_ptr<Evaluator>>("distance"));
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {fp_f_evaluator, d_evaluator});
		return std::make_shared<BoundedCostExplicitEstimationSearch<2>>(opts);
	} else {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {fp_f_evaluator});
		return std::make_shared<BoundedCostExplicitEstimationSearch<1>>(opts);
	}
}

static std::shared_ptr<SearchEngine> _parse_beeps(OptionParser &parser) {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	add_warm_start_options(parser);
	add_percentage_based_error_option(parser);
	add_f_hat_then_d_tie_breaking_option(parser);
	bounded_cost_search::add_options_to_parser(parser);

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

	auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	auto f_hat_evaluator = std::make_shared<SumEvaluator>(
			std::vector<std::shared_ptr<FloatingPointEvaluator>>{std::make_shared<FloatingPointEvaluatorWrapper>(g_evaluator), debiased_heuristic});

	auto potential_opts = options::Options();
	potential_opts.set("cache_estimates", false);
	potential_opts.set("bound", opts.get<int>("bound"));
	potential_opts.set<std::shared_ptr<FloatingPointEvaluator>>("heuristic", debiased_heuristic);
	const auto potential_evaluator = std::make_shared<LinearRelativeErrorPotentialEvaluator>(potential_opts);

	auto heuristic_error_observers = opts.get_list<std::shared_ptr<heuristic_error::HeuristicError>>("error");
	heuristic_error_observers.push_back(distance_error);
	heuristic_error_observers.push_back(heuristic_error);
	opts.set("error", std::move(heuristic_error_observers));

	opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat_evaluator", f_hat_evaluator);
	opts.set<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator", debiased_distance);

	if (opts.get<bool>("enable_tie_breaking")) {
		auto d_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(opts.get<std::shared_ptr<Evaluator>>("distance"));
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {potential_evaluator, f_hat_evaluator, d_evaluator});
		return std::make_shared<BoundedCostExplicitEstimationSearch<3>>(opts);
	} else {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {potential_evaluator});
		return std::make_shared<BoundedCostExplicitEstimationSearch<1>>(opts);
	}
}

template <std::size_t N>
const typename BoundedCostExplicitEstimationPercentileSearch<N>::OpenListCreationFunction BoundedCostExplicitEstimationPercentileSearch<N>::create_focal_open_list =
		[]() {
			return std::make_unique<FocalOpenList<N, StateID>>();
		};

template <std::size_t N>
BoundedCostExplicitEstimationPercentileSearch<N>::BoundedCostExplicitEstimationPercentileSearch(const Options &opts)
	: EagerBoundedCostSearch<N>(opts, create_focal_open_list),
	  d_hat_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator")),
	  probability_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("probability_evaluator")),
	  focal_threshold(opts.get<double>("focal_threshold")) {}

template <std::size_t N>
void BoundedCostExplicitEstimationPercentileSearch<N>::insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id,
                                                              bool preferred) {
	EagerBoundedCostSearch<N>::insert(eval_context, evaluator_values, state_id, preferred);
	assert(!eval_context.get_calculate_preferred());
	const auto probability_value = probability_evaluator->compute_result(eval_context);
	if (probability_value >= focal_threshold) {
		const auto d_hat_value = d_hat_evaluator->compute_result(eval_context);
		EagerBoundedCostSearch<N>::open_list->push_focal(d_hat_value, state_id, preferred);
	}
}

static auto _parse_pbees(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	parser.add_option<double>("focal_threshold",
	                          "if a search node has at least the given probability to lead to a solution within the bound it is put into the focal list",
	                          ".95", Bounds("0", "1"));
	add_warm_start_options(parser);
	add_percentage_based_error_option(parser);
	add_online_variance_option(parser);
	add_d_tie_breaking_option(parser);
	bounded_cost_search::add_options_to_parser(parser);

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

	auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	auto f_evaluator = std::make_shared<sum_evaluator::SumEvaluator>(
			std::vector<std::shared_ptr<Evaluator>>{g_evaluator, opts.get<std::shared_ptr<Evaluator>>("heuristic")});
	auto f_hat_evaluator = std::make_shared<SumEvaluator>(
			std::vector<std::shared_ptr<FloatingPointEvaluator>>{std::make_shared<FloatingPointEvaluatorWrapper>(g_evaluator), debiased_heuristic});

	auto nancy_assumptions_opts = options::Options();
	nancy_assumptions_opts.set("cache_estimates", false);
	nancy_assumptions_opts.set("cost_bound", opts.get<int>("bound"));
	nancy_assumptions_opts.set<std::shared_ptr<Evaluator>>("f", f_evaluator);
	nancy_assumptions_opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat", f_hat_evaluator);
	if (opts.get<bool>("use_online_variance"))
		nancy_assumptions_opts.set<std::shared_ptr<heuristic_error::HeuristicError>>("heuristic_error", heuristic_error);
	auto nancy_assumptions_evaluator = std::make_shared<bounded_cost_search::NancyAssumptionsCBSEvaluator>(nancy_assumptions_opts);

	auto fp_f_opts = options::Options();
	fp_f_opts.set("cache_estimates", false);
	fp_f_opts.set<std::shared_ptr<Evaluator>>("eval", f_evaluator);
	const auto fp_f_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(fp_f_opts);

	auto heuristic_error_observers = opts.get_list<std::shared_ptr<heuristic_error::HeuristicError>>("error");
	heuristic_error_observers.push_back(distance_error);
	heuristic_error_observers.push_back(heuristic_error);
	opts.set("error", std::move(heuristic_error_observers));

	opts.set<std::shared_ptr<FloatingPointEvaluator>>("probability_evaluator", nancy_assumptions_evaluator);
	opts.set<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator", debiased_distance);

	if (opts.get<bool>("enable_tie_breaking")) {
		auto d_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(opts.get<std::shared_ptr<Evaluator>>("distance"));
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {fp_f_evaluator, d_evaluator});
		return std::make_shared<BoundedCostExplicitEstimationPercentileSearch<2>>(opts);
	} else {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {fp_f_evaluator});
		return std::make_shared<BoundedCostExplicitEstimationPercentileSearch<1>>(opts);
	}
}

static auto _parse_pbeeps(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	parser.add_option<double>("focal_threshold",
	                          "if a search node has at least the given probability to lead to a solution within the bound it is put into the focal list", ".95",
	                          Bounds("0", "1"));
	add_warm_start_options(parser);
	add_percentage_based_error_option(parser);
	add_online_variance_option(parser);
	add_d_tie_breaking_option(parser);
	bounded_cost_search::add_options_to_parser(parser);

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

	auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	auto f_evaluator = std::make_shared<sum_evaluator::SumEvaluator>(
			std::vector<std::shared_ptr<Evaluator>>{g_evaluator, opts.get<std::shared_ptr<Evaluator>>("heuristic")});
	auto f_hat_evaluator = std::make_shared<SumEvaluator>(
			std::vector<std::shared_ptr<FloatingPointEvaluator>>{std::make_shared<FloatingPointEvaluatorWrapper>(g_evaluator), debiased_heuristic});

	auto potential_opts = options::Options();
	potential_opts.set("cache_estimates", false);
	potential_opts.set("bound", opts.get<int>("bound"));
	potential_opts.set<std::shared_ptr<FloatingPointEvaluator>>("heuristic", debiased_heuristic);
	const auto potential_evaluator = std::make_shared<LinearRelativeErrorPotentialEvaluator>(potential_opts);

	auto nancy_assumptions_opts = options::Options();
	nancy_assumptions_opts.set("cache_estimates", false);
	nancy_assumptions_opts.set("cost_bound", opts.get<int>("bound"));
	nancy_assumptions_opts.set<std::shared_ptr<Evaluator>>("f", f_evaluator);
	nancy_assumptions_opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat", f_hat_evaluator);
	if (opts.get<bool>("use_online_variance"))
		nancy_assumptions_opts.set<std::shared_ptr<heuristic_error::HeuristicError>>("heuristic_error", heuristic_error);
	auto nancy_assumptions_evaluator = std::make_shared<bounded_cost_search::NancyAssumptionsCBSEvaluator>(nancy_assumptions_opts);

	auto heuristic_error_observers = opts.get_list<std::shared_ptr<heuristic_error::HeuristicError>>("error");
	heuristic_error_observers.push_back(distance_error);
	heuristic_error_observers.push_back(heuristic_error);
	opts.set("error", std::move(heuristic_error_observers));

	opts.set<std::shared_ptr<FloatingPointEvaluator>>("probability_evaluator", nancy_assumptions_evaluator);
	opts.set<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator", debiased_distance);

	if (opts.get<bool>("enable_tie_breaking")) {
		auto d_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(opts.get<std::shared_ptr<Evaluator>>("distance"));
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {potential_evaluator, f_hat_evaluator, d_evaluator});
		return std::make_shared<BoundedCostExplicitEstimationPercentileSearch<3>>(opts);
	} else {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {potential_evaluator});
		return std::make_shared<BoundedCostExplicitEstimationPercentileSearch<1>>(opts);
	}
}

static options::Plugin<SearchEngine> _plugin_bees("bees", _parse_bees);
static options::Plugin<SearchEngine> _plugin_beeps("beeps", _parse_beeps);
static options::Plugin<SearchEngine> _plugin_pbees("pbees", _parse_pbees);
static options::Plugin<SearchEngine> _plugin_pbeeps("pbeeps", _parse_pbeeps);

template class BoundedCostExplicitEstimationSearch<1>;
template class BoundedCostExplicitEstimationSearch<2>;
template class BoundedCostExplicitEstimationSearch<3>;
template class BoundedCostExplicitEstimationPercentileSearch<1>;
template class BoundedCostExplicitEstimationPercentileSearch<2>;
} // namespace bounded_cost_search
