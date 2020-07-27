#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator_wrapper.h"
#include "../floating_point_evaluator/fp_division_evaluator.h"
#include "../floating_point_evaluator/fp_sum_evaluator.h"
#include "../heuristic_error/debiased_distance.h"
#include "../heuristic_error/debiased_heuristic.h"
#include "../heuristic_error/one_step_distance_error.h"
#include "../heuristic_error/one_step_heuristic_error.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "cost_bound_assumptions_nancy_evaluator.h"
#include "eager_bounded_cost_search.h"
#include "util.h"

using namespace floating_point_evaluator;

namespace bounded_cost_search {
static std::shared_ptr<SearchEngine> _parse(options::OptionParser &parser) {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	add_warm_start_options(parser);
	add_percentage_based_error_option(parser);
	add_online_variance_option(parser);
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

	auto expected_work = std::make_shared<DivisionEvaluator>(debiased_distance, nancy_assumptions_evaluator);

	auto heuristic_error_observers = opts.get_list<std::shared_ptr<heuristic_error::HeuristicError>>("error");
	heuristic_error_observers.push_back(distance_error);
	heuristic_error_observers.push_back(heuristic_error);
	opts.set("error", std::move(heuristic_error_observers));

	if (opts.get<bool>("enable_tie_breaking")) {
		auto d_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(opts.get<std::shared_ptr<Evaluator>>("distance"));
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {expected_work, f_hat_evaluator, d_evaluator});
		return std::make_shared<EagerBoundedCostSearch<3>>(opts);
	} else {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {expected_work});
		return std::make_shared<EagerBoundedCostSearch<1>>(opts);
	}
}

static options::Plugin<SearchEngine> _plugin("xes", _parse);
} // namespace bounded_cost_search
