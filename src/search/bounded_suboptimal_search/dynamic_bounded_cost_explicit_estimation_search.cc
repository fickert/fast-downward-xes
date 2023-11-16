#include "dynamic_bounded_cost_explicit_estimation_search.h"

#include <cassert>

#include "../evaluation_context.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator_wrapper.h"
#include "../floating_point_evaluator/fp_sum_evaluator.h"
#include "../floating_point_open_list/best_first_open_list.h"
#include "../floating_point_open_list/floating_point_open_list.h"
#include "../heuristic_error/debiased_distance.h"
#include "../heuristic_error/one_step_distance_error.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../suboptimal_search/util.h"

using namespace floating_point_evaluator;
using namespace floating_point_open_list;
using namespace suboptimal_search;

namespace bounded_suboptimal_search {
template <bool tie_breaking>
DynamicBoundedCostExplicitEstimationSearch<tie_breaking>::DynamicBoundedCostExplicitEstimationSearch(const Options &opts)
	: EagerSuboptimalSearch<N>(opts),
	  f_min(0),
	  suboptimality_factor(opts.get<double>("suboptimality_factor")),
	  heuristic(opts.get<std::shared_ptr<Evaluator>>("heuristic")),
	  f_evaluator(std::make_shared<sum_evaluator::SumEvaluator>(
			  std::vector<std::shared_ptr<Evaluator>>{std::make_shared<g_evaluator::GEvaluator>(), opts.get<std::shared_ptr<Evaluator>>("heuristic")})),
	  d_hat_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator")) {
	if (!opts.get_list<std::shared_ptr<Evaluator>>("preferred").empty()) {
		std::cerr << "DBEES currently does not support preferred operators, exiting." << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
	}
}

template <bool tie_breaking>
void DynamicBoundedCostExplicitEstimationSearch<tie_breaking>::update_focal() {
	// remove closed nodes from the front of the open list
	while (true) {
		if (open_list.empty())
			return;
		auto &state_ids = std::begin(open_list)->second;
		const auto id = state_ids.front().second;
		const auto s = state_registry.lookup_state(id);
		const auto node = search_space.get_node(s);
		if (!node.is_closed())
			break;
		std::pop_heap(std::begin(state_ids), std::end(state_ids), open_list_compare);
		state_ids.pop_back();
		if (state_ids.empty())
			open_list.erase(std::begin(open_list));
	}

	assert(!open_list.empty());
	assert(!std::begin(open_list)->second.empty());
	const auto current_f_min = std::begin(open_list)->first;

	if (current_f_min <= f_min)
		return;

	// f_min increased --> fix focal/f-hat by copying all nodes that were previously
	// outside the bound (i.e. suboptimality factor * current_f_min) and are now
	// inside it (i.e. suboptimality_factor * f_min)
	const auto lower = open_list.upper_bound(suboptimality_factor * f_min);
	const auto upper = open_list.upper_bound(suboptimality_factor * current_f_min);
	for (auto it = lower; it != upper; ++it) {
		for (auto [_, state_id] : it->second) {
			const auto state = state_registry.lookup_state(state_id);
			const auto node = search_space.get_node(state);
			if (node.is_closed())
				continue;
			auto eval_context = EvaluationContext(state, node.get_g(), false, &statistics);
			auto evaluator_values = this->compute_results(eval_context);
			focal_list.emplace(std::move(evaluator_values), std::move(state_id), false);
		}
	}
	f_min = current_f_min;
}

template <bool tie_breaking>
auto DynamicBoundedCostExplicitEstimationSearch<tie_breaking>::fetch_next_node() -> std::optional<SearchNode> {
	const auto fetch_top = [this](auto &open_list) {
		while (!open_list.empty()) {
			const auto id = open_list.top();
			const auto state = state_registry.lookup_state(id);
			auto node = search_space.get_node(state);
			if (!node.is_closed())
				return std::optional(node);
			open_list.pop();
		}
		return std::optional<SearchNode>();
	};

	const auto close_node = [this](auto &node) {
		assert(node);
		assert(!node->is_closed());
		node->close();
		assert(!node->is_dead_end());
		statistics.inc_expanded();
	};

	update_focal();

	if (open_list.empty())
		return {};

	auto best_d_hat = fetch_top(focal_list);
	if (best_d_hat) {
		auto best_d_hat_eval_context = EvaluationContext(best_d_hat->get_state(), best_d_hat->get_g(), false, &statistics, false);
		focal_list.pop();
		close_node(best_d_hat);
		return best_d_hat;
	}

	assert(!std::begin(open_list)->second.empty());
	const auto id = std::begin(open_list)->second.front().second;
	assert(!search_space.get_node(state_registry.lookup_state(id)).is_closed());

	const auto state = state_registry.lookup_state(id);
	auto node = std::optional<SearchNode>(search_space.get_node(state));
	close_node(node);
	return node;
}

template <bool tie_breaking>
void DynamicBoundedCostExplicitEstimationSearch<tie_breaking>::insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values,
                                                                      StateID state_id, bool preferred) {
	assert(!eval_context.is_evaluator_value_infinite(f_evaluator.get()));
	const auto f = eval_context.get_evaluator_value(f_evaluator.get());
	open_list[f].emplace_back(eval_context.get_g_value(), state_id);
	std::push_heap(std::begin(open_list[f]), std::end(open_list[f]), open_list_compare);

	if (f <= suboptimality_factor * f_min)
		focal_list.push(evaluator_values, state_id, preferred);
}

template <bool tie_breaking>
void DynamicBoundedCostExplicitEstimationSearch<tie_breaking>::reward_progress() {
	// preferred operators are not (yet?) implemented for DBEES --> nothing to do here
}

static auto _parse_dbees(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	parser.add_option<double>("suboptimality_factor", "Suboptimality factor.", "2.0", Bounds("1.0", "infinity"));
	parser.add_option<bool>("enable_tie_breaking", "Break ties by h and h-hat in the queues ordered by f respectively f-hat.", "true");
	parser.add_option<bool>("admissible_h", "Indicate that the heuristic is admissible for the debiased heuristic.", "true");

	add_warm_start_options(parser);
	add_percentage_based_error_option(parser);
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

	// for EES, the debiased heuristic values must be cached to ensure correct synchronization between the queues
	auto debiased_heuristic = get_debiased_heuristic(opts.get<bool>("percentage_based_error"), opts.get<std::shared_ptr<Evaluator>>("heuristic"),
	                                                 debiased_distance, heuristic_error, opts.get<bool>("admissible_h"), true);

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

	opts.set<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator", debiased_distance);
	opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat_evaluator", f_hat_evaluator);

	if (opts.get<bool>("enable_tie_breaking")) {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {debiased_distance, f_hat_evaluator, fp_f_evaluator});
		return std::static_pointer_cast<SearchEngine>(std::make_shared<DynamicBoundedCostExplicitEstimationSearch<true>>(opts));
	} else {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {debiased_distance});
		return std::static_pointer_cast<SearchEngine>(std::make_shared<DynamicBoundedCostExplicitEstimationSearch<false>>(opts));
	}
	
	return opts.get<bool>("enable_tie_breaking") ? std::static_pointer_cast<SearchEngine>(std::make_shared<DynamicBoundedCostExplicitEstimationSearch<true>>(opts))
	                                             : std::static_pointer_cast<SearchEngine>(std::make_shared<DynamicBoundedCostExplicitEstimationSearch<false>>(opts));
}

static auto _parse_astarepsilon(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	parser.add_option<double>("suboptimality_factor", "Suboptimality factor.", "2.0", Bounds("1.0", "infinity"));
	parser.add_option<bool>("enable_tie_breaking", "Break ties by h and h-hat in the queues ordered by f respectively f-hat.", "true");
	parser.add_option<bool>("admissible_h", "Indicate that the heuristic is admissible for the debiased heuristic.", "true");

	add_warm_start_options(parser);
	add_percentage_based_error_option(parser);
	add_options_to_parser(parser);

	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;

	auto fp_h_opts = options::Options();
	fp_h_opts.set("cache_estimates", false);
	fp_h_opts.set("eval", opts.get<std::shared_ptr<Evaluator>>("heuristic"));
	const auto fp_h_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(fp_h_opts);

	auto fp_d_opts = options::Options();
	fp_d_opts.set("cache_estimates", false);
	fp_d_opts.set("eval", opts.get<std::shared_ptr<Evaluator>>("distance"));
	const auto fp_d_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(fp_d_opts);

	auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	auto f_evaluator = std::make_shared<sum_evaluator::SumEvaluator>(
			std::vector<std::shared_ptr<Evaluator>>{g_evaluator, opts.get<std::shared_ptr<Evaluator>>("heuristic")});

	auto fp_f_opts = options::Options();
	fp_f_opts.set("cache_estimates", false);
	fp_f_opts.set<std::shared_ptr<Evaluator>>("eval", f_evaluator);
	const auto fp_f_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(fp_f_opts);

	opts.set<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator", fp_d_evaluator);

	if (opts.get<bool>("enable_tie_breaking")) {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {fp_d_evaluator, fp_f_evaluator, fp_h_evaluator});
		return std::static_pointer_cast<SearchEngine>(std::make_shared<DynamicBoundedCostExplicitEstimationSearch<true>>(opts));
	} else {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {fp_d_evaluator});
		return std::static_pointer_cast<SearchEngine>(std::make_shared<DynamicBoundedCostExplicitEstimationSearch<false>>(opts));
	}

	return opts.get<bool>("enable_tie_breaking")
	             ? std::static_pointer_cast<SearchEngine>(std::make_shared<DynamicBoundedCostExplicitEstimationSearch<true>>(opts))
	             : std::static_pointer_cast<SearchEngine>(std::make_shared<DynamicBoundedCostExplicitEstimationSearch<false>>(opts));
}

static options::Plugin<SearchEngine> _plugin_dbees("dbees", _parse_dbees);
static options::Plugin<SearchEngine> _plugin_astarepsilon("astarepsilon", _parse_astarepsilon);

template class DynamicBoundedCostExplicitEstimationSearch<true>;
template class DynamicBoundedCostExplicitEstimationSearch<false>;
} // namespace bounded_suboptimal_search
