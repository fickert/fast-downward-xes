#include "extended_dynamic_expected_effort_search.h"

#include <cassert>
#include <numeric>

#include "../evaluation_context.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../expansion_delay/expansion_delay_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator_wrapper.h"
#include "../floating_point_evaluator/fp_division_evaluator.h"
#include "../floating_point_evaluator/fp_sum_evaluator.h"
#include "../floating_point_open_list/best_first_open_list.h"
#include "../floating_point_open_list/floating_point_open_list.h"
#include "../heuristic_error/debiased_distance.h"
#include "../heuristic_error/one_step_distance_error.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../suboptimal_search/util.h"
#include "remaining_expansions_evaluator.h"
#include "suboptimality_bound_assumptions_nancy_evaluator.h"

using namespace expansion_delay;
using namespace floating_point_evaluator;
using namespace floating_point_open_list;
using namespace suboptimal_search;

namespace bounded_suboptimal_search {
template <std::size_t N>
ExtendedDynamicExpectedEffortSearch<N>::ExtendedDynamicExpectedEffortSearch(const Options &opts, std::shared_ptr<ExpansionDelay> expansion_delay)
	: EagerSuboptimalSearch<N>(opts),
	  reopen_closed_nodes(opts.get<bool>("reopen_closed")),
	  suboptimality_factor(opts.get<double>("suboptimality_factor")),
	  expected_work_error_margin(opts.get<double>("expected_work_error_margin")),
	  f_min(0),
	  h_error(opts.get<std::shared_ptr<heuristic_error::HeuristicError>>("h_error")),
	  expansion_delay(expansion_delay),
	  expected_work_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("expected_work_evaluator")),
	  f_evaluator(std::make_shared<sum_evaluator::SumEvaluator>(
			  std::vector<std::shared_ptr<Evaluator>>{std::make_shared<g_evaluator::GEvaluator>(), opts.get<std::shared_ptr<Evaluator>>("heuristic")})),
	  f_hat_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("f_hat_evaluator")),
	  d_hat_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator")),
	  f_hat_min_evaluator(std::static_pointer_cast<FHatMinEvaluator>(opts.get<std::shared_ptr<FloatingPointEvaluator>>("f_hat_min_evaluator"))),
	  best_f_expansions(0) {
	if (opts.contains("preferred") && !opts.get_list<std::shared_ptr<Evaluator>>("preferred").empty()) {
		std::cerr << "Dynamic expected effort search currently does not support preferred operators, exiting." << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
	}
	expansion_delay->initialize(statistics);
}

template <std::size_t N>
void ExtendedDynamicExpectedEffortSearch<N>::initialize_extra(EvaluationContext &eval_context) {
	const auto f_hat = f_hat_evaluator->compute_result(eval_context);
	const auto d_hat = d_hat_evaluator->compute_result(eval_context);
	if (!FloatingPointEvaluator::is_dead_end(f_hat) && !FloatingPointEvaluator::is_dead_end(d_hat))
		f_hat_min_evaluator->update(f_hat, d_hat);
}

template <std::size_t N>
auto ExtendedDynamicExpectedEffortSearch<N>::fetch_next_node() -> std::optional<SearchNode> {
	if (open_list.empty())
		return {};

	// open list buckets should not be empty
	assert(!std::begin(open_list)->second.empty());
	// the open list should never contain any closed nodes
	assert(!search_space.get_node(state_registry.lookup_state(std::begin(open_list)->second.top().second)).is_closed());
	const auto current_f_min = std::begin(open_list)->first;

	// update f_min if necessary
	if (current_f_min > f_min) {
		// f_min increased --> fix focal by copying all nodes that were previously
		// outside the bound (i.e. suboptimality factor * current_f_min) and are now
		// inside it (i.e. suboptimality_factor * f_min)
		const auto lower = open_list.upper_bound(suboptimality_factor * f_min);
		const auto upper = open_list.upper_bound(suboptimality_factor * current_f_min);
		for (auto it = lower; it != upper; ++it) {
			for (auto it2 = std::begin(it->second); it2 != std::end(it->second); ++it2) {
				auto state_id = it2->second;
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

	auto node = std::optional<SearchNode>();
	while (!node) {
		// the focal list should contain at least one (non-closed) node: the one with minimal f value from the open list
		assert(!focal_list.empty());

		auto id = focal_list.top();
		const auto s = state_registry.lookup_state(id);
		node.emplace(search_space.get_node(s));
		if (node->is_closed()) {
			node.reset();
			focal_list.pop();
			continue;
		}
		if (expected_work_error_margin < 0)
			break;
		const auto old_expected_work = focal_list.top_key()[0];
		if (old_expected_work == 0. || std::isinf(old_expected_work) || std::isnan(old_expected_work))
			break;
		auto eval_context = EvaluationContext(s, node->get_g(), false, &statistics);
		auto evaluator_values = this->compute_results(eval_context);
		const auto expected_work = evaluator_values.front();
		assert(expected_work == expected_work_evaluator->compute_result(eval_context));
		if (std::abs((expected_work - old_expected_work) / old_expected_work) > expected_work_error_margin) {
			focal_list.pop();
			focal_list.emplace(std::move(evaluator_values), std::move(id), false);
			node.reset();
		}
	}

	auto eval_context = EvaluationContext(node->get_state(), node->get_g(), false, &statistics);
	const auto expected_work = expected_work_evaluator->compute_result(eval_context);
	// the specific evaluation context does not matter for the f-hat-min evaluator -- any is fine
	const auto f_hat_min = f_hat_min_evaluator->compute_result(eval_context);

	auto number_of_nodes_for_f_value = std::unordered_map<int, int>();
	number_of_nodes_for_f_value[std::begin(open_list)->first] = std::begin(open_list)->second.size();

	auto expand_best_f = false;

	for (auto it = std::next(std::begin(open_list)); it != std::end(open_list) && it->first <= suboptimality_factor * f_hat_min; ++it) {
		const auto target_f_value = it->first;
		number_of_nodes_for_f_value[target_f_value] = it->second.size();

		const auto open_list_state_id = it->second.top().second;
		const auto open_list_state = state_registry.lookup_state(open_list_state_id);
		auto open_list_eval_context = EvaluationContext(open_list_state, node->get_g(), false, &statistics);
		const auto potential_expected_work = expected_work_evaluator->compute_result(open_list_eval_context);

		if (potential_expected_work >= expected_work)
			// the best node for this f-value is not better than the best we can do otherwise, so we don't need to consider it
			continue;

		const auto expected_required_f_min_expansions =
				h_error->get_average_heuristic_error() == 0.
						? std::numeric_limits<double>::infinity()
						: (std::accumulate(std::begin(open_list), it, 0ll,
		                                   [&number_of_nodes_for_f_value, target_f_value](const auto sum, const auto &open_list_entry) {
											   const auto f = open_list_entry.first;
											   assert(number_of_nodes_for_f_value.find(f) != std::end(number_of_nodes_for_f_value));
											   assert(f < target_f_value);
											   return sum + number_of_nodes_for_f_value.at(f) * (target_f_value - f);
										   })
		                   / h_error->get_average_heuristic_error());

		if (potential_expected_work + expected_required_f_min_expansions < expected_work) {
			// taking into account the expected number of required best-f expansions, we still believe that it will pay off to bump up f-min until this node is available in the focal list
			// ==> expand best-f instead of the top node of the focal list
			expand_best_f = true;
			break;
		}
	}

	if (expand_best_f) {
		const auto best_f_id = std::begin(open_list)->second.top().second;
		const auto best_f = state_registry.lookup_state(best_f_id);
		node.emplace(search_space.get_node(best_f));
		++best_f_expansions;
	} else {
		assert(node->get_state_id() == focal_list.top());
		focal_list.pop();
	}

	// clean up all corresponding nodes from the open list
	auto handles_it = open_list_handles.find(node->get_state_id());
	assert(handles_it != std::end(open_list_handles));
	for (auto &[f, handle] : handles_it->second) {
		auto open_list_it = open_list.find(f);
		assert(open_list_it != std::end(open_list));
		open_list_it->second.erase(handle);
		if (open_list_it->second.empty())
			open_list.erase(open_list_it);
	}
	open_list_handles.erase(handles_it);

	// we update f-hat-min after we have found a node for expansion but before closing it to ensure that the f-hat list is non-empty when updating f-hat-min
	update_f_hat_min();
	node->close();
	statistics.inc_expanded();

	expansion_delay->update_expansion_delay(statistics.get_expanded() - open_list_insertion_time[node->get_state()]);
	return node;
}

template <std::size_t N>
void ExtendedDynamicExpectedEffortSearch<N>::insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) {
	open_list_insertion_time[eval_context.get_state()] = statistics.get_expanded();
	assert(!eval_context.is_evaluator_value_infinite(f_evaluator.get()));
	const auto f = eval_context.get_evaluator_value(f_evaluator.get());
	auto open_list_it = open_list.find(f);
	if (open_list_it == std::end(open_list))
		open_list_it = open_list.emplace(f, open_list_compare).first;
	const auto handle = open_list_it->second.emplace(eval_context.get_g_value(), state_id);
	open_list_handles[state_id].emplace_front(f, handle);

	const auto f_hat = f_hat_evaluator->compute_result(eval_context);
	f_hat_list.push({f_hat}, state_id, preferred);

	if (f <= suboptimality_factor * f_min)
		focal_list.push(evaluator_values, state_id, preferred);
}

template <std::size_t N>
void ExtendedDynamicExpectedEffortSearch<N>::update_f_hat_min() {
	while (true) {
		assert(!f_hat_list.empty());
		const auto state_id = f_hat_list.top();
		const auto state = state_registry.lookup_state(state_id);
		const auto node = search_space.get_node(state);

		if (node.is_closed()) {
			f_hat_list.pop();
			continue;
		}

		auto eval_context = EvaluationContext(state, node.get_g(), false, &statistics);
		assert(!FloatingPointEvaluator::is_dead_end(f_hat_evaluator->compute_result(eval_context)));
		assert(!FloatingPointEvaluator::is_dead_end(d_hat_evaluator->compute_result(eval_context)));
		const auto f_hat_min = f_hat_evaluator->compute_result(eval_context);
		if (!std::isinf(f_hat_min)) // this may happen if we get infinite d-hat values due to an error of 1
			f_hat_min_evaluator->update(f_hat_evaluator->compute_result(eval_context), d_hat_evaluator->compute_result(eval_context));
		return;
	}
}

template <std::size_t N>
void ExtendedDynamicExpectedEffortSearch<N>::reward_progress() {
	// preferred operators are not (yet?) implemented for DXES --> nothing to do here
}

template <std::size_t N>
void ExtendedDynamicExpectedEffortSearch<N>::print_statistics() const {
	EagerSuboptimalSearch<N>::print_statistics();
	utils::g_log << "Number of best-f expansions: " << best_f_expansions << " (ratio: " << (static_cast<double>(best_f_expansions) / statistics.get_expanded())
				 << ")" << std::endl;
}

static auto _parse_edxes(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	parser.add_option<double>("suboptimality_factor", "Suboptimality factor.", "2.0", Bounds("1.0", "infinity"));
	parser.add_option<double>(
			"expected_work_error_margin",
			"Fractional error margin for the expected work. For example, using a value of 0.05 means that if the expected work upon expansion deviates by more "
			"than five percent from the expected work value at insertion, the node is reinserted with the updated value. This setting is disabled if given a "
			"negative value.",
			"-1");
	parser.add_option<bool>("admissible_h", "Indicate that the heuristic is admissible for the debiased heuristic.", "true");
	parser.add_enum_option<NancyAssumptionsSBSEvaluator::CostBoundVarianceMethod>(
			"cost_bound_variance_method", {"HEURISTIC_ERROR", "F_MIN_VARIANCE", "F_MIN_VARIANCE_TIMES_DISTANCE", "ZERO", "ZERO_IMPROVED"},
			"how to model the variance of the cost bound", "F_MIN_VARIANCE");
	parser.add_option<int>("moving_average_size", "size of the moving average to be considered by the expansion delay (0 to average over all states)", "100",
	                       Bounds("0", "infinity"));

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
	                                                 debiased_distance, heuristic_error, opts.get<bool>("admissible_h"));

	auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	auto f_evaluator = std::make_shared<sum_evaluator::SumEvaluator>(
			std::vector<std::shared_ptr<Evaluator>>{g_evaluator, opts.get<std::shared_ptr<Evaluator>>("heuristic")});
	auto f_hat_evaluator = std::make_shared<SumEvaluator>(
			std::vector<std::shared_ptr<FloatingPointEvaluator>>{std::make_shared<FloatingPointEvaluatorWrapper>(g_evaluator), debiased_heuristic});

	const auto cost_bound_variance_method = opts.get<NancyAssumptionsSBSEvaluator::CostBoundVarianceMethod>("cost_bound_variance_method");

	auto f_hat_min_opts = options::Options();
	f_hat_min_opts.set("cache_estimates", false);
	auto f_hat_min_evaluator = std::make_shared<FHatMinEvaluator>(
			f_hat_min_opts, cost_bound_variance_method == NancyAssumptionsSBSEvaluator::CostBoundVarianceMethod::F_MIN_VARIANCE
									|| cost_bound_variance_method == NancyAssumptionsSBSEvaluator::CostBoundVarianceMethod::F_MIN_VARIANCE_TIMES_DISTANCE);

	auto nancy_assumptions_opts = options::Options();
	nancy_assumptions_opts.set("cache_estimates", false);
	nancy_assumptions_opts.set("suboptimality_factor", opts.get<double>("suboptimality_factor"));
	nancy_assumptions_opts.set<std::shared_ptr<Evaluator>>("f", f_evaluator);
	nancy_assumptions_opts.set<std::shared_ptr<FloatingPointEvaluator>>("d_hat", debiased_distance);
	nancy_assumptions_opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat", f_hat_evaluator);
	nancy_assumptions_opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat_min", f_hat_min_evaluator);
	nancy_assumptions_opts.set<std::shared_ptr<heuristic_error::HeuristicError>>("heuristic_error", heuristic_error);
	nancy_assumptions_opts.set("use_online_variance", opts.get<bool>("use_online_variance"));
	nancy_assumptions_opts.set("admissible_h", opts.get<bool>("admissible_h"));
	nancy_assumptions_opts.set<NancyAssumptionsSBSEvaluator::CostBoundVarianceMethod>("cost_bound_variance_method", cost_bound_variance_method);
	auto nancy_assumptions_evaluator = std::make_shared<NancyAssumptionsSBSEvaluator>(nancy_assumptions_opts);

	auto expansion_delay = std::make_shared<ExpansionDelay>(opts.get<int>("moving_average_size"));
	auto expansion_delay_evaluator_opts = options::Options();
	expansion_delay_evaluator_opts.set("cache_estimates", false);
	auto expansion_delay_evaluator = std::make_shared<ExpansionDelayEvaluator>(expansion_delay_evaluator_opts, *expansion_delay);
	auto remaining_expansions = std::make_shared<RemainingExpansionsEvaluator>(debiased_distance, expansion_delay_evaluator);

	auto expected_work_evaluator = std::make_shared<DivisionEvaluator>(remaining_expansions, nancy_assumptions_evaluator);

	auto heuristic_error_observers = opts.get_list<std::shared_ptr<heuristic_error::HeuristicError>>("error");
	heuristic_error_observers.push_back(distance_error);
	heuristic_error_observers.push_back(heuristic_error);
	opts.set("error", std::move(heuristic_error_observers));

	opts.set("h_error", heuristic_error);

	opts.set<std::shared_ptr<FloatingPointEvaluator>>("expected_work_evaluator", expected_work_evaluator);
	opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat_evaluator", f_hat_evaluator);
	opts.set<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator", debiased_distance);
	opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat_min_evaluator", f_hat_min_evaluator);

	if (opts.get<bool>("enable_tie_breaking")) {
		auto d_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(opts.get<std::shared_ptr<Evaluator>>("distance"));
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {expected_work_evaluator, f_hat_evaluator, d_evaluator});
		return std::make_shared<ExtendedDynamicExpectedEffortSearch<3>>(opts, expansion_delay);
	} else {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {expected_work_evaluator});
		return std::make_shared<ExtendedDynamicExpectedEffortSearch<1>>(opts, expansion_delay);
	}
}

static options::Plugin<SearchEngine> _plugin_dxes("edxes", _parse_edxes);

template class ExtendedDynamicExpectedEffortSearch<1>;
template class ExtendedDynamicExpectedEffortSearch<3>;
} // namespace bounded_suboptimal_search
