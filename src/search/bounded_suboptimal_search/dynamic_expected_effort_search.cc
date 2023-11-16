#include "dynamic_expected_effort_search.h"

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
#include "suboptimality_bound_assumptions_nancy_evaluator.h"
#include "../floating_point_evaluator/fp_division_evaluator.h"

using namespace floating_point_evaluator;
using namespace floating_point_open_list;
using namespace suboptimal_search;

namespace bounded_suboptimal_search {
template <std::size_t N>
DynamicExpectedEffortSearch<N>::DynamicExpectedEffortSearch(const Options &opts)
	: EagerSuboptimalSearch<N>(opts),
	  reopen_closed_nodes(opts.get<bool>("reopen_closed")),
	  suboptimality_factor(opts.get<double>("suboptimality_factor")),
	  expected_work_error_margin(opts.get<double>("expected_work_error_margin")),
	  alternation_mode(static_cast<AlternationMode>(opts.get<int>("alternation_mode"))),
	  f_min(0),
	  f_evaluator(std::make_shared<sum_evaluator::SumEvaluator>(
			  std::vector<std::shared_ptr<Evaluator>>{std::make_shared<g_evaluator::GEvaluator>(), opts.get<std::shared_ptr<Evaluator>>("heuristic")})),
	  f_hat_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("f_hat_evaluator")),
	  d_hat_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator")),
	  f_hat_min_evaluator(std::static_pointer_cast<FHatMinEvaluator>(opts.get<std::shared_ptr<FloatingPointEvaluator>>("f_hat_min_evaluator"))) {
	if (opts.contains("preferred") && !opts.get_list<std::shared_ptr<Evaluator>>("preferred").empty()) {
		std::cerr << "Dynamic expected effort search currently does not support preferred operators, exiting." << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
	}
}

template <std::size_t N>
void DynamicExpectedEffortSearch<N>::initialize_extra(EvaluationContext &eval_context) {
	const auto f_hat = f_hat_evaluator->compute_result(eval_context);
	const auto d_hat = d_hat_evaluator->compute_result(eval_context);
	if (!FloatingPointEvaluator::is_dead_end(f_hat) && !FloatingPointEvaluator::is_dead_end(d_hat))
		f_hat_min_evaluator->update(f_hat, d_hat);
}

template <std::size_t N>
auto DynamicExpectedEffortSearch<N>::fetch_next_node() -> std::optional<SearchNode> {
	// remove closed nodes from the front of the open list
	while (true) {
		if (open_list.empty())
			return {};
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

	// update f_min if necessary
	if (current_f_min > f_min) {
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
				if (alternation_mode == AlternationMode::F_HAT || alternation_mode == AlternationMode::BOTH) {
					// we are alternating with f-hat, so the f-hat queue must also adhere to the suboptimality bound
					const auto f_hat = f_hat_evaluator->compute_result(eval_context);
					f_hat_list.push({f_hat}, state_id, eval_context.is_preferred());
				}
			}
		}
		f_min = current_f_min;
	}

	auto node = std::optional<SearchNode>();
	while (true) {
		// the focal list should contain at least one (non-closed) node: the one with minimal f value from the open list
		assert(!focal_list.empty());

		auto id = StateID::no_state;
		auto focal_key = std::optional<double>();
		switch (get_active_queue()) {
		case Queue::DXES:
			id = focal_list.top();
			if (expected_work_error_margin >= 0)
				focal_key = focal_list.top_key()[0];
			focal_list.pop();
			break;
		case Queue::F_HAT:
			id = f_hat_list.top();
			f_hat_list.pop();
			break;
		case Queue::F:
			id = std::begin(open_list)->second.front().second;
			assert(!search_space.get_node(state_registry.lookup_state(id)).is_closed());
			// the open list will be cleaned up in the next call of fetch_next_state
			break;
		}
		const auto s = state_registry.lookup_state(id);
		node.emplace(search_space.get_node(s));
		if (node->is_closed())
			continue;

		if (get_active_queue() == Queue::DXES && expected_work_error_margin >= 0) {
			// check if the current expected work value is sufficiently close to the one at insertion time
			assert(focal_key);
			const auto old_expected_work = *focal_key;
			if (!(old_expected_work == 0. || std::isinf(old_expected_work) || std::isnan(old_expected_work))) {
				auto eval_context = EvaluationContext(s, node->get_g(), false, &statistics);
				auto evaluator_values = this->compute_results(eval_context);
				const auto expected_work = evaluator_values.front();
				if (std::abs((expected_work - old_expected_work) / old_expected_work) > expected_work_error_margin) {
					// the difference in expected work values is too large ==> re-insert the node and continue
					focal_list.emplace(std::move(evaluator_values), std::move(id), false);
					continue;
				}
			}
		}

		// we update f-hat-min after we have found a node for expansion but before closing it to ensure that the f-hat list is non-empty when updating f-hat-min
		update_f_hat_min();
		node->close();
		assert(!node->is_dead_end());
		statistics.inc_expanded();
		break;
	}
	return node;
}

template <std::size_t N>
void DynamicExpectedEffortSearch<N>::insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) {
	assert(!eval_context.is_evaluator_value_infinite(f_evaluator.get()));
	const auto f = eval_context.get_evaluator_value(f_evaluator.get());
	open_list[f].emplace_back(eval_context.get_g_value(), state_id);
	std::push_heap(std::begin(open_list[f]), std::end(open_list[f]), open_list_compare);

	const auto check_bound_for_f_hat = alternation_mode == AlternationMode::F_HAT || alternation_mode == AlternationMode::BOTH;
	if (!check_bound_for_f_hat) {
		const auto f_hat = f_hat_evaluator->compute_result(eval_context);
		f_hat_list.push({f_hat}, state_id, preferred);
	}
	if (f <= suboptimality_factor * f_min) {
		focal_list.push(evaluator_values, state_id, preferred);
		if (check_bound_for_f_hat) {
			const auto f_hat = f_hat_evaluator->compute_result(eval_context);
			f_hat_list.push({f_hat}, state_id, preferred);
		}
	}
}

template <std::size_t N>
auto DynamicExpectedEffortSearch<N>::get_active_queue() const->Queue {
	switch (alternation_mode) {
	case AlternationMode::NONE:
		return Queue::DXES;
	case AlternationMode::F:
		return statistics.get_expanded() % 2 == 0 ? Queue::DXES : Queue::F;
	case AlternationMode::F_HAT:
		return statistics.get_expanded() % 2 == 0 ? Queue::DXES : Queue::F_HAT;
	case AlternationMode::BOTH:
		return statistics.get_expanded() % 3 == 0 ? Queue::DXES : statistics.get_expanded() % 3 == 1 ? Queue::F_HAT : Queue::F;
	}
	utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
}

template <std::size_t N>
void DynamicExpectedEffortSearch<N>::update_f_hat_min() {
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
void DynamicExpectedEffortSearch<N>::reward_progress() {
	// preferred operators are not (yet?) implemented for DXES --> nothing to do here
}

static auto _parse_dxes(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	parser.add_option<double>("suboptimality_factor", "Suboptimality factor.", "2.0", Bounds("1.0", "infinity"));
	parser.add_option<double>(
			"expected_work_error_margin",
			"Fractional error margin for the expected work. For example, using a value of 0.05 means that if the expected work upon expansion deviates by more "
			"than five percent from the expected work value at insertion, the node is reinserted with the updated value. This setting is disabled if given a "
			"negative value.",
			"-1");
	// using an int enum option for simplicity
	parser.add_enum_option<int>("alternation_mode", {"NONE", "F", "F_HAT", "BOTH"}, "alternate DXES expansions with f/f-hat expansions", "NONE");
	parser.add_option<bool>("admissible_h", "Indicate that the heuristic is admissible for the debiased heuristic.", "true");
	parser.add_enum_option<NancyAssumptionsSBSEvaluator::CostBoundVarianceMethod>(
			"cost_bound_variance_method", {"HEURISTIC_ERROR", "F_MIN_VARIANCE", "F_MIN_VARIANCE_TIMES_DISTANCE", "ZERO", "ZERO_IMPROVED"},
			"how to model the variance of the cost bound", "F_MIN_VARIANCE");

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

	auto expected_work = std::make_shared<DivisionEvaluator>(debiased_distance, nancy_assumptions_evaluator);

	auto heuristic_error_observers = opts.get_list<std::shared_ptr<heuristic_error::HeuristicError>>("error");
	heuristic_error_observers.push_back(distance_error);
	heuristic_error_observers.push_back(heuristic_error);
	opts.set("error", std::move(heuristic_error_observers));

	opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat_evaluator", f_hat_evaluator);
	opts.set<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator", debiased_distance);
	opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat_min_evaluator", f_hat_min_evaluator);

	if (opts.get<bool>("enable_tie_breaking")) {
		auto d_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(opts.get<std::shared_ptr<Evaluator>>("distance"));
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {expected_work, f_hat_evaluator, d_evaluator});
		return std::make_shared<DynamicExpectedEffortSearch<3>>(opts);
	} else {
		opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {expected_work});
		return std::make_shared<DynamicExpectedEffortSearch<1>>(opts);
	}
}

static options::Plugin<SearchEngine> _plugin_dxes("dxes", _parse_dxes);


template class DynamicExpectedEffortSearch<1>;
template class DynamicExpectedEffortSearch<3>;
} // namespace bounded_suboptimal_search
