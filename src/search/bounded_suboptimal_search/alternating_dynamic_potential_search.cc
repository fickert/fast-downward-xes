#include "alternating_dynamic_potential_search.h"

#include <array>
#include <cassert>

#include "../evaluation_context.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "../heuristic_error/heuristic_error.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../suboptimal_search/util.h"
#include "../task_utils/successor_generator.h"
#include "../heuristic_error/one_step_distance_error.h"
#include "../heuristic_error/debiased_distance.h"
#include "../floating_point_evaluator/floating_point_evaluator_wrapper.h"
#include "../floating_point_evaluator/fp_sum_evaluator.h"

using namespace suboptimal_search;

namespace bounded_suboptimal_search {
AlternatingDynamicPotentialSearch::AlternatingDynamicPotentialSearch(const Options &opts)
	: SearchEngine(opts),
	  reopen_closed_nodes(opts.get<bool>("reopen_closed")),
	  suboptimality_factor(opts.get<double>("suboptimality_factor")),
	  f_min(0),
	  compare_potential(suboptimality_factor),
	  heuristic(opts.get<std::shared_ptr<Evaluator>>("heuristic")),
	  f_evaluator(std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{std::make_shared<g_evaluator::GEvaluator>(), heuristic})),
	  f_hat_evaluator(opts.get<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>>("f_hat_evaluator")),
	  heuristic_error(opts.get_list<std::shared_ptr<heuristic_error::HeuristicError>>("error")) {
	if (opts.contains("preferred") && !opts.get_list<std::shared_ptr<Evaluator>>("preferred").empty()) {
		std::cerr << "Alternating dynamic potential search currently does not support preferred operators, exiting." << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
	}
	for (const auto &h_error : heuristic_error) {
		h_error->initialize(state_registry);
		h_error->notify_initial_state();
	}
}

auto AlternatingDynamicPotentialSearch::fetch_next_node() -> std::optional<SearchNode> {
	// remove closed nodes from the front of the open list
	while (true) {
		if (f_list.empty())
			return {};
		auto &state_ids = std::begin(f_list)->second;
		const auto id = state_ids.front().second;
		const auto s = state_registry.lookup_state(id);
		const auto node = search_space.get_node(s);
		if (!node.is_closed())
			break;
		std::pop_heap(std::begin(state_ids), std::end(state_ids), f_list_compare);
		state_ids.pop_back();
		if (state_ids.empty())
			f_list.erase(std::begin(f_list));
	}

	assert(!f_list.empty());
	assert(!std::begin(f_list)->second.empty());
	const auto current_f_min = std::begin(f_list)->first;

	// update f_min if necessary
	if (current_f_min > f_min) {
		// f_min increased --> fix f-hat list by copying all nodes that were previously
		// outside the bound (i.e. suboptimality factor * current_f_min) and are now
		// inside it (i.e. suboptimality_factor * f_min)
		const auto lower = f_list.upper_bound(suboptimality_factor * f_min);
		const auto upper = f_list.upper_bound(suboptimality_factor * current_f_min);
		for (auto it = lower; it != upper; ++it) {
			for (auto [_, state_id] : it->second) {
				const auto state = state_registry.lookup_state(state_id);
				const auto node = search_space.get_node(state);
				if (node.is_closed())
					continue;
				auto eval_context = EvaluationContext(state, node.get_g(), false, &statistics);
				const auto f_hat = f_hat_evaluator->compute_result(eval_context);
				f_hat_list.push({f_hat}, state_id, eval_context.is_preferred());
			}
		}
		f_min = current_f_min;
	}

	auto node = std::optional<SearchNode>();
	while (true) {
		if (open_list.empty())
			return {};

		auto id = StateID::no_state;

		switch (statistics.get_expanded() % 3) {
		case 0: {
			// expand by DPS ordering
			// reorder open list if necessary
			assert(!f_counts.empty());
			const auto f_min2 = std::begin(f_counts)->first;
			if (f_min2 != compare_potential.get_f_min()) {
				compare_potential.set_f_min(f_min2);
				std::make_heap(std::begin(open_list), std::end(open_list), compare_potential);
			}

			assert(!open_list.front().second->empty());
			auto &value_set = *open_list.front().second;
			id = value_set.back();
			value_set.pop_back();

			if (value_set.empty()) {
				// removed the last state id for this (g, h) pair
				const auto key = open_list.front().first;
				open_list_value_sets.erase(key);
				std::pop_heap(std::begin(open_list), std::end(open_list), compare_potential);
				open_list.pop_back();
				const auto f_value = key.first + key.second;
				if (--f_counts[f_value] == 0)
					f_counts.erase(f_value);
			}
			break;
		}
		case 1: {
			assert(!f_hat_list.empty());
			id = f_hat_list.top();
			f_hat_list.pop();
			break;
		}
		case 2: {
			auto &state_ids = std::begin(f_list)->second;
			id = state_ids.front().second;
			std::pop_heap(std::begin(state_ids), std::end(state_ids), f_list_compare);
			state_ids.pop_back();
			if (state_ids.empty())
				f_list.erase(std::begin(f_list));
			break;
		}
		default:
			utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
		}

		assert(id != StateID::no_state);

		// TODO is there a way we can avoid creating the state here and then
		//      recreate it outside of this function with node.get_state()?
		//      One way would be to store GlobalState objects inside SearchNodes
		//      instead of StateIDs
		auto s = state_registry.lookup_state(id);
		node.emplace(search_space.get_node(s));

		if (node->is_closed())
			continue;

		node->close();
		assert(!node->is_dead_end());
		statistics.inc_expanded();
		break;
	}
	return node;
}

void AlternatingDynamicPotentialSearch::insert(EvaluationContext &eval_context) {
	assert(!eval_context.is_evaluator_value_infinite(heuristic.get()));
	const auto key = std::pair(eval_context.get_g_value(), eval_context.get_evaluator_value(heuristic.get()));
	auto value_set_it = open_list_value_sets.find(key);
	if (value_set_it == std::end(open_list_value_sets)) {
		auto value_set = std::make_shared<OpenListValueSet::element_type>();
		value_set_it = open_list_value_sets.emplace(key, value_set).first;
		open_list.emplace_back(key, value_set);
		std::push_heap(std::begin(open_list), std::end(open_list), compare_potential);
		++f_counts[key.first + key.second];
	}

	const auto state_id = eval_context.get_state().get_id();
	value_set_it->second->push_back(state_id);
	const auto f = static_cast<double>(eval_context.get_evaluator_value(f_evaluator.get()));
	f_list[f].emplace_back(eval_context.get_g_value(), state_id);
	std::push_heap(std::begin(f_list[f]), std::end(f_list[f]), f_list_compare);

	if (f <= suboptimality_factor * f_min) {
		const auto f_hat = f_hat_evaluator->compute_result(eval_context);
		f_hat_list.push({f_hat}, state_id, eval_context.is_preferred());
	}
}

void AlternatingDynamicPotentialSearch::initialize() {
	utils::g_log << "Conducting best first search" << (reopen_closed_nodes ? " with" : " without") << " reopening closed nodes, (real) bound = " << bound << std::endl;

	const auto &initial_state = state_registry.get_initial_state();

	/*
	  Note: we consider the initial state as reached by a preferred
	  operator.
	*/
	auto eval_context = EvaluationContext(initial_state, 0, true, &statistics);
	statistics.inc_evaluated_states();
	if (eval_context.is_evaluator_value_infinite(heuristic.get())) {
		utils::g_log << "Initial state is a dead end." << std::endl;
	} else {
		if (search_progress.check_progress(eval_context))
			statistics.print_checkpoint_line(0);
		auto node = search_space.get_node(initial_state);
		node.open_initial();

		insert(eval_context);
	}

	print_initial_evaluator_values(eval_context);
}

void AlternatingDynamicPotentialSearch::print_statistics() const {
	statistics.print_detailed_statistics();
	search_space.print_statistics();
}

auto AlternatingDynamicPotentialSearch::step() -> SearchStatus {
	auto node = fetch_next_node();
	if (!node) {
		utils::g_log << "Completely explored state space -- no solution!" << std::endl;
		return FAILED;
	}

	auto s = node->get_state();
	if (check_goal_and_set_plan(s))
		return SOLVED;

	auto applicable_ops = std::vector<OperatorID>();
	successor_generator.generate_applicable_ops(s, applicable_ops);

	// This evaluates the expanded state (again) to get preferred ops
	auto eval_context = EvaluationContext(s, node->get_g(), false, &statistics, true);

	for (const auto &h_error : heuristic_error)
		h_error->set_expanding_state(s);

	for (const auto op_id : applicable_ops) {
		const auto op = task_proxy.get_operators()[op_id];
		// We need to use > instead of >= here!
		if ((node->get_real_g() + op.get_cost()) > bound)
			continue;

		const auto succ_state = state_registry.get_successor_state(s, op);
		statistics.inc_generated();
		const auto is_preferred = false;

		auto succ_node = search_space.get_node(succ_state);

		// Previously encountered dead end. Don't re-evaluate.
		if (succ_node.is_dead_end())
			continue;

		if (succ_node.is_new()) {
			// We have not seen this state before.
			// Evaluate and create a new node.

			// Careful: succ_node.get_g() is not available here yet,
			// hence the stupid computation of succ_g.
			// TODO: Make this less fragile.
			const auto succ_g = node->get_g() + get_adjusted_cost(op);

			auto succ_eval_context = EvaluationContext(succ_state, succ_g, is_preferred, &statistics);
			statistics.inc_evaluated_states();

			if (succ_eval_context.is_evaluator_value_infinite(heuristic.get())) {
				succ_node.mark_as_dead_end();
				statistics.inc_dead_ends();
				continue;
			}
			succ_node.open(*node, op, get_adjusted_cost(op));

			insert(succ_eval_context);
			if (search_progress.check_progress(succ_eval_context)) {
				statistics.print_checkpoint_line(succ_node.get_g());
				reward_progress();
			}
		} else if (succ_node.get_g() > node->get_g() + get_adjusted_cost(op)) {
			// We found a new cheapest path to an open or closed state.
			if (reopen_closed_nodes) {
				if (succ_node.is_closed()) {
					/*
					  TODO: It would be nice if we had a way to test
					  that reopening is expected behaviour, i.e., exit
					  with an error when this is something where
					  reopening should not occur (e.g. A* with a
					  consistent heuristic).
					*/
					statistics.inc_reopened();
				}
				succ_node.reopen(*node, op, get_adjusted_cost(op));

				auto succ_eval_context = EvaluationContext(succ_state, succ_node.get_g(), is_preferred, &statistics);

				/*
				  Note: our old code used to retrieve the h value from
				  the search node here. Our new code recomputes it as
				  necessary, thus avoiding the incredible ugliness of
				  the old "set_evaluator_value" approach, which also
				  did not generalize properly to settings with more
				  than one evaluator.

				  Reopening should not happen all that frequently, so
				  the performance impact of this is hopefully not that
				  large. In the medium term, we want the evaluators to
				  remember evaluator values for states themselves if
				  desired by the user, so that such recomputations
				  will just involve a look-up by the Evaluator object
				  rather than a recomputation of the evaluator value
				  from scratch.
				*/
				insert(succ_eval_context);
			} else {
				// If we do not reopen closed nodes, we just update the parent pointers.
				// Note that this could cause an incompatibility between
				// the g-value and the actual path that is traced back.
				succ_node.update_parent(*node, op, get_adjusted_cost(op));
			}
		}

		for (const auto &h_error : heuristic_error)
			h_error->add_successor(succ_node, get_adjusted_cost(op));
	}

	for (const auto &h_error : heuristic_error)
		h_error->update_error();
	return IN_PROGRESS;
}

void AlternatingDynamicPotentialSearch::reward_progress() {
	// preferred operators are not (yet?) implemented for DPS --> nothing to do here
}

static auto _parse_alternating_dps(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance");
	parser.add_option<double>("suboptimality_factor", "Suboptimality factor.", "2.0", Bounds("1.0", "infinity"));
	parser.add_option<bool>("admissible_h", "Indicate that the heuristic is admissible for the debiased heuristic.", "true");
	parser.add_option<bool>("reopen_closed", "reopen closed nodes", "true");
	add_warm_start_options(parser);
	add_percentage_based_error_option(parser);

	SearchEngine::add_options_to_parser(parser);

	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;

	using g_evaluator::GEvaluator;
	using floating_point_evaluator::SumEvaluator;
	using floating_point_evaluator::FloatingPointEvaluator;
	using floating_point_evaluator::FloatingPointEvaluatorWrapper;

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

	auto g_evaluator = std::make_shared<GEvaluator>();
	auto f_hat_evaluator = std::make_shared<SumEvaluator>(
			std::vector<std::shared_ptr<FloatingPointEvaluator>>{std::make_shared<FloatingPointEvaluatorWrapper>(g_evaluator), debiased_heuristic});

	auto heuristic_error_observers = std::vector<std::shared_ptr<heuristic_error::HeuristicError>>();
	heuristic_error_observers.push_back(heuristic_error);
	opts.set("error", std::move(heuristic_error_observers));

	opts.set<std::shared_ptr<FloatingPointEvaluator>>("f_hat_evaluator", f_hat_evaluator);

	return std::make_shared<AlternatingDynamicPotentialSearch>(opts);
}

static options::Plugin<SearchEngine> _plugin_dps("alt_dps", _parse_alternating_dps);
} // namespace bounded_suboptimal_search
