#include "dynamic_potential_search.h"

#include <cassert>

#include "../evaluation_context.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../suboptimal_search/util.h"
#include "../task_utils/successor_generator.h"

using namespace suboptimal_search;

namespace bounded_suboptimal_search {
DynamicPotentialSearch::DynamicPotentialSearch(const Options &opts)
	: SearchEngine(opts),
	  reopen_closed_nodes(opts.get<bool>("reopen_closed")),
	  suboptimality_factor(opts.get<double>("suboptimality_factor")),
	  compare_potential(suboptimality_factor),
	  heuristic(opts.get<std::shared_ptr<Evaluator>>("heuristic")),
	  f_evaluator(std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{std::make_shared<g_evaluator::GEvaluator>(), heuristic})) {
	if (opts.contains("preferred") && !opts.get_list<std::shared_ptr<Evaluator>>("preferred").empty()) {
		std::cerr << "Dynamic potential search currently does not support preferred operators, exiting." << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
	}
}

auto DynamicPotentialSearch::fetch_next_node() -> std::optional<SearchNode> {
	auto node = std::optional<SearchNode>();
	while (true) {
		if (open_list.empty())
			return {};

		// reorder open list if necessary
		assert(!f_counts.empty());
		const auto f_min = std::begin(f_counts)->first;
		if (f_min != compare_potential.get_f_min()) {
			compare_potential.set_f_min(f_min);
			std::make_heap(std::begin(open_list), std::end(open_list), compare_potential);
		}

		assert(!open_list.front().second->empty());
		auto &value_set = *open_list.front().second;
		const auto id = value_set.back();
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

void DynamicPotentialSearch::insert(EvaluationContext &eval_context) {
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
	value_set_it->second->push_back(eval_context.get_state().get_id());
}

void DynamicPotentialSearch::initialize() {
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

void DynamicPotentialSearch::print_statistics() const {
	statistics.print_detailed_statistics();
	search_space.print_statistics();
}

auto DynamicPotentialSearch::step() -> SearchStatus {
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
	}
	return IN_PROGRESS;
}

void DynamicPotentialSearch::reward_progress() {
	// preferred operators are not (yet?) implemented for DPS --> nothing to do here
}

static auto _parse_dps(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<double>("suboptimality_factor", "Suboptimality factor.", "2.0", Bounds("1.0", "infinity"));
	parser.add_option<bool>("reopen_closed", "reopen closed nodes", "true");

	SearchEngine::add_options_to_parser(parser);

	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;

	return std::make_shared<DynamicPotentialSearch>(opts);
}

static options::Plugin<SearchEngine> _plugin_dps("dps", _parse_dps);
} // namespace bounded_suboptimal_search
