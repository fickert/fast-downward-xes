#include "eager_bounded_cost_search.h"

#include <cassert>
#include <memory>
#include <optional.hh>

#include "../algorithms/ordered_set.h"
#include "../evaluation_context.h"
#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "../floating_point_open_list/alternation_open_list.h"
#include "../floating_point_open_list/best_first_open_list.h"
#include "../floating_point_open_list/floating_point_open_list.h"
#include "../heuristic_error/heuristic_error.h"
#include "../option_parser.h"
#include "../pruning_method.h"
#include "../task_utils/successor_generator.h"
#include "../utils/logging.h"

using namespace floating_point_evaluator;
using namespace floating_point_open_list;

namespace bounded_cost_search {
template <std::size_t N>
const typename EagerBoundedCostSearch<N>::OpenListCreationFunction EagerBoundedCostSearch<N>::create_best_first_open_list = []() {
	return std::make_unique<floating_point_open_list::BestFirstOpenList<N, StateID>>();
};

template <std::size_t N>
auto EagerBoundedCostSearch<N>::create_open_list(const options::Options &opts, OpenListCreationFunction create_default_open_list)
		-> std::unique_ptr<OpenListType> {
	auto preferred_operator_evaluators = opts.get_list<std::shared_ptr<Evaluator>>("preferred");
	if (preferred_operator_evaluators.empty())
		return create_default_open_list();
	auto sublists = std::vector<std::unique_ptr<OpenListType>>();
	sublists.emplace_back(create_default_open_list());
	auto preferred_sublists = std::vector<std::unique_ptr<OpenListType>>();
	preferred_sublists.reserve(preferred_operator_evaluators.size());
	for (auto i = 0u; i < preferred_operator_evaluators.size(); ++i)
		preferred_sublists.emplace_back(create_default_open_list());
	return std::make_unique<AlternationOpenList<N, StateID>>(opts.get<int>("boost"), std::move(sublists), std::move(preferred_sublists));
}

template <std::size_t N>
EagerBoundedCostSearch<N>::EagerBoundedCostSearch(const Options &opts) : EagerBoundedCostSearch(opts, create_best_first_open_list) {}

template <std::size_t N>
static auto get_evaluators(const Options &opts) -> std::array<std::shared_ptr<FloatingPointEvaluator>, N> {
	auto evaluators = std::array<std::shared_ptr<FloatingPointEvaluator>, N>();
	auto evaluators_vector = opts.get_list<std::shared_ptr<FloatingPointEvaluator>>("evals");
	assert(evaluators_vector.size() == N);
	std::copy(std::begin(evaluators_vector), std::end(evaluators_vector), std::begin(evaluators));
	return evaluators;
}

template <std::size_t N>
EagerBoundedCostSearch<N>::EagerBoundedCostSearch(const Options &opts, OpenListCreationFunction create_default_open_list)
	: SearchEngine(opts),
	  reopen_closed_nodes(opts.get<bool>("reopen_closed")),
	  initialize_error_with_cost_bound(opts.get<bool>("initialize_error_with_cost_bound")),
	  evaluators(get_evaluators<N>(opts)),
	  preferred_operator_evaluators(opts.get_list<std::shared_ptr<Evaluator>>("preferred")),
	  distance_evaluator(opts.get<std::shared_ptr<Evaluator>>("distance", nullptr)),
	  pruning_method(opts.get<std::shared_ptr<PruningMethod>>("pruning")),
	  heuristic_error(opts.get_list<std::shared_ptr<heuristic_error::HeuristicError>>("error")),
	  best_evaluator_value(std::numeric_limits<double>::infinity()),
	  open_list(create_open_list(opts, create_default_open_list)) {
	for (const auto &h_error : heuristic_error)
		h_error->initialize(state_registry);
}

template <std::size_t N>
void EagerBoundedCostSearch<N>::initialize() {
	utils::g_log << "Conducting best first search" << (reopen_closed_nodes ? " with" : " without") << " reopening closed nodes, (real) bound = " << bound << std::endl;
	assert(open_list);

	const auto &initial_state = state_registry.get_initial_state();

	/*
	  Note: we consider the initial state as reached by a preferred
	  operator.
	*/
	auto eval_context = EvaluationContext(initial_state, 0, true, &statistics);
	// let the heuristic error perform optional initialization before evaluating the main evaluator
	for (const auto &h_error : heuristic_error) {
		if (initialize_error_with_cost_bound) {
			assert(distance_evaluator);
			const auto distance = eval_context.get_evaluator_value_or_infinity(distance_evaluator.get());
			h_error->notify_initial_state(eval_context, bound, distance);
		} else {
			h_error->notify_initial_state();
		}
	}
	const auto initial_values = compute_results(eval_context);
	statistics.inc_evaluated_states();
	if (is_dead_end(initial_values)) {
		utils::g_log << "Initial state is a dead end." << std::endl;
	} else {
		if (check_progress(initial_values.front()))
			statistics.print_checkpoint_line(0);
		auto node = search_space.get_node(initial_state);
		node.open_initial();

		insert(eval_context, initial_values, initial_state.get_id(), eval_context.is_preferred());
	}

	print_initial_evaluator_values(eval_context);

	pruning_method->initialize(task);
}

template <std::size_t N>
void EagerBoundedCostSearch<N>::print_statistics() const {
	statistics.print_detailed_statistics();
	search_space.print_statistics();
	pruning_method->print_statistics();
}

template <std::size_t N>
auto EagerBoundedCostSearch<N>::step() -> SearchStatus {
	auto node = tl::optional<SearchNode>();
	while (true) {
		if (open_list->empty()) {
			utils::g_log << "Completely explored state space -- no solution!" << std::endl;
			return FAILED;
		}
		auto id = open_list->top();
		open_list->pop();
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

	auto s = node->get_state();
	if (check_goal_and_set_plan(s))
		return SOLVED;

	auto applicable_ops = std::vector<OperatorID>();
	successor_generator.generate_applicable_ops(s, applicable_ops);

	/*
	  TODO: When preferred operators are in use, a preferred operator will be
	  considered by the preferred operator queues even when it is pruned.
	*/
	pruning_method->prune_operators(s, applicable_ops);

	// This evaluates the expanded state (again) to get preferred ops
	auto eval_context = EvaluationContext(s, node->get_g(), false, &statistics, true);
	auto preferred_operators = ordered_set::OrderedSet<OperatorID>();
	for (const auto &preferred_operator_evaluator : preferred_operator_evaluators)
		if (!eval_context.is_evaluator_value_infinite(preferred_operator_evaluator.get()))
			for (const auto &op_id : eval_context.get_preferred_operators(preferred_operator_evaluator.get()))
				preferred_operators.insert(op_id);
	for (const auto &h_error : heuristic_error)
		h_error->set_expanding_state(s);

	for (const auto op_id : applicable_ops) {
		const auto op = task_proxy.get_operators()[op_id];
		// We need to use > instead of >= here!
		if ((node->get_real_g() + op.get_cost()) > bound)
			continue;

		const auto succ_state = state_registry.get_successor_state(s, op);
		statistics.inc_generated();
		const auto is_preferred = preferred_operators.contains(op_id);

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
			const auto evaluator_values = compute_results(succ_eval_context);
			statistics.inc_evaluated_states();

			if (is_dead_end(evaluator_values)) {
				succ_node.mark_as_dead_end();
				statistics.inc_dead_ends();
				continue;
			}
			succ_node.open(*node, op, get_adjusted_cost(op));

			// NOTE: we put nodes into the open list even if their main evaluator evaluates to infinity because we can't rule our rounding errors
			const auto main_evaluator_value = evaluator_values.front();
			insert(succ_eval_context, evaluator_values, succ_state.get_id(), is_preferred);
			if (check_progress(main_evaluator_value)) {
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
				const auto evaluator_values = compute_results(succ_eval_context);

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
				insert(succ_eval_context, evaluator_values, succ_state.get_id(), is_preferred);
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

template <std::size_t N>
auto EagerBoundedCostSearch<N>::compute_results(EvaluationContext &eval_context) -> EvaluatorValues {
	auto values = EvaluatorValues();
	for (auto i = 0u; i < N; ++i)
		values[i] = evaluators[i]->compute_result(eval_context);
	return values;
}

template <std::size_t N>
auto EagerBoundedCostSearch<N>::is_dead_end(const EvaluatorValues &values) -> bool {
	return std::any_of(std::begin(values), std::end(values), [](const auto value) { return FloatingPointEvaluator::is_dead_end(value); });
}

template <std::size_t N>
void EagerBoundedCostSearch<N>::insert(EvaluationContext &, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) {
	open_list->push(evaluator_values, state_id, preferred);
}

template <std::size_t N>
void EagerBoundedCostSearch<N>::reward_progress() {
	// Boost the "preferred operator" open lists somewhat whenever
	// one of the heuristics finds a state with a new best h value.
	open_list->boost_preferred();
}

template <std::size_t N>
auto EagerBoundedCostSearch<N>::check_progress(double evaluator_value) -> bool {
	if (evaluator_value < best_evaluator_value) {
		best_evaluator_value = evaluator_value;
		return true;
	}
	return false;
}

template <std::size_t N>
void EagerBoundedCostSearch<N>::dump_search_space() const {
	search_space.dump(task_proxy);
}

void add_options_to_parser(OptionParser &parser) {
	parser.add_list_option<std::shared_ptr<Evaluator>>("preferred", "use preferred operators of these evaluators", "[]");
	parser.add_list_option<std::shared_ptr<heuristic_error::HeuristicError>>("error", "Heuristic error observers", "[]");
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance evaluator (only used for error initialization with the cost bound)", OptionParser::NONE);
	parser.add_option<bool>("initialize_error_with_cost_bound",
	                        "assume that the initial heuristic value should be equal to the cost bound for the initialization of the heuristic error", "false");
	parser.add_option<bool>("reopen_closed", "reopen closed nodes", "true");
	parser.add_option<int>("boost", "boost value for preferred operator open lists", "0");
	SearchEngine::add_pruning_option(parser);
	SearchEngine::add_options_to_parser(parser);
}

template class EagerBoundedCostSearch<1>;
template class EagerBoundedCostSearch<2>;
template class EagerBoundedCostSearch<3>;
} // namespace bounded_cost_search
