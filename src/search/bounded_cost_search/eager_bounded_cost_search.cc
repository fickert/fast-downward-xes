#include "eager_bounded_cost_search.h"

#include <cassert>

#include "../algorithms/ordered_set.h"
#include "../evaluation_context.h"
#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "../floating_point_open_list/alternation_open_list.h"
#include "../floating_point_open_list/best_first_open_list.h"
#include "../heuristic_error/heuristic_error.h"
#include "../option_parser.h"

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
EagerBoundedCostSearch<N>::EagerBoundedCostSearch(const Options &opts, OpenListCreationFunction create_default_open_list)
	: suboptimal_search::EagerSuboptimalSearch<N>(opts),
	  initialize_error_with_cost_bound(opts.get<bool>("initialize_error_with_cost_bound")),
	  distance_evaluator(opts.get<std::shared_ptr<Evaluator>>("distance", nullptr)),
	  open_list(create_open_list(opts, create_default_open_list)) {}

template <std::size_t N>
void EagerBoundedCostSearch<N>::initialize_heuristic_error(EvaluationContext &eval_context) {
	for (const auto &h_error : heuristic_error) {
		if (initialize_error_with_cost_bound) {
			assert(distance_evaluator);
			const auto distance = eval_context.get_evaluator_value_or_infinity(distance_evaluator.get());
			h_error->notify_initial_state(eval_context, bound, distance);
		} else {
			h_error->notify_initial_state();
		}
	}
}

template <std::size_t N>
auto EagerBoundedCostSearch<N>::fetch_next_node() -> std::optional<SearchNode> {
	auto node = std::optional<SearchNode>();
	while (true) {
		if (open_list->empty())
			break;
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
	return node;
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

void add_options_to_parser(OptionParser &parser) {
	parser.add_option<std::shared_ptr<Evaluator>>("distance", "distance evaluator (only used for error initialization with the cost bound)", OptionParser::NONE);
	parser.add_option<bool>("initialize_error_with_cost_bound",
	                        "assume that the initial heuristic value should be equal to the cost bound for the initialization of the heuristic error", "false");
	suboptimal_search::add_options_to_parser(parser);
}

template class EagerBoundedCostSearch<1>;
template class EagerBoundedCostSearch<2>;
template class EagerBoundedCostSearch<3>;
} // namespace bounded_cost_search
