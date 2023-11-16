#include "weighted_astar_search.h"

#include <cassert>

#include "../evaluation_context.h"
#include "../evaluators/g_evaluator.h"
#include "../floating_point_evaluator/floating_point_evaluator_wrapper.h"
#include "../floating_point_evaluator/fp_sum_evaluator.h"
#include "../floating_point_evaluator/fp_weighted_evaluator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../suboptimal_search/util.h"
#include "../task_utils/successor_generator.h"

using namespace floating_point_evaluator;
using namespace suboptimal_search;

namespace bounded_suboptimal_search {
WeightedAstarSearch::WeightedAstarSearch(const Options &opts) : suboptimal_search::EagerSuboptimalSearch<2>(opts) {
	if (opts.contains("preferred") && !opts.get_list<std::shared_ptr<Evaluator>>("preferred").empty()) {
		std::cerr << "Dynamic potential search currently does not support preferred operators, exiting." << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
	}
}

void WeightedAstarSearch::reward_progress() {
	// Boost the "preferred operator" open lists somewhat whenever
	// one of the heuristics finds a state with a new best h value.
	open_list.boost_preferred();
}

auto WeightedAstarSearch::fetch_next_node() -> std::optional<SearchNode> {
	auto node = std::optional<SearchNode>();
	while (true) {
		if (open_list.empty())
			break;
		auto id = open_list.top();
		open_list.pop();
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

void WeightedAstarSearch::insert(EvaluationContext &, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) {
	open_list.push(evaluator_values, state_id, preferred);
}

static auto _parse_wastar(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "heuristic");
	parser.add_option<double>("weight", "weight");

	add_options_to_parser(parser);

	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;

	auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	auto fp_h_evaluator = std::make_shared<FloatingPointEvaluatorWrapper>(opts.get<std::shared_ptr<Evaluator>>("heuristic"));
	auto weighted_h_evaluator = std::make_shared<WeightedEvaluator>(fp_h_evaluator, opts.get<double>("weight"));
	auto f_evaluator = std::make_shared<SumEvaluator>(
			std::vector<std::shared_ptr<FloatingPointEvaluator>>{std::make_shared<FloatingPointEvaluatorWrapper>(g_evaluator), weighted_h_evaluator});

	opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {f_evaluator, fp_h_evaluator});

	return std::make_shared<WeightedAstarSearch>(opts);
}

static options::Plugin<SearchEngine> _plugin_wastar("fp_wastar", _parse_wastar);
} // namespace bounded_suboptimal_search
