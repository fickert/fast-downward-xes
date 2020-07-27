#include "ff_distance_wrapper.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace ff_heuristic {
FFDistanceWrapper::FFDistanceWrapper(const options::Options &opts)
	: Heuristic(opts), ff_heuristic(std::static_pointer_cast<FFHeuristic>(opts.get<std::shared_ptr<Evaluator>>("heuristic"))) {}

auto FFDistanceWrapper::compute_heuristic(const GlobalState &global_state) -> int {
	if (ff_heuristic->last_evaluated != global_state.get_id())
		ff_heuristic->compute_heuristic_for_distance_wrapper(global_state);
	assert(ff_heuristic->last_evaluated == global_state.get_id());
	return ff_heuristic->last_relaxed_plan_length;
}

static std::shared_ptr<Heuristic> _parse(OptionParser &parser) {
	parser.add_option<std::shared_ptr<Evaluator>>("heuristic", "underlying FF heuristic");
	Heuristic::add_options_to_parser(parser);
	auto opts = parser.parse();
	if (parser.dry_run() || parser.help_mode())
		return nullptr;
	return std::make_shared<FFDistanceWrapper>(opts);
}

static Plugin<Evaluator> _plugin("ff_distance", _parse);
} // namespace ff_heuristic
