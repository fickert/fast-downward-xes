#include "greedy_explicit_estimation_search.h"

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
GreedyExplicitEstimationSearch<tie_breaking>::GreedyExplicitEstimationSearch(const Options &opts)
	: EagerSuboptimalSearch<3>(opts),
	  suboptimality_factor(opts.get<double>("suboptimality_factor")),
	  heuristic(opts.get<std::shared_ptr<Evaluator>>("heuristic")),
	  d_hat_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("d_hat_evaluator")),
	  f_hat_evaluator(opts.get<std::shared_ptr<FloatingPointEvaluator>>("f_hat_evaluator")),
	  f_min(0),
	  focal_list(focal_list_compare) {
	if (!opts.get_list<std::shared_ptr<Evaluator>>("preferred").empty()) {
		std::cerr << "EES currently does not support preferred operators, exiting." << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
	}
}

template <bool tie_breaking>
auto GreedyExplicitEstimationSearch<tie_breaking>::open_list_top() -> std::optional<SearchNode> {
	while (!open_list.empty()) {
		auto &state_ids = std::begin(open_list)->second;
		auto id = StateID::no_state;
		if constexpr (tie_breaking)
			id = std::get<StateID>(state_ids.front());
		else
			id = std::get<StateID>(state_ids.back());
		const auto state = state_registry.lookup_state(id);
		const auto node = search_space.get_node(state);
		if (!node.is_closed())
			return node;
		open_list_pop();
	}
	return {};
}

template <bool tie_breaking>
void GreedyExplicitEstimationSearch<tie_breaking>::open_list_pop() {
	assert(!open_list.empty());
	auto &state_ids = std::begin(open_list)->second;
	if constexpr (tie_breaking)
		std::pop_heap(std::begin(state_ids), std::end(state_ids), tie_breaking_open_list_compare);
	state_ids.pop_back();
	if (state_ids.empty())
		open_list.erase(std::begin(open_list));
}

template <bool tie_breaking>
void GreedyExplicitEstimationSearch<tie_breaking>::open_list_push(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id,
                                                            std::optional<FocalListHandleType> focal_handle) {
	const auto f_hat = evaluator_values[F_HAT_INDEX];
	auto &state_ids = open_list[f_hat];
	if constexpr (tie_breaking) {
		state_ids.emplace_back(eval_context.get_g_value(), state_id, focal_handle);
		std::push_heap(std::begin(state_ids), std::end(state_ids), tie_breaking_open_list_compare);
	} else {
		state_ids.emplace_back(state_id, focal_handle);
	}
}

template <bool tie_breaking>
void GreedyExplicitEstimationSearch<tie_breaking>::update_focal(int current_f_min) {
	if (current_f_min <= f_min)
		return;
	// the bound increased --> we need to copy some nodes into the focal list
	open_list_top(); // just call this for the side effect of cleaning up the front of the open list
	const auto lower = open_list.upper_bound(f_min);
	const auto upper = open_list.upper_bound(current_f_min);
	for (auto it = lower; it != upper; ++it) {
		for (auto &open_list_element : it->second) {
			assert(!std::get<std::optional<FocalListHandleType>>(open_list_element));
			const auto state_id = std::get<StateID>(open_list_element);
			const auto state = state_registry.lookup_state(state_id);
			const auto node = search_space.get_node(state);
			auto eval_context = EvaluationContext(state, node.get_g(), false, &statistics);
			if (node.is_closed() || f_hat_evaluator->compute_result(eval_context) != it->first)
				// ignore nodes that are closed or reopened and this one doesn't have the updated g-value
				continue;
			const auto d_hat = d_hat_evaluator->compute_result(eval_context);
			// update handle
			if constexpr (tie_breaking) {
				const auto f_hat = f_hat_evaluator->compute_result(eval_context);
				assert(!eval_context.is_evaluator_value_infinite(heuristic.get()));
				const auto h = eval_context.get_evaluator_value(heuristic.get());
				std::get<std::optional<FocalListHandleType>>(open_list_element).emplace(focal_list.emplace(std::make_tuple(d_hat, f_hat, h), std::move(state_id)));
			} else {
				std::get<std::optional<FocalListHandleType>>(open_list_element).emplace(focal_list.emplace(d_hat, std::move(state_id)));
			}
		}
	}
	// update f_min
	f_min = current_f_min;
}

template <bool tie_breaking>
auto GreedyExplicitEstimationSearch<tie_breaking>::fetch_next_node() -> std::optional<SearchNode> {
	const auto fetch_top = [this](auto &open_list, auto get_state_id) {
		while (!open_list.empty()) {
			const auto id = get_state_id(open_list.top());
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

	auto best_f = fetch_top(cleanup_list, [](const auto &state_id) { return state_id; });
	if (!best_f)
		// exhausted search space
		return {};
	auto best_f_eval_context = EvaluationContext(best_f->get_state(), best_f->get_g(), false, &statistics, false);
	assert(!best_f_eval_context.is_evaluator_value_infinite(heuristic.get()));
	const auto current_f_min = best_f->get_g() + best_f_eval_context.get_evaluator_value(heuristic.get());

	update_focal(current_f_min);

	auto best_d_hat = fetch_top(focal_list, [](const auto &focal_pair) { return focal_pair.second; });
	if (best_d_hat) {
		focal_list.pop();
		close_node(best_d_hat);
		return best_d_hat;
	} else {
		cleanup_list.pop();
		close_node(best_f);
		return best_f;
	}
}

template <bool tie_breaking>
void GreedyExplicitEstimationSearch<tie_breaking>::insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) {
	auto focal_handle = std::optional<typename decltype(focal_list)::handle_type>();
	if (evaluator_values[F_HAT_INDEX] <= suboptimality_factor * f_min) {
		if constexpr (tie_breaking) {
			assert(!eval_context.is_evaluator_value_infinite(heuristic.get()));
			const auto h = eval_context.get_evaluator_value(heuristic.get());
			focal_handle.emplace(focal_list.emplace(std::make_tuple(evaluator_values[D_HAT_INDEX], evaluator_values[F_HAT_INDEX], h), state_id));
		} else {
			focal_handle.emplace(focal_list.emplace(evaluator_values[D_HAT_INDEX], state_id));
		}
	}
	open_list_push(eval_context, evaluator_values, state_id, focal_handle);
	if constexpr (tie_breaking) {
		assert(!eval_context.is_evaluator_value_infinite(heuristic.get()));
		const auto h = eval_context.get_evaluator_value(heuristic.get());
		cleanup_list.emplace({evaluator_values[F_INDEX], static_cast<double>(h)}, std::move(state_id), preferred);
	} else {
		cleanup_list.emplace({evaluator_values[F_INDEX]}, std::move(state_id), preferred);
	}
}

template <bool tie_breaking>
void GreedyExplicitEstimationSearch<tie_breaking>::reward_progress() {
	// preferred operators are not (yet?) implemented for EES --> nothing to do here
}

static auto _parse_gees(OptionParser &parser) -> std::shared_ptr<SearchEngine> {
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

	opts.set<std::vector<std::shared_ptr<FloatingPointEvaluator>>>("evals", {debiased_distance, f_hat_evaluator, fp_f_evaluator});
	return opts.get<bool>("enable_tie_breaking") ? std::static_pointer_cast<SearchEngine>(std::make_shared<GreedyExplicitEstimationSearch<true>>(opts))
	                                             : std::static_pointer_cast<SearchEngine>(std::make_shared<GreedyExplicitEstimationSearch<false>>(opts));
}

static options::Plugin<SearchEngine> _plugin_bees("gees", _parse_gees);

template class GreedyExplicitEstimationSearch<true>;
template class GreedyExplicitEstimationSearch<false>;
} // namespace bounded_suboptimal_search
