#ifndef BOUNDED_SUBOPTIMAL_SEARCH_ALTERNATING_SPEEDY_SEARCH_H
#define BOUNDED_SUBOPTIMAL_SEARCH_ALTERNATING_SPEEDY_SEARCH_H

#include <map>
#include <optional>

#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "../floating_point_open_list/best_first_open_list.h"
#include "../heuristic_error/heuristic_error.h"
#include "../search_engine.h"

namespace bounded_suboptimal_search {
class AlternatingSpeedySearch : public SearchEngine {
	const bool reopen_closed_nodes;
	const double suboptimality_factor;

	int f_min;

	// an integer-based priority queue would make more sense here, but this does not make a practical difference
	floating_point_open_list::BestFirstOpenList<1, StateID> d_list;
	floating_point_open_list::BestFirstOpenList<1, StateID> f_hat_list;

	// iterable bucket-based open list (ordered by f)
	// each bucket is a heap sorted by high g
	std::map<int, std::vector<std::pair<int, StateID>>> f_list;

	static constexpr auto f_list_compare = [](const std::pair<int, StateID> &lhs, const std::pair<int, StateID> &rhs) {
		return lhs.first < rhs.first;
	};

	void reward_progress();

	std::shared_ptr<Evaluator> distance;
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<Evaluator> f_evaluator;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> f_hat_evaluator;

	std::vector<std::shared_ptr<heuristic_error::HeuristicError>> heuristic_error;

protected:
	auto fetch_next_node() -> std::optional<SearchNode>;
	void insert(EvaluationContext &eval_context);

	void initialize() override;
	SearchStatus step() override;

public:
	explicit AlternatingSpeedySearch(const options::Options &opts);
	virtual ~AlternatingSpeedySearch() = default;

	void print_statistics() const override;

	void dump_search_space() const;
};
} // namespace bounded_suboptimal_search

#endif
