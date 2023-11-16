#ifndef BOUNDED_SUBOPTIMAL_SEARCH_DYNAMIC_POTENTIAL_SEARCH_H
#define BOUNDED_SUBOPTIMAL_SEARCH_DYNAMIC_POTENTIAL_SEARCH_H

#include <boost/container_hash/hash.hpp>
#include <limits>
#include <map>
#include <optional>

#include "../search_engine.h"

namespace bounded_suboptimal_search {
class DynamicPotentialSearch : public SearchEngine {
	const bool reopen_closed_nodes;
	const double suboptimality_factor;

	// ((g, h), state_ids)
	using OpenListKey = std::pair<int, int>;
	using OpenListValueSet = std::shared_ptr<std::vector<StateID>>;
	using OpenListElement = std::pair<OpenListKey, OpenListValueSet>;

	class compare_potential_t {
		int f_min;
		const double suboptimality_factor;

		auto compute_potential(const OpenListElement &element) const -> double {
			const auto [g, h] = element.first;
			return h == 0 ? std::numeric_limits<double>::infinity() : (suboptimality_factor * f_min - g) / h;
		}

	public:
		compare_potential_t(double suboptimality_factor) : f_min(0), suboptimality_factor(suboptimality_factor) {}
		auto get_f_min() const -> int { return f_min; }
		void set_f_min(int f_min) { this->f_min = f_min; }
		auto operator()(const OpenListElement &lhs, const OpenListElement &rhs) const -> bool {
			const auto lhs_potential = compute_potential(lhs);
			const auto rhs_potential = compute_potential(rhs);
			if (lhs_potential != rhs_potential)
				return lhs_potential < rhs_potential;
			return lhs.first.second < rhs.first.second;
		}
	} compare_potential;

	std::vector<OpenListElement> open_list;
	std::unordered_map<OpenListKey, OpenListValueSet, boost::hash<OpenListKey>> open_list_value_sets;
	std::map<int, int> f_counts; // number of value sets (i.e. distinct (g, h) pairs) per f-value

	void reward_progress();

	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<Evaluator> f_evaluator;

protected:
	auto fetch_next_node() -> std::optional<SearchNode>;
	void insert(EvaluationContext &eval_context);

	void initialize() override;
	SearchStatus step() override;

public:
	explicit DynamicPotentialSearch(const options::Options &opts);
	virtual ~DynamicPotentialSearch() = default;

	void print_statistics() const override;

	void dump_search_space() const;
};
} // namespace bounded_suboptimal_search

#endif
