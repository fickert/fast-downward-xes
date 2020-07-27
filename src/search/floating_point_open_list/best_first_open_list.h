#ifndef FLOATING_POINT_OPEN_LIST_BEST_FIRST_OPEN_LIST_H
#define FLOATING_POINT_OPEN_LIST_BEST_FIRST_OPEN_LIST_H

#include <queue>

#include "floating_point_open_list.h"

namespace floating_point_open_list {
template <std::size_t N, class T>
class BestFirstOpenList : public FloatingPointOpenList<N, T> {
	using typename FloatingPointOpenList<N, T>::internal_value_type;
	using typename FloatingPointOpenList<N, T>::compare_type;

	std::priority_queue<internal_value_type, std::vector<internal_value_type>, compare_type> queue;

public:
	using typename FloatingPointOpenList<N, T>::key_type;
	using typename FloatingPointOpenList<N, T>::value_type;

	auto top() const -> const value_type & override { return queue.top().second; }
	[[nodiscard]] auto empty() const -> bool override { return queue.empty(); }
	void push(const key_type &key, const value_type &value, bool) override { queue.push({key, value}); }
	void emplace(const key_type &key, value_type &&value, bool) override { queue.emplace(key, value); }
	void pop() override { queue.pop(); }

	BestFirstOpenList() : BestFirstOpenList(FloatingPointOpenList<N, T>::default_compare) {}
	explicit BestFirstOpenList(compare_type compare) : FloatingPointOpenList<N, T>(), queue(compare) {}
};
} // namespace floating_point_open_list

#endif
