#ifndef FLOATING_POINT_OPEN_LIST_FOCAL_OPEN_LIST_H
#define FLOATING_POINT_OPEN_LIST_FOCAL_OPEN_LIST_H

#include <queue>

#include "floating_point_open_list.h"

namespace floating_point_open_list {
template <std::size_t N, class T>
class FocalOpenList : public FloatingPointOpenList<N, T> {
	using typename FloatingPointOpenList<N, T>::internal_value_type;
	using typename FloatingPointOpenList<N, T>::compare_type;
	using focal_internal_value_type = std::pair<double, T>;
	using focal_compare_type = std::function<bool(const focal_internal_value_type &, const focal_internal_value_type &)>;

	std::priority_queue<internal_value_type, std::vector<internal_value_type>, compare_type> queue;
	std::priority_queue<focal_internal_value_type, std::vector<focal_internal_value_type>, focal_compare_type> focal;

	static constexpr auto focal_default_compare = [](const auto &lhs, const auto &rhs) {
		return lhs.first > rhs.first;
	};

public:
	using typename FloatingPointOpenList<N, T>::key_type;
	using typename FloatingPointOpenList<N, T>::value_type;

	[[nodiscard]] auto top() const -> const value_type & override { return !focal.empty() ? focal.top().second : queue.top().second; }
	[[nodiscard]] auto empty() const -> bool override { return focal.empty() && queue.empty(); }
	void push(const key_type &key, const value_type &value, bool) override { queue.push({key, value}); }
	void emplace(const key_type &key, value_type &&value, bool) override { queue.emplace(key, value); }
	void push_focal(double key, const value_type &value, bool) override { focal.push({key, value}); }
	void emplace_focal(double key, value_type &&value, bool) override { focal.emplace(key, value); }
	void pop() override { if (!focal.empty()) focal.pop(); else queue.pop(); }

	FocalOpenList() : FocalOpenList(FloatingPointOpenList<N, T>::get_default_compare(), focal_default_compare) {}
	FocalOpenList(compare_type compare, focal_compare_type compare_focal) : FloatingPointOpenList<N, T>(), queue(compare), focal(compare_focal) {}
};
} // namespace floating_point_open_list

#endif
