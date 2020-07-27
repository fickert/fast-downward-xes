#ifndef FLOATING_POINT_OPEN_LIST_ALTERNATION_OPEN_LIST_H
#define FLOATING_POINT_OPEN_LIST_ALTERNATION_OPEN_LIST_H

#include <memory>

#include "floating_point_open_list.h"

namespace floating_point_open_list {
template <std::size_t N, class T>
class AlternationOpenList : public FloatingPointOpenList<N, T> {
	const int boost_amount;

	struct sublist_type {
		std::unique_ptr<FloatingPointOpenList<N, T>> open_list;
		int priority;
		bool only_preferred;
	};

	std::vector<sublist_type> sublists;

	static auto initialize_sublists(std::vector<std::unique_ptr<FloatingPointOpenList<N, T>>> &&sublists,
	                                std::vector<std::unique_ptr<FloatingPointOpenList<N, T>>> &&preferred_sublists) -> std::vector<sublist_type>;

	auto get_best_list() const -> const sublist_type &;
	auto get_best_list() -> sublist_type & { return const_cast<sublist_type &>(const_cast<const AlternationOpenList *>(this)->get_best_list()); }

public:
	using typename FloatingPointOpenList<N, T>::key_type;
	using typename FloatingPointOpenList<N, T>::value_type;

	auto top() const -> const value_type & override;
	[[nodiscard]] auto empty() const -> bool override;
	void push(const key_type &key, const T &value, bool preferred) override;
	void emplace(const key_type &key, T &&value, bool preferred) override;
	void push_focal(double key, const T &value, bool preferred) override;
	void emplace_focal(double key, T &&value, bool preferred) override;
	void pop() override;
	void boost_preferred() override;

	AlternationOpenList(int boost_amount, std::vector<std::unique_ptr<FloatingPointOpenList<N, T>>> &&sublists, std::vector<std::unique_ptr<FloatingPointOpenList<N, T>>> &&preferred_sublists)
		: FloatingPointOpenList<N, T>(), boost_amount(boost_amount), sublists(initialize_sublists(std::move(sublists), std::move(preferred_sublists))) {}
};

template <std::size_t N, class T>
auto AlternationOpenList<N, T>::initialize_sublists(std::vector<std::unique_ptr<FloatingPointOpenList<N, T>>> &&sublists,
                                                    std::vector<std::unique_ptr<FloatingPointOpenList<N, T>>> &&preferred_sublists) -> std::vector<sublist_type> {
	auto all_sublists = std::vector<sublist_type>();
	all_sublists.reserve(sublists.size() + preferred_sublists.size());
	for (auto &&sublist : sublists)
		all_sublists.push_back({std::move(sublist), 0, false});
	for (auto &&sublist : preferred_sublists)
		all_sublists.push_back({std::move(sublist), 0, true});
	return all_sublists;
}

template <std::size_t N, class T>
auto AlternationOpenList<N, T>::get_best_list() const -> const sublist_type & {
	return *std::min_element(std::begin(sublists), std::end(sublists), [](const auto &lhs, const auto &rhs) {
		if (lhs.open_list->empty() != rhs.open_list->empty())
			return rhs.open_list->empty();
		return lhs.priority < rhs.priority;
	});
}

template <std::size_t N, class T>
auto AlternationOpenList<N, T>::top() const -> const value_type & {
	return get_best_list().open_list->top();
}

template <std::size_t N, class T>
auto AlternationOpenList<N, T>::empty() const -> bool {
	return get_best_list().open_list->empty();
}

template <std::size_t N, class T>
void AlternationOpenList<N, T>::push(const key_type &key, const T &value, bool preferred) {
	for (auto &sublist : sublists)
		if (preferred || !sublist.only_preferred)
			sublist.open_list->push(key, value, preferred);
}

template <std::size_t N, class T>
void AlternationOpenList<N, T>::emplace(const key_type &key, T &&value, bool preferred) {
	for (auto &sublist : sublists)
		if (preferred || !sublist.only_preferred)
			sublist.open_list->push(key, std::move(value), preferred);
}

template <std::size_t N, class T>
void AlternationOpenList<N, T>::push_focal(double key, const T &value, bool preferred) {
	for (auto &sublist : sublists)
		if (preferred || !sublist.only_preferred)
			sublist.open_list->push_focal(key, value, preferred);
}

template <std::size_t N, class T>
void AlternationOpenList<N, T>::emplace_focal(double key, T &&value, bool preferred) {
	for (auto &sublist : sublists)
		if (preferred || !sublist.only_preferred)
			sublist.open_list->emplace_focal(key, std::move(value), preferred);
}

template <std::size_t N, class T>
void AlternationOpenList<N, T>::pop() {
	auto &best_list = get_best_list();
	++best_list.priority;
	best_list.open_list->pop();
}

template <std::size_t N, class T>
void AlternationOpenList<N, T>::boost_preferred() {
	for (auto &sublist : sublists)
		if (sublist.only_preferred)
			sublist.priority -= boost_amount;
}
} // namespace floating_point_open_list

#endif
