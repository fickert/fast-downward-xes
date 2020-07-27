#ifndef FLOATING_POINT_OPEN_LIST_FLOATING_POINT_OPEN_LIST_H
#define FLOATING_POINT_OPEN_LIST_FLOATING_POINT_OPEN_LIST_H

#include <functional>
#include <utility>

#include "../utils/system.h"

namespace floating_point_open_list {
template <std::size_t N, class T>
class FloatingPointOpenList {
	static_assert(N > 0, "The open list must have at least one key.");

	[[noreturn]] static void not_focal() {
		std::cerr << "Tried to do a focal insert on a non-focal open list." << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}

public:
	using key_type = std::array<double, N>;
	using value_type = T;

protected:
	using internal_value_type = std::pair<key_type, value_type>;
	using compare_type = std::function<bool(const internal_value_type &, const internal_value_type &)>;

	// default comparison function for open lists based on std::priority_queue (lexicographical comparison)
	static constexpr auto default_compare = [](const auto &lhs, const auto &rhs) {
		return lhs.first > rhs.first;
	};

public:
	[[nodiscard]] virtual auto top() const -> const value_type & = 0;
	[[nodiscard]] virtual auto empty() const -> bool = 0;
	virtual void push(const key_type &key, const value_type &value, bool preferred) = 0;
	virtual void emplace(const key_type &key, value_type &&value, bool preferred) = 0;
	virtual void push_focal(double, const value_type &, bool) { not_focal(); }
	virtual void emplace_focal(double, value_type &&, bool) { not_focal(); }
	virtual void pop() = 0;
	virtual void boost_preferred() {}

	FloatingPointOpenList() = default;
	virtual ~FloatingPointOpenList() = default;

	FloatingPointOpenList(const FloatingPointOpenList &other) = delete;
	FloatingPointOpenList(FloatingPointOpenList &&other) = delete;

	auto operator=(const FloatingPointOpenList &other) -> FloatingPointOpenList & = delete;
	auto operator=(FloatingPointOpenList &&other) -> FloatingPointOpenList & = delete;
};
} // namespace floating_point_open_list

#endif
