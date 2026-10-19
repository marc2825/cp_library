#include "basic_utility/debug.hpp"
#include "basic_utility/debug.hpp"
#include <array>
#include <forward_list>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>
#include "basic_utility/big_integer.hpp"
struct Custom { int value; };
template <> struct cp::DebugFormatter<Custom> {
    static void write(std::ostream& out, const Custom& value) { out << "custom:" << value.value; }
};
struct Streamed { int value; };
std::ostream& operator<<(std::ostream& out, const Streamed& value) { return out << "stream:" << value.value; }
struct Opaque {};
void check(bool condition) { if (!condition) throw std::runtime_error("debug check failed"); }
int main() {
    check(cp::debug_string(std::vector<int>{1, -2, 3}) == "[1, -2, 3]");
    check(cp::debug_string(std::vector<bool>{true, false}) == "[true, false]");
    check(cp::debug_string(std::make_tuple(1, "a\nb", std::pair<int, bool>{2, true})) == "(1, \"a\\nb\", (2, true))");
    check(cp::debug_string(std::tuple<>()) == "()");
    check(cp::debug_string(std::array<int, 2>{3, 4}) == "(3, 4)");
    check(cp::debug_string(std::map<int, std::vector<int>>{{1, {2, 3}}}) == "[(1, [2, 3])]");
    check(cp::debug_string(std::set<int>{3, 1}) == "[1, 3]");
    check(cp::debug_string(std::forward_list<int>{1, 2}) == "[1, 2]");
    check(cp::debug_string(std::optional<int>{}) == "nullopt");
    check(cp::debug_string(std::optional<int>{5}) == "optional(5)");
    check(cp::debug_string(std::variant<int, std::string>{std::string("ok")}) == "variant[1](\"ok\")");
    check(cp::debug_string(std::monostate{}) == "monostate");
    check(cp::debug_string(Custom{7}) == "custom:7");
    check(cp::debug_string(Streamed{8}) == "stream:8");
    check(cp::debug_string(Opaque{}) == "<unprintable>");
    check(cp::debug_string(cp::BigInt("12345678901234567890")) == "12345678901234567890");
    const auto minimum = std::numeric_limits<__int128_t>::min();
    check(cp::debug_string(minimum) == "-170141183460469231731687303715884105728");
    check(cp::debug_string(std::numeric_limits<__uint128_t>::max()) == "340282366920938463463374607431768211455");
    for (int value = -128; value < 128; ++value) check(cp::debug_string(static_cast<signed char>(value)) == std::to_string(value));
    const char embedded[] = {'a', '\0', 'b'};
    check(cp::debug_string(embedded) == "\"a\\x00b\"");
    const char* null = nullptr;
    check(cp::debug_string(null) == "null");
    const char* text = "abcdef";
    check(cp::debug_string(text, {8, 32, 3}) == "\"abc...\"");
    check(cp::debug_string('"') == "'\"'");
    check(cp::debug_string('\\') == "'\\\\'");
    check(cp::debug_string(std::string("abc"), {8, 32, 2}) == "\"ab...\"");
    check(cp::debug_string(std::vector<int>{1, 2, 3}, {8, 2, 256}) == "[1, 2, ...]");
    check(cp::debug_string(std::vector<int>{1, 2}, {0, 32, 256}) == "[..., ...]");
    check(cp::debug_string(std::vector<int>{1}, {8, 0, 256}) == "[...]");
    check(cp::debug_string(std::pair<int, int>{1, 2}, {8, 1, 256}) == "(1, ...)");
    std::ostringstream out;
    int calls = 0;
    CP_DEBUG_TO(out, ++calls);
#if defined(CP_ENABLE_DEBUG) && !defined(CP_DISABLE_DEBUG) && !defined(ONLINE_JUDGE)
    check(calls == 1 && out.str().find("++calls = (1)") != std::string::npos);
#else
    check(calls == 0 && out.str().empty());
    CP_DEBUG(does_not_exist);
    CP_DEBUG_TO(also_does_not_exist, invalid_expression());
#endif
    std::cout << "debug checks passed\n";
}
