// basic_utility の素朴解比較テスト（issue #3 の精査で追加）。
// 各関数を小さな入力の全探索・標準ライブラリと比べ、境界と回帰ケースも確認する。
#include "basic_utility/all.hpp"

#include <algorithm>
#include <climits>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#if defined(__unix__)
#include <unistd.h>
#endif

namespace {
#define REQUIRE(condition) \
    do { if (!(condition)) throw std::runtime_error(std::string("line ") + std::to_string(__LINE__) + ": " #condition); } while (0)

struct Rng {
    std::uint64_t state;
    std::uint64_t next() {
        state += 0x9e3779b97f4a7c15ULL;
        std::uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }
    long long range(long long lo, long long hi) { // [lo, hi]
        return lo + static_cast<long long>(next() % static_cast<std::uint64_t>(hi - lo + 1));
    }
};

// 回帰：グローバル変数の初期化中でも静的な還元器が使える（以前は 0 除算で落ちた）。
using GlobalMontgomery = cp::MontgomeryModInt64<1000000007>;
using GlobalDynamic = cp::DynamicModInt<7>;
const GlobalMontgomery global_inverse_two = GlobalMontgomery(2).inv();
const GlobalDynamic global_minus_one(-1);

void global_modints() {
    REQUIRE(global_inverse_two.val() == 500000004);
    REQUIRE(global_minus_one.val() == 998244352);
}

#if defined(__unix__)
// 回帰：届いた分だけで読み進める（以前は 64KiB 溜まるまで fread が戻らず、インタラクティブ問題で止まった）。
void scanner_does_not_wait_for_full_buffer() {
    int fds[2];
    REQUIRE(::pipe(fds) == 0);
    REQUIRE(::write(fds[1], "12 -34 ", 7) == 7);
    std::FILE* file = ::fdopen(fds[0], "r");
    REQUIRE(file != nullptr);
    ::alarm(10); // 古い実装ではここで止まるので、テストを失敗として終わらせる
    {
        cp::FastScanner in(file);
        int a = 0, b = 0;
        REQUIRE(in.read(a, b) && a == 12 && b == -34);
        REQUIRE(::write(fds[1], "56\n", 3) == 3);
        ::close(fds[1]);
        int c = 0;
        REQUIRE(in.read(c) && c == 56 && !in.read(c));
    }
    ::alarm(0);
    std::fclose(file);
    // 先に stdio で読んだ後でも、stdio が先読みした残りを失わない（この場合は fread で読む）。
    REQUIRE(::pipe(fds) == 0);
    REQUIRE(::write(fds[1], "1 2 3\n", 6) == 6);
    ::close(fds[1]);
    std::FILE* mixed = ::fdopen(fds[0], "r");
    REQUIRE(mixed != nullptr);
    int first = 0;
    REQUIRE(std::fscanf(mixed, "%d", &first) == 1 && first == 1);
    {
        cp::FastScanner in(mixed);
        int second = 0, third = 0;
        REQUIRE(in.read(second, third) && second == 2 && third == 3 && !in.read(third));
    }
    std::fclose(mixed);
    // bool は std::cout と同じく 1 / 0 で出力する（以前は生のバイト 0x01 だった）。
    std::FILE* out_file = std::tmpfile();
    REQUIRE(out_file != nullptr);
    {
        cp::FastPrinter out(out_file);
        out.writeln(true, false);
        out.flush();
    }
    std::rewind(out_file);
    char printed[8] = {};
    REQUIRE(std::fread(printed, 1, sizeof(printed), out_file) == 4 && std::string(printed, 4) == "1 0\n");
    std::fclose(out_file);
    // 記述子を持たない FILE* は fread で読む。
    char text[] = "7 8";
    std::FILE* memory = ::fmemopen(text, 3, "r");
    REQUIRE(memory != nullptr);
    {
        cp::BasicFastScanner<2> in(memory);
        int x = 0, y = 0;
        REQUIRE(in.read(x, y) && x == 7 && y == 8 && !in.read(x));
    }
    std::fclose(memory);
}
#endif

void fibonacci_search(Rng& rng) {
    for (int round = 0; round < 3000; ++round) {
        const int n = static_cast<int>(rng.range(1, 70));
        const int down = static_cast<int>(rng.range(0, n - 1)), flat = static_cast<int>(rng.range(1, n - down));
        std::vector<long long> f(n);
        long long value = rng.range(-50, 50);
        for (int i = 0; i < n; ++i) {
            if (i < down) value = f[i] = value + rng.range(1, 5) * (i ? -1 : 0) + (i ? 0 : 100);
            else if (i < down + flat) f[i] = i == down ? (value -= rng.range(1, 5)) : value;
            else f[i] = value += rng.range(1, 5);
        }
        // 先頭の down 個は狭義単調減少、続く flat 個は最小値の平坦部、残りは狭義単調増加。
        for (int i = 1; i < down; ++i) REQUIRE(f[i] < f[i - 1]);
        const long long left = rng.range(-1000, 1000);
        const auto result = cp::fibonacci_search(left, left + n, [&](std::int64_t x) { return f[x - left]; });
        const auto best = std::min_element(f.begin(), f.end()) - f.begin();
        REQUIRE(result.position == left + best && result.value == f[best]);
    }
}

void fractional_cascading(Rng& rng) {
    for (int round = 0; round < 500; ++round) {
        std::vector<std::vector<int>> lists(rng.range(0, 6));
        for (auto& list : lists) {
            list.resize(rng.range(0, 12));
            for (auto& x : list) x = static_cast<int>(rng.range(0, 20));
            std::sort(list.begin(), list.end());
        }
        const cp::FractionalCascading<int> cascade(lists);
        for (int x = -2; x <= 23; ++x) {
            const auto got = cascade.lower_bounds(x);
            REQUIRE(got.size() == lists.size());
            for (std::size_t i = 0; i < lists.size(); ++i)
                REQUIRE(got[i] == std::lower_bound(lists[i].begin(), lists[i].end(), x) - lists[i].begin());
        }
    }
}

void two_pointers_and_subarray(Rng& rng) {
    for (int round = 0; round < 1000; ++round) {
        const std::size_t n = rng.range(0, 15);
        std::vector<int> a(n);
        for (auto& x : a) x = static_cast<int>(rng.range(0, 6));
        const int limit = static_cast<int>(rng.range(0, 12));
        int sum = 0;
        const auto result = cp::two_pointers(n, [&](std::size_t r) { return sum + a[r] <= limit; },
                                             [&](std::size_t r) { sum += a[r]; }, [&](std::size_t l) { sum -= a[l]; });
        std::uint64_t count = 0;
        for (std::size_t l = 0; l < n; ++l) {
            std::size_t r = l;
            int total = 0;
            while (r < n && total + a[r] <= limit) total += a[r++];
            REQUIRE(result.right[l] == r);
            count += r - l;
        }
        REQUIRE(result.count() == count);

        std::vector<long long> b(n);
        for (auto& x : b) x = rng.range(-4, 4);
        for (bool allow_empty : {false, true}) {
            const auto best = cp::maximum_subarray(b, allow_empty);
            std::optional<std::tuple<long long, std::size_t, std::size_t>> expected; // (-sum, l, r) の最小
            if (allow_empty) expected = std::make_tuple(0LL, std::size_t(0), std::size_t(0));
            for (std::size_t l = 0; l < n; ++l) {
                long long s = 0;
                for (std::size_t r = l + 1; r <= n; ++r) {
                    s += b[r - 1];
                    const auto candidate = std::make_tuple(-s, l, r);
                    if (!expected || candidate < *expected) expected = candidate;
                }
            }
            REQUIRE(bool(best) == bool(expected));
            if (best) REQUIRE(std::make_tuple(-best->sum, best->left, best->right) == *expected);
        }
    }
}

void meet_in_the_middle(Rng& rng) {
    for (int round = 0; round < 300; ++round) {
        std::vector<std::int64_t> values(rng.range(0, 11));
        for (auto& x : values) x = rng.range(-6, 9);
        const cp::MeetInTheMiddle mitm(values);
        std::vector<std::pair<long long, std::uint64_t>> sums;
        for (std::uint64_t mask = 0; mask < (std::uint64_t(1) << values.size()); ++mask) {
            long long s = 0;
            for (std::size_t i = 0; i < values.size(); ++i) if (mask >> i & 1) s += values[i];
            sums.emplace_back(s, mask);
        }
        for (long long target = -30; target <= 40; target += 3) {
            const auto leq = std::count_if(sums.begin(), sums.end(), [&](auto& s) { return s.first <= target; });
            const auto eq = std::count_if(sums.begin(), sums.end(), [&](auto& s) { return s.first == target; });
            const auto in_range = std::count_if(sums.begin(), sums.end(), [&](auto& s) { return target <= s.first && s.first < target + 5; });
            REQUIRE(mitm.count_leq(target) == std::uint64_t(leq));
            REQUIRE(mitm.count_equal(target) == std::uint64_t(eq));
            REQUIRE(mitm.count_range(target, target + 5) == std::uint64_t(in_range)); // 半開区間 [low, high)
            std::optional<std::pair<long long, std::uint64_t>> best;
            for (auto& s : sums)
                if (s.first <= target && (!best || s.first > best->first || (s.first == best->first && s.second < best->second))) best = s;
            const auto got = mitm.best_leq(target);
            REQUIRE(bool(got) == bool(best));
            if (got) REQUIRE(static_cast<long long>(got->sum) == best->first && got->mask == best->second);
        }
    }
}

bool palindrome(std::uint64_t x) {
    const auto s = std::to_string(x);
    return std::equal(s.begin(), s.end(), s.rbegin());
}

void special_integers(Rng& rng) {
    for (int round = 0; round < 300; ++round) {
        const std::uint64_t first = rng.range(0, 30000), last = first + rng.range(0, 3000);
        std::vector<std::uint64_t> got, expected;
        cp::enumerate_palindromic_integers(first, last, [&](std::uint64_t x) { got.push_back(x); return true; });
        for (auto x = first; x < last; ++x) if (palindrome(x)) expected.push_back(x);
        REQUIRE(got == expected);

        std::vector<int> digits;
        for (int d = 0; d < 10; ++d) if (rng.range(0, 2) == 0) digits.push_back(d);
        got.clear(); expected.clear();
        cp::enumerate_digit_restricted_integers(first, last, digits, [&](std::uint64_t x) { got.push_back(x); return true; });
        for (auto x = first; x < last; ++x) {
            const auto s = std::to_string(x);
            if (std::all_of(s.begin(), s.end(), [&](char c) { return std::count(digits.begin(), digits.end(), c - '0'); }))
                expected.push_back(x);
        }
        REQUIRE(got == expected);

        std::vector<std::uint64_t> factors(rng.range(0, 3));
        for (auto& f : factors) f = rng.range(2, 12);
        const std::uint64_t limit = rng.range(0, 3000);
        got.clear();
        cp::enumerate_generated_products(limit, factors, [&](std::uint64_t x) { got.push_back(x); return true; });
        std::set<std::uint64_t> reach; // 1 から因子を掛けて limit 未満で届く数（乗法的閉包）
        std::vector<std::uint64_t> queue;
        if (limit > 1) { reach.insert(1); queue.push_back(1); }
        while (!queue.empty()) {
            const auto v = queue.back(); queue.pop_back();
            for (auto f : factors) if (v * f < limit && reach.insert(v * f).second) queue.push_back(v * f);
        }
        REQUIRE(got == std::vector<std::uint64_t>(reach.begin(), reach.end()));
    }
    // 上端付近：桁あふれせずに終わる。
    std::vector<std::uint64_t> top;
    cp::enumerate_palindromic_integers(UINT64_MAX - 20000000000ULL, UINT64_MAX,
                                       [&](std::uint64_t x) { top.push_back(x); return true; });
    for (auto x : top) REQUIRE(palindrome(x) && x >= UINT64_MAX - 20000000000ULL);
    std::uint64_t count = 0;
    cp::enumerate_digit_restricted_integers(UINT64_MAX - 1000, UINT64_MAX, {1, 8, 9},
                                            [&](std::uint64_t) { ++count; return true; });
    REQUIRE(count == 0);
}

void combinatorial_enumeration() {
    for (int n = 0; n <= 22; ++n) {
        std::vector<int> previous;
        std::uint64_t count = 0;
        cp::enumerate_integer_partitions(n, [&](const std::vector<int>& parts) {
            REQUIRE(std::accumulate(parts.begin(), parts.end(), 0) == n);
            REQUIRE(std::is_sorted(parts.rbegin(), parts.rend()));
            if (count) REQUIRE(std::lexicographical_compare(parts.begin(), parts.end(), previous.begin(), previous.end()));
            previous = parts; ++count; return true;
        });
        REQUIRE(cp::BigInt(count) == cp::integer_partition_count(n));
    }
    for (int n = 0; n <= 9; ++n) {
        std::vector<int> previous;
        std::uint64_t count = 0;
        cp::enumerate_set_partitions(n, [&](const std::vector<int>& rgs) {
            int maximum = -1;
            for (int x : rgs) { REQUIRE(x <= maximum + 1); maximum = std::max(maximum, x); }
            if (count) REQUIRE(previous < rgs);
            previous = rgs; ++count; return true;
        });
        REQUIRE(cp::BigInt(count) == cp::set_partition_count(n));
    }
    for (int pairs = 0; pairs <= 7; ++pairs) {
        const cp::ParenthesesTable table(pairs);
        std::string previous;
        for (long long i = 0; cp::BigInt(i) < table.count(); ++i) {
            const auto word = table.unrank(i);
            REQUIRE(cp::is_balanced_parentheses(word) && word.size() == std::size_t(2 * pairs));
            REQUIRE(table.rank(word) == cp::BigInt(i));
            if (i) REQUIRE(previous < word);
            REQUIRE(cp::tree_to_parentheses(cp::parentheses_to_tree(word)) == word);
            previous = word;
        }
    }
    for (int n = 0; n <= 6; ++n) {
        std::vector<int> p(n);
        std::iota(p.begin(), p.end(), 0);
        std::uint64_t rank = 0;
        do {
            REQUIRE(cp::permutation_rank(p) == rank && cp::permutation_unrank(n, rank) == p);
            ++rank;
        } while (std::next_permutation(p.begin(), p.end()));
    }
}

void permutation_tree(Rng& rng) {
    for (int round = 0; round < 400; ++round) {
        std::vector<int> p(rng.range(0, 40));
        std::iota(p.begin(), p.end(), 0);
        if (round % 3 == 0) for (std::size_t i = 1; i < p.size(); ++i) std::swap(p[i], p[rng.range(0, i)]);
        else for (std::size_t i = 0; i + 1 < p.size(); ++i) if (rng.range(0, 3) == 0) std::swap(p[i], p[i + 1]);
        std::uint64_t brute = 0;
        for (std::size_t l = 0; l < p.size(); ++l) {
            int lo = p[l], hi = p[l];
            for (std::size_t r = l; r < p.size(); ++r) {
                lo = std::min(lo, p[r]); hi = std::max(hi, p[r]);
                if (std::size_t(hi - lo) == r - l) ++brute;
            }
        }
        const cp::PermutationTree tree(p);
        REQUIRE(tree.common_interval_count() == brute);
        for (const auto& node : tree.nodes())
            REQUIRE(node.high - node.low == node.right - node.left);
    }
}

void fractions(Rng& rng) {
    for (std::uint64_t p = 1; p <= 60; ++p)
        for (std::uint64_t q = 1; q <= 60; ++q) {
            const auto f = cp::stern_brocot_fraction(cp::stern_brocot_path(p, q));
            const auto g = std::gcd(p, q);
            REQUIRE(f.numerator == p / g && f.denominator == q / g);
        }
    for (int round = 0; round < 300; ++round) {
        const std::uint64_t a = rng.range(0, 40), b = rng.range(1, 40), limit = rng.range(1, 30);
        const auto [low, high] = cp::fraction_binary_search(limit, [&](std::uint64_t p, std::uint64_t q) { return p * b <= a * q; });
        // 素朴解：分子・分母 <= limit の分数で、条件を満たす最大と満たさない最小。
        std::pair<std::uint64_t, std::uint64_t> best_true{0, 1}, best_false{1, 0};
        for (std::uint64_t p = 0; p <= limit; ++p)
            for (std::uint64_t q = 1; q <= limit; ++q) {
                if (std::gcd(p, q) != 1) continue;
                if (p * b <= a * q) { if (p * best_true.second > best_true.first * q) best_true = {p, q}; }
                else if (best_false.second == 0 || p * best_false.second < best_false.first * q) best_false = {p, q};
            }
        REQUIRE(low.numerator == best_true.first && low.denominator == best_true.second);
        REQUIRE(high.numerator == best_false.first && high.denominator == best_false.second);
    }
}

std::string concatenate(std::string x, std::string y) { return x + y; }
std::string empty_string() { return std::string(); }

void searches_and_structures(Rng& rng) {
    for (int round = 0; round < 500; ++round) {
        const std::uint64_t n = rng.range(1, 40), l = rng.range(0, 5);
        std::vector<long long> f(n);
        for (auto& x : f) x = rng.range(-20, 20);
        const auto slack = rng.range(0, 3);
        const auto result = cp::nonmonotone_argmin<long long>(l, l + n, [&](std::uint64_t x) { return f[x - l]; },
            [&](std::uint64_t a, std::uint64_t b) { return *std::min_element(f.begin() + (a - l), f.begin() + (b - l)) - slack; });
        const auto best = std::min_element(f.begin(), f.end()) - f.begin();
        REQUIRE(result && result->first == l + best && result->second == f[best]);
    }
    { // HashMap と std::map を同じ操作列で比べる。
        cp::HashMap<long long, int> map(cp::SeededHash(12345));
        std::map<long long, int> reference;
        for (int step = 0; step < 200000; ++step) {
            const long long key = rng.range(-300, 300);
            switch (rng.range(0, 5)) {
            case 0: map[key] += 1; reference[key] += 1; break;
            case 1: REQUIRE(map.erase(key) == bool(reference.erase(key))); break;
            case 2: { const auto* v = map.find(key); const auto it = reference.find(key);
                      REQUIRE(bool(v) == (it != reference.end()) && (!v || *v == it->second)); break; }
            case 3: { const int value = static_cast<int>(rng.range(0, 9));
                      map.insert_or_assign(key, value); reference[key] = value; break; }
            case 4: if (step % 5000 == 0) { auto copy = map; map = std::move(copy); map.reserve(rng.range(0, 700)); } break;
            default: map.try_emplace(key, 7); reference.try_emplace(key, 7); break;
            }
            REQUIRE(map.size() == reference.size());
        }
        std::map<long long, int> listed;
        map.for_each([&](long long k, int v) { listed[k] = v; });
        REQUIRE(listed == reference);
    }
    { // DynamicMex と素朴な集合。
        cp::DynamicMex mex;
        std::multiset<long long> values;
        for (int step = 0; step < 20000; ++step) {
            const long long x = rng.range(-3, 25);
            if (rng.range(0, 2)) { mex.insert(x); values.insert(x); }
            else { const auto it = values.find(x); REQUIRE(mex.erase(x) == (it != values.end())); if (it != values.end()) values.erase(it); }
            long long expected = 0;
            while (values.count(expected)) ++expected;
            REQUIRE(mex.mex() == std::uint64_t(expected) && mex.size() == values.size());
        }
    }
    { // 並べ替え順・挿入順が違っても同じ多重集合なら指紋が一致する。
        cp::MultisetHash<int> a(99), b(99);
        std::vector<int> items;
        for (int i = 0; i < 200; ++i) items.push_back(static_cast<int>(rng.range(0, 30)));
        for (int x : items) a.insert(x);
        for (auto it = items.rbegin(); it != items.rend(); ++it) b.insert(*it);
        REQUIRE(a.fingerprint() == b.fingerprint());
        b.erase(items[0]);
        REQUIRE(a.fingerprint() != b.fingerprint());
        b.insert(items[0]);
        REQUIRE(a.fingerprint() == b.fingerprint());
    }
    { // 非可換モノイド（文字列の連結）の平方分割。
        for (int round = 0; round < 200; ++round) {
            std::vector<std::string> values(rng.range(0, 30));
            for (auto& s : values) s = std::string(1, char('a' + rng.range(0, 25)));
            cp::SqrtDecomposition<std::string, concatenate, empty_string> sqrt(values);
            for (int step = 0; step < 50; ++step) {
                if (!values.empty() && rng.range(0, 1)) {
                    const int p = static_cast<int>(rng.range(0, values.size() - 1));
                    values[p] = std::string(1, char('a' + rng.range(0, 25)));
                    sqrt.set(p, values[p]);
                }
                const int l = static_cast<int>(rng.range(0, values.size())), r = static_cast<int>(rng.range(l, values.size()));
                std::string expected;
                for (int i = l; i < r; ++i) expected += values[i];
                REQUIRE(sqrt.prod(l, r) == expected);
            }
        }
    }
}

void integers(Rng& rng) {
    const auto power_fits = [](std::uint64_t x, unsigned k, std::uint64_t n) {
        __uint128_t p = 1;
        for (unsigned i = 0; i < k; ++i) { p *= x; if (p > n) return false; }
        return true;
    };
    for (unsigned k = 1; k < 70; ++k)
        for (int round = 0; round < 200; ++round) {
            std::uint64_t n = rng.next() >> rng.range(0, 63);
            if (round < 3) n = round == 0 ? UINT64_MAX : round == 1 ? 0 : 1;
            const auto x = cp::integer_kth_root(n, k);
            REQUIRE(power_fits(x, k, n) && (x == UINT64_MAX || !power_fits(x + 1, k, n)));
        }
    for (int round = 0; round < 300; ++round) {
        const std::uint64_t l = rng.range(0, 200), r = l + rng.range(0, 200), step = rng.range(1, 30);
        std::vector<std::uint64_t> got, expected;
        cp::for_each_multiple(l, r, step, [&](std::uint64_t x) { got.push_back(x); });
        for (auto x = l; x < r; ++x) if (x % step == 0) expected.push_back(x);
        REQUIRE(got == expected);
        const std::uint64_t n = rng.range(0, 5000);
        std::uint64_t next = 1;
        cp::for_each_quotient(n, [&](std::uint64_t a, std::uint64_t b, std::uint64_t q) {
            REQUIRE(a == next && a <= b && n / a == q && n / b == q && (b == n || n / (b + 1) != q));
            next = b + 1;
        });
        REQUIRE(next == n + 1);
    }
    std::vector<std::uint64_t> top;
    cp::for_each_multiple(UINT64_MAX - 10, UINT64_MAX, 3, [&](std::uint64_t x) { top.push_back(x); });
    REQUIRE(top.size() == 3 || top.size() == 4);
    for (auto x : top) REQUIRE(x % 3 == 0 && x >= UINT64_MAX - 10);
    for (int round = 0; round < 200; ++round) { // 安定な基数ソート
        std::vector<std::pair<long long, int>> values(rng.range(0, 300));
        for (std::size_t i = 0; i < values.size(); ++i) values[i] = {rng.range(-5, 5) * (round % 2 ? 1 : 1000000007LL), int(i)};
        auto expected = values;
        std::stable_sort(expected.begin(), expected.end(), [](auto& a, auto& b) { return a.first < b.first; });
        cp::radix_sort_by(values, [](const auto& v) { return v.first; });
        REQUIRE(values == expected);
    }
}
} // namespace

int main() {
    Rng rng{20261123};
    global_modints();
#if defined(__unix__)
    scanner_does_not_wait_for_full_buffer();
#endif
    fibonacci_search(rng);
    fractional_cascading(rng);
    two_pointers_and_subarray(rng);
    meet_in_the_middle(rng);
    special_integers(rng);
    combinatorial_enumeration();
    permutation_tree(rng);
    fractions(rng);
    searches_and_structures(rng);
    integers(rng);
    std::cout << "basic_utility random tests passed\n";
}
