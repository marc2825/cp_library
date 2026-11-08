#include "linear_recurrence/holonomic.hpp"
#include "linear_recurrence/recurrence_slice.hpp"
#include "linear_recurrence/reeds_sloane.hpp"
#include "set_function/intersection_convolution.hpp"
#include "set_function/online_subset_convolution.hpp"
#include "set_function/sps_composition.hpp"
#include "set_function/sps_power_projection.hpp"
#include "test_common.hpp"
std::vector<int> naive_subset(const std::vector<int> &a, const std::vector<int> &b, int modulus = prime) {
    std::vector<int> c(a.size());
    for (std::size_t s = 0; s < a.size(); ++s) {
        auto t = s;
        for (;;) {
            c[s] = norm(c[s] + static_cast<long long>(a[t]) * b[s ^ t], modulus);
            if (!t)
                break;
            t = (t - 1) & s;
        }
    }
    return c;
}
bool recurrence_valid(const std::vector<int> &s, const std::vector<int> &c, int m) {
    for (std::size_t i = c.size(); i < s.size(); ++i) {
        int value = 0;
        for (std::size_t j = 0; j < c.size(); ++j)
            value = norm(value + static_cast<long long>(c[j]) * s[i - 1 - j], m);
        if (value != norm(s[i], m))
            return false;
    }
    return true;
}
void exhaustive_rs(int modulus, int length) {
    std::size_t count = 1;
    for (int i = 0; i < length; ++i)
        count *= modulus;
    for (std::size_t mask = 0; mask < count; ++mask) {
        auto value = mask;
        std::vector<int> s(length);
        for (auto &x : s) {
            x = value % modulus;
            value /= modulus;
        }
        auto c = cp::reeds_sloane(s, modulus);
        CHECK(recurrence_valid(s, c, modulus));
        std::size_t choices = 1;
        for (std::size_t d = 0; d < c.size(); ++d) {
            for (std::size_t code = 0; code < choices; ++code) {
                auto t = code;
                std::vector<int> candidate(d);
                for (auto &x : candidate) {
                    x = t % modulus;
                    t /= modulus;
                }
                CHECK(!recurrence_valid(s, candidate, modulus));
            }
            choices *= modulus;
        }
    }
    std::cout << "Reeds-Sloane mod " << modulus << ": " << count << " exhaustive minimality checks\n";
}
int main() {
    exhaustive_rs(4, 5);
    exhaustive_rs(6, 4);
    exhaustive_rs(8, 4);
    exhaustive_rs(9, 4);
    CHECK(cp::reeds_sloane({6, 3, 1, 5, 6}, 9) == std::vector<int>({5, 2, 8}));
    CHECK(cp::reeds_sloane({1, 0, 0}, 4).size() == 1);
    CHECK(cp::reeds_sloane({}, 12).empty());
    CHECK(cp::reeds_sloane({4, 8}, 1).empty());
    for (int t = 0; t < 80; ++t) {
        auto d = 1 + random_engine() % 15;
        auto c = random_poly(d), initial = random_poly(d);
        auto sequence = initial;
        for (int i = 0; i < 300; ++i) {
            int v = 0;
            for (std::size_t j = 0; j < d; ++j)
                v = norm(v + static_cast<long long>(c[j]) * sequence[sequence.size() - 1 - j]);
            sequence.push_back(v);
        }
        auto start = random_engine() % 100, count = random_engine() % 200;
        auto slice = cp::linear_recurrence_slice(initial, c, start, count);
        CHECK(slice == std::vector<int>(sequence.begin() + start, sequence.begin() + start + count));
        auto huge = random_engine() % UINT64_C(1000000000000000000);
        slice = cp::linear_recurrence_slice(initial, c, huge, 30);
        for (std::size_t i = 0; i < slice.size(); ++i)
            CHECK(slice[i] == cp::kitamasa(initial, c, huge + i));
        int modulus = 2 + random_engine() % 100;
        auto ss = initial;
        for (auto &x : ss)
            x %= modulus;
        for (int i = 0; i < 80; ++i) {
            int v = 0;
            for (std::size_t j = 0; j < d; ++j)
                v = norm(v + static_cast<long long>(c[j] % modulus) * ss[ss.size() - 1 - j], modulus);
            ss.push_back(v);
        }
        auto feedback = cp::reeds_sloane(ss, modulus);
        CHECK(recurrence_valid(ss, feedback, modulus));
        CHECK(feedback.size() <= d);
    }
    std::vector<int> factorial(40, 1);
    for (std::size_t i = 1; i < factorial.size(); ++i)
        factorial[i] = norm(static_cast<long long>(factorial[i - 1]) * i);
    auto inferred = cp::infer_holonomic(factorial, 1, 1);
    CHECK(inferred);
    auto full = cp::p_recursive_prefix({1}, *inferred, 200);
    for (std::size_t i = 0; i < full.size(); ++i)
        CHECK(cp::holonomic_nth({1}, *inferred, i) == full[i]);
    int expected = 1;
    for (int i = 1; i <= 100000; ++i)
        expected = norm(static_cast<long long>(expected) * i);
    CHECK(cp::holonomic_nth({1}, {{-1, -1}, {1}}, 100000) == expected);
    for (int bits = 0; bits <= 8; ++bits) {
        std::size_t size = std::size_t(1) << bits;
        auto a = random_poly(size), b = random_poly(size);
        auto c = naive_subset(a, b);
        cp::OnlineSubsetConvolution<> online(bits);
        for (std::size_t s = 0; s < size; ++s)
            CHECK(online.push(a[s], b[s]) == c[s]);
        rejects<std::length_error>([&] { online.push(0, 0); });
        cp::OnlineSubsetConvolution<> adaptive(bits);
        int last = 0;
        for (std::size_t s = 0; s < size; ++s) {
            a[s] = norm(last + 11);
            b[s] = norm(static_cast<long long>(last) * 3 + 7);
            int want = 0;
            auto t = s;
            for (;;) {
                want = norm(want + static_cast<long long>(a[t]) * b[s ^ t]);
                if (!t)
                    break;
                t = (t - 1) & s;
            }
            last = adaptive.push(a[s], b[s]);
            CHECK(last == want);
        }
        auto moments = cp::sps_power_projection(a, b, bits + 5);
        std::vector<int> power(size);
        power[0] = 1;
        for (int i = 0; i < bits + 5; ++i) {
            CHECK(moments[i] == dot(a, power));
            power = naive_subset(power, b);
        }
        auto intersection = cp::intersection_size_convolution(a, b);
        std::vector<std::vector<int>> want(bits + 1, std::vector<int>(size));
        for (std::size_t i = 0; i < size; ++i)
            for (std::size_t j = 0; j < size; ++j) {
                int k = __builtin_popcountll(i & j);
                want[k][i | j] = norm(want[k][i | j] + static_cast<long long>(a[i]) * b[j]);
            }
        CHECK(intersection == want);
        CHECK(intersection[0] == naive_subset(a, b));
    }
    CHECK(cp::sps_power_projection<2>({1, 1, 1, 1}, {1, 1, 1, 1}, 6) == std::vector<int>({1, 0, 1, 0, 1, 0}));
    rejects<std::invalid_argument>([] { cp::reeds_sloane({1}, 0); });
    rejects<std::invalid_argument>([] { cp::infer_holonomic({1, 2}, 2, 3); });
    rejects<std::length_error>([] { cp::linear_recurrence_slice({1}, {1}, UINT64_MAX, 2); });
    auto many = cp::linear_recurrence_slice({0, 1}, {1, 1}, UINT64_C(1000000000000), 10000);
    for (std::size_t i = 2; i < many.size(); ++i)
        CHECK(many[i] == norm(many[i - 1] + many[i - 2]));
    cp::OnlineSubsetConvolution<> large(12);
    for (int i = 0; i < (1 << 12); ++i)
        CHECK(large.push(1, 1) == (1 << __builtin_popcount(unsigned(i))));
    std::cout << "sequences/sets: composite-ring minimality, large-index blocks, holonomic inference and "
                 "adaptive online streams passed\n";
}
