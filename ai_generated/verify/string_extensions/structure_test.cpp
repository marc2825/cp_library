#include "static_query/static_range_lis.hpp"
#include "string_algorithm/border_tree.hpp"
#include "string_algorithm/online_z.hpp"
#include "string_algorithm/semi_local_lcs.hpp"
#include "string_algorithm/substring_period.hpp"
#include "test_common.hpp"
int main() {
    for (const auto &s : words(8, 2)) {
        cp::BorderTree tree;
        cp::OnlineZ z;
        cp::SubstringPeriodIndex index(s);
        for (std::size_t n = 0; n <= s.size(); ++n) {
            for (std::size_t i = 0; i <= n; ++i) {
                int length = 0;
                while (i + length < n && s[length] == s[i + length])
                    ++length;
                CHECK(z.query(i) == length);
            }
            for (int p = 0; p <= int(n); ++p) {
                std::vector<int> borders, periods;
                std::uint64_t count = 0;
                for (int k = 0; k <= p; ++k) {
                    bool ok = s.substr(0, k) == s.substr(p - k, k);
                    CHECK(tree.is_border(k, p) == ok);
                    if (ok && k > 0 && k < p)
                        borders.push_back(k);
                }
                for (int k = 1; k <= p; ++k)
                    if (s.substr(0, p - k) == s.substr(k, p - k))
                        periods.push_back(k);
                for (std::size_t i = 0; i + std::size_t(p) <= n; ++i)
                    if (s.substr(i, p) == s.substr(0, p))
                        ++count;
                CHECK(tree.borders(p) == borders);
                CHECK(tree.periods(p) == periods);
                CHECK(tree.prefix_occurrences()[p] == count);
            }
            if (n < s.size()) {
                tree.append(static_cast<unsigned char>(s[n]));
                z.append(static_cast<unsigned char>(s[n]));
            }
        }
        for (int l = 0; l <= int(s.size()); ++l)
            for (int r = l; r <= int(s.size()); ++r) {
                int root = 0;
                for (int p = 1; p <= r - l; ++p) {
                    bool ok = s.substr(l, r - l - p) == s.substr(l + p, r - l - p);
                    CHECK(index.is_period(l, r, p) == ok);
                    if (!root && ok && (r - l) % p == 0)
                        root = p;
                }
                CHECK(index.minimum_repetition_period(l, r) == root);
                CHECK(index.repetition_count(l, r) == (root ? (r - l) / root : 0));
            }
    }
    auto cases = words(4, 2);
    for (const auto &a : cases)
        for (const auto &b : cases) {
            cp::SemiLocalLCS semi(a, b);
            cp::PrefixSubstringLCS prefix(a, b);
            auto permutation = semi.seaweed();
            std::sort(permutation.begin(), permutation.end());
            for (std::size_t i = 0; i < permutation.size(); ++i)
                CHECK(permutation[i] == int(i) - int(a.size()));
            for (int p = 0; p <= int(a.size()); ++p)
                for (int l = 0; l <= int(b.size()); ++l)
                    for (int r = l; r <= int(b.size()); ++r)
                        CHECK(prefix.query(p, l, r) == lcs(a.substr(0, p), b.substr(l, r - l)));
            for (int l = 0; l <= int(b.size()); ++l)
                for (int r = l; r <= int(b.size()); ++r)
                    CHECK(semi.string_substring(l, r) == lcs(a, b.substr(l, r - l)));
            for (int l = 0; l <= int(a.size()); ++l)
                for (int r = l; r <= int(a.size()); ++r)
                    CHECK(semi.substring_string(l, r) == lcs(a.substr(l, r - l), b));
            for (int p = 0; p <= int(a.size()); ++p)
                for (int q = 0; q <= int(b.size()); ++q) {
                    CHECK(semi.prefix_suffix(p, q) == lcs(a.substr(0, p), b.substr(q)));
                    CHECK(semi.suffix_prefix(p, q) == lcs(a.substr(p), b.substr(0, q)));
                }
        }
    for (int t = 0; t < 200; ++t) {
        std::vector<int> a(random_s() % 20);
        for (int &x : a)
            x = random_s() % 10;
        cp::StaticRangeLIS<int> lis(a);
        for (int l = 0; l <= int(a.size()); ++l)
            for (int r = l; r <= int(a.size()); ++r) {
                std::vector<int> dp;
                for (int i = l; i < r; ++i) {
                    auto p = std::lower_bound(dp.begin(), dp.end(), a[i]);
                    if (p == dp.end())
                        dp.push_back(a[i]);
                    else
                        *p = a[i];
                }
                CHECK(lis.query(l, r) == int(dp.size()));
            }
    }
    throws<std::out_of_range>([] { cp::OnlineZ("a").query(2); });
    throws<std::invalid_argument>([] { cp::SubstringPeriodIndex("a").is_period(0, 1, 0); });
    throws<std::out_of_range>([] { cp::SemiLocalLCS("a", "b").substring_string(0, 2); });
    throws<std::length_error>([] { cp::PrefixSubstringLCS("aa", "bb", 5); });
    cp::BorderTree self("abc");
    self.append(self.text());
    CHECK(self.text() == "abcabc" && self.parent(6) == 3);
    cp::OnlineZ many;
    for (int i = 0; i < 200000; ++i)
        many.append('a');
    for (int i = 0; i < 200000; i += 997)
        CHECK(many.query(i) == 200000 - i);
    std::cout << "string structure: 511 online/border/period texts, 961 all semi-local LCS pairs, 200 shared "
                 "range-LIS cases, 200000 online symbols passed\n";
}
