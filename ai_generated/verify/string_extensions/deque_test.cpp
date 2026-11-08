#include "string_algorithm/double_ended_eertree.hpp"
#include "test_common.hpp"
void check(const cp::DoubleEndedEertree &e, const std::string &s) {
    CHECK(e.text() == s);
    CHECK(e.size() == int(s.size()));
    std::set<std::string> distinct;
    std::uint64_t total = 0;
    int prefix = 0, suffix = 0;
    for (int l = 0; l < int(s.size()); ++l)
        for (int r = l + 1; r <= int(s.size()); ++r) {
            auto p = s.substr(l, r - l), q = p;
            std::reverse(q.begin(), q.end());
            if (p == q) {
                distinct.insert(p);
                ++total;
                if (!l)
                    prefix = std::max(prefix, r);
                if (r == int(s.size()))
                    suffix = std::max(suffix, r - l);
            }
        }
    CHECK(e.distinct_count() == int(distinct.size()));
    CHECK(e.total_occurrences() == total);
    CHECK(e.longest_prefix() == prefix);
    CHECK(e.longest_suffix() == suffix);
}
int main() {
    cp::DoubleEndedEertree e;
    std::string s;
    for (int t = 0; t < 30000; ++t) {
        int op = random_s() % 4;
        if (s.empty())
            op %= 2;
        if (s.size() > 32)
            op = 2 + random_s() % 2;
        const auto c = static_cast<unsigned char>(random_s() % 5);
        if (op == 0) {
            e.push_back(c);
            s += char(c);
        }
        if (op == 1) {
            e.push_front(c);
            s.insert(s.begin(), char(c));
        }
        if (op == 2) {
            CHECK(e.pop_back() == static_cast<unsigned char>(s.back()));
            s.pop_back();
        }
        if (op == 3) {
            CHECK(e.pop_front() == static_cast<unsigned char>(s.front()));
            s.erase(s.begin());
        }
        check(e, s);
    }
    for (const auto &w : words(7, 2))
        for (int pattern = 0; pattern < 4; ++pattern) {
            e.clear();
            s.clear();
            for (unsigned char c : w) {
                if (pattern & 1) {
                    e.push_front(c);
                    s.insert(s.begin(), char(c));
                } else {
                    e.push_back(c);
                    s += char(c);
                }
                check(e, s);
            }
            while (!s.empty()) {
                if (pattern & 2) {
                    e.pop_front();
                    s.erase(s.begin());
                } else {
                    e.pop_back();
                    s.pop_back();
                }
                check(e, s);
            }
        }
    e.clear();
    for (int i = 1; i <= 100000; ++i) {
        if (i & 1)
            e.push_back(255);
        else
            e.push_front(255);
        CHECK(e.distinct_count() == i);
        CHECK(e.total_occurrences() == std::uint64_t(i) * (i + 1) / 2);
        CHECK(e.longest_prefix() == i);
    }
    for (int i = 99999; i >= 0; --i) {
        if (i & 1)
            e.pop_back();
        else
            e.pop_front();
        CHECK(e.distinct_count() == i);
        CHECK(e.total_occurrences() == std::uint64_t(i) * (i + 1) / 2);
        CHECK(e.longest_suffix() == i);
    }
    for (int i = 0; i < 10000; ++i) {
        e.push_back(0);
        e.push_front(255);
        e.pop_back();
        e.pop_front();
    }
    CHECK(e.empty());
    throws<std::out_of_range>([&] { e.pop_back(); });
    throws<std::out_of_range>([&] { e.pop_front(); });
    e.clear();
    CHECK(!e.allocated_nodes());
    std::cout << "deque eertree: 30000 independent random steps, 1020 push/pop histories, 200000 adversarial "
                 "updates and 40000 reuse steps passed\n";
}
