#include "basic_utility/big_integer.hpp"
#include "string_algorithm/burrows_wheeler.hpp"
#include "string_algorithm/fm_index.hpp"
#include "string_algorithm/generalized_suffix_automaton.hpp"
#include "string_algorithm/lempel_ziv.hpp"
#include "string_algorithm/subsequence_automaton.hpp"
#include "string_algorithm/suffix_automaton_utilities.hpp"
#include "test_common.hpp"
void check_text(const std::string &s) {
    cp::SubsequenceAutomaton sub(s);
    std::set<std::string> subsequences;
    if (s.size() < 12) {
        for (unsigned mask = 0; mask < (1U << s.size()); ++mask) {
            std::string t;
            for (std::size_t j = 0; j < s.size(); ++j)
                if (mask >> j & 1)
                    t += s[j];
            subsequences.insert(t);
        }
        for (auto m : {1ULL, 2ULL, 6ULL, 998244353ULL, 18446744073709551615ULL})
            for (bool empty : {false, true})
                CHECK(cp::distinct_subsequences(s, m, empty) == (subsequences.size() - !empty) % m);
    }
    for (auto p : words(3, 3)) {
        std::vector<int> want;
        std::size_t at = 0;
        for (char c : p) {
            while (at < s.size() && s[at] != c)
                ++at;
            if (at == s.size())
                break;
            want.push_back(static_cast<int>(at++));
        }
        auto got = sub.match(p);
        CHECK(bool(got) == (want.size() == p.size()));
        if (got)
            CHECK(*got == want);
    }
    const auto factors = cp::lempel_ziv_factorization(s);
    CHECK(cp::restore_lz77(factors) == s);
    int cursor = 0;
    for (auto f : factors) {
        int best = 0;
        for (int j = 0; j < cursor; ++j) {
            int k = 0;
            while (cursor + k < int(s.size()) && s[j + k] == s[cursor + k])
                ++k;
            best = std::max(best, k);
        }
        CHECK(f.length == std::max(1, best));
        CHECK((f.source == -1) == (best == 0));
        cursor += f.length;
    }
    auto b = cp::burrows_wheeler_transform(s);
    CHECK(cp::inverse_burrows_wheeler(b) == s);
    std::vector<int> text;
    for (unsigned char c : s)
        text.push_back(c + 1);
    text.push_back(0);
    std::vector<std::pair<std::vector<int>, int>> rotations;
    for (int i = 0; i < int(text.size()); ++i) {
        std::vector<int> r;
        for (int j = 0; j < int(text.size()); ++j)
            r.push_back(text[(i + j) % text.size()]);
        rotations.emplace_back(r, i);
    }
    std::sort(rotations.begin(), rotations.end());
    for (std::size_t i = 0; i < rotations.size(); ++i) {
        CHECK(b.last[i] == rotations[i].first.back());
        if (!rotations[i].second)
            CHECK(b.primary == int(i));
    }
    cp::FMIndex fm(s, 3);
    cp::SuffixAutomatonUtilities sam(s);
    std::set<std::string> patterns;
    patterns.insert("");
    patterns.insert(std::string(1, char(255)));
    for (std::size_t i = 0; i < s.size(); ++i)
        for (std::size_t j = i + 1; j <= s.size(); ++j)
            patterns.insert(s.substr(i, j - i));
    std::vector<std::string> distinct;
    for (const auto &p : patterns)
        if (!p.empty() && !occurrences(s, p).empty())
            distinct.push_back(p);
    CHECK(sam.distinct_substrings() == distinct.size());
    for (std::size_t k = 0; k < distinct.size(); ++k)
        CHECK(sam.kth_substring(k) == distinct[k]);
    CHECK(!sam.kth_substring(distinct.size()));
    for (const auto &p : patterns) {
        auto want = occurrences(s, p);
        CHECK(fm.locate(p) == want);
        CHECK(fm.count(p) == int(want.size()));
        CHECK(sam.occurrences(p) == want);
    }
    for (int rate : {1, 17, 1000000}) {
        cp::FMIndex other(s, rate);
        CHECK(other.locate("") == occurrences(s, ""));
        CHECK(other.locate(s) == occurrences(s, s));
    }
}
int main() {
    auto cases = words(7, 2);
    for (const auto &s : cases)
        check_text(s);
    for (int t = 0; t < 100; ++t) {
        std::string s(random_s() % 24, ' ');
        for (auto &c : s)
            c = static_cast<char>(random_s() % 256);
        check_text(s);
    }
    std::string alphabet;
    for (int i = 0; i < 256; ++i)
        alphabet += char(i);
    cp::BigInt count = 1;
    for (int i = 0; i < 256; ++i)
        count *= 2;
    for (auto m : {6ULL, 998244353ULL, 18446744073709551615ULL})
        CHECK(cp::distinct_subsequences(alphabet, m) == (count % cp::BigInt(m)).to_integral<std::uint64_t>());
    auto b = cp::burrows_wheeler_transform(alphabet);
    CHECK(cp::inverse_burrows_wheeler(b) == alphabet);
    cp::GeneralizedSuffixAutomaton general;
    CHECK(!general.contains(""));
    CHECK(general.count_occurrences("") == 0);
    std::vector<std::string> documents;
    for (int t = 0; t < 90; ++t) {
        std::string s(random_s() % 10, ' ');
        for (auto &c : s)
            c = char(random_s() % 4);
        documents.push_back(s);
        CHECK(general.add_document(s) == t);
        std::set<std::string> distinct;
        for (auto doc : documents)
            for (std::size_t i = 0; i < doc.size(); ++i)
                for (std::size_t j = i + 1; j <= doc.size(); ++j)
                    distinct.insert(doc.substr(i, j - i));
        CHECK(general.distinct_substrings() == distinct.size());
        std::size_t k = 0;
        for (const auto &p : distinct)
            CHECK(general.kth_substring(k++) == p);
        CHECK(!general.kth_substring(k));
        for (auto p : words(3, 4)) {
            std::vector<cp::GeneralizedSuffixAutomaton::Occurrence> want;
            std::vector<int> ids;
            for (int d = 0; d <= t; ++d) {
                auto positions = occurrences(documents[d], p);
                if (!positions.empty())
                    ids.push_back(d);
                for (int pos : positions)
                    want.push_back({d, pos});
            }
            CHECK(general.occurrences(p) == want);
            CHECK(general.count_occurrences(p) == want.size());
            CHECK(general.contains(p) == !want.empty());
            CHECK(general.containing_documents(p) == ids);
        }
    }

    cp::GeneralizedSuffixAutomaton binary(std::vector<std::string>{alphabet, std::string("\0\xff", 2), ""});
    CHECK(binary.count_occurrences(std::string("\0\xff", 2)) == 1);
    CHECK(!binary.contains(std::string("\xff\0", 2)));
    std::string repeated(100000, 'a');
    cp::SuffixAutomatonUtilities large_sam(repeated);
    CHECK(large_sam.distinct_substrings() == 100000);
    CHECK(large_sam.kth_substring(99999) == repeated);
    cp::FMIndex large_fm(repeated, 32);
    CHECK(large_fm.count("aaa") == 99998);
    auto located = large_fm.locate("aaa");
    CHECK(located.size() == 99998 && located.front() == 0 && located.back() == 99997);
    auto large_lz = cp::lempel_ziv_factorization(repeated);
    CHECK(large_lz.size() == 2 && large_lz[1].length == 99999);
    throws<std::invalid_argument>([] { cp::distinct_subsequences("", 0); });
    throws<std::out_of_range>([] { cp::SubsequenceAutomaton("x").next(2, 0); });
    throws<std::invalid_argument>([] { cp::restore_lz77({{0, 1, 0}}); });
    throws<std::length_error>([] { cp::restore_lz77({{-1, 1, 0}, {0, 10, 0}}, 5); });
    throws<std::invalid_argument>([] { cp::inverse_burrows_wheeler({{0, 1}, 0}); });
    throws<std::invalid_argument>([] { cp::inverse_burrows_wheeler({{257}, 0}); });
    throws<std::invalid_argument>([] { cp::FMIndex("x", 0); });
    std::cout << "string indexes: 355 exhaustive/byte texts, greedy LZ77, BWT rotations, FM sampling, "
                 "subsequences, 90 document updates passed\n";
}
