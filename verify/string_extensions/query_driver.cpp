#include "string_algorithm/fm_index.hpp"
#include "string_algorithm/online_z.hpp"
#include "string_algorithm/subsequence_automaton.hpp"
#include "string_algorithm/suffix_automaton_utilities.hpp"
#include <iostream>
std::string unhex(const std::string &hex) {
    std::string out;
    if (hex == "-")
        return out;
    for (std::size_t i = 0; i < hex.size(); i += 2)
        out += static_cast<char>(std::stoi(hex.substr(i, 2), nullptr, 16));
    return out;
}
int main() {
    int queries;
    std::cin >> queries;
    while (queries--) {
        std::string s, p;
        int rate, position;
        std::uint64_t modulus;
        std::cin >> s >> p >> rate >> modulus >> position;
        s = unhex(s);
        p = unhex(p);
        cp::FMIndex fm(s, rate);
        cp::SuffixAutomatonUtilities sam(s);
        cp::OnlineZ z(s);
        std::cout << fm.count(p) << ' ' << sam.distinct_substrings() << ' '
                  << cp::distinct_subsequences(s, modulus) << ' ' << z.query(position);
        for (int i : fm.locate(p))
            std::cout << ' ' << i;
        std::cout << '\n';
    }
}
