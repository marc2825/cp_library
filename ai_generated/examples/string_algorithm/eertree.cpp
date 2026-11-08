#include <iostream>
#include "string_algorithm/eertree.hpp"
int main() {
    cp::Eertree t("ababa"); auto counts = t.occurrence_counts();
    std::cout << t.distinct_count() << ' ' << t.nodes()[t.longest_suffix_node()].length << ' ' << counts.back() << '\n';
}
