#include "parsing/automata.hpp"
#include <iostream>
using namespace cp;
int main() {
    NFA nfa(3, 2);
    nfa.set_accepting(2);
    for (int c : {0, 1}) {
        nfa.add_transition(0, c, 0);
        nfa.add_transition(2, c, 2);
    }
    nfa.add_transition(0, 0, 1);
    nfa.add_transition(1, 1, 2);
    auto dfa = determinize(nfa);
    auto minimal = minimize_dfa(dfa.automaton);
    std::cout << dfa.automaton.size() << " " << minimal.automaton.size() << "\n";
    const std::vector<std::vector<int>> words{{}, {0}, {0, 1}, {1, 0, 0}, {1, 1, 0, 1, 1}};
    for (std::size_t i = 0; i < words.size(); ++i)
        std::cout << (i ? " " : "") << minimal.automaton.accepts(words[i]);
    std::cout << "\n";
}
