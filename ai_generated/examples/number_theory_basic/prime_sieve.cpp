#include <iostream>
#include "number_theory_basic/prime_sieve.hpp"
int main() {
    cp::PrimeSieve sieve(20);
    for (int p : sieve.primes()) std::cout << p << ' ';
    std::cout << '\n' << sieve.is_prime(1) << ' ' << sieve.is_prime(19) << '\n';
}
