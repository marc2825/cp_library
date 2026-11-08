// PROBLEM https://judge.yosupo.jp/problem/associative_array
#include <cstdint>
#include "basic_utility/fast_io.hpp"
#include "basic_utility/hash_map.hpp"
int main(){
    cp::FastScanner in;cp::FastPrinter out;
    cp::HashMap<std::uint64_t,std::uint64_t> values(cp::SeededHash(0));
    int q=0;in.read(q);
    while(q--){int op=0;std::uint64_t key=0,value=0;in.read(op,key);
        if(op==0){in.read(value);values.insert_or_assign(key,value);}
        else {const auto* p=values.find(key);out.writeln(p?*p:0ULL);}
    }
    out.flush();
}
