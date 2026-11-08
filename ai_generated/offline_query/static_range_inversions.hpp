#pragma once
#include <cassert>
#include <utility>
#include <vector>
#include "mo.hpp"
#include "../ordered_structure/coordinate_compression.hpp"
#include "../range_query/fenwick_tree.hpp"
namespace cp {
// 質問をまとめて受け取り、入力順の狭義転倒数を返す。Mo + Fenwick を共有。
template<class T,class Compare=std::less<T>>
std::vector<long long> static_range_inversions(const std::vector<T>& values,
        const std::vector<std::pair<int,int>>& queries,Compare compare=Compare{},int block_size=0) {
    CoordinateCompression<T,Compare> compression(values,compare);
    int n=int(values.size()); std::vector<int> a(n);
    for(int i=0;i<n;++i) a[i]=*compression.index(values[i]);
    Mo mo(n); for(auto [l,r]:queries) mo.add_query(l,r);
    FenwickTree<int> frequency(compression.size());
    int length=0; long long inversions=0; std::vector<long long> answer(queries.size());
    mo.run([&](int i){inversions+=frequency.sum(a[i]);frequency.add(a[i],1);++length;},
           [&](int i){inversions+=length-frequency.sum(a[i]+1);frequency.add(a[i],1);++length;},
           [&](int i){frequency.add(a[i],-1);--length;inversions-=frequency.sum(a[i]);},
           [&](int i){frequency.add(a[i],-1);--length;inversions-=length-frequency.sum(a[i]+1);},
           [&](int id){answer[id]=inversions;},block_size);
    return answer;
}
} // namespace cp
