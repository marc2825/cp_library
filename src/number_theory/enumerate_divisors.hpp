/*
★ Name        ：約数列挙 (O(sqrt N))
 ■ Snipet      : Yes
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/arc026/submissions/44844789
                 https://atcoder.jp/contests/arc108/submissions/44844868
 ■ References  :

 ■ TODO        :
*/


#pragma once
#include <vector>
#include <algorithm>

using namespace std;


/// 約数列挙
// O(sqrt N)
// Nの約数全てを昇順に出力する
vector<long long> enum_divisors(long long N, const bool do_sort = true) {
    vector<long long> ret;

    for (long long i = 1; i * i <= N; i++) {
        if (N % i != 0) continue;
        ret.push_back(i);
        if (i * i != N) ret.push_back(N / i);
    }

    if (do_sort) sort(ret.begin(), ret.end());
    return ret;
}
