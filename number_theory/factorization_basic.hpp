/*
★ Name        ：素因数分解 基本形 (O(sqrtN))
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc052/submissions/44845872
                 https://atcoder.jp/contests/caddi2018/submissions/44845995
                 https://atcoder.jp/contests/abc280/submissions/44849234
                 https://atcoder.jp/contests/arc034/submissions/44849656
 ■ References  :

 ■ TODO        :
*/


#pragma once
#include <vector>


using namespace std;


/// N を素因数分解する
// O(sqrtN), より高速な方法は他にもある
// (素因数、次数)の配列を返す
vector<pair<long long, int> > factorization(long long N) {
    vector<pair<long long, int> > ret;
    for (long long i = 2; i * i <= N; i++) {
        int cnt = 0;
        while (N % i == 0) {
            cnt++;
            N /= i;
        }
        if (cnt > 0) ret.emplace_back(i, cnt);
    }
    if (N != 1) ret.emplace_back(N, 1);
    return ret;
}