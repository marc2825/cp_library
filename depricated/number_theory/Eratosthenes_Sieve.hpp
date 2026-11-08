/*
★ Name        ：Sieve of Eratosthenes
 ■ Snipet      : Yes
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc084/submissions/44844032
                 https://judge.yosupo.jp/submission/156893
                 https://atcoder.jp/contests/abc300/submissions/44848778
                 https://atcoder.jp/contests/abc250/submissions/44848985
 ■ References  : https://uwitenpen.hatenadiary.org/entries/2011/12/03
                 https://qiita.com/peria/items/a4ff4ddb3336f7b81d50
 
 ■ TODO        : エラトステネスの定数倍高速化（特に2,3,5の倍数を予め弾くやつ、bitとして持つやつ）、Atkinの篩や線形篩、区間篩等に関して
*/


#pragma once
#include <vector>

using namespace std;


/// エラトステネスの篩
// N 以下の整数が素数かどうかのテーブルを返す
// O(NloglogN)
vector<bool> Eratosthenes(int N) {
    vector<bool> isprime(N+1, true);

    isprime[0] = false;
    isprime[1] = false;

    for (long long p = 2; p <= N; ++p) {
        if (!isprime[p]) continue;

        if(p * p > N) break;

        for (long long q = p * p; q <= N; q += p) {
            isprime[q] = false;
        }
    }

    return isprime;
}