/*
★ Name        ：素数判定 (O(sqrt N))
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc149/submissions/44846118
 ■ References  :

 ■ TODO        :
*/


#pragma once


using namespace std;


/// is N prime or not
// O(sqrtN)
bool is_prime(long long N) {
    for (long long i = 2; i * i <= N; i++) {
        if (N % i == 0) return false;
    }
    return true;
}