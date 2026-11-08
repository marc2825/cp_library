/*
★ Name        ：拡張ユークリッドの互除法
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      :
 ■ References  :

 ■ TODO        :
*/


#pragma once


using namespace std;


/// 拡張ユークリッドの互除法
// ax + by = gcd(a, b) を満たす (x, y) が格納される (返り値: a と b の最大公約数)
long long extGCD(long long a, long long b, long long &x, long long &y) {
    if (b == 0) {
        x = 1;
        y = 0;
        return a;
    }
    // a = a/b * b + a % b を上式に再帰的に適用
    long long d = extGCD(b, a%b, y, x);
    y -= a/b * x; 
    return d;
}
