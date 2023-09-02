/*
★ Name         : 順列全探索
 ■ Snipet      : Yes
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc183/submissions/45203934
                 https://atcoder.jp/contests/abc317/submissions/45204121
 ■ References  :

 ■ TODO        :
*/


#pragma once
#include <vector>
#include <algorithm>


using namespace std;


    auto perm_bruteforce = [&](int N) -> void{
        vector<int> perm(N);
        for(int i = 0; i < N; i++) perm[i] = i;

        do {


        } while (next_permutation(perm.begin(), perm.end()));

    };