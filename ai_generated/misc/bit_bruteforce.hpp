/*
★ Name         : bit全探索
 ■ Snipet      : Yes
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/arc029/tasks/arc029_1
                 https://atcoder.jp/contests/arc061/submissions/45204049
 ■ References  :

 ■ TODO        :
*/


#pragma once


using namespace std;

    auto bit_bruteforce = [&](int N) -> void{

        for(int i = 0; i < (1 << N); i++) {
            
            for(int j = 0; j < N; j++) {
                if((i >> j) & 1) {


                }
            }
        }
    };