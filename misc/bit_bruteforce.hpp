/*
★ Name         : bit全探索
 ■ Snipet      : Yes
 
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

    auto bit_bruteforce = [&](int N) -> void{

        for(int i = 0; i < (1 << N); i++) {
            
            for(int j = 0; j < N; j++) {
                if((i >> j) & 1) {


                }
            }
        }
    };