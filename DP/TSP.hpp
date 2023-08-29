/*
★ Name         : DP used for Travel Salesman Problem ( Permutation to power2 O(N!) -> O(N^2*2^N) )
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


    auto TSP = [&](int N) -> void{
        for(int i = 0; i < (1 << N); i++) {
            for(int j = 0; j < N; j++) {
                if((i >> j) & 1) {
                    for(int k = 0; k < N; k++) {
                        if(((i >> k) & 1) == 0) {
                            chmin(dp[i | (1 << k)][k], dp[i][j] + cost[j][k]);
                        }
                    }
                }
            }
        }
    };

    
