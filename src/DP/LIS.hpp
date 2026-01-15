/*
★ Name         : LIS (最長増加部分列)
 ■ Snipet      : No
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/tessoku-book/submissions/45203191 (狭義単調増加)
                 https://atcoder.jp/contests/arc149/submissions/45203524 (〃)
                 https://atcoder.jp/contests/chokudai_S002/submissions/45203697 (〃)
                 https://atcoder.jp/contests/abc038/submissions/45203716 (〃)
                 https://atcoder.jp/contests/abc134/submissions/45203339 (広義単調減少)
 ■ References  :

 ■ TODO        :
*/


#pragma once
#include <vector>
#include <algorithm>

using namespace std;


/// LISの長さを(in-place DPで)求める 
// strictly : 狭義/広義, increase : 増加/減少
// O(NlogN)
template< typename T >
int LIS(vector< T > &A, bool strictly = true, bool increase = true) {
    int N = A.size();
    if(!increase) for(int i = 0; i < N; i++) A[i] *= -1;
    vector< T > ret;

    for(int i = 0; i < N; i++) {
        typename decltype(ret)::iterator it;
        if(strictly) it = lower_bound(ret.begin(), ret.end(), A[i]);
        else it = upper_bound(ret.begin(), ret.end(), A[i]);
        
        if(it == ret.end()) ret.push_back(A[i]);
        else *it = A[i];
    }

    if(!increase) for(auto &x : ret) x *= -1;
    return (int)ret.size();
}