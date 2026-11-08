/*
★ Name         : ヒストグラム中の最大長方形を monotonic stackを用いて O(N)求める
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9385662#1 (histogram)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9385739#1 (rectangle)
                 
 ■ References  : http://algorithms.blog55.fc2.com/blog-entry-133.html

 ■ TODO        :
*/


#pragma once
#include <vector>
#include <stack>
#include <cassert>


using namespace std;


/// ヒストグラム中の最大長方形の面積を求める
// monotonic stack を用いて O(N)
long long Largest_Rectangle_inHistogram(const vector<long long> &H) {
    int N = H.size();
    
    long long ans = 0;
    stack<pair<long long, int> > st; // (height, index) -> heightが狭義単調増加になるようにする -> 右端で弾かれる時に精算
    vector<long long> L(N); // indexに対して左端がどこになるか？
    st.push({-1,-1});
    for(int i=0; i<N; i++) {
        while(st.top().first >= H[i]) { // iで弾かれる分を計算
            long long area = (i-L[st.top().second]) * st.top().first;
            if(ans < area) ans = area;
            st.pop();
        }

        L[i] = st.top().second+1;
        st.push({H[i], i});
    }

    while(!st.empty()) { // 最後まで残った分
        auto [h1,i1] = st.top();
        st.pop();
        if(st.empty()) break;
        
        long long area = h1 * (N - L[i1]);
        if(ans < area) ans = area;
    }

    return ans;
}


/// 二次元領域H*W中の、0(false)のみを含む、最大長方形の面積を求める（1はブロック的なイメージ）
// 各要素について上に向かって 1 が何個連続しているか？を示すテーブルを作る
// 各行に対して、上述のヒストグラム中の最大長方形を求めれば良いので、計算量は O(HW)
long long Largest_Rectangle_inRectangle(const vector<vector<bool> > &B) {
    int H = B.size();
    assert(H > 0);
    int W = B[0].size();
    
    vector<vector<long long> > T(H, vector<long long>(W));
    for(int i=0; i<H; i++) {
        for(int j=0; j<W; j++) {
            if(i == 0) T[i][j] = !B[i][j];
            else {
                if(B[i][j]) T[i][j] = 0;
                else T[i][j] = T[i-1][j] + 1;
            }
        }
    }

    long long ans = 0;
    for(int i=0; i<H; i++) {
        long long area = Largest_Rectangle_inHistogram(T[i]);
        if(ans < area) ans = area;
    }

    return ans;
}