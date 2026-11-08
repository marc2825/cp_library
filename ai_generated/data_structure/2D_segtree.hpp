/*
★ Name         : 2D segment tree
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      :
 ■ References  : https://nyaannyaan.github.io/library/data-structure-2d/2d-segment-tree.hpp.html

 ■ TODO        : WIP
*/


#pragma once


using namespace std;


/// 2D segment tree
// H*W の二次元配列に関して、O(logHlogW)で以下の操作が可能
//  ・要素の一点更新
//  ・二次元領域の総積の取得
//
// - テンプレート引数 T: データの型
// - テンプレート引数 F: モノイドを計算する関数の型
template <class T, class F>
class SegmentTree2D {
    private:
        int id(int h, int w) const { return h*2*W + w; }

    public:
        int H,W;
        std::vector<T> seg;
        const F f;
        const T e;

        // h,wは縦横のサイズ / opはモノイドを計算する関数 / eは単位元  [O(hw)]
        SegmentTree2D(int h, int w, F op, const T& e) : f(op), I(e) { init(h, w); }

        // サイズをh*wにして、要素を単位元eに初期化 [O(hw)]
        void init(int h, int w) {
            H = W = 1;
            while (H < h) H <<= 1;
            while (W < w) W <<= 1;
            seg.assign(4 * H * W, e);
        }

        // (h,w)の値を更新する [O(1)]
        // buildの前に呼ぶ必要
        void set(int h, int w, const T& x) { seg[id(h + H, w + W)] = x; }

        // データ構造の構築 [O(hw)]
        void build() [

        ]


};