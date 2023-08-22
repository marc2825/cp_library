/*
★ Name        ：Convex Hull Trick
 ■ Snipet      : No
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description : https://satanic0258.hatenablog.com/entry/2016/08/16/181331 と 蟻本 がわかりやすい
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc289/submissions/44832380 (maximize/non-monotonic-add/general-query)
                 https://yukicoder.me/submissions/906759
                 https://atcoder.jp/contests/abc289/submissions/44822693 (maximize/monotonic-add/general-query)
                 https://atcoder.jp/contests/abc289/submissions/44823314 (maximize/non-monotonic-add/monotonic-query)
                 https://atcoder.jp/contests/abc289/submissions/44823110 (maximize/monotonic-add/monotonic-query)
                 https://atcoder.jp/contests/dp/submissions/44832419 (minimize/non-monotonic-add/general-query)


 ■ References  : https://github.com/kth-competitive-programming/kactl/blob/main/content/data-structures/ConvexHullTrick.h
                 https://atcoder.jp/contests/abc289/submissions/38793756
                 https://satanic0258.hatenablog.com/entry/2016/08/16/181331
                 https://noshi91.hatenablog.com/entry/2021/03/23/200810 (オーバーフロー関連)

 ■ TODO        :
*/


#pragma once
#include <set>
#include <deque>
#include <limits>
#include <cmath>
#include <cassert>

using namespace std;


/// 直線 y = kx+m, p は不要な直線か？の判定 / クエリ処理における二分探索 に用いる値 (具体的には、傾き降順で次の直線との交点のx座標 = (y切片の差) / (傾きの差) )
template <typename T = long long>
struct Line {
    mutable T k, m, p; 
    bool operator<(const Line& o) const { return k < o.k; }
    bool operator<(T x) const { return p < x; }

    T get_value(T x) const {
        return k * x + m;
    }
};

/// div(a,b) = (for doubles, use inf = 1/.0, div(a,b) = a/b)
template <typename T = long long>
T lc_inf() {
    return numeric_limits<T>::max();
}
template <>
long double lc_inf<long double>() {
    return 1 / .0;
}

template <typename T = long long>
T lc_div(T a, T b) { // クエリが整数ならfloored division でよい, considered negative
    return a / b - (((a ^ b) < 0) && (a % b));
}
template <>
long double lc_div(long double a, long double b) {
    return a / b;
}
template <>
double lc_div(double a, double b) {
    return a / b;
}


/// Convex Hull Trick
// 直線群 kx+m を管理（追加可能）、任意のxでの最大値/最小値取得クエリをそれぞれ O(log N) で処理可能 (N = 直線の数)
// Minimize でクエリの種類（最小/最大取得）を指定、MonotonicAdd で 追加する直線の傾きの単調性（max->降順/min->昇順）が保証されているか（単調なら線形時間に出来る）を指定
// 何をしてるかは Reference[3] がわかりやすい（グラフをイメージすると更によい）
template <typename T, bool Minimize = true, bool MonotonicAdd = false>
class ConvexHullTrick : multiset<Line<T>, less<> > {
    private:
        using base = multiset<Line<T>, less<> >;
        using base::begin, base::end, base::insert, base::erase, base::rbegin, base::rend;
        using base::empty, base::lower_bound;
        const T inf = lc_inf<T>();
        deque<Line<T> > deq;

        /// 直線bが不要かどうかを判定（要するに直線bが凸包を構成しているかを判定しており、これで交点pのx座標の単調性が保たれる）
        bool noneed(const Line<T> &a, const Line<T> &b, const Line<T> &c) { // TODO : オーバーフロー関連
            if((c.k != b.k && abs(b.m - a.m) >= inf / abs(c.k - b.k)) || (c.m != b.m && abs(b.k - a.k) >= inf / abs(c.m - b.m))) return lc_div(b.m - a.m, a.k - b.k) >= lc_div(c.m - b.m, b.k - c.k);
            else return ((b.m - a.m) * (c.k - b.k)) >= ((b.k - a.k) * (c.m - b.m));
        }

        /// 直線a,bとの交点のx座標(a->p)を計算してから、直線bが不要かどうかを判定
        bool calc_intersect_noneed(typename base::iterator a, typename base::iterator b) {
            if (b == end()) {
                a->p = inf;
                return false;
            }
            if (a->k == b->k) a->p = (a->m > b->m ? inf : -inf);
            else a->p = lc_div(b->m - a->m, a->k - b->k);
            return a->p >= b->p;
        }

        /// multiset -> deque に格納（単調クエリ用）
        void build_deque() {
            assert(!empty());
            auto it = rbegin();
            while(it != rend()) {
                deq.push_back(*it);
                it++;
            }
        }

    public:
        /// 直線 kx+m を追加（同時に不要な直線たちは削除される）
        void add(T k, T m) {
            if (Minimize) k = -k, m = -m;
            
            if(!MonotonicAdd) {
                auto cur = insert({k, m, 0});
                auto nxt = cur; nxt++;
                while (calc_intersect_noneed(cur, nxt)) nxt = erase(nxt);

                auto prev = cur;
                if (prev != begin() && calc_intersect_noneed(--prev, cur)) {
                    cur = erase(cur);
                    calc_intersect_noneed(prev, cur);
                }
                while ((cur = prev) != begin() && (--prev)->p >= cur->p) calc_intersect_noneed(prev, erase(cur));
            }
            else {
                Line<T> l = {k, m, 0};
                int sz = deq.size();
                while ((sz = deq.size()) >= 2 && noneed(deq[sz-2], deq[sz-1], l)) deq.pop_back();
                deq.push_back(l);
            }
        }

        /// 直線群の x における 最小値/最大値 を取得
        // O(log N)
        T query(T x) {
            T value;
            if (!MonotonicAdd) {
                assert(!empty());
                auto l = *lower_bound(x);
                value = l.get_value(x);
            }
            else {
                assert(!deq.empty());
                int ng = -1, ok = (int)deq.size() - 1;
                while(ok - ng > 1) {
                    int mid = (ok + ng) / 2;
                    if(deq[mid].get_value(x) <= deq[mid + 1].get_value(x)) ng = mid;
                    else ok = mid;
                }
                value = deq[ok].get_value(x);
            }
            return (Minimize ? -value : value);
        }

        /// xが(max->降順、min->昇順) という条件で、直線群の x における 最小値/最大値 を取得
        // O(N) 、破壊的操作、non-monotonic-addの場合はこの後に更に追加される場合に対応していない
        T query_monotonic(T x) {
            if(!MonotonicAdd && deq.empty()) build_deque();
            assert(!deq.empty());
            
            T value = deq.front().get_value(x);
            while (deq.size() >= 2 && (value <= (deq[1].get_value(x)))) {
                deq.pop_front();
                value = deq.front().get_value(x);
            }  
            return (Minimize ? -value : value);
        }
};

template <typename T>
using CHT_min = ConvexHullTrick<T, true, false>;
template <typename T>
using CHT_min_monotonicadd = ConvexHullTrick<T, true, true>;
template <typename T>
using CHT_max = ConvexHullTrick<T, false, false>;
template <typename T>
using CHT_max_monotonicadd = ConvexHullTrick<T, false, true>;