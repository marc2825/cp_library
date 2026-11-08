/*
★ Name         : Priority-Sum-Structure (スライド区間の昇順k個の和, priority_queue(multiset)2個管理するアレ)
 ■ Snipet      : No
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc306/submissions/47665313 (降順K個、MaximumSum)
                 https://atcoder.jp/contests/abc281/submissions/47665421 (昇順K個、MinimunSum)
 ■ References  : https://ei1333.github.io/luzhiled/snippets/structure/priority-sum-structure.html

 ■ TODO        :
*/

#include <queue>

/// 昇順（降順）K個の和クエリ（変更可能）を高速に取得するためのデータ構造
template< typename T, typename Compare = less< T >, typename RCompare = greater< T > >
struct PrioritySumStructure {
    size_t k; 
    T sum; // 昇順（降順）K個の和

    // d_in, out は遅延削除用のpriority_queue
    priority_queue< T, vector< T >, Compare > in, d_in; // 昇順(降順)K個に含まれる数の集合
    priority_queue< T, vector< T >, RCompare > out, d_out; // 含まれない数の集合

    PrioritySumStructure(int k) : k(k), sum(0) { }

    /// 順番を整える（実質内部関数）
    void modify() {
        while(in.size() - d_in.size() < k && !out.empty()) {
            auto p = out.top();
            out.pop();
            if(!d_out.empty() && p == d_out.top()) {
                d_out.pop();
            } else {
                sum += p;
                in.emplace(p);
            }
        }
        while(in.size() - d_in.size() > k) {
            auto p = in.top();
            in.pop();
            if(!d_in.empty() && p == d_in.top()) {
                d_in.pop();
            } else {
                sum -= p;
                out.emplace(p);
            }
        }
        while(!d_in.empty() && in.top() == d_in.top()) {
            in.pop();
            d_in.pop();
        }
    }

    /// 上位k個の和(要素数がkに満たないとき, 要素すべての和) を返す
    // O(1)
    T query() const {
        return sum;
    }

    /// xを挿入（順番を整えるのは自動でmodify()でやってくれる）
    // O(logN)
    void insert(T x) {
        in.emplace(x);
        sum += x;
        modify();
    }

    /// xを削除（順番を整えるのは自動でmodify()でやってくれる）
    // O(logN)
    void erase(T x) {
        assert(size());
        if(!in.empty() && in.top() == x) {
            sum -= x;
            in.pop();
        } else if(!in.empty() && RCompare()(in.top(), x)) {
            sum -= x;
            d_in.emplace(x);
        } else {
            d_out.emplace(x);
        }
        modify();
    }

    /// Kの変更
    // O((それ以前のkとの差)logN)
    void set_k(size_t kk) {
        k = kk;
        modify();
    }

    size_t get_k() const {
        return k;
    }

    size_t size() const {
        return in.size() + out.size() - d_in.size() - d_out.size();
    }
};

/// 降順K個の総和
template< typename T >
using MaximumSum = PrioritySumStructure< T, greater< T >, less< T > >;

/// 昇順K個の総和
template< typename T >
using MinimumSum = PrioritySumStructure< T, less< T >, greater< T > >;