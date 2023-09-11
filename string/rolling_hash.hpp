/*
★ Name         : Rolling Hash (mod:2^61 - 1, 基数:共通random or 指定)
 ■ Snipet      : No
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc284/submissions/45324077 (const_base, get, connect)
                 https://atcoder.jp/contests/abc284/submissions/45324439 (const_base複数, get, connect)
                 https://atcoder.jp/contests/abc284/submissions/45324120 (unique_base, get, connect)
                 https://atcoder.jp/contests/abc284/submissions/45324216 (unique_base複数, get, connect)
                 https://atcoder.jp/contests/abc141/submissions/45324895
                 https://judge.yosupo.jp/submission/160022 (LCP)

 ■ References  : https://nyaannyaan.github.io/library/string/rolling-hash.hpp
                 https://atcoder.jp/contests/abc284/submissions/37954573
                 https://qiita.com/keymoon/items/11fac5627672a6d6a9f6

 ■ TODO        : baseをどうするか（複数のロリハで共通になるようにするか、分けるか）
*/


#pragma once
#include <vector>
#include <string>
#include <random>

using namespace std;


/// mod 2^61 - 1 の Rolling Hash
// <配列の型、基数> を指定（基数はデフォルトなら乱数）
// 配列の要素に0がない状態で渡す
// baseは(LCP等用に)const_baseで共通がデフォルト、複数使いたい場合は適宜BASE_NUMをコンストラクタに渡す
template <typename Str = string>
class RollingHash {
private:
    static const unsigned long long mod = 0x1fffffffffffffff; // 2^61 - 1
    unsigned long long base; // 実際に使うbase
    static unsigned long long const_base; // 共通base (static変数)
    vector<unsigned long long> hashed;
    vector<unsigned long long> power;
    Str data;
    size_t sz;

    /// オーバーフロー対策の乗算
    // https://qiita.com/keymoon/items/11fac5627672a6d6a9f6
    inline unsigned long long mul(unsigned long long a, unsigned long long b) const {
        unsigned long long au = a >> 31;
        unsigned long long ad = a & ((1UL << 31) - 1);
        unsigned long long bu = b >> 31;
        unsigned long long bd = b & ((1UL << 31) - 1);
        unsigned long long mid = ad * bu + au * bd;
        unsigned long long midu = mid >> 30;
        unsigned long long midd = mid & ((1UL << 30) - 1);
        unsigned long long ret = au * bu * 2 + midu + (midd << 31) + ad * bd;
        
        // mod
        ret = (ret >> 61) + (ret & mod);
        if (ret >= mod) ret -= mod;
        return ret;
    }

public:
    /// Sを対象としたRollingHashの構築を行うコンストラクタ
    // 共通base(const_base) 以外を使う場合は、BASE_NUMで2以上の整数を指定する
    // O(|S|)
    RollingHash(const Str &S, unsigned long long BASE_NUM = 0) {
        data = S;
        int N = S.size();
        sz = N;
        hashed.resize(N + 1);
        power.resize(N + 1);
        power[0] = 1;
        hashed[0] = 0;
        if(const_base < 2) const_base += 2;
        if(BASE_NUM < 2) base = const_base;
        else base = BASE_NUM;

        for(int i = 0; i < N; i++) {
            power[i + 1] = mul(power[i], base);
            hashed[i + 1] = mul(hashed[i], base) + S[i];
            if(hashed[i + 1] >= mod) hashed[i + 1] -= mod;
        }
    }

    /// 配列のサイズを返す
    size_t size() const { return sz; }

    /// Rolling Hash に実際に使われてる基数baseを確認する
    unsigned long long base_check() const { return base; }
    
    /// 区間 [l,r) のハッシュを返す
    // O(1)
    unsigned long long get(int l, int r) const {
        unsigned long long ret = hashed[r] + mod - mul(hashed[l], power[r - l]);
        if(ret >= mod) ret -= mod;
        return ret;
    }

    /// 配列Tのハッシュを返す
    // O(|T|)
    unsigned long long calc_hash(const Str &T) {
        unsigned long long ret = 0;
        for (int i = 0; i < (int)T.size(); i++) {
            ret = mul(ret, base) + T[i];
            if(ret >= mod) ret -= mod;
        }
        return ret;
    }
    
    /// ある区間(ハッシュ h1) の後ろに 長さ h2len のある区間(ハッシュ h2) を繋げた時のハッシュ値を返す
    // O(1)
    unsigned long long connect(unsigned long long h1, unsigned long long h2, ll h2len) const {
        unsigned long long ret = mul(h1, power[h2len]) + h2;
        if(ret >= mod) ret -= mod;
        return ret;
    }
    
    /// 配列Tを新たに繋げる
    // O(|T|)
    void combine_newarray(const Str &T) {
        int N = sz;
        int M = T.size();
        sz += M;
        hashed.resize(N + M + 1);
        power.resize(N + M + 1);
        for(int i = N; i < N + M; i++) {
            power[i + 1] = mul(power[i], base);
            hashed[i + 1] = mul(hashed[i], base) + T[i - N];
            if(hashed[i + 1] >= mod) hashed[i + 1] -= mod;
            data.push_back(T[i - N]);
        }
    }
    
    /// ロリハ済み配列Sの lower 文字目以降で、初めて引数の配列 T が出現する位置を返す (存在しない場合は -1)
    // O(|S|)
    int find(Str &T, int lower = 0) const {
        unsigned long long Thash = calc_hash(T);
        for (int i = lower; i <= sz - (int)T.size(); i++)
            if (Thash == get(i, i + (int)T.size())) return i;
        return -1;
    }

    /// 自分A[al, ar)と引数B[bl, br)の大小を比較する(返り値はstd::strcmpの仕様に準ずる)
    // O(log(|S| + |T|)), RollingHash の baseが共通である必要
    int strcmp(const RollingHash &B, int al, int bl, int ar = -1, int br = -1) {
    if (ar == -1) ar = sz;
    if (br == -1) br = B.size();
    int n = min<int>({LCP(B, al, bl), ar - al, br - bl});
    return   al + n == ar                    ? bl + n == br ? 0 : -1
           : bl + n == br                    ? 1
           : data[al + n] < B.data[bl + n]   ? -1
                                             : 1;
    }

    /// 自分Aのal文字目から始まるsuffixとBのbl文字目から始まるsuffixのLCP(Longest common prefix)を返す
    // suffix の連続部分列(A[al, ar), B[bl, br))を指定してもよい（デフォルトはsuffix全て）
    // O(log(|S| + |T|)), RollingHash の baseが共通である必要
    int LCP(const RollingHash &B, int al, int bl, int ar = -1, int br = -1) {
        if (ar == -1) ar = sz;
        if (br == -1) br = B.size();
        int len = min(ar - al, br - bl);
        int ok = 0, ng = len + 1;
        while(ng - ok > 1) {
            int mid = (ok + ng) / 2;
            if(get(al, al + mid) == B.get(bl, bl + mid)) ok = mid;
            else ng = mid;
        }
        return ok;
    }
};
mt19937_64 mt1{(unsigned int)time(NULL)};
template <typename Str>
unsigned long long RollingHash<Str>::const_base = mt1() % RollingHash<Str>::mod;