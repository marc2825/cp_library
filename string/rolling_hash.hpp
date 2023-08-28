/*
★ Name         : Rolling Hash
 ■ Snipet      : No
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      :
 ■ References  : https://atcoder.jp/contests/abc284/submissions/37954573
                 https://nyaannyaan.github.io/library/string/rolling-hash.hpp

 ■ TODO        :
*/


#pragma once
#include <vector>
#include <string>
#include <random>


using namespace std;

struct RollingHash {
private:
    using ull = unsigned long long;
    using ll = long long;
    static const ull _mod = 0x1fffffffffffffff;
    static ull _base;
    vector<ull> _hashed, _power;

    inline ull _mul(ull a, ull b) const {
        ull au = a >> 31;
        ull ad = a & ((1UL << 31) - 1);
        ull bu = b >> 31;
        ull bd = b & ((1UL << 31) - 1);
        ull mid = ad * bu + au * bd;
        ull midu = mid >> 30;
        ull midd = mid & ((1UL << 30) - 1);
        ull ans = au * bu * 2 + midu + (midd << 31) + ad * bd;
        
        ans = (ans >> 61) + (ans & _mod);
        if (ans >= _mod) ans -= _mod;
        return ans;
    }
public:
    RollingHash(const string &s) {
        ll n = s.size();
        _hashed.assign(n + 1, 0);
        _power.assign(n + 1, 0);
        _power[0] = 1;
        for(ll i = 0; i < n; i++) {
            _power[i + 1] = _mul(_power[i], _base);
            _hashed[i + 1] = _mul(_hashed[i], _base) + s[i];
            if(_hashed[i + 1] >= _mod) _hashed[i + 1] -= _mod;
        }
    }
    
    ull get(ll l, ll r) const {
        ull ret = _hashed[r] + _mod - _mul(_hashed[l], _power[r - l]);
        if(ret >= _mod) ret -= _mod;
        return ret;
    }
    
    ull connect(ull h1, ull h2, ll h2len) const {
        ull ret = _mul(h1, _power[h2len]) + h2;
        if(ret >= _mod) ret -= _mod;
        return ret;
    }
    
    void connect(const string &s) {
        ll n = _hashed.size() - 1, m = s.size();
        _hashed.resize(n + m + 1);
        _power.resize(n + m + 1);
        for(ll i = n; i < n + m; i++) {
            _power[i + 1] = _mul(_power[i], _base);
            _hashed[i + 1] = _mul(_hashed[i], _base) + s[i - n];
            if(_hashed[i + 1] >= _mod) _hashed[i + 1] -= _mod;
        }
    }
    
    ll LCP(const RollingHash &b, ll l1, ll r1, ll l2, ll r2) const {
        ll len = min(r1 - l1, r2 - l2);
        ll low = -1, high = len + 1;
        while(high - low > 1) {
            ll mid = (low + high) / 2;
            if(get(l1, l1 + mid) == b.get(l2, l2 + mid)) low = mid;
            else high = mid;
        }
        return low;
    }
};

mt19937_64 mt{(unsigned int)time(NULL)};
RollingHash::ull RollingHash::_base = mt() % RollingHash::_mod;