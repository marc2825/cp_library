#pragma once
// 検証用の小さな環型。ライブラリの実行依存にはしない。
struct BModInt {
    static constexpr long long mod=998244353;
    long long value;
    BModInt(long long x=0) : value((x%mod+mod)%mod) {}
    friend BModInt operator+(BModInt a,BModInt b) { return a.value+b.value; }
    friend BModInt operator-(BModInt a,BModInt b) { return a.value-b.value; }
    friend BModInt operator*(BModInt a,BModInt b) { return a.value*b.value; }
    BModInt& operator+=(BModInt b) { return *this=*this+b; }
};
