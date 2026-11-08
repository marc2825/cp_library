#pragma once
// 検証・ジャッジ用。積は合成数法・法1も可、/は素数法だけで使用。
template <int P> struct TestModInt {
    int value;
    TestModInt(long long x = 0) : value(static_cast<int>((x % P + P) % P)) {}
    TestModInt& operator+=(const TestModInt& x) { value = static_cast<int>((1LL * value + x.value) % P); return *this; }
    TestModInt& operator-=(const TestModInt& x) { value = static_cast<int>((1LL * value - x.value + P) % P); return *this; }
    friend TestModInt operator*(const TestModInt& a, const TestModInt& b) { return 1LL * a.value * b.value % P; }
    friend TestModInt operator/(const TestModInt& a, const TestModInt& b) { TestModInt inverse(1), base=b; for(int e=P-2;e;e>>=1,base=base*base)if(e&1)inverse=inverse*base;return a*inverse; }
    friend bool operator==(const TestModInt& a, const TestModInt& b) { return a.value == b.value; }
};
