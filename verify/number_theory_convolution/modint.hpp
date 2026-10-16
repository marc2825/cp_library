#pragma once
// ジャッジ用の小さなmod型。実装本体は外部modintライブラリに依存しない。
struct ModInt {
    static constexpr int modulus = 998244353;
    int value;
    ModInt(long long x = 0) : value(static_cast<int>(x % modulus)) { if (value < 0) value += modulus; }
    ModInt& operator+=(const ModInt& other) { value += other.value; if (value >= modulus) value -= modulus; return *this; }
    ModInt& operator-=(const ModInt& other) { value -= other.value; if (value < 0) value += modulus; return *this; }
    friend ModInt operator*(const ModInt& a, const ModInt& b) { return ModInt(1LL * a.value * b.value); }
    friend bool operator==(const ModInt& a, const ModInt& b) { return a.value == b.value; }
};
