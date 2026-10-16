#pragma once

// 検証用の最小限の可換群。ライブラリ本体には依存しない。
namespace union_find_test {

struct ModValue {
    static constexpr int modulus = 998244353;
    int value;
    explicit ModValue(long long x = 0) : value(static_cast<int>(x % modulus)) {
        if (value < 0) value += modulus;
    }
    friend ModValue operator+(ModValue a, ModValue b) { return ModValue(a.value + b.value); }
    friend ModValue operator-(ModValue a, ModValue b) { return ModValue(a.value - b.value); }
    friend bool operator==(ModValue a, ModValue b) { return a.value == b.value; }
};

struct XorValue {
    unsigned value;
    explicit XorValue(unsigned x = 0) : value(x) {}
    friend XorValue operator+(XorValue a, XorValue b) { return XorValue(a.value ^ b.value); }
    friend XorValue operator-(XorValue a, XorValue b) { return a + b; }
    friend bool operator==(XorValue a, XorValue b) { return a.value == b.value; }
};

} // namespace union_find_test
