/*
★ Name        ：有理数型 (x/y)
 ■ Snipet      : No
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc274/submissions/48246521
				 https://atcoder.jp/contests/past16-open/submissions/48246572
				 https://atcoder.jp/contests/abc168/submissions/48246461 (T = i128)
 ■ References  :

 ■ TODO        :
*/


#pragma once
#include <iostream>
#include <cmath>

using namespace std;


/// 有理数型 (x/y)
// 分母分子が longlong 内に収まっても、比較の際の乗算でオーバーフローする可能性があることに注意 -> T = i128を使う
template< class T = long long>
class Fraction{
	public:
		T x,y;
		/// 約分
		void reduc(){
			int minus = 1;
			T absx = x;
            T absy = y;
            if(x < 0) minus *= -1, absx *= -1;
			if(y < 0) minus *= -1, absy *= -1;
			T g = gcd(absx, absy);
			x = minus * absx / g;
			y = absy / g;
		}
		
		Fraction(T x = 0, T y = 1): x(x), y(y) {reduc();};

        // 比較の際の乗算でオーバーフローすることがあることに注意
		bool operator<(const Fraction& right) const {return x*right.y < y*right.x;}
		bool operator<=(const Fraction& right) const {return x*right.y <= y*right.x;}
		bool operator>(const Fraction& right) const {return x*right.y > y*right.x;}
		bool operator>=(const Fraction& right) const {return x*right.y >= y*right.x;}
		bool operator==(const Fraction& right) const {return x == right.x && y == right.y;}
		Fraction operator-() const {return Fraction(-x, y);}
		Fraction& operator+=(const Fraction& v){
			x = x*v.y + y*v.x;
			y *= v.y;
			reduc();
			return *this;
		}
		Fraction operator+(const Fraction& v) const {return Fraction(*this) += v;}
		Fraction& operator-=(const Fraction& v){
			x = x*v.y - y*v.x;
			y *= v.y;
			reduc();
			return *this;
		}
		Fraction operator-(const Fraction& v) const {return Fraction(*this) -= v;}
		Fraction& operator*=(const Fraction& v){
			x *= v.x;
			y *= v.y;
			reduc();
			return *this;
		}
		Fraction operator*(const Fraction& v) const {return Fraction(*this) *= v;}
		Fraction& operator/=(const Fraction& v){
			x *= v.y;
			y *= v.x;
			reduc();
			return *this;
		}
		Fraction operator/(const Fraction& v) const {return Fraction(*this) /= v;}
		Fraction inv() const {return Fraction(y,x);}
		Fraction pow(long long t) const {
			if(t < 0) return inv().pow(-t);
			Fraction a(1, 1), d = *this;
			while(t){
				d *= d;
				if(t & 1) a *= d;
				t >>= 1;
			}
			return a;
		}

		friend ostream& operator << (ostream& os, const Fraction& v){ return os << v.x << '/' << v.y;}

    private:
        T gcd(T a, T b){
            if(a < b) swap(a, b);

            if(b == 0) return a;
            if(a%b == 0) return b;
            else return gcd(b, a%b);
        }
};

using Fractionll = Fraction<long long>;