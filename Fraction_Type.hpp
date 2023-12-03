/*
★ Name        ：有理数型 (x/y)
 ■ Snipet      : No
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc274/submissions/44851168
				 https://atcoder.jp/contests/past16-open/submissions/48071804
 ■ References  :

 ■ TODO        :
*/


#pragma once
#include <iostream>
#include <cmath>

using namespace std;


/// 有理数型 (x/y)
class Fraction{
	public:
		long long x,y;
		/// 約分
		void reduc(){
			long long minus = 1;
			if(x < 0) minus *= -1;
			if(y < 0) minus *= -1;
			long long g = gcd(abs(x), abs(y));
			x = minus * abs(x) / g;
			y = abs(y) / g;
		}
		
		Fraction(long long x = 0, long long y = 1): x(x), y(y) {reduc();};

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
        long long gcd(long long a, long long b){
            if(a < b) swap(a, b);

            if(b == 0) return a;
            if(a%b == 0) return b;
            else return gcd(b, a%b);
        }
};