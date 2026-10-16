#define PROBLEM "https://atcoder.jp/contests/dp/tasks/dp_s"
#include <iostream>
#include <string>
#include "dp_optimization/digit_dp.hpp"
struct Count {
    int value;
    Count(int x=0):value(x){}
    Count& operator+=(const Count& x){value+=x.value;if(value>=1000000007)value-=1000000007;return *this;}
    bool operator==(const Count& x)const{return value==x.value;}
};
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::string upper;int d;std::cin>>upper>>d;auto answer=cp::digit_dp<Count>(upper,d,0,[=](int state,int digit,bool,bool){return (state+digit)%d;},[](int state,bool started){return !state&&started;});std::cout<<answer.value<<'\n';}
