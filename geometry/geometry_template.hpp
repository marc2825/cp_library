/*
★ Name        ：計算幾何テンプレート
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      :
 ■ References  :

 ■ TODO        : 完成させる
*/


#pragma once
#include <cmath>

using namespace std;


class Point{
    public:
        double x, y;
 
        Point(double x = 0, double y = 0): x(x), y(y) {}
 
        
        Point operator + (Point p) {return Point(x + p.x, y + p.y); }
        Point operator - (Point p) {return Point(x - p.x, y - p.y); }
        Point operator * (double a) {return Point(a * x, a * y); }
        Point operator / (double a) {return Point(x / a, y / a); }
        
 
        double norm() {return x * x + y * y; }
        double abs() {return sqrt(norm()); }
 
 
        bool operator == (const Point &p) const{
            return fabs(x - p.x) < EPS && fabs(y - p.y) < EPS;
        }
 
        bool operator < (const Point &p) const{
            return x != p.x ? x < p.x : y < p.y;
        }
    private :
        const double EPS = (1e-10);
};


/// aとbの 外積
double dot(Point a, Point b){return a.x * b.x + a.y * b.y;}

/// aとbの 内積
double cross(Point a, Point b){return a.x * b.y - a.y * b.x;}