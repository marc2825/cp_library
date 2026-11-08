#pragma once
#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>
namespace cp {
// 非負整数列: floor除算・非負加算・代入・和。min/maxから作用できれば枝刈り。
class DivisionSegmentTreeBeats {
    struct Node {long long low=0,high=0,sum=0,add=0;int length=0;bool assigned=false;};
    int n_;std::vector<Node> nodes_;
    void assign(int k,long long x){auto& s=nodes_[k];s.low=s.high=x;s.sum=x*s.length;s.add=0;s.assigned=true;}
    void add(int k,long long x){auto& s=nodes_[k];s.low+=x;s.high+=x;s.sum+=x*s.length;if(!s.assigned)s.add+=x;}
    void pull(int k){auto& s=nodes_[k];auto a=nodes_[2*k],b=nodes_[2*k+1];s.low=std::min(a.low,b.low);s.high=std::max(a.high,b.high);s.sum=a.sum+b.sum;}
    void push(int k){auto& s=nodes_[k];if(s.assigned){assign(2*k,s.low);assign(2*k+1,s.low);s.assigned=false;}if(s.add){add(2*k,s.add);add(2*k+1,s.add);s.add=0;}}
    void build(int k,int l,int r,const std::vector<long long>& a){nodes_[k].length=r-l;if(r-l==1){assert(a[l]>=0);assign(k,a[l]);return;}int m=l+(r-l)/2;build(2*k,l,m,a);build(2*k+1,m,r,a);pull(k);}
    void update(int k,int l,int r,int ql,int qr,long long x,int type){
        if(qr<=l || r<=ql)return;
        if(ql<=l && r<=qr){
            if(type==0){assign(k,x);return;}if(type==1){add(k,x);return;}
            long long a=nodes_[k].low,b=nodes_[k].high;
            if(a/x==b/x){assign(k,a/x);return;}
            if(a/x-a==b/x-b){add(k,a/x-a);return;}
        }
        push(k);int m=l+(r-l)/2;update(2*k,l,m,ql,qr,x,type);update(2*k+1,m,r,ql,qr,x,type);pull(k);
    }
    long long query(int k,int l,int r,int ql,int qr,int type){
        if(qr<=l || r<=ql)return type==1?std::numeric_limits<long long>::max():0;
        if(ql<=l && r<=qr)return type==0?nodes_[k].sum:type==1?nodes_[k].low:nodes_[k].high;
        push(k);int m=l+(r-l)/2;auto a=query(2*k,l,m,ql,qr,type),b=query(2*k+1,m,r,ql,qr,type);
        return type==0?a+b:type==1?std::min(a,b):std::max(a,b);
    }
    void bounds(int l,int r)const{assert(0<=l && l<=r && r<=n_);(void)l;(void)r;}
public:
    explicit DivisionSegmentTreeBeats(const std::vector<long long>& a={}) :n_(int(a.size())),nodes_(std::max(std::size_t(1),4*a.size())){if(n_)build(1,0,n_,a);}
    int size()const{return n_;}
    void range_assign(int l,int r,long long x){bounds(l,r);assert(x>=0);if(l<r)update(1,0,n_,l,r,x,0);}
    void range_add(int l,int r,long long x){bounds(l,r);assert(x>=0);if(l<r)update(1,0,n_,l,r,x,1);}
    void range_divide(int l,int r,long long x){bounds(l,r);assert(x>=1);if(l<r && x!=1)update(1,0,n_,l,r,x,2);}
    long long range_sum(int l,int r){bounds(l,r);return query(1,0,n_,l,r,0);}
    long long range_min(int l,int r){bounds(l,r);return query(1,0,n_,l,r,1);}
    long long range_max(int l,int r){bounds(l,r);return query(1,0,n_,l,r,2);}
    long long get(int p){assert(0<=p && p<n_);return range_sum(p,p+1);}
};
} // namespace cp
