#pragma once
#include <array>
#include <climits>
#include <cstdint>
#include <string>
#include <string_view>
#include "rolling_hash.hpp"
#include "../ordered_structure/implicit_treap.hpp"

namespace cp {
namespace dynamic_hash_detail {
struct Value {RollingHash::Hash hash{0,0},power{1,1};};
inline Value identity(){return {};}
inline Value combine(Value a,Value b){for(std::size_t j=0;j<2;++j){a.hash[j]=std::uint32_t((std::uint64_t(a.hash[j])*b.power[j]+b.hash[j])%RollingHash::moduli[j]);a.power[j]=std::uint32_t(std::uint64_t(a.power[j])*b.power[j]%RollingHash::moduli[j]);}return a;}
}
// ImplicitTreapの順/逆集約を共有。衝突あり。削除ノードはclearまで保持。
class DynamicRollingHash {
    using Value=dynamic_hash_detail::Value;
    ImplicitTreap<Value,dynamic_hash_detail::combine,dynamic_hash_detail::identity> sequence_;
    std::uint32_t base_;std::size_t allocated_=0;
    Value value(unsigned char c)const{return {{std::uint32_t(c)+1,std::uint32_t(c)+1},{base_,base_}};}
public:
    using Hash=RollingHash::Hash;
    explicit DynamicRollingHash(std::string_view text={},std::uint32_t base=RollingHash::default_base,std::uint64_t seed=0x51f15eULL):sequence_(seed),base_(base){
        (void)RollingHash({},base);string_algorithm_detail::checked_size(text.size());for(unsigned char c:text)insert(size(),c);
    }
    int size()const{return sequence_.size();}
    std::uint32_t base()const{return base_;}
    void clear(){sequence_.clear();allocated_=0;}
    void insert(int position,unsigned char c){string_algorithm_detail::range(size(),position,position);if(allocated_>=static_cast<std::size_t>(INT_MAX-1))throw std::length_error("dynamic hash node IDs exceed int");sequence_.insert(position,value(c));++allocated_;}
    void erase(int position){string_algorithm_detail::range(size(),position,position);if(position==size())throw std::out_of_range("dynamic hash point index");sequence_.erase(position);}
    void set(int position,unsigned char c){string_algorithm_detail::range(size(),position,position);if(position==size())throw std::out_of_range("dynamic hash point index");sequence_.set(position,value(c));}
    Hash get(int l,int r){string_algorithm_detail::range(size(),l,r);return sequence_.prod(l,r).hash;}
    void reverse(int l,int r){string_algorithm_detail::range(size(),l,r);sequence_.reverse(l,r);}
    bool equal(DynamicRollingHash& other,int l,int r,int a,int b){if(base_!=other.base_)throw std::invalid_argument("dynamic hash bases differ");string_algorithm_detail::range(size(),l,r);string_algorithm_detail::range(other.size(),a,b);return r-l==b-a&&get(l,r)==other.get(a,b);}
    int lcp(DynamicRollingHash& other,int l,int r,int a,int b){if(base_!=other.base_)throw std::invalid_argument("dynamic hash bases differ");string_algorithm_detail::range(size(),l,r);string_algorithm_detail::range(other.size(),a,b);int low=0,high=std::min(r-l,b-a);while(low<high){int mid=low+(high-low)/2+1;if(get(l,l+mid)==other.get(a,a+mid))low=mid;else high=mid-1;}return low;}
    std::string str(){std::string text;for(auto v:sequence_.to_vector())text.push_back(static_cast<char>(v.hash[0]-1));return text;}
};
} // namespace cp
