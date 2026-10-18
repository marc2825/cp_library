#pragma once
#include <algorithm>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>
namespace cp {
enum class PartizanOrder {less,equal,greater,fuzzy};
enum class PartizanOutcome {left_wins,right_wins,previous_wins,next_wins};
// 有限のshort partizan game {L|R} のarena。ID同値とゲーム値の等価は別。
// 正規形への簡約はせず、再帰的な和・負号・部分順序をmemo化する。
class PartizanGameArena {
public:
    struct Node {std::vector<int> left,right;int birthday=0;};
private:
    std::vector<Node> nodes_{{}};
    std::map<std::pair<std::vector<int>,std::vector<int>>,int> intern_;
    std::map<std::pair<int,int>,int> sums_;
    std::map<int,int> negatives_;
    std::map<std::pair<int,int>,bool> comparisons_;
    std::size_t max_nodes_;int max_birthday_;
    void check(int id)const{if(id<0||static_cast<std::size_t>(id)>=nodes_.size())throw std::out_of_range("partizan game ID");}
public:
    explicit PartizanGameArena(std::size_t max_nodes=100000,int max_birthday=128):max_nodes_(max_nodes),max_birthday_(max_birthday){if(!max_nodes||max_nodes>static_cast<std::size_t>(INT_MAX)||max_birthday<0||max_birthday>256)throw std::invalid_argument("invalid partizan arena budget");intern_[{{},{}}]=0;}
    std::size_t size()const{return nodes_.size();}
    const Node& node(int id)const{check(id);return nodes_[id];}
    int make(std::vector<int> left,std::vector<int> right){
        int birthday=0;for(const auto&options:{left,right})for(int id:options){check(id);birthday=std::max(birthday,nodes_[id].birthday+1);}
        if(birthday>max_birthday_)throw std::length_error("partizan birthday budget exceeded");
        for(auto*options:{&left,&right}){std::sort(options->begin(),options->end());options->erase(std::unique(options->begin(),options->end()),options->end());}
        const auto key=std::make_pair(left,right);auto found=intern_.find(key);if(found!=intern_.end())return found->second;
        if(nodes_.size()==max_nodes_)throw std::length_error("partizan node budget exceeded");
        const int id=int(nodes_.size());nodes_.push_back({std::move(left),std::move(right),birthday});intern_[key]=id;return id;
    }
    int integer(int value){
        if(value>max_birthday_||value< -max_birthday_)throw std::length_error("partizan integer exceeds birthday budget");
        int id=0;if(value>=0)for(int i=0;i<value;++i)id=make({id},{});else for(int i=0;i>value;--i)id=make({},{id});return id;
    }
    int negate(int id){
        check(id);if(!id)return 0;auto found=negatives_.find(id);if(found!=negatives_.end())return found->second;
        const auto original=nodes_[id];std::vector<int> left,right;for(int option:original.right)left.push_back(negate(option));
        for(int option:original.left)right.push_back(negate(option));
        const int result=make(std::move(left),std::move(right));negatives_[id]=result;negatives_[result]=id;return result;
    }
    int add(int a,int b){
        check(a);check(b);if(!a)return b;if(!b)return a;if(a>b)std::swap(a,b);const auto key=std::make_pair(a,b);auto found=sums_.find(key);if(found!=sums_.end())return found->second;
        const auto x=nodes_[a],y=nodes_[b];std::vector<int> left,right;
        for(int option:x.left)left.push_back(add(option,b));
        for(int option:y.left)left.push_back(add(a,option));
        for(int option:x.right)right.push_back(add(option,b));
        for(int option:y.right)right.push_back(add(a,option));
        const int result=make(std::move(left),std::move(right));sums_[key]=result;return result;
    }
    int subtract(int a,int b){return add(a,negate(b));}
    bool less_equal(int a,int b){
        check(a);check(b);if(a==b)return true;const auto key=std::make_pair(a,b);auto found=comparisons_.find(key);if(found!=comparisons_.end())return found->second;
        for(int option:nodes_[a].left)if(less_equal(b,option))return comparisons_[key]=false;
        for(int option:nodes_[b].right)if(less_equal(option,a))return comparisons_[key]=false;
        return comparisons_[key]=true;
    }
    PartizanOrder compare(int a,int b){const bool le=less_equal(a,b),ge=less_equal(b,a);return le?(ge?PartizanOrder::equal:PartizanOrder::less):(ge?PartizanOrder::greater:PartizanOrder::fuzzy);}
    PartizanOutcome outcome(int id){switch(compare(id,0)){case PartizanOrder::less:return PartizanOutcome::right_wins;case PartizanOrder::greater:return PartizanOutcome::left_wins;case PartizanOrder::equal:return PartizanOutcome::previous_wins;case PartizanOrder::fuzzy:return PartizanOutcome::next_wins;}throw std::logic_error("invalid game order");}
};
} // namespace cp
