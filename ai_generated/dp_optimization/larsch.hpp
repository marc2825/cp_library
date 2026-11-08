#pragma once
// 説明: docs/dp_optimization/larsch.md
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>
namespace cp {
// 下三角totally monotone行列のオンライン最左argmin。
// noshi91/Library (CC0), algorithm/larsch.cpp の行・列縮約を移植。
// https://noshi91.github.io/Library/algorithm/larsch.cpp.html
// f(i,j)はj>iなら全ての有限値より大きい値を返すこと。
template<class T> class Larsch {
    struct Row;
    struct Column;
    struct Row {
        int n,current=0,state=0;std::function<T(int,int)> f;std::unique_ptr<Column> child;
        explicit Row(int count):n(count){if(n/2)child=std::make_unique<Column>(n/2);}
        void bind(std::function<T(int,int)> function){f=std::move(function);if(child)child->bind([this](int i,int j){return f(2*i+1,j);});}
        int next(){int i=current++;if(i%2){return f(i,i)<f(i,state)?i:state;}int previous=state;state=i+1==n?n-1:child->next();int answer=previous;for(int j=previous+1;j<=state;++j)if(f(i,j)<f(i,answer))answer=j;return answer;}
    };
    struct Column {
        int n,current=0;std::function<T(int,int)> f;std::vector<int> columns;Row child;
        explicit Column(int count):n(count),child(count){columns.reserve(n);}
        void bind(std::function<T(int,int)> function){f=std::move(function);child.bind([this](int i,int j){return f(i,columns[j]);});}
        int next(){int i=current++;auto insert=[&](int column){while(static_cast<int>(columns.size())!=i && f(static_cast<int>(columns.size())-1,column)<f(static_cast<int>(columns.size())-1,columns.back()))columns.pop_back();if(static_cast<int>(columns.size())!=n)columns.push_back(column);};if(i==0)insert(0);else{insert(2*i-1);insert(2*i);}return columns[child.next()];}
    };
    int n_,current_=0;std::function<T(int,int)> function_;std::unique_ptr<Row> base_;
public:
    Larsch(int n,std::function<T(int,int)> function):n_(n),function_(std::move(function)){if(n<0)throw std::invalid_argument("negative LARSCH dimension");
    reset();}
    int size()const{return n_;}
    int processed_rows()const{return current_;}
    int next_argmin(){if(current_==n_)throw std::out_of_range("LARSCH exhausted");
    ++current_;return base_->next();}
    void reset(){current_=0;base_=std::make_unique<Row>(n_);base_->bind(function_);}
};
} // namespace cp
