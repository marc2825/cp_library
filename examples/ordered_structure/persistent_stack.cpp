#include <iostream>
#include "ordered_structure/persistent_stack.hpp"
int main() {
    cp::PersistentStack<int> s; int a=s.push(0,4),b=s.push(a,9);
    std::cout<<s.top(a)<<" "<<s.top(b)<<" "<<s.top(s.pop(b))<<"\n";
}
