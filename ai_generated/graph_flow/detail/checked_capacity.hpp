#pragma once
#include <limits>
#include <stdexcept>
#include <type_traits>
namespace cp { namespace detail {
template<class T> T bounded_flow_add(T a,T b) {
    static_assert(std::is_integral<T>::value && std::is_signed<T>::value,"signed integer required");
    if((b>0 && a>std::numeric_limits<T>::max()-b) || (b<0 && a<std::numeric_limits<T>::lowest()-b))
        throw std::overflow_error("flow arithmetic overflow");
    return a+b;
}
template<class T> T bounded_flow_subtract(T a,T b) {
    if((b>0 && a<std::numeric_limits<T>::lowest()+b) || (b<0 && a>std::numeric_limits<T>::max()+b))
        throw std::overflow_error("flow arithmetic overflow");
    return a-b;
}
}} // namespace cp::detail
