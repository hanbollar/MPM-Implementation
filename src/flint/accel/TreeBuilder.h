
#pragma once

#include <type_traits>
#include <vector>
#include "geometry/Geometry.h"
#include "Tree.h"

namespace accel {

template <typename Object, typename Impl>
class TreeBuilder {

public:
    static constexpr unsigned int kDimension = std::remove_pointer<Object>::type::kDimension;

    template <typename GeometryIterator, typename Tree> 
    using Build_t = typename std::enable_if<
        std::is_same<typename std::iterator_traits<GeometryIterator>::value_type, typename std::remove_const<Object>::type>::value ||
        std::is_same<typename std::iterator_traits<GeometryIterator>::value_type, typename std::add_pointer<const typename std::remove_pointer<Object>::type>::type>::value ||
        std::is_same<typename std::iterator_traits<GeometryIterator>::value_type, typename std::add_pointer<typename std::remove_const<typename std::remove_pointer<Object>::type>::type>::type>::value,
    Tree*>::type;

    // template <typename GeometryIterator>
    // typename std::enable_if<
    //     std::is_same<typename std::iterator_traits<GeometryIterator>::value_type, typename std::remove_const<Object>::type>::value ||
    //     std::is_same<typename std::iterator_traits<GeometryIterator>::value_type, typename std::add_pointer<const typename std::remove_pointer<Object>::type>::type>::value ||
    //     std::is_same<typename std::iterator_traits<GeometryIterator>::value_type, typename std::add_pointer<typename std::remove_const<typename std::remove_pointer<Object>::type>::type>::type>::value,
    // Tree<Object>*>::type
    // template <typename GeometryIterator>
    // Build_t<GeometryIterator, Tree<Object>> Build(GeometryIterator objectBegin, GeometryIterator objectEnd) const {
    //     return static_cast<const Impl*>(this)->BuildImpl(objectBegin, objectEnd);
    // }

};

}

