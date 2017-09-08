
#pragma once

#include <type_traits>
#include <core/Iterator.h>

namespace accel {

class TreeBase {

};

template <typename _Object>
class Tree : public TreeBase {
    public:
        static constexpr unsigned int kDimension = std::remove_pointer<_Object>::type::kDimension;
        using Object = _Object;
        
        virtual ~Tree() {}

        class Node {
            protected:
                bool _isLeaf;

            public:

                Node() {}
                virtual ~Node() {}

                bool IsLeaf() const {
                    return this->_isLeaf;
                }

                virtual core::iterator::polymorphic::RangeIterator<const Node*> IterateChildren() const = 0;
                virtual core::iterator::polymorphic::RangeIterator<typename std::add_const<_Object>::type> IterateObjects() const = 0;
                virtual core::iterator::polymorphic::RangeIterator<Node*> IterateChildren() = 0;
                virtual core::iterator::polymorphic::RangeIterator<_Object> IterateObjects() = 0;
        };

        virtual const Node* GetRoot() const = 0;

};

}

