
#pragma once

#include "accel/Tree.h"
#include "core/AxisAlignedBox.h"

namespace accel {
namespace BVH {

template <typename Object, unsigned int B, unsigned int L> class TreeBuilder;

template <typename Object, unsigned int B, unsigned int L>
class Tree : public accel::Tree<Object> {
    using Base = accel::Tree<Object>;
    friend class TreeBuilder<Object, B, L>;

    public:
        class Node : public accel::Tree<Object>::Node {
            friend class TreeBuilder<Object, B, L>;
            using Base = typename accel::Tree<Object>::Node;
            using Bound = core::AxisAlignedBox<Tree::kDimension, float>;

            Node(const Bound &bound) : Base(), bound(bound) { }

            std::array<Node*, B> children;
            std::array<Object, L> objects;

            Bound bound;

            public:
                const Bound& GetBound()  const {
                    return bound;
                }

                core::iterator::polymorphic::RangeIterator<const Base*> IterateChildren() const {
                    using namespace core::iterator::polymorphic;
                    auto& constChildren = *reinterpret_cast<const std::array<const Node*, B>*>(const_cast<void*>(reinterpret_cast<const void*>(&children)));
                    return RangeIterator<const Base*>(new FixedArrayIteratorImpl<const Base*, const Node*, B>(constChildren, [](const Node* n) {
                        return n != nullptr;
                    }));
                }

                core::iterator::polymorphic::RangeIterator<Base*> IterateChildren() {
                    using namespace core::iterator::polymorphic;
                    return RangeIterator<Base*>(new FixedArrayIteratorImpl<Base*, Node*, B>(children, [](Node* n) {
                        return n != nullptr;
                    }));
                }

                core::iterator::polymorphic::RangeIterator<typename std::add_const<Object>::type> IterateObjects() const {
                    using object_t = typename std::add_const<Object>::type;
                    using namespace core::iterator::polymorphic;
                    auto& constObjects = *reinterpret_cast<const std::array<object_t, L>*>(const_cast<void*>(reinterpret_cast<const void*>(&objects)));
                    return RangeIterator<object_t>(new FixedArrayIteratorImpl<object_t, object_t, L>(constObjects, [](Object object) {
                        return object != nullptr;
                    }));
                }

                core::iterator::polymorphic::RangeIterator<Object> IterateObjects() {
                    using namespace core::iterator::polymorphic;
                    auto& _objects = *reinterpret_cast<const std::array<Object, L>*>(const_cast<void*>(reinterpret_cast<void*>(&objects)));
                    return RangeIterator<Object>(new FixedArrayIteratorImpl<Object, Object, L>(_objects, [](Object object) {
                        return object != nullptr;
                    }));
                }
        };

        Tree(Node* root) : root(root) {

        }

        template <typename ShouldVisit, typename VisitNode, typename Data, typename NextData>
        void PreorderDFS(ShouldVisit shouldVisit, VisitNode visitNode, Data initialData, NextData nextData) const {
            struct DFSNode {
                const Node* node;
                Data data;
            };

            std::vector<DFSNode> stack;

            stack.emplace_back(DFSNode { root, initialData });
            do {
                auto current = stack.back();
                stack.pop_back();

                if (shouldVisit(current.node, current.data)) {
                    visitNode(current.node, current.data);
                    for (const auto* _child : current.node->IterateChildren()) {
                        const auto* child = static_cast<const typename Tree::Node*>(_child);
                        stack.emplace_back(DFSNode { child, nextData(child, current.node, current.data) });
                    }
                }
            } while (stack.size() > 0);
        }

        template <typename ShouldVisit, typename VisitNode>
        void PreorderDFS(ShouldVisit shouldVisit, VisitNode visitNode) const {
            std::vector<const Node*> stack;
            stack.push_back(root);
            do {
                auto current = stack.back();
                stack.pop_back();

                if (shouldVisit(current)) {
                    visitNode(current);
                    for (const auto* _child : current->IterateChildren()) {
                        const auto* child = static_cast<const typename Tree::Node*>(_child);
                        stack.push_back(child);
                    }
                }
            } while (stack.size() > 0);
        }

        template <typename VisitNode, typename Data, typename NextData>
        void PreorderDFS(VisitNode visitNode, Data initialData, NextData nextData) const {
            PreorderDFS([](const Node*, const Data&){ return true; }, visitNode, initialData, nextData);
        }

        template <typename VisitNode>
        void PreorderDFS(VisitNode visitNode) const {
            PreorderDFS([](const Node*){ return true; }, visitNode);
        }

        ~Tree() {
            std::vector<core::iterator::polymorphic::RangeIterator<typename Base::Node*>> iteratorStack;
            std::vector<std::pair<unsigned int, Node*>> nodeStack;

            if (root) {
                nodeStack.push_back(std::make_pair(iteratorStack.size(), root));
                iteratorStack.push_back(std::move(root->IterateChildren()));
            }

            while (iteratorStack.size() > 0) {
                auto currentIterator = std::move(iteratorStack.back());
                iteratorStack.pop_back();

                for (auto* _node : currentIterator) {
                    Node* node = reinterpret_cast<Node*>(_node);
                    if (node) {
                        nodeStack.push_back(std::make_pair(iteratorStack.size(), node));
                        iteratorStack.push_back(std::move(node->IterateChildren()));
                    }
                }

                while (nodeStack.size() > 0 && iteratorStack.size() == nodeStack.back().first) {
                    Node* node = nodeStack.back().second;
                    nodeStack.pop_back();
                    delete node;
                }
            }

            assert(nodeStack.size() == 0);
        }

        const Node* GetRoot() const override {
            return root;
        }

    private:
        Node* root = nullptr;

};

}
}
