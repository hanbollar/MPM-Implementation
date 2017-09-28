
#pragma once

#include <atomic>
#include <cassert>
#include <thread>
#include <Eigen/Dense>
#include "accel/TreeBuilder.h"
#include "TreeBuilder.h"
#include "Tree.h"

namespace accel {
namespace BVH {

template <typename Object, unsigned int B, unsigned int L>
class TreeBuilder : public accel::TreeBuilder<Object, TreeBuilder<Object, B, L>> {
    static_assert(B > 1, "Node branching factor must be at least 2");
    static_assert(std::is_pointer<Object>::value, "BVH tree builder operates on Geometry*");

protected:
    using Base = accel::TreeBuilder<Object, TreeBuilder<Object, B, L>>;
    using bound_t = core::AxisAlignedBox<TreeBuilder::kDimension, float>;
    using centroid_t = Eigen::Matrix<float, TreeBuilder::kDimension, 1>;

    friend Base;

    struct ObjectInfo {
        Object object;
        bound_t bound;
        centroid_t centroid;
    };

    using Tree_t = Tree<Object, B, L>;
    using Node_t = typename Tree_t::Node;

private:
    bool FindSplit(unsigned int begin, unsigned int end, ObjectInfo* objectInfos, unsigned int* axisPtr, float* split, bool* firstSplitSmaller) const {
        constexpr unsigned int kBinCount = 12;
        constexpr unsigned int kSplits = kBinCount - 1;
        constexpr float kEpsilon = 0.99f;

        // Get a bounding box of all object centroids
        bound_t centroidBound = Merge<TreeBuilder::kDimension, float>(core::Optional<bound_t>(), objectInfos[begin].centroid);
        for (unsigned int i = begin + 1; i < end; ++i) {
            Merge<TreeBuilder::kDimension, float>(centroidBound, objectInfos[i].centroid);
        }

        *axisPtr = centroidBound.GreatestExtent();
        unsigned int axis = *axisPtr;

        bool empty = (centroidBound.Extent(axis) == 0);

        if (!empty) {
            std::array<float, 2 * kSplits> costs;
            std::array<core::Optional<bound_t>, kBinCount> binBounds_;

            // Sort the objects into bins
            for (unsigned int i = begin; i < end; ++i) {
                unsigned int bin = static_cast<unsigned int>(
					kBinCount * kEpsilon * (objectInfos[i].centroid[axis] - centroidBound.min(axis)) /
					(centroidBound.max(axis) - centroidBound.min(axis))
				);
                Merge<TreeBuilder::kDimension, float>(binBounds_[bin], objectInfos[i].bound);
            }

            // Compute the cost of each bin split
            for (unsigned int s = 0; s < kSplits; ++s) {
                core::Optional<bound_t> lower_;
                core::Optional<bound_t> upper_;
                for (unsigned int b = 0; b < kBinCount; ++b) {
                    if (b <= s) {
                        Merge<TreeBuilder::kDimension, float>(lower_, binBounds_[b]);
                    } else {
                        Merge<TreeBuilder::kDimension, float>(upper_, binBounds_[b]);
                    }
                }

                costs[2 * s] = lower_.hasValue() ? lower_.value().SurfaceArea() : 0.f;
                costs[2 * s + 1] = upper_.hasValue() ? upper_.value().SurfaceArea() : 0.f;
            }

            // Find the cheapest split
            unsigned int binSplit = 0;
            for (unsigned int s = 0; s < kSplits; ++s) {
                if (costs[2 * s] + costs[2 * s + 1] < costs[2 * binSplit] + costs[2 * binSplit + 1]) {
                    binSplit = s;
                }
            }
            *split =  static_cast<float>(binSplit + 1) / kBinCount * kEpsilon * centroidBound.Extent(axis) + centroidBound.min(axis);
            *firstSplitSmaller = (costs[2 * binSplit] < costs[2 * binSplit + 1]);
            return true;
        }
        return false;
    }

    std::array<std::pair<unsigned int, unsigned int>, B> Partition(unsigned int begin, unsigned int end, ObjectInfo* objectInfos) const {
        std::array<std::pair<unsigned int, unsigned int>, B> partitions;
        unsigned int count = 0;
        do {
            unsigned int axis = 0;
            float split = 0;
            bool firstSplitSmaller = true;

            ObjectInfo* ptr;
            if (FindSplit(begin, end, objectInfos, &axis, &split, &firstSplitSmaller)) {
                // Partition the objects about the split point
                ptr = std::partition(objectInfos + begin, objectInfos + end, [&split, &axis](const ObjectInfo& object) {
                    return object.centroid[axis] < split;
                });
            } else {
                // No split found. Split in half
                ptr = std::partition(objectInfos + begin, objectInfos + end, [begin, end, objectInfos](const ObjectInfo& object) {
                    return (&object - &objectInfos[begin]) < (&objectInfos[end] - &objectInfos[begin] + 1) / 2;
                });
            }

            // A split has been made. Continue splitting with the larger side until the desired branching factor is reached
            unsigned int mid = static_cast<unsigned int>(ptr - objectInfos);

            if (firstSplitSmaller) {
                partitions[count].first = begin;
                partitions[count].second = mid;
                partitions[count + 1].first = mid;
                partitions[count + 1].second = end;

                begin = mid;
                end = end;
            } else {
                partitions[count + 1].first = begin;
                partitions[count + 1].second = mid;
                partitions[count].first = mid;
                partitions[count].second = end;

                begin = begin;
                end = mid;
            }

            count += 1;

        } while (count < B - 1);

        return partitions;
    }

    template <typename NodeAllocator>
    Node_t* RecursiveBuild(unsigned int begin, unsigned int end, ObjectInfo* objectInfos, NodeAllocator& nodeAllocator) const {
        if (end - begin == 0) {
            return nullptr;
        }

        core::Optional<bound_t> bound;
        // Allocate the node without constructing because the bound would be uninitialized
        Node_t* node = nodeAllocator.allocate(1);
        node->children.fill(nullptr);
        node->objects.fill(nullptr);

        if (end - begin <= L) {
            node->_isLeaf = true;
            for (unsigned int i = 0; i < L; ++i) {
                if (begin + i < end) {
                    node->objects[i] = objectInfos[begin + i].object;
                    Merge(bound, objectInfos[begin + i].bound);
                } else {
                    node->objects[i] = nullptr;
                }
            }
        } else {
            // Partition the objects into `B` sections
            auto partitions = Partition(begin, end, objectInfos);

            node->_isLeaf = false;

            for (unsigned int i = 0; i < B; ++i) {
                // Recursively build each section
                node->children[i] = RecursiveBuild(partitions[i].first, partitions[i].second, objectInfos, nodeAllocator);
                if (node->children[i]) {
                    Merge(bound, node->children[i]->bound);
                }
            }
        }

        // Now initialize in the same location with placement new
        // This will set up the virtual function table as well as initialize the bound
        assert(bound.hasValue());
        new(node) Node_t(bound.value());

        return node;
    }

    template <typename NodeAllocator>
    Node_t* IterativeBuild(unsigned int begin, unsigned int end, ObjectInfo* objectInfos, NodeAllocator& nodeAllocator) const {
        struct IterationInfo {
            unsigned int begin;
            unsigned int end;
            Node_t** node;
        };

        std::vector<IterationInfo> stack;
        std::vector<std::pair<unsigned int, Node_t*>> parentStack;
        stack.reserve(end - begin);
        parentStack.reserve(end - begin);

        Node_t* root = nullptr;
        stack.emplace_back(IterationInfo {begin, end, &root });

#ifndef NDEBUG
        uint32_t allocated = 0;
        uint32_t constructed = 0;
#endif

        while (stack.size() > 0) {
            IterationInfo curr = stack.back();
            stack.pop_back();

            assert(curr.end >= curr.begin);
            if (curr.end - curr.begin != 0) {
                // Allocate the node without constructing because the bound would be uninitialized
                Node_t* node = nodeAllocator.allocate(1);
#ifndef NDEBUG
                allocated++;
#endif
                node->children.fill(nullptr);
                node->objects.fill(nullptr);
                *curr.node = node;

                if (curr.end - curr.begin <= L) {

                    // Now initialize in the same location with placement new
                    // This will set up the virtual function table as well as initialize the bound
                    new(node) Node_t(objectInfos[curr.begin].bound);
                    node->objects[0] = objectInfos[curr.begin].object;
#ifndef NDEBUG
                    constructed++;
#endif

                    node->_isLeaf = true;
                    for (unsigned int i = 1; i < L; ++i) {
                        if (curr.begin + i < curr.end) {
                            node->objects[i] = objectInfos[curr.begin + i].object;
                            Merge(node->bound, objectInfos[curr.begin + i].bound);
                        } else {
                            node->objects[i] = nullptr;
                        }
                    }
                } else {
                    // Partition the objects into `B` sections
                    auto partitions = Partition(curr.begin, curr.end, objectInfos);

                    node->_isLeaf = false;

                    // Save a pair of the current stack size and the current node
                    // When the stack gets back to this size, we know we have finished processing this node's children
                    parentStack.emplace_back(std::make_pair(stack.size(), node));

                    for (unsigned int i = 0; i < B; ++i) {
                        stack.emplace_back(IterationInfo { partitions[i].first, partitions[i].second, &node->children[i] });
                    }
                }
            }

            while (parentStack.size() > 0 && stack.size() == parentStack.back().first) {
                Node_t* parent = parentStack.back().second;
                // All children of `parent` have been processed. Set the bound of `parent` to be the union of its children
                parentStack.pop_back();
                assert(parent);

                // Now initialize in the same location with placement new
                // This will set up the virtual function table as well as initialize the bound
                bool initialized = false;
                for (unsigned int i = 0; i < B; ++i) {
                    if (parent->children[i]) {
                        if (!initialized) {
                            initialized = true;
                            new(parent) Node_t(parent->children[i]->bound);
#ifndef NDEBUG
                            constructed++;
#endif
                        }
                        Merge(parent->bound, parent->children[i]->bound);
                    }
                }
                assert(initialized);
            }
        }

        assert(parentStack.size() == 0);

#ifndef NDEBUG
        assert(allocated == constructed);
#endif
        return root;
    }

    // static Node_t* Worker(std::atomic<bool> &running, unsigned int &begin, unsigned int &end, std::array<std::pair<unsigned int, unsigned int>, B>* outputPartition) {

    // }

    // template <typename NodeAllocator>
    // Node_t* ThreadedBuild(size_t begin, size_t end, ObjectInfo* objectInfos) const {
    //     unsigned int concurentThreadsSupported = std::thread::hardware_concurrency();
    //     assert(concurentThreadsSupported != 0);
    //     if (concurentThreadsSupported == 0 || end - begin <= L) {
    //         NodeAllocator nodeAllocator;
    //         return IterativeBuild(begin, end, objectInfos, nodeAllocator);
    //     }

    //     std::vector<NodeAllocator> nodeAllocators(concurentThreadsSupported);
    //     std::vector<std::atomic<bool>> threadsRunning(concurentThreadsSupported);
    //     std::vector<std::array<std::pair<unsigned int, unsigned int>, B>> partitions(concurentThreadsSupported);
    //     std::vector<std::atomic<bool>> availablePartitions(concurentThreadsSupported * B);
    //     std::vector<std::thread> threads;
    //     threads.reserve(concurentThreadsSupported);

    //     bool done = false;
    //     for (unsigned int i = 0; i < concurentThreadsSupported; ++i) {
    //         threads.emplace_back(Worker, std::ref(threadsRunning[i]), &partitions[i]);
    //     }

    //     // while (!done) {

    //     // }

    //     partitions[0] = Partition(begin, end, objectInfos);
    //     availablePartitions[0] = true;
    // }

    template <typename GeometryIterator, typename NodeAllocator = std::allocator<Node_t> >
    Tree_t* BuildImpl(GeometryIterator objectBegin, GeometryIterator objectEnd) const {
        if (objectEnd - objectBegin == 0) {
            return nullptr;
        }

        NodeAllocator nodeAllocator;

        std::vector<ObjectInfo> objectInfos;
        objectInfos.reserve(std::distance(objectBegin, objectEnd));

        std::for_each(objectBegin, objectEnd, [&objectInfos](auto object) {
            auto bound = object->getAxisAlignedBound();
            auto centroid = object->getCentroid();
            if (bound.hasValue() && centroid.hasValue()) {
                objectInfos.emplace_back(ObjectInfo {
                    object,
                    bound.value(),
                    centroid.value(),
                });
            }
        });

        // Node_t* root = ThreadedBuild<NodeAllocator>(0, static_cast<unsigned int>(objectInfos.size()), objectInfos.data());
        Node_t* root = IterativeBuild(0, static_cast<unsigned int>(objectInfos.size()), objectInfos.data(), nodeAllocator);
        // Node_t* root = RecursiveBuild(0, static_cast<unsigned int>(objectInfos.size()), objectInfos.data(), nodeAllocator);
        return new Tree_t(root);
    }

public:
    template <typename GeometryIterator>
    typename Base::template Build_t<GeometryIterator, Tree_t> Build(GeometryIterator objectBegin, GeometryIterator objectEnd) const {
        return this->BuildImpl(objectBegin, objectEnd);
    }

};

}
}
