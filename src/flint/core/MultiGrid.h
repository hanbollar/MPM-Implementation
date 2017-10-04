
#pragma once
#include <array>
#include <vector>

namespace core {

template <typename T, unsigned int Dimension>
class MultiGridBase {
    public:
		using Index = std::array<unsigned int, Dimension>;

        virtual void Fill(const T& value) = 0;
};

template <typename T, unsigned int Dimension>
class MultiGrid : public MultiGridBase<T, Dimension> {
    Index sizes;
    Index strides;
    std::vector<T> contents;

    unsigned int computeIndex(const Index &indices) const {
        unsigned int index = 0;
        for (unsigned int d = 0; d < Dimension; ++d) {
            index += indices[d] * strides[d];
        }
        return index;
    }

    public:
        MultiGrid() { }

        template<typename... Args>
        MultiGrid(Args&&... sizes) : sizes { sizes... } {
            unsigned int accum = 1;
            for (unsigned int d = 0; d < Dimension; ++d) {
                strides[d] = accum;
                accum *= sizes[d];
            }
            contents.resize(accum);
        }

        MultiGrid(const Index &sizes) : sizes(sizes) {
            unsigned int accum = 1;
            for (unsigned int d = 0; d < Dimension; ++d) {
                strides[d] = accum;
                accum *= sizes[d];
            }
            contents.resize(accum);
        }

        template<typename... Args>
        const T& operator()(Args&&... indices) const {
            Index i = { indices... };
            return contents[computeIndex(i)];
        }

        template<typename... Args>
        T& operator()(Args&&... indices) {
            Index i = { indices... };
            return contents[computeIndex(i)];
        }

        const T& operator[](const Index &indices) const {
            return contents[computeIndex(indices)];
        }

        T& operator[](const Index &indices) {
            return contents[computeIndex(indices)];
        }

        void Fill(const T& value) override {
            for (unsigned int i = 0; i < contents.size(); ++i) {
                contents[i] = value;
            }
        }
};

template <typename T, unsigned int Dimension>
class ResizeableMultiGrid : public MultiGridBase<T, Dimension> {
    using InnerGrid = ResizeableMultiGrid<T, Dimension - 1>;
    std::vector<InnerGrid> contents;

    public:
        ResizeableMultiGrid() { }

        template<typename... Args>
        ResizeableMultiGrid(unsigned int size, Args&&... sizes) {
            Resize(size, std::forward<Args>(sizes)...);
        }

        ResizeableMultiGrid(const Index &sizes) {
            Resize(sizes);
        }

        template<typename... Args>
        const T& operator()(unsigned int index, Args&&... indices) const {
            return contents[index](std::forward<Args>(indices)...);
        }

        template<typename... Args>
        T& operator()(unsigned int index, Args&&... indices) {
            return contents[index](std::forward<Args>(indices)...);
        }

        const T& operator[](const Index &indices) const {
            const InnerGrid::Index& innerIndices = *reinterpret_cast<const InnerGrid::Index*>(&indices[1]);
            return contents[indices[0]][innerIndices];
        }

        T& operator[](const Index &indices) {
            const InnerGrid::Index& innerIndices = *reinterpret_cast<const InnerGrid::Index*>(&indices[1]);
            return contents[indices[0]][innerIndices];
        }

        template<typename... Args>
        void Resize(unsigned int size, Args&&... sizes) {
            contents.resize(size, InnerGrid(std::forward<Args>(sizes)...));
            for (auto& content : contents) {
                content.Resize(std::forward<Args>(sizes)...);
            }
        }

        void Resize(const Index &sizes) {
            const InnerGrid::Index& innerSizes = *reinterpret_cast<const InnerGrid::Index*>(&sizes[1]);
            contents.resize(sizes[0], InnerGrid(innerSizes));
            for (auto& content : contents) {
                content.Resize(innerSizes);
            }
        }

        void Fill(const T& value) override {
            for (auto& content : contents) {
                content.Fill(value);
            }
        }
};

template <typename T>
class ResizeableMultiGrid<T, 1> : public MultiGridBase<T, 1> {
    std::vector<T> contents;

    public:
        ResizeableMultiGrid() { }

        ResizeableMultiGrid(unsigned int size) : contents(size) { }

        ResizeableMultiGrid(const Index &size) : contents(size[0]) { }

        const T& operator()(unsigned int index) const {
            return contents[index];
        }

        T& operator()(unsigned int index) {
            return contents[index];
        }

        const T& operator[](const Index &indices) const {
            return contents[indices[0]];
        }

        T& operator[](const Index &indices) {
            return contents[indices[0]];
        }

        void Resize(unsigned int size) {
            contents.resize(size);
        }

        void Resize(const Index &size) {
            contents.resize(size[0]);
        }

        void Fill(const T& value) override {
            for (unsigned int i = 0; i < contents.size(); ++i) {
                contents[i] = value;
            }
        }
};

template <typename T, unsigned int Dimension, unsigned int... Args>
class StaticMultiGrid : public MultiGridBase<T, sizeof...(Args)> {
    using InnerGrid = StaticMultiGrid<T, Args...>;
    std::array<InnerGrid, Dimension> contents;

    public:
        template<typename... Args>
        const T& operator()(unsigned int index, Args&&... indices) const {
            return contents[index](std::forward<Args>(sizes)...);
        }

        template<typename... Args>
        T& operator()(unsigned int index, Args&&... indices) {
            return contents[index](std::forward<Args>(sizes)...);
        }

        const T& operator[](const Index &indices) const {
            const InnerGrid::Index& innerIndices = *reinterpret_cast<const InnerGrid::Index*>(&indices[1]);
            return contents[indices[0]][innerIndices];
        }

        T& operator[](const Index &indices) {
            const InnerGrid::Index& innerIndices = *reinterpret_cast<const InnerGrid::Index*>(&indices[1]);
            return contents[indices[0]][innerIndices];
        }

        void Fill(const T& value) override {
            for (auto& content : contents) {
                content.Fill(value);
            }
        }
};

template <typename T, unsigned int Dimension>
class StaticMultiGrid<T, Dimension> : public MultiGridBase<T, Dimension> {
    std::array<T, Dimension> contents;

    public:
        const T& operator()(unsigned int index) const {
            return contents[index];
        }

        T& operator()(unsigned int index) {
            return contents[index];
        }

        const T& operator[](const Index &indices) const {
            return contents[indices[0]];
        }

        T& operator[](const Index &indices) {
            return contents[indices[0]];
        }

        void Fill(const T& value) override {
            for (unsigned int i = 0; i < contents.size(); ++i) {
                contents[i] = value;
            }
        }
};

}
