
#pragma once
#include <array>
#include <vector>
#include <Eigen/Dense>

namespace core {

template <typename T, unsigned int Dimension>
class MultiGridBase {
    public:
        using Index = Eigen::Array<unsigned int, Dimension, 1>;

        virtual void Fill(const T& value) = 0;
};

template <typename T, unsigned int Dimension>
class MultiGrid : public MultiGridBase<T, Dimension> {
    

    public:
        using Index = typename MultiGridBase<T, Dimension>::Index;

        MultiGrid() { }

        template<typename... Args>
        MultiGrid(Args&&... _sizes) : sizes { _sizes... } {
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

        const T* at(const Index &indices) const {
            if ((indices >= sizes).any()) {
                return nullptr;
            }
            return &contents[computeIndex(indices)];
        }

        T* at(const Index &indices) {
            if ((indices >= sizes).any()) {
                return nullptr;
            }
            return &contents[computeIndex(indices)];
        }

        void Fill(const T& value) override {
            for (unsigned int i = 0; i < contents.size(); ++i) {
                contents[i] = value;
            }
        }

        class CellIterator {
            class iterator {
                public:
                    iterator(const Index& index, MultiGrid* grid) : index(index), grid(grid) { }
                    
                    iterator& operator++() {
                        for (unsigned int d = 0; d < Dimension; ++d) {
                            index[d]++;
                            if (index[d] >= grid->sizes[d]) {
                                index[d] = 0;
                            } else {
                                break;
                            }
                            if (d == Dimension - 1) {
                                done = true;
                            }
                        }
                        return *this;
                    }

                    T& operator*() const {
                        return (*grid)[index];
                    }

                    friend bool operator!=(const iterator& a, const iterator& b) {
                        for (unsigned int d = 0; d < Dimension; ++d) {
                            if (a.done != b.done || a.index[d] != b.index[d]) {
                                return true;
                            }
                        }
                        return false;
                    }

                    friend bool operator==(const iterator& a, const iterator& b) {
                        return !operator!=(a, b);
                    }

                private:
                    bool done = false;
                    Index index;
                    MultiGrid* grid;
            };
            
            public:
                CellIterator(MultiGrid* grid) : grid(grid) { }

                iterator begin() const {
                    Index index;
                    for (unsigned int d = 0; d < Dimension; ++d) {
                        index[d] = 0;
                    }

                    return iterator(index, grid);
                }
                
                iterator end() const {
                    Index index = grid->sizes;
                    iterator iter(index, grid);
                    iter.operator++();
                    return iter;
                }

            private:
                MultiGrid* grid;
        };

        CellIterator IterateCells() {
            return CellIterator(this);
        }

    private:
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
};

template <typename T, unsigned int Dimension>
class ResizeableMultiGrid : public MultiGridBase<T, Dimension> {
    using InnerGrid = ResizeableMultiGrid<T, Dimension - 1>;
    std::vector<InnerGrid> contents;

    public:
        using Index = typename MultiGridBase<T, Dimension>::Index;

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
            const typename InnerGrid::Index& innerIndices = *reinterpret_cast<const typename InnerGrid::Index*>(&indices[1]);
            return contents[indices[0]][innerIndices];
        }

        T& operator[](const Index &indices) {
            const typename InnerGrid::Index& innerIndices = *reinterpret_cast<const typename InnerGrid::Index*>(&indices[1]);
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
            const typename InnerGrid::Index& innerSizes = *reinterpret_cast<const typename InnerGrid::Index*>(&sizes[1]);
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
        using Index = typename MultiGridBase<T, 1>::Index;

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
        using Index = typename MultiGridBase<T, sizeof...(Args)>::Index;

        template<typename... _Args>
        const T& operator()(unsigned int index, _Args&&... indices) const {
            return contents[index](std::forward<Args>(indices)...);
        }

        template<typename... _Args>
        T& operator()(unsigned int index, _Args&&... indices) {
            return contents[index](std::forward<_Args>(indices)...);
        }

        const T& operator[](const Index &indices) const {
            const typename InnerGrid::Index& innerIndices = *reinterpret_cast<const typename InnerGrid::Index*>(&indices[1]);
            return contents[indices[0]][innerIndices];
        }

        T& operator[](const Index &indices) {
            const typename InnerGrid::Index& innerIndices = *reinterpret_cast<const typename InnerGrid::Index*>(&indices[1]);
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
        using Index = typename MultiGridBase<T, 1>::Index;

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
