
#pragma once
#include <array>
#include <vector>
#include <Eigen/Dense>

namespace core {

template <typename T, unsigned int Dimension>
class MultiGridBase {
    public:
        using Index = Eigen::Array<unsigned int, Dimension, 1>;

        virtual const T& operator[](const Index &indices) const = 0;
        virtual T& operator[](const Index &indices) = 0;

        virtual void Fill(const T& value) = 0;
        virtual Index GetSizes() const = 0;

        class CellIterator {
            class iterator {
            public:
                iterator(const Index& index, MultiGridBase* grid) : index(index), grid(grid), sizes(grid->GetSizes()) { }

                iterator& operator++() {
                    for (unsigned int d = 0; d < Dimension; ++d) {
                        index[d]++;
                        if (index[d] >= sizes[d]) {
                            index[d] = 0;
                        }
                        else {
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
                Index sizes;
                MultiGridBase* grid;
            };

        public:
            CellIterator(MultiGridBase* grid) : grid(grid) { }

            iterator begin() const {
                Index index;
                for (unsigned int d = 0; d < Dimension; ++d) {
                    index[d] = 0;
                }

                return iterator(index, grid);
            }

            iterator end() const {
                Index index = grid->GetSizes();
                iterator iter(index, grid);
                iter.operator++();
                return iter;
            }

        private:
            MultiGridBase* grid;
        };

        class IndexIterator {
            class iterator {
            public:
                iterator(const Index& index, const MultiGridBase* grid) : index(index), grid(grid), sizes(grid->GetSizes()) { }

                iterator& operator++() {
                    for (unsigned int d = 0; d < Dimension; ++d) {
                        index[d]++;
                        if (index[d] >= sizes[d]) {
                            index[d] = 0;
                        }
                        else {
                            break;
                        }
                        if (d == Dimension - 1) {
                            done = true;
                        }
                    }
                    return *this;
                }

                const Index& operator*() const {
                    return index;
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
                Index sizes;
                const MultiGridBase* grid;
            };

        public:
            IndexIterator(const MultiGridBase* grid) : grid(grid) { }

            iterator begin() const {
                Index index;
                for (unsigned int d = 0; d < Dimension; ++d) {
                    index[d] = 0;
                }

                return iterator(index, grid);
            }

            iterator end() const {
                Index index = grid->GetSizes();
                iterator iter(index, grid);
                iter.operator++();
                return iter;
            }

        private:
            const MultiGridBase* grid;
        };

        CellIterator IterateCells() {
            return CellIterator(this);
        }

        IndexIterator IterateIndices() const {
            return IndexIterator(this);
        }

        template <typename F>
        void ApplyOverCells(F func) const {
            for (const auto& cell : IterateCells()) {
                func(cell);
            }
        }

        template <typename F>
        void ApplyOverCells(F func) {
            for (auto& cell : IterateCells()) {
                func(cell);
            }
        }

        template <typename F>
        void ApplyOverIndices(F func) const {
            for (const auto& index : IterateIndices()) {
                func(index);
            }
        }

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

        const T& operator[](const Index &indices) const override {
            return contents[computeIndex(indices)];
        }

        T& operator[](const Index &indices) override {
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

        Index GetSizes() const override {
            return sizes;
        }

    private:
        Index sizes;
        Index strides;
        std::vector<T> contents;

        unsigned int computeIndex(const Index &indices) const {
            return (indices * strides).sum();
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

        const T& operator[](const Index &indices) const override {
            const typename InnerGrid::Index& innerIndices = *reinterpret_cast<const typename InnerGrid::Index*>(&indices[1]);
            return contents[indices[0]][innerIndices];
        }

        T& operator[](const Index &indices) override {
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

        template <unsigned int... I>
        Index GetSizesImpl(std::index_sequence<I...>) const {
            const auto& innerSize = contents[0].GetSizes();
            return Index(contents.size(), innerSize[I]...);
        }

        Index GetSizes() const override {
            return GetSizesImpl(std::make_index_sequence<Dimension - 1>{});
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

        const T& operator[](const Index &indices) const override {
            return contents[indices[0]];
        }

        T& operator[](const Index &indices) override {
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

        Index GetSizes() const override {
            return Index(contents.size());
        }
};

template <typename T, unsigned int Dimension, unsigned int... Args>
class StaticMultiGrid : public MultiGridBase<T, 1 + sizeof...(Args)> {
    using InnerGrid = StaticMultiGrid<T, Args...>;
    std::array<InnerGrid, Dimension> contents;

    public:
        using Index = typename MultiGridBase<T, 1 + sizeof...(Args)>::Index;

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

        Index GetSizes() const override {
            return Index(Dimension, Args...);
        }
};

template <typename T, unsigned int Dimension>
class StaticMultiGrid<T, Dimension> : public MultiGridBase<T, 1> {
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

        Index GetSizes() const override {
            return Index(Dimension);
        }
};

}
