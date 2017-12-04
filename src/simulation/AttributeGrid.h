
#pragma once

#include <tuple>
#include <Eigen/Dense>
#include "flint/core/MultiGrid.h"
#include "AttributeStorage.h"

namespace simulation {

template <unsigned int Dimension, typename T, typename Attribute, typename AttributeDefinitions, Attribute... Attributes>
class AttributeGrid {
    using AttributeStorage_ = AttributeStorage<Attribute, Attributes...>;
    using Storage = std::tuple<core::MultiGrid<typename AttributeDefinitions::template AttributeInfo<Attributes>::type, Dimension>...>;

    public:
        AttributeGrid() { }

        AttributeGrid(T cellSize, const Eigen::Array<T, Dimension, 1> &sizes) {
            Resize(cellSize, sizes);
        }

        void Resize(T cellSize, const Eigen::Array<T, Dimension, 1> &sizes) {
            this->cellSize = cellSize;
            AttributeStorage_::ForEach(this->storage, [&](auto& grid, unsigned int) {
                using Grid = typename std::remove_reference<decltype(grid)>::type;
                typename Grid::Index index;
                for (unsigned int i = 0; i < Dimension; ++i) {
                    index[i] = sizes[i] / cellSize;
                }
                grid = Grid(index);
            });
        }

        template <Attribute A>
        decltype(auto) GetGrid() {
            return std::get<AttributeStorage_::template AttributeToIndex<A>::value>(storage);
        }

        template <Attribute A>
        decltype(auto) GetGrid() const {
            return std::get<AttributeStorage_::template AttributeToIndex<A>::value>(storage);
        }

        T CellSize() const {
            return cellSize;
        }

    private:
        T cellSize;
        Storage storage;
};

}
