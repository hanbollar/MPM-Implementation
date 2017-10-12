
#pragma once

#include <Eigen/Dense>

namespace simulation {

template <unsigned int Dimension, typename T, typename Attribute, Attribute Key>
struct AttributeTransfer {

    // TODO: actually interpolate with basis functions

    template <Attribute A, typename ParticleSet, typename AttributeGrid>
    static void ParticleToGrid(const ParticleSet &particleSet, AttributeGrid &attributeGrid, const Eigen::Array<T, Dimension, 1> &origin) {
        const auto& keyAttributes = particleSet.GetAttributeList<Key>();
        const auto& sourceAttributes = particleSet.GetAttributeList<A>();
        auto& targetGrid = attributeGrid.GetGrid<A>();

        for (unsigned int p = 0; p < particleSet.Size(); ++p) {
            const auto& key = keyAttributes[p];
            Eigen::Array<unsigned int, Dimension, 1> ll = ((key - origin) / attributeGrid.CellSize()).cast<unsigned int>();
            auto* gridAttributell = targetGrid.at(ll);
            if (gridAttributell) {
                *gridAttributell = sourceAttributes[p];
            }
        }
    }

    template <Attribute A, typename ParticleSet, typename AttributeGrid>
    static void GridToParticle(const AttributeGrid &attributeGrid, const Eigen::Array<T, Dimension, 1> &origin, ParticleSet &particleSet) {
        const auto& keyAttributes = particleSet.GetAttributeList<Key>();
        auto& targetAttributes = particleSet.GetAttributeList<A>();
        const auto& sourceGrid = attributeGrid.GetGrid<A>();

        for (unsigned int p = 0; p < particleSet.Size(); ++p) {
            const auto& key = keyAttributes[p];
            Eigen::Array<unsigned int, Dimension, 1> ll = ((key - origin) / attributeGrid.CellSize()).cast<unsigned int>();
            auto* gridAttributell = sourceGrid.at(ll);
            if (gridAttributell) {
                targetAttributes[p] = *gridAttributell;
            }
        }
    }
};


}