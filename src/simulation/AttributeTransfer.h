
#pragma once

#include <Eigen/Dense>

namespace simulation {

template <unsigned int Dimension, typename T, typename Attribute, Attribute Key>
struct AttributeTransfer {

	class WeightVals {

	public:
		static constexpr unsigned int kLen = 3 + (Dimension - 1) * 2;
		Eigen::Array<float, kLen, 1> w;
		Eigen::Array<Eigen::Vector3f, kLen, 1> w_deriv;

		WeightVals() :
			w(Eigen::Array<float, kLen, 1>()),
			w_deriv(Eigen::Array<Eigen::Vector3f, kLen, 1>()) {}

		// first col = wip
		// second col = gradWip

		// loc template
		//     Y -Z
		// -X  0  X
		//  Z -Y

		// N:
		//   D G
		// A B C
		// F E
		// N':
		//   K M
		// H I J
		// N L

		// ordering in 2d array
		// ABC DE FG -- w
		// HIJ KL MN -- w_deriv

	private:

		// the function N used to define wip and grad wip for 
		static float basisFunction(float x) {
			float first = 0.75f - pow(abs(x), 2);
			float second = 0.5f * pow((1.5f - abs(x)), 2);
			return (x >= 0.0f && x > 0.5f) ? first : ((x >= 0.5 && x < 1.5f) ? second : 0);
		}
		static Eigen::Vector3f basisFunction(Eigen::Vector3f v) {
			return Eigen::Vector3f(basisFunction(v[0]), basisFunction(v[1]), basisFunction(v[2]));
		}

		// N' 
		static float derivOfBasisFunction(float x) {
			auto first = -2.0f * abs(x);
			auto second = -1.5f + abs(x);
			return (x >= 0.0f && x > 0.5f) ? first : ((x >= 0.5 && x < 1.5f) ? second : 0);
		}
		static Eigen::Vector3f derivOfBasisFunction(Eigen::Vector3f v) {
			return Eigen::Vector3f(derivOfBasisFunction(v[0]), derivOfBasisFunction(v[1]), derivOfBasisFunction(v[2]));
		}

		// wip for a particle to a specific grid location
		static float calcWIP(Eigen::Vector3f xp, Eigen::Vector3f xi, float cellSize) {
			Eigen::Vector3f wip = basisFunction((xp - xi) / cellSize);
			return wip[0] * wip[1] * wip[2];
		}

		// grad wip 
		static Eigen::Vector3f calcGradWIP(Eigen::Vector3f xp, Eigen::Vector3f xi, float cellSize) {
			float inv_cSize = 1.0f / cellSize;
			auto basisVals = basisFunction(inv_cSize * (xp - xi));
			auto derivBasisVals = derivOfBasisFunction(inv_cSize * (xp - xi));

			return Eigen::Vector3f(inv_cSize * derivBasisVals[0] * basisVals[1] * basisVals[2],
				inv_cSize * basisVals[0] * derivBasisVals[1] * basisVals[2],
				inv_cSize * basisVals[0] * basisVals[1] * derivBasisVals[2]);
		}

	public:
		template <typename AttributeGrid>
		//NEED TO RE-ADD IN GRID CHECK HERE -------------------------------------------------------------------------
		void fill_Weights(Eigen::Vector3f xp, float cellSize, AttributeGrid &attributeGrid) {

			auto& sourceGrid = attributeGrid;
			auto intCast = Eigen::Vector3f(floor(xp[0]),
				floor(xp[1]),
				floor(xp[2]));

			// calc weights for grid positions being checked
			for (unsigned int i = 0; i < kLen; i++) {
				Eigen::Vector3f loc = (i < 3) ? Eigen::Vector3f(intCast[0] - 1 + i, intCast[1], intCast[2])
					: ((i < 5) ? Eigen::Vector3f(intCast[0], intCast[1] + ((i == 3) ? -1 : 1), intCast[2])
						: Eigen::Vector3f(intCast[0], intCast[1], intCast[2] + ((i == 5) ? -1 : 1)
						));
				auto wip = calcWIP(xp, loc, cellSize);
				Eigen::Vector3f gradwip = calcGradWIP(xp, loc, cellSize);

				// existing loc in grid then want to have valid weight otherwise 0
				auto index = Eigen::Array<unsigned int, kDimension, 1>();
				for (int j = 0; j < 3; j++) {
					index[j] = static_cast<unsigned int>(loc[j]);
				}
				auto* checkLoc = sourceGrid.at(index);
				if (checkLoc) {
					w[i] = wip;
					w_deriv[i] = gradwip;
				}
				else {
					w[i] = 0.f;
					w_deriv[i] = Eigen::Vector3f(0.f, 0.f, 0.f);
				}
			}
		}
	};

	/* PARTICLE MOVEMENT */

	// P2G
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

		//const auto& sourceAttributes = particleSet.GetAttributeList<Weights>();
    }

	// G2P
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