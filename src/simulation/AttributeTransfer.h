
#pragma once

#include <Eigen/Dense>

namespace simulation {

template <unsigned int Dimension, typename T, typename Attribute, Attribute Key>
struct AttributeTransfer {

	class WeightVals {

	public:
		static constexpr unsigned int kLen = 3 + (Dimension - 1) * 2;
		Eigen::Array<Eigen::Vector3f, kLen, 1> N;
		Eigen::Array<Eigen::Vector3f, kLen, 1> N_deriv;

		WeightVals() :
			N(Eigen::Array<Eigen::Vector3f, kLen, 1>()),
			N_deriv(Eigen::Array<Eigen::Vector3f, kLen, 1>()) {}

		// N: used for wip
		// N': used for gradWip

		// loc template
		//     Y -Z
		// -X  0  X
		//  Z -Y

		// N:
		//   E F
		// A B C
		// G D
		// N':
		//   L M
		// H I J
		// N K

		// ordering in 2d array
		// ABC DE FG -- for w
		// HIJ KL MN -- for w_deriv

	private:

		// the function N used to define wip and grad wip for 
		static float basisFunction(float x) {
			float first = 0.75f - pow(abs(x), 2);
			float second = 0.5f * pow((1.5f - abs(x)), 2);
			return (x >= 0.0f && x > 0.5f) ? first : ((x >= 0.5 && x < 1.5f) ? second : 0);
		}
		static Eigen::Array<T, Dimension, 1> basisFunction_V(Eigen::Array<T, Dimension, 1> v) {
			Eigen::Array<T, Dimension, 1> returning = Eigen::Array<T, Dimension, 1>();
			for (int i = 0; i < Dimension; i++) {
				returning[i] = basisFunction(v[i]);
			}
			return returning;
		}

		// N' 
		static float derivOfBasisFunction(float x) {
			auto first = -2.0f * abs(x);
			auto second = -1.5f + abs(x);
			return (x >= 0.0f && x > 0.5f) ? first : ((x >= 0.5 && x < 1.5f) ? second : 0);
		}
		static Eigen::Array<T, Dimension, 1> derivOfBasisFunction_V(Eigen::Array<T, Dimension, 1> v) {
			Eigen::Array<T, Dimension, 1> returning = Eigen::Array<T, Dimension, 1>();
			for (int i = 0; i < Dimension; i++) {
				returning[i] = derivOfBasisFunction(v[i]);
			}
			return returning;
		}

		// N for particle grid location
		static Eigen::Array<T, Dimension, 1> calcN(Eigen::Array<T, Dimension, 1> xp, Eigen::Array<T, Dimension, 1> xi, float cellSize) {
			return basisFunction_V((xp - xi) / cellSize);
		}

		// N' for particle to grid location
		static Eigen::Array<T, Dimension, 1> calcN_deriv_V(Eigen::Array<T, Dimension, 1> xp, Eigen::Array<T, Dimension, 1> xi, float cellSize) {
			return derivOfBasisFunction_V((xp - xi) / cellSize);
		} 

		static Eigen::Array<T, Dimension, 1> indexN(Eigen::Array<T, Dimension, 1> xp, Eigen::Array<T, Dimension, 1> xi, float cellSize) {
			if (i < 3) {
				loc[0] += static_cast<float>(-cellSize + i * cellSize);
			}
			else {
				// index for arb sized grid
				int index = (i - 3) / 2 + 1;
				loc[index] += ((i + 1) % 2 == 0) ? -cellSize : cellSize;
			}

			Eigen::Array<T, Dimension, 1> fixedParam = ((xp - xi) / cellSize);
			Eigen::Array<T, Dimension, 1> index = Eigen::Array<T, Dimension, 1>();
			index.fill(0.f);

			for (int i = 0; i < Dimension; i++) {

				float val = fixedParam[i];
				float aVal = abs(fixedParam[i]);
				if (val > 2 || val < -1) {
					// outside of checking region
					return -1;
				}

				if (i == 0) {
					// x val has 3 components all others have 2
					loc[i] = (aVal > 1) ? 0 : ((val >= 0) ? 1 : 2) ;
				}
				else {
					float offset = (i - 1) * 2 + 3;
					loc[i] = (aVal > 1) ? offset : offset + 1;
				}
			}

			return index;
		}

		// calculating a weight property [wip or gradwip] with reference to a specific grid location
		Eigen::Array<T, Dimension, 1> calcWeightProp(Eigen::Array<T, Dimension, 1> xp, Eigen::Array<T, Dimension, 1> xi, float cellSize, bool funcOrDeriv) {
			Eigen::Array<T, Dimension, 1> index = indexN(xp, xi, cellSize);

			Eigen::Array<T, Dimension, 1>  calc = Eigen::Array<T, Dimension, 1>();
			calc.fill(1.f);

			for (int i = 0; i < Dimension; i++) {
				calc *= (funcOrDeriv) ? N[index[i]] : N_deriv[index[i]];
			}

			return calc;
		}

	public:

		// calc weights for curr particle positions
		void fill_Weights(Eigen::Array<T, Dimension, 1> xp, float cellSize) {

			auto intCast = Eigen::Array<T, Dimension, 1>();
			for (int i = 0; i < Dimension; i++) {
				intCast[i] = floor(xp[i]);
			}

			// calc weights for grid positions being checked
			for (unsigned int i = 0; i < kLen; i++) {
				Eigen::Array<T, Dimension, 1> loc = intCast;

				if (i < 3) {
					loc[0] += static_cast<float>(-cellSize + i * cellSize);
				} else {
					// index for arb sized grid
					int index = (i - 3) / 2 + 1;
					loc[index] += ((i + 1) % 2 == 0) ? -cellSize : cellSize;
				}

				auto Nval = calcN(xp, loc, cellSize);
				auto N_derivVal = calcN_deriv_V(xp, loc, cellSize);

				N[i] = Nval;
				N_deriv[i] = N_derivVal;
			}
		}

		// calculating Wip with reference to a specific cell xi
		Eigen::Array<T, Dimension, 1> calcWip(Eigen::Array<T, Dimension, 1> xp, Eigen::Array<T, Dimension, 1> xi, float cellSize) {
			return calcWeightProp(xp, xi, cellSize, true);
		}

		// calculating gradWip with reference to a specific cell xi
		Eigen::Array<T, Dimension, 1> calcGradWip(Eigen::Array<T, Dimension, 1> xp, float cellSize) {
			return calcWeightProp(xp, xi, cellSize, false);
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