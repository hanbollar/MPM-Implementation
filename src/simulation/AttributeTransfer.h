
#pragma once
#include <functional>
#include <Eigen/Dense>
#include "flint/core/MultiGrid.h"
#include "flint/core/VectorUtils.h"

namespace simulation {

	template <unsigned int Dimension, typename T, typename Attribute, Attribute Key>
	struct AttributeTransfer {

		/* PARTICLE TRANSFER CALCULATIONS */
        using GridIndex = Eigen::Array<unsigned int, Dimension, 1>;

		class WeightVals {

		public:
			Eigen::Array<float, Dimension, 3> N;
			Eigen::Array<float, Dimension, 3> N_deriv;

			WeightVals() :
				N(Eigen::Array<float, Dimension, 3>()),
				N_deriv(Eigen::Array<float, Dimension, 3>()) {}

			// N: used for wip
			// N': used for gradWip

			// loc template
			//     Y -Z
			// -X  0  X
			//  Z -Y

			// N:
			//		   F	  G
			// A B C   E	 H    (always indexes from neg to pos version of the component)
			//		   D	I
			// N':
			//		   O	  P
			// J K L   N	 Q
			//		   M	R

			// ordering in array
			// ABC
			// DEF
			// GHI -- for N
			// JKL
			// MNO
			// PQR -- for N_deriv

		private:

			// checks if inputted grid value [whose components should now range from -1 to 2] are proper
			static bool withinTest(Eigen::Array<T, Dimension, 1> v) {

				for (int i = 0; i < 3; ++i) {
					float val = v[i];

					bool valid = (val < 2 || floatEquals(val, 2)) && (val > -2 || floatEquals(val, -2));
					if (!valid) {
						// outside of checking region
						throw;
					}
				}
			}

			// N for particle grid location
			void calcN(int i, int j, float component) {

				float x = abs(component);

				/*
				// linear
				float first = 1 - fabs(x);
				float second = 0;

				N(i, j) = (x >= 0 && x < 1) ? first : second;
				*/

				// quadratic
				float first = 0.75f - pow(x, 2.f);
				float second = 0.5f * pow((1.5f - x), 2.f);

				N(i, j) = (x >= 0.0f && x < 0.5f) ? first : ((x >= 0.5f && x < 1.5f) ? second : 0);

			}

			// N' for particle to grid location
			void calcN_deriv(int i, int j, float component) {

				float x = abs(component);

				/*
				// linear

				float first = -fabs(x);
				float second = 0;

				N_deriv(i, j) = (x >= 0 && x < 1) ? first : second;
				*/

				//quadratic
				auto first = -2.0f * x;
				auto second = -1.5f + x;

				N_deriv(i, j) = (x >= 0.0f && x < 0.5f) ? first : ((x >= 0.5 && x < 1.5f) ? second : 0);

			}

			// indexing helper to get N or N' value from appropriate arrays
			static Eigen::Array<T, Dimension, 1> indexInto(Eigen::Array<T, Dimension, 1> xp_modif, Eigen::Array<T, Dimension, 1> xi_modif, float cellSize) {

				Eigen::Array<T, Dimension, 1> scaled = ((xp_modif - xi_modif) / cellSize);
				Eigen::Array<T, Dimension, 1> index = Eigen::Array<T, Dimension, 1>();
				index.fill(0.f);

				withinTest(scaled);

				for (int i = 0; i < Dimension; ++i) {
					float val = scaled[i];

					index[i] = (val > 0.f || floatEquals(val, 0.f)) ? ((val > 1.f || floatEquals(val, 1.f) ) ? 0 : 1) : 2;
				}

				return index;
			}

		public:

			static bool floatEquals(float a, float b) {
				float epsilon = 0.000005f;
				return (fabs(a - b) < epsilon);
			}

			static Eigen::Array<T, Dimension, 1> particleLoc(Eigen::Array<T, Dimension, 1> xp, const Eigen::Array<T, Dimension, 1> &origin) {
				return xp - origin;
			}

			static Eigen::Array<T, Dimension, 1> baseNodeLoc(Eigen::Array < T, Dimension, 1> xp, const Eigen::Array<T, Dimension, 1> &origin, float cellSize) {
				Eigen::Array<float, Dimension, 1> base = Eigen::Array<float, Dimension, 1>();
				for (int d = 0; d < Dimension; ++d) {
					base[d] = floor((xp[d] - origin[d]) / cellSize) - 1.f;
				}
				return base * cellSize;
			}

			// calc N and N' values for curr particle positions
			void fillWeights(Eigen::Array<T, Dimension, 1> xp, float cellSize, const Eigen::Array<T, Dimension, 1> &origin) {
				// here xp, xi are orig values (no modifications)

				// location of lower left hand node [ie if grid system is of cellSize 1, and point p is at 2,2 then
				// this node at 0, 0]
				auto pLoc = particleLoc(xp, origin);
				auto baseNode = baseNodeLoc(xp, origin, cellSize);

				// calc weights for grid positions being checked
				for (int i = 0; i < Dimension; ++i) {

					// iterating temp location on grid nodes
					auto tempGridLoc = baseNode;
					for (int j = 0; j < 3; ++j) {

						tempGridLoc[i] = baseNode[i] + j * cellSize;

						Eigen::Array<T, Dimension, 1> scaled = (pLoc - tempGridLoc) / cellSize;
						withinTest(scaled);

						calcN(i, j, scaled[i]);
						calcN_deriv(i, j, scaled[i]);
					}
				}

			}

			// calculating Wip with reference to a specific cell xi
			float calcWip(Eigen::Array<T, Dimension, 1> xp_modif, Eigen::Array<T, Dimension, 1> xi_modif, float cellSize) const {
				// here xp, xi have already been modified by origin but have not yet been divided by cellSize

				Eigen::Array<T, Dimension, 1> index = indexInto(xp_modif, xi_modif, cellSize);

				// calc weights for grid positions being checked
				float val = 1.f;
				for (int i = 0; i < Dimension; ++i) {
					float temp = N(i, index[i]);
					val *= N(i, index[i]);
				}

				return val;
			}

			// calculating gradWip with reference to a specific cell xi
			Eigen::Array<T, Dimension, Dimension> calcGradWip(Eigen::Array<T, Dimension, 1> xp_modif, Eigen::Array<T, Dimension, 1> xi_modif, float cellSize) {
				// here xp, xi have already been modified by origin but have not yet been divided by cellSize

				Eigen::Array<T, Dimension, 1> index = indexInto(xp_modif, xi_modif, cellSize);

				// calc weights for grid positions being checked
				Eigen::Array<T, Dimension, Dimension>  calc = Eigen::Array<T, Dimension, Dimension>();
				calc.fill(1.f);

				for (int d = 0; d < Dimension; ++d) {
					for (int i_d = 0; i_d < Dimension; ++i_d) {
						calc[d] *= (i_d == d) ? N_deriv(d, index[d]) : N(d, index[d]);
					}
				}

				calc /= cellSize;

				return calc;
			}

		};

		/* PARTICLE MOVEMENT */

		// P2G

		template <unsigned int... I>
        static decltype(auto) KernelGridImpl(std::index_sequence<I...>) {
			return core::StaticMultiGrid<unsigned int, (I, Dimension)...>();
		}

		static const void ApplyOverKernel(const std::function<void(const GridIndex&)> &func) {
			//static constexpr auto kernel = KernelGridImpl(std::make_index_sequence<Dimension>());
            static const auto kernel = core::StaticMultiGrid<unsigned int, Dimension, Dimension, Dimension>();
            kernel.ApplyOverIndices(func);
		}

        template <typename ParticleSet, typename AttributeGrid>
        static void IterateParticleKernel(
                const ParticleSet &particleSet,
                AttributeGrid &attributeGrid,
                const Eigen::Array<T, Dimension, 1> &origin,
                const std::function<void(unsigned int, float, const GridIndex&, const GridIndex&)> &func) {

            const auto& keyAttributes = particleSet.GetAttributeList<Key>(); // pos
            const auto& weightAttributes = particleSet.GetAttributeList<SimulationAttribute::Weights>();
            auto cellSize = attributeGrid.CellSize();

            core::VectorUtils::ApplyOverIndices(keyAttributes, [&](unsigned int p) {
                auto& key = keyAttributes[p];
                auto& weights = weightAttributes[p];

                Eigen::Array<T, Dimension, 1> baseNodeLoc = WeightVals::baseNodeLoc(key, origin, cellSize);
                Eigen::Array<T, Dimension, 1> pLoc = WeightVals::particleLoc(key, origin);

                ApplyOverKernel([&](const auto& index) {
                    Eigen::Array<float, Dimension, 1> gridLoc = baseNodeLoc + index.cast<float>() * cellSize;
					float weightVal_calc = weights.calcWip(pLoc, gridLoc, cellSize);
					Eigen::Array<unsigned int, Dimension, 1> gridCell = (gridLoc / cellSize).cast<unsigned int>();
                    func(p, weightVal_calc, index, gridCell);
				});
            });
        }

		template <Attribute A, typename ParticleSet, typename AttributeGrid>
		static void ParticleToGrid(const ParticleSet &particleSet, AttributeGrid &attributeGrid, const Eigen::Array<T, Dimension, 1> &origin) {

			const auto& keyAttributes = particleSet.GetAttributeList<Key>(); // pos
			const auto& sourceAttributes = particleSet.GetAttributeList<A>(); // attribute
			const auto& weightAttributes = particleSet.GetAttributeList<SimulationAttribute::Weights>();
			auto& targetGrid = attributeGrid.GetGrid<A>();

			auto cellSize = attributeGrid.CellSize();

			core::VectorUtils::ApplyOverIndices(keyAttributes, [&](unsigned int p) {
				auto& key = keyAttributes[p];
				auto& sourceVal = sourceAttributes[p];
				auto weights = weightAttributes[p];

				// location of lower left hand node [ie if grid system is of cellSize 1, and point p is at 2,2 then
				// this node at 0, 0
				Eigen::Array<T, Dimension, 1> baseNodeLoc = WeightVals::baseNodeLoc(key, origin, cellSize);
				Eigen::Array<T, Dimension, 1> pLoc = WeightVals::particleLoc(key, origin);

				// iterating gridLoc for particle
				Eigen::Array<float, Dimension, 1> gridLoc = Eigen::Array<float, Dimension, 1>();
				gridLoc.fill(0.f);

				float sumWeights = 0.f;

				ApplyOverKernel([&](const auto& index) {
					gridLoc = baseNodeLoc + index.cast<float>() * cellSize;

					auto weightVal_calc = weights.calcWip(pLoc, gridLoc, cellSize);

					sumWeights += weightVal_calc;

					Eigen::Array<unsigned int, Dimension, 1> checkLoc = (gridLoc / cellSize).cast<unsigned int>();
					auto* gridAttributeAtLoc = targetGrid.at(checkLoc);

					// if grid location exits in grid then alter it
					if (gridAttributeAtLoc) {
						*gridAttributeAtLoc += sourceVal * weightVal_calc;
					}
				});

				// -- turn this check back on once we have bounds to prohibit our particles from stepping out of the grid box
				//if ( !(WeightVals::floatEquals(sumWeights, 1.f)) ) {
				//	throw;
				//}
			}); // end: looping through each particle

		}

		// G2P
		template <Attribute A, typename ParticleSet, typename AttributeGrid>
		static void GridToParticle(const AttributeGrid &attributeGrid, const Eigen::Array<T, Dimension, 1> &origin, ParticleSet &particleSet) {

			const auto& keyAttributes = particleSet.GetAttributeList<Key>(); // pos
			auto& targetAttributes = particleSet.GetAttributeList<A>(); // attribute
            using TargetType = std::remove_reference<decltype(targetAttributes[0])>::type;
			const auto& weightAttributes = particleSet.GetAttributeList<SimulationAttribute::Weights>();
			const auto& sourceGrid = attributeGrid.GetGrid<A>();

			//int cellSize_int = static_cast<unsigned int>(attributeGrid.CellSize());
			auto cellSize = attributeGrid.CellSize();

			core::VectorUtils::ApplyOverIndices(keyAttributes, [&](unsigned int p) {
				auto& key = keyAttributes[p];
				auto weights = weightAttributes[p];

				// location of lower left hand node [ie if grid system is of cellSize 1, and point p is at 2,2 then
				// this node at 0, 0
				Eigen::Array<T, Dimension, 1> baseNodeLoc = WeightVals::baseNodeLoc(key, origin, cellSize);
				Eigen::Array<T, Dimension, 1> pLoc = WeightVals::particleLoc(key, origin);

				// iterating gridLoc for particle
				Eigen::Array<float, Dimension, 1> gridLoc = Eigen::Array<float, Dimension, 1>();
				gridLoc.fill(0.f);

                TargetType gridVal_weighted;// Eigen::Array<float, Dimension, 1>();
				gridVal_weighted.fill(0.f);

				ApplyOverKernel([&](const auto& index) {
					gridLoc = baseNodeLoc + index.cast<float>() * cellSize;

					Eigen::Array<unsigned int, Dimension, 1> checkLoc = (gridLoc / cellSize).cast<unsigned int>();
					auto* gridAttributeAtLoc = sourceGrid.at(checkLoc);
					// if grid location exits in grid then alter particle val based on grid val
					if (gridAttributeAtLoc) {
						gridVal_weighted += *gridAttributeAtLoc * weights.calcWip(pLoc, gridLoc, cellSize);
					}
				});

				targetAttributes[p] = gridVal_weighted;
			}); // end: looping through each particle

		}

	};


}
