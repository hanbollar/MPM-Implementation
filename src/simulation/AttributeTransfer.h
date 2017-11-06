
#pragma once

#include <Eigen/Dense>

namespace simulation {

	template <unsigned int Dimension, typename T, typename Attribute, Attribute Key>
	struct AttributeTransfer {

		/* PARTICLE TRANSFER CALCULATIONS */

		class WeightVals {

		public:
			static constexpr unsigned int kLen = 3 * Dimension;
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

			// ordering in 2d array
			// ABC DEF GHI -- for N
			// JKL MNO PQR -- for N_deriv

		private:

			// the function N used to define wip and grad wip for 
			float basisFunction(float x) {
				float first = 0.75f - pow(abs(x), 2.f);
				float second = 0.5f * pow((1.5f - abs(x)), 2.f);

				return (x >= 0.0f && x <= 1.5f) ? first : ((x >= 0.5 && x <= 1.5f) ? second : 0);
			}
			void basisFunction_V(int i, int j, Eigen::Array<T, Dimension, 1> v) {
				for (int d = 0; d < Dimension; ++d) {
					N(i,j) = basisFunction(v[d]);
				}
			}

			// N' 
			float derivOfBasisFunction(float x) {
				auto first = -2.0f * abs(x);
				auto second = -1.5f + abs(x);
				return (x >= 0.0f && x <= 1.5f) ? first : ((x >= 0.5 && x <= 1.5f) ? second : 0);
			}
			void derivOfBasisFunction_V(int i, int j, Eigen::Array<T, Dimension, 1> v) {
				for (int d = 0; d < Dimension; ++d) {
					N_deriv(i,j) = derivOfBasisFunction(v[d]);
				}
			}

			// checks if inputted grid value [whose components should now range from -1 to 2] are proper
			static bool withinTest(Eigen::Array<T, Dimension, 1> v) {
				
				for (int i = 0; i < 3; ++i) {
					float val = v[i];

					bool valid = (val < 2 || floatEquals(val, 2)) && (val > -2 || floatEquals(val, -2));
					if (!valid) {
						// outside of checking region
						std::cout << "ERROR: givin improper indexing location for weights array" << std::endl;
						throw;
					}
				}
			}

			// N for particle grid location
			void calcN(int i, int j, Eigen::Array<T, Dimension, 1> xp_modif, Eigen::Array<T, Dimension, 1> xi_modif, float cellSize) {
				Eigen::Array<float, Dimension, 1> scaled = (xp_modif - xi_modif) / cellSize;
				withinTest(scaled);
				
				basisFunction_V(i, j, scaled);
			}

			// N' for particle to grid location
			void calcN_deriv(int i, int j, Eigen::Array<T, Dimension, 1> xp_modif, Eigen::Array<T, Dimension, 1> xi_modif, float cellSize) {
				Eigen::Array<float, Dimension, 1> scaled = (xp_modif - xi_modif) / cellSize;
				withinTest(scaled);

				derivOfBasisFunction_V(i, j, scaled);
			}

			static bool floatEquals(float a, float b) {
				float epsilon = 0.000005f;
				return (fabs(a - b) < epsilon);
			}

			// indexing helper to get N or N' value from appropriate arrays
			static Eigen::Array<T, Dimension, 1> indexInto(Eigen::Array<T, Dimension, 1> xp_modif, Eigen::Array<T, Dimension, 1> xi_modif, float cellSize) {
				
				Eigen::Array<T, Dimension, 1> scaled = ((xp_modif - xi_modif) / cellSize);
				Eigen::Array<T, Dimension, 1> index = Eigen::Array<T, Dimension, 1>();
				index.fill(0.f);

				withinTest(scaled);

				for (int i = 0; i < Dimension; i++) {
					float val = scaled[i];

					index[i] = (val > 0.f || floatEquals(val, 0.f)) ? ((val > 1.f || floatEquals(val, 1.f) ) ? 0 : 1) : 2;
				}

				return index;
			}

		public:

			static Eigen::Array<T, Dimension, 1> particleLoc(Eigen::Array<T, Dimension, 1> xp, const Eigen::Array<T, Dimension, 1> &origin) {
				return xp - origin;
			}

			static Eigen::Array<T, Dimension, 1> baseNodeLoc(Eigen::Array < T, Dimension, 1> xp, const Eigen::Array<T, Dimension, 1> &origin, float cellSize) {
				Eigen::Array<float, Dimension, 1> fill_cellSize = Eigen::Array<float, Dimension, 1>();
				fill_cellSize.fill(cellSize);
				return (xp - origin) - fill_cellSize;
			}

			// calc N and N' values for curr particle positions
			void fillWeights(Eigen::Array<T, Dimension, 1> xp, float cellSize, const Eigen::Array<T, Dimension, 1> &origin) {
				// here xp, xi are orig values (no modifications)

				// location of lower left hand node [ie if grid system is of cellSize 1, and point p is at 2,2 then 
				// this node at 0, 0
				auto pLoc = particleLoc(xp, origin);
				auto baseNode = baseNodeLoc(xp, origin, cellSize);

				// calc weights for grid positions being checked
				for (int i = 0; i < Dimension; ++i) {

					// iterating temp location on grid nodes
					auto temp = baseNode;
					for (int j = 0; j < 3; ++j) {
						temp[i] += j * cellSize;

						calcN(i, j, pLoc, temp, cellSize);
						calcN_deriv(i, j, pLoc, temp, cellSize);
					}
				}

			}

			// calculating Wip with reference to a specific cell xi
			float calcWip(Eigen::Array<T, Dimension, 1> xp_modif, Eigen::Array<T, Dimension, 1> xi_modif, float cellSize) {
				// here xp, xi have already been modified by origin but have not yet been divided by cellSize

				Eigen::Array<T, Dimension, 1> index = indexInto(xp_modif, xi_modif, cellSize);

				// calc weights for grid positions being checked
				float val = 1.f;
				for (int i = 0; i < Dimension; ++i) {
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
		template <Attribute A, typename ParticleSet, typename AttributeGrid>
		static void ParticleToGrid(const ParticleSet &particleSet, AttributeGrid &attributeGrid, const Eigen::Array<T, Dimension, 1> &origin) {
			
			/*
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
			}*/
			
			
			const auto& keyAttributes = particleSet.GetAttributeList<Key>(); // pos
			const auto& sourceAttributes = particleSet.GetAttributeList<A>(); // attribute
			const auto& weightAttributes = particleSet.GetAttributeList<SimulationAttribute::Weights>();
			auto& targetGrid = attributeGrid.GetGrid<A>();

			//int cellSize_int = static_cast<unsigned int>(attributeGrid.CellSize());
			auto cellSize = attributeGrid.CellSize();

			for (unsigned int p = 0; p < particleSet.Size(); p++) {
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

				auto sumGradWeights = Eigen::Array<float, Dimension, 1>();
				sumGradWeights.fill(0.f);

				// THIS PART MANUALLY CODED FOR 3 DIMENSIONS
				for (unsigned int i = 0; i < 3; i++) {
					gridLoc[0] = baseNodeLoc[0] + i * cellSize;
					for (unsigned int j = 0; j < 3; j++) {
						gridLoc[1] = baseNodeLoc[1] + j * cellSize;
						for (unsigned int k = 0; k < 3; k++) {
							gridLoc[2] = baseNodeLoc[2] + k * cellSize;

							auto weightVal_calc = weights.calcWip(pLoc, gridLoc, cellSize);
							Eigen::Array<T, Dimension, 1> sourceVal_weighted = sourceVal * weightVal_calc;

							sumGradWeights += weightVal_calc;
							if (p == 0) {
								std::cout << "P2G gridVal_w: weights num: " << (i*k*j + k*j + k) << " =" << weightVal_calc << std::endl;
							}

							Eigen::Array<unsigned int, Dimension, 1> checkLoc = (gridLoc / cellSize).cast<unsigned int>();
							auto* gridAttributeAtLoc = targetGrid.at(checkLoc);
							// if grid location exits in grid then alter it
							if (gridAttributeAtLoc) {
								*gridAttributeAtLoc += sourceVal_weighted;
							}
						}
					}
				} // end: looping through all three dimensions for interpolating grid value

				if (p == 0) {
					std::cout << "sumGradWeights: " << sumGradWeights << std::endl;
				}

			} // end: looping through each particle

			
		}

		// G2P
		template <Attribute A, typename ParticleSet, typename AttributeGrid>
		static void GridToParticle(const AttributeGrid &attributeGrid, const Eigen::Array<T, Dimension, 1> &origin, ParticleSet &particleSet) {
			
			/*
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
			*/

			const auto& keyAttributes = particleSet.GetAttributeList<Key>(); // pos
			auto& targetAttributes = particleSet.GetAttributeList<A>(); // attribute
			const auto& weightAttributes = particleSet.GetAttributeList<SimulationAttribute::Weights>();
			const auto& sourceGrid = attributeGrid.GetGrid<A>();

			//int cellSize_int = static_cast<unsigned int>(attributeGrid.CellSize());
			auto cellSize = attributeGrid.CellSize();

			for (unsigned int p = 0; p < particleSet.Size(); p++) {
				auto& key = keyAttributes[p];
				auto weights = weightAttributes[p];

				// location of lower left hand node [ie if grid system is of cellSize 1, and point p is at 2,2 then 
				// this node at 0, 0
				Eigen::Array<T, Dimension, 1> baseNodeLoc = WeightVals::baseNodeLoc(key, origin, cellSize);
				Eigen::Array<T, Dimension, 1> pLoc = WeightVals::particleLoc(key, origin);

				// iterating gridLoc for particle
				Eigen::Array<float, Dimension, 1> gridLoc = Eigen::Array<float, Dimension, 1>();
				gridLoc.fill(0.f);

				auto& gridVal_weighted = Eigen::Array<float, Dimension, 1>();
				gridVal_weighted.fill(0.f);

				// THIS PART MANUALLY CODED FOR 3 DIMENSIONS
				for (unsigned int i = 0; i < 3; i++) {
					gridLoc[0] = baseNodeLoc[0] + i * cellSize;
					for (unsigned int j = 0; j < 3; j++) {
						gridLoc[1] = baseNodeLoc[1] + j * cellSize;
						for (unsigned int k = 0; k < 3; k++) {
							gridLoc[2] = baseNodeLoc[2] + k * cellSize;

							Eigen::Array<unsigned int, Dimension, 1> checkLoc = (gridLoc / cellSize).cast<unsigned int>();
							auto* gridAttributeAtLoc = sourceGrid.at(checkLoc);
							// if grid location exits in grid then alter particle val based on grid val
							if (gridAttributeAtLoc) {

								gridVal_weighted += *gridAttributeAtLoc * weights.calcWip(pLoc, gridLoc, cellSize);
							}
						}
					}
				} // end: looping through all three dimensions for interpolating grid value

				targetAttributes[p] = gridVal_weighted;

			} // end: looping through each particle
		
		}
		
	};


}