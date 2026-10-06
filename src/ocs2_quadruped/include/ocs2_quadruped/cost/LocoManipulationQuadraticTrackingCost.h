/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include "ocs2_quadruped/common/utils.h"
#include "ocs2_quadruped/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace quadruped {

/**
 * State-input tracking cost used for intermediate times
 */
        class LocoManipulationStateInputQuadraticCost final : public QuadraticStateInputCost {
        public:
            LocoManipulationStateInputQuadraticCost(matrix_t Q, matrix_t R, CentroidalModelInfo info)
                    : QuadraticStateInputCost(std::move(Q), std::move(R)), info_(std::move(info)) {}

            ~LocoManipulationStateInputQuadraticCost() override = default;
            LocoManipulationStateInputQuadraticCost* clone() const override { return new LocoManipulationStateInputQuadraticCost(*this); }

        private:
            LocoManipulationStateInputQuadraticCost(const LocoManipulationStateInputQuadraticCost& rhs) = default;

            std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& targetTrajectories) const override {
              // Reference is the end-effector trajectory
              vector_t nominal_traj(info_.stateDim);
              nominal_traj << vector_t::Zero(6), info_.qPinocchioNominal;
              const vector_t xNominal = nominal_traj;
              
              // Assume always on the ground
              const vector_t uNominal = weightCompensatingInput(info_, {true, true, true, true});
              return {state - xNominal, input - uNominal};
            }

            const CentroidalModelInfo info_;
            // const SwitchedModelReferenceManager* referenceManagerPtr_;
        };

/**
 * State tracking cost used for the final time
 */
        class LocoManipulationStateQuadraticCost final : public QuadraticStateCost {
        public:
            LocoManipulationStateQuadraticCost(matrix_t Q, CentroidalModelInfo info)
                    : QuadraticStateCost(std::move(Q)), info_(std::move(info)) {}

            ~LocoManipulationStateQuadraticCost() override = default;
            LocoManipulationStateQuadraticCost* clone() const override { return new LocoManipulationStateQuadraticCost(*this); }

        private:
            LocoManipulationStateQuadraticCost(const LocoManipulationStateQuadraticCost& rhs) = default;

            vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
              // Reference is the end-effector trajectory
              vector_t nominal_traj(info_.stateDim);
              nominal_traj << vector_t::Zero(6), info_.qPinocchioNominal;
              const vector_t xNominal = nominal_traj;
              return state - xNominal;
            }

            const CentroidalModelInfo info_;
            // const SwitchedModelReferenceManager* referenceManagerPtr_;
        };

    }  // namespace quadruped
}  // namespace ocs2
