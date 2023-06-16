// Copyright 2018 CNRS - Airbus SAS
// Author: Joseph Mirabel
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef AGIMUS_SOT_HOLONOMIC_CONSTRAINT_HH
# define AGIMUS_SOT_HOLONOMIC_CONSTRAINT_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-geometry.hh>

#include <agimus/sot/config.hh>

namespace dynamicgraph {
  namespace agimus {
      /// Holonomic constraints
      /// Implements command law from section IV.4.3 of http://homepages.laas.fr/florent/publi/these.pdf
      class AGIMUS_SOT_DLLAPI HolonomicConstraint : public dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

        HolonomicConstraint (const std::string& name);

        ~HolonomicConstraint () {}

        /// Header documentation of the python class
        virtual std::string getDocString () const
        {
          return
            "Compute the control (v, w) of size 2 for a mobile base.\n"
            "  Signal velocityRef is the desired velocity (v_r, w_r) expressed in the desired frame.\n";
        }

        void setNumberDoF (const unsigned int& d)
        { dim_ = d; }

        private:
        Vector& computeError     (Vector& error    , const sigtime_t& time);
        Vector& computeControl   (Vector& control  , const sigtime_t& time);
        Matrix& computeProjector (Matrix& projector, const sigtime_t& time);

        unsigned int dim_;

        SignalPtr <double, sigtime_t> g1SIN;
        SignalPtr <double, sigtime_t> g2SIN;
        SignalPtr <double, sigtime_t> g3SIN;

        SignalPtr <sot::MatrixHomogeneous, sigtime_t> positionSIN;
        SignalPtr <sot::MatrixHomogeneous, sigtime_t> positionRefSIN;
        SignalPtr <Vector, sigtime_t> velocityRefSIN;

        Signal <Vector, sigtime_t> errorSOUT;
        Signal <Vector, sigtime_t> controlSOUT;
        Signal <Matrix, sigtime_t> projectorSOUT;
      };
  } // namespace agimus
} // namespace dynamicgraph
#endif // AGIMUS_SOT_HOLONOMIC_CONSTRAINT_HH
