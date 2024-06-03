/****************************************************************************
* Copyright (c) 2022, CEA
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/

#ifndef Multiplicateur_diphasique_homogene_included
#define Multiplicateur_diphasique_homogene_included
#include <Multiplicateur_diphasique_base.h>

/*! @brief classe Multiplicateur_diphasique_homogene multiplicateur diphasique homogene : Phi^2 = 1 + x (rho_l / rho_g - 1)
 *
 *     raccord vers la phase vapeur a partir de alpha_min et jusqu'a alpha_max
 *
 *
 */

class Multiplicateur_diphasique_homogene : public Multiplicateur_diphasique_base
{
  Declare_instanciable(Multiplicateur_diphasique_homogene);
public:
  void coefficient(const double *alpha, const double *rho, const double *v, const double *f,
                   const double *mu, const double Dh, const double gamma, const double *Fk,
                   const double Fm, DoubleTab& coeff) const override;
protected:
  double alpha_min_ = 0.9995, alpha_max_ = 1;
  int n_l = -1, n_g = -1; //indices des phases frottantes (liquide/gaz)
};

#endif
