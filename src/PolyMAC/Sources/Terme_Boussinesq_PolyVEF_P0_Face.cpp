/****************************************************************************
* Copyright (c) 2023, CEA
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

#include <Convection_Diffusion_Temperature.h>
#include <Terme_Boussinesq_PolyVEF_P0_Face.h>
#include <Fluide_Incompressible.h>
#include <Neumann_sortie_libre.h>
#include <Champ_Face_PolyVEF_P0.h>
#include <Domaine_PolyVEF_P0.h>
#include <Domaine_Cl_PolyMAC.h>
#include <Navier_Stokes_std.h>
#include <Champ_Uniforme.h>
#include <Pb_Multiphase.h>
#include <Synonyme_info.h>
#include <Dirichlet.h>

Implemente_instanciable(Terme_Boussinesq_PolyVEF_P0_Face, "Boussinesq_PolyVEF_P0_Face", Terme_Boussinesq_PolyMAC_Face);
Add_synonym(Terme_Boussinesq_PolyVEF_P0_Face, "Boussinesq_temperature_Face_PolyVEF_P0");
Add_synonym(Terme_Boussinesq_PolyVEF_P0_Face, "Boussinesq_concentration_PolyVEF_P0_Face");

Sortie& Terme_Boussinesq_PolyVEF_P0_Face::printOn(Sortie& s) const { return Terme_Boussinesq_PolyMAC_Face::printOn(s); }

Entree& Terme_Boussinesq_PolyVEF_P0_Face::readOn(Entree& s) { return Terme_Boussinesq_PolyMAC_Face::readOn(s); }

void Terme_Boussinesq_PolyVEF_P0_Face::ajouter_blocs(matrices_t matrices, DoubleTab& secmem, const tabs_t& semi_impl) const
{
  const Domaine_PolyMAC& domaine = le_dom_PolyMAC.valeur();
  const DoubleTab& param = equation_scalaire().inconnue().valeurs();
  const DoubleTab& beta_valeurs = beta().valeur().valeurs();
  const IntTab& f_e = domaine.face_voisins();
  const DoubleTab& rho = equation().milieu().masse_volumique().passe(), &vfd = domaine.volumes_entrelaces_dir(),
                   *alp = sub_type(Pb_Multiphase, equation().probleme()) ? &ref_cast(Pb_Multiphase, equation().probleme()).equation_masse().inconnue().passe() : nullptr;
  const DoubleVect& pf = equation().milieu().porosite_face(), &grav = gravite().valeurs();

  // Verifie la validite de T0:
  check();
  int e, i, f, n, nb_dim = param.line_size(), cR = (rho.dimension_tot(0) == 1), d, D = dimension;
  for (f = 0; f < domaine.nb_faces(); f++)
    for (i = 0; i < 2 && (e = f_e(f, i)) >= 0; i++) //contributions amont/aval
      {
        double coeff = 0;
        for (n = 0; n < nb_dim; n++)
          coeff += (alp ? (*alp)(e, n) * rho(!cR * e, n) : 1) * valeur(beta_valeurs, e, e, n) * (Scalaire0(n) - valeur(param, e, n));

        for (d = 0; d < D; d++)
          secmem(f, d) += coeff * grav(d) * vfd(f, i) * pf(f);
      }
}
