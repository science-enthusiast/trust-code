/****************************************************************************
* Copyright (c) 2024, CEA
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

#include <Eval_Dift_VDF_leaves.h>

void Eval_Dift_VDF_Elem::init_ind_fluctu_term()
{
  ind_Fluctu_Term = 0;
  if (loipar.est_nul()) ind_Fluctu_Term = 1;
}

void Eval_Dift_VDF_Elem::associer_loipar(const Turbulence_paroi_scal_base& loi_paroi)
{
  Eval_Dift_VDF::associer_loipar(loi_paroi);
  ind_Fluctu_Term = 0;
}

void Eval_Dift_VDF_Face::mettre_a_jour()
{
  Eval_Dift_VDF::mettre_a_jour();
  if (le_modele_turbulence->has_loi_paroi_hyd())
    {
      // Modif E. Saikali : on fait le ref seulement si le tableau a ete initialise, sinon pointeur nulle
      const DoubleTab& tab = le_modele_turbulence->loi_paroi().Cisaillement_paroi();
      if (tab.size_array() > 0) tau_tan_.ref(tab);
    }
}

double Eval_Dift_VDF_Face::tau_tan_impl(int face, int k) const
{
  const int nb_faces = le_dom->nb_faces();
  const ArrOfInt& ind_faces_virt_bord = le_dom->ind_faces_virt_bord();
  int f = (face >= tau_tan_.dimension(0)) ? ind_faces_virt_bord[face-nb_faces] : face;
  if(f >= tau_tan_.dimension_tot(0))
    {
      Cerr << "Erreur dans tau_tan " << finl;
      Cerr << "dimension : " << tau_tan_.dimension(0) << finl;
      Cerr << "dimension_tot : " << tau_tan_.dimension_tot(0) << finl;
      Cerr << "face : " << face << finl;
      Process::exit();
    }
  return tau_tan_(f,k);
}
