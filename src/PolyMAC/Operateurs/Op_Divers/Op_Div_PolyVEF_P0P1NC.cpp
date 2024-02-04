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

#include <Champ_Face_PolyVEF.h>
#include <Op_Div_PolyVEF_P0P1NC.h>
#include <Check_espace_virtuel.h>
#include <Domaine_Cl_PolyMAC.h>
#include <Navier_Stokes_std.h>
#include <Schema_Temps_base.h>
#include <Probleme_base.h>
#include <EcrFicPartage.h>
#include <Matrice_Morse.h>
#include <Matrix_tools.h>
#include <Array_tools.h>
#include <Debog.h>

Implemente_instanciable(Op_Div_PolyVEF_P0P1NC, "Op_Div_PolyVEF_P0P1NC", Op_Div_PolyMAC);

Sortie& Op_Div_PolyVEF_P0P1NC::printOn(Sortie& s) const { return s << que_suis_je(); }

Entree& Op_Div_PolyVEF_P0P1NC::readOn(Entree& s) { return s; }

void Op_Div_PolyVEF_P0P1NC::dimensionner_blocs(matrices_t matrices, const tabs_t& semi_impl) const
{
  const Domaine_PolyVEF_P0P1NC& domaine = ref_cast(Domaine_PolyVEF_P0P1NC, le_dom_PolyMAC.valeur());
  const Champ_Face_PolyVEF& ch = ref_cast(Champ_Face_PolyVEF, equation().inconnue().valeur());
  const DoubleTab& inco = ch.valeurs(), &press = ref_cast(Navier_Stokes_std, equation()).pression()->valeurs();
  const IntTab& e_f = domaine.elem_faces(), &f_e = domaine.face_voisins(), &fcl = ch.fcl();
  int i, j, e, f, ne_tot = domaine.nb_elem_tot(), d, D = dimension;

  Matrice_Morse *matv = matrices.count("vitesse") ? matrices["vitesse"] : nullptr, *matp = matrices.count("pression") ? matrices["pression"] : nullptr, matv2, matp2;
  IntTab sten_v(0, 2), sten_p(0, 2);
  DoubleTab w2; //matrice w2 aux elements (la meme que dans Op_Grad et Assembleur_P)

  for (f = 0; matv && f < domaine.nb_faces(); f++) /* dependance en v : divergence par elem + v = v_imp aux faces de Dirichlet */
    for (i = 0; i < 2; i++)
      if ((e = f_e(f, i)) >= 0 && e < domaine.nb_elem())
        for (d = 0; d < D; d++)
          sten_v.append_line(e, D * f + d); /* contribution a la divergence aux elems */
      else if (e < 0 && fcl(f, 0) > 1)
        for (d = 0; d < D; d++)
          sten_v.append_line(ne_tot + f, D * f + d); /* CL a vitesse normale imposee : equation v_norm = v_cl */

  for (e = 0; matp && e < domaine.nb_elem_tot(); e++)
    for (domaine.W2(nullptr, e, w2), i = 0; i < w2.dimension(0); i++) /* dependance en p : equation sur p_f */
      if (fcl(f = e_f(e, i), 0) == 1)
        sten_p.append_line(ne_tot + f, ne_tot + f); /* aux faces de Neumann : p_f = p_imp */
      else if (!fcl(f, 0))
        for (sten_p.append_line(ne_tot + f, e), j = 0; j < w2.dimension(1); j++) /* aux faces internes : egalite des deux gradients */
          if (w2(i, j, 0))
            sten_p.append_line(ne_tot + f, ne_tot + e_f(e, j));
  if (matp)
    for (e = 0; e < domaine.nb_elem(); e++)
      sten_p.append_line(e, e); //diagonale du vide!

  if (matv)
    tableau_trier_retirer_doublons(sten_v), Matrix_tools::allocate_morse_matrix(press.size_totale(), inco.size_totale(), sten_v, matv2);
  if (matp)
    tableau_trier_retirer_doublons(sten_p), Matrix_tools::allocate_morse_matrix(press.size_totale(), press.size_totale(), sten_p, matp2);
  if (matv)
    matv->nb_colonnes() ? *matv += matv2 : *matv = matv2;
  if (matp)
    matp->nb_colonnes() ? *matp += matp2 : *matp = matp2;
}

void Op_Div_PolyVEF_P0P1NC::ajouter_blocs_ext(const DoubleTab& vit, matrices_t matrices, DoubleTab& secmem, const tabs_t& semi_impl) const
{
  const Domaine_PolyVEF_P0P1NC& domaine = ref_cast(Domaine_PolyVEF_P0P1NC, le_dom_PolyMAC.valeur());
  const Champ_Face_PolyVEF& ch = ref_cast(Champ_Face_PolyVEF, equation().inconnue().valeur());
  const Conds_lim& cls = le_dcl_PolyMAC->les_conditions_limites();
  const DoubleTab& press = ref_cast(Navier_Stokes_std, equation()).pression()->valeurs(), &nf = domaine.face_normales();
  const IntTab& e_f = domaine.elem_faces(), &f_e = domaine.face_voisins(), &fcl = ch.fcl();
  const DoubleVect& pf = equation().milieu().porosite_face();
  int i, j, e, f, fb, ne_tot = domaine.nb_elem_tot(), d, D = dimension, has_f = secmem.dimension_tot(0) > ne_tot;
  Matrice_Morse *matv = matrices.count("vitesse") ? matrices["vitesse"] : nullptr, *matp = matrices.count("pression") ? matrices["pression"] : nullptr; //, matv2, matp2;

  DoubleTrav w2; //matrice w2 aux elements (la meme que dans Op_Grad et Assembleur_P)

  DoubleTab& tab_flux_bords = flux_bords_;
  tab_flux_bords.resize(domaine.nb_faces_bord(), 1), tab_flux_bords = 0;

  for (f = 0; f < domaine.nb_faces(); f++) /* divergence aux elements + equations aux bords */
    {
      for (i = 0; i < 2 && (e = f_e(f, i)) >= 0; i++)
        if (e < domaine.nb_elem()) /* divergence aux elems */
          for (d = 0; d < D; d++)
            {
              secmem(e) -= (i ? 1 : -1) * nf(f, d) * pf(f) * vit(f, d);
              if (matv)
                (*matv)(e, D * f + d) += (i ? 1 : -1) * pf(f) * nf(f, d);
            }

      if (f < domaine.premiere_face_int())
        for (d = 0; d < D; d++)
          tab_flux_bords(f) += nf(f, d) * pf(f) * vit(f, d);

      /* equations v = v_imp ou p = p_imp aux faces de bord */
      if (has_f && fcl(f, 0) > 1)
        for (d = 0; d < D; d++)
          {
            secmem(ne_tot + f) += nf(f, d) * pf(f) * ((fcl(f, 0) == 3 ? ref_cast(Dirichlet, cls[fcl(f, 1)].valeur()).val_imp(fcl(f, 2), d) : 0) - vit(f, d));
            if (matv)
              (*matv)(ne_tot + f, D * f + d) += nf(f, d) * pf(f);
          }
      else if (has_f && fcl(f, 0) == 1)
        {
          secmem(ne_tot + f) += ref_cast(Neumann, cls[fcl(f, 1)].valeur()).flux_impose(fcl(f, 2), 0) - press(ne_tot + f);
          if (matp)
            (*matp)(ne_tot + f, ne_tot + f) += 1;
        }
    }

  /* equations aux faces internes : egalite des gradients */
  if (!has_f) return;
  for (e = 0; e < domaine.nb_elem_tot(); e++)
    for (domaine.W2(nullptr, e, w2), i = 0; i < w2.dimension(0); i++)
      if ((f = e_f(e, i)) < domaine.nb_faces() && !fcl(f, 0))
        {
          double coeff_e = 0;
          for (j = 0; j < w2.dimension(1); j++)
            if (w2(i, j, 0))
              {
                fb = e_f(e, j);
                secmem(ne_tot + f) -= pf(f) * w2(i, j, 0) * (press(ne_tot + fb) - press(e));
                if (matp)
                  (*matp)(ne_tot + f, ne_tot + fb) += pf(f) * w2(i, j, 0), coeff_e += pf(f) * w2(i, j, 0);
              }
          if (matp)
            (*matp)(ne_tot + f, e) -= coeff_e;
        }
}
