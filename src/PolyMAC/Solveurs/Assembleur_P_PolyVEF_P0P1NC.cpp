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

#include <Champ_front_instationnaire_base.h>
#include <Champ_front_var_instationnaire.h>
#include <Assembleur_P_PolyVEF_P0P1NC.h>
#include <Op_Grad_PolyVEF_P0P1NC_Face.h>
#include <Op_Div_PolyVEF_P0P1NC.h>
#include <Champ_Face_PolyVEF.h>
#include <Domaine_PolyVEF.h>
#include <Neumann_sortie_libre.h>
#include <Domaine_Cl_PolyMAC.h>
#include <Matrice_Morse_Sym.h>
#include <Matrice_Diagonale.h>
#include <Static_Int_Lists.h>
#include <Matrice_Bloc_Sym.h>
#include <Operateur_Grad.h>
#include <TRUSTTab_parts.h>
#include <Pb_Multiphase.h>
#include <Matrix_tools.h>
#include <Statistiques.h>
#include <Array_tools.h>
#include <Dirichlet.h>
#include <Debog.h>

extern Stat_Counter_Id assemblage_sys_counter_;

Implemente_instanciable(Assembleur_P_PolyVEF_P0P1NC, "Assembleur_P_PolyVEF_P0P1NC", Assembleur_P_PolyMAC);

Sortie& Assembleur_P_PolyVEF_P0P1NC::printOn(Sortie& s) const { return s << que_suis_je() << " " << le_nom(); }

Entree& Assembleur_P_PolyVEF_P0P1NC::readOn(Entree& s) { return Assembleur_base::readOn(s); }


int  Assembleur_P_PolyVEF_P0P1NC::assembler_mat(Matrice& la_matrice,const DoubleVect& diag,int incr_pression,int resoudre_en_u)
{
  set_resoudre_increment_pression(incr_pression);
  set_resoudre_en_u(resoudre_en_u);
  la_matrice.typer("Matrice_Morse");
  Matrice_Morse& mat = ref_cast(Matrice_Morse, la_matrice.valeur()), mat_div_v, mat_div_p, mat_grad;
  const Navier_Stokes_std& eq = ref_cast(Navier_Stokes_std, equation());
  const Op_Div_PolyVEF_P0P1NC& div = ref_cast(Op_Div_PolyVEF_P0P1NC, eq.operateur_divergence().valeur());
  const Op_Grad_PolyVEF_P0P1NC_Face& grad = ref_cast(Op_Grad_PolyVEF_P0P1NC_Face, eq.operateur_gradient().valeur());

  const Domaine_PolyVEF_P0P1NC& dom = ref_cast(Domaine_PolyVEF_P0P1NC, le_dom_PolyMAC.valeur());
  int ne_tot = dom.nb_elem_tot(), nf_tot = dom.nb_faces_tot(), d, D = dimension, f, i;
  const DoubleVect& pf = equation().milieu().porosite_face(), &vf = dom.volumes_entrelaces();

  /* 1. stencil : seulement au premier passage */
  if (!stencil_done)
    {
      //dimensionner_blocs de div et grad, puis produit
      div.dimensionner_blocs({ { "vitesse", &mat_div_v }, { "pression", &mat_div_p } });
      grad.dimensionner_blocs_ext({ { "pression", &mat_grad } }, 1); //avec les faces virtuelles
      mat.affecte_prod(mat_div_v, mat_grad), mat.set_nb_columns(ne_tot + nf_tot);
      mat += mat_div_p;
      //stockage des stencils : pour mat, on ne peut stocker que la taille
      tab1.resize(1), tab1(0) = mat.get_set_tab1().size(), tab2.resize(1), tab2(0) = mat.get_set_tab2().size();
      div_v_tab1.ref_array(mat_div_v.get_set_tab1()), div_v_tab2.ref_array(mat_div_v.get_set_tab2());
      div_p_tab1.ref_array(mat_div_p.get_set_tab1()), div_p_tab2.ref_array(mat_div_p.get_set_tab2());
      grad_tab1.ref_array(mat_grad.get_set_tab1()), grad_tab2.ref_array(mat_grad.get_set_tab2());
      stencil_done = 1;
    }
  else //sinon, on recycle
    {
      mat.get_set_tab1().resize(tab1(0)), mat.get_set_tab2().resize(tab2(0)), mat.get_set_coeff().resize(tab2(0)), mat.set_nb_columns(ne_tot + nf_tot);
      mat_div_v.get_set_tab1().ref_array(div_v_tab1), mat_div_v.get_set_tab2().ref_array(div_v_tab2), mat_div_v.get_set_coeff().resize(div_v_tab2.size()), mat_div_v.set_nb_columns(D * nf_tot);
      mat_div_p.get_set_tab1().ref_array(div_p_tab1), mat_div_p.get_set_tab2().ref_array(div_p_tab2), mat_div_p.get_set_coeff().resize(div_p_tab2.size()), mat_div_p.set_nb_columns(ne_tot + nf_tot);
      mat_grad.get_set_tab1().ref_array(grad_tab1), mat_grad.get_set_tab2().ref_array(grad_tab2), mat_grad.get_set_coeff().resize(grad_tab2.size()), mat_grad.set_nb_columns(ne_tot + nf_tot);
    }

  /* gradient : par op_grad, puis produit par -1 / diag */
  DoubleTrav sec_grad(nf_tot, D), sec_div(ne_tot + nf_tot, 1), inv_diag(D * nf_tot);
  grad.ajouter_blocs_ext({ { "pression", &mat_grad } }, sec_grad, 1); //avec lignes virtuelles
  div.ajouter_blocs({ { "vitesse", &mat_div_v }, { "pression", &mat_div_p } }, sec_div); //avec lignes virtuelles
  for (f = 0, i = 0; f < nf_tot; f++)
    for (d = 0; d < D; d++, i++) inv_diag(i) = 1. / (diag.size() ? diag(i) : pf(f) * vf(f));
  mat_grad.diagmulmat(inv_diag);
  mat.affecte_prod(mat_div_v, mat_grad);
  mat_div_p *= -1;
  mat += mat_div_p; //partie (p, p) (pressions imposees) de la divergence

  //en l'absence de CLs en pression, on ajoute P(0) = 0 sur le process premier process dont le domaine est non vide
  has_P_ref=0;
  for (int n_bord=0; n_bord<le_dom_PolyMAC->nb_front_Cl(); n_bord++)
    if (sub_type(Neumann_sortie_libre, le_dom_Cl_PolyMAC->les_conditions_limites(n_bord).valeur()) )
      has_P_ref=1;

  return 1;
}

/* equations sum_k alpha_k = 1, [grad p]_{fe} = [grad p]_{fe'} en Pb_Multiphase */
void Assembleur_P_PolyVEF_P0P1NC::dimensionner_continuite(matrices_t matrices, int aux_only) const
{
  const Domaine_PolyVEF_P0P1NC& domaine = ref_cast(Domaine_PolyVEF_P0P1NC, le_dom_PolyMAC.valeur());
  int i, j, e, f, fb, d, D = dimension, n, N = equation().inconnue()->valeurs().line_size() / D, m, M = equation().get_champ("pression").valeurs().line_size(),
                         ne_tot = domaine.nb_elem_tot(), nf_tot = domaine.nb_faces_tot();
  const IntTab& fcl = ref_cast(Champ_Face_PolyVEF, mon_equation->inconnue().valeur()).fcl(), &e_f = domaine.elem_faces();
  IntTrav sten_a(0, 2), sten_p(0, 2), sten_v(0, 2);
  DoubleTrav w2;
  /* equations sum alpha_k = 1 */
  if (!aux_only)
    for (e = 0; e < domaine.nb_elem(); e++)
      for (n = 0; n < N; n++) sten_a.append_line(e, N * e + n);
  /* equations sur les p_f : continuite du gradient si interne, p = p_f si Neumann, sum_k alpha_k v_k = sum_k alpha_k v_k,imp si Dirichlet */
  for (e = 0; e < domaine.nb_elem_tot(); e++)
    for (domaine.W2(nullptr, e, w2), i = 0; i < w2.dimension(0); i++)
      if ((f = e_f(e, i)) >= domaine.nb_faces()) continue; //faces virtuelles
      else if (!fcl(f, 0))
        for (sten_p.append_line(!aux_only * ne_tot + f, e), j = 0; j < w2.dimension(1); j++) //face interne
          {
            if (w2(i, j, 0) && fcl(fb = e_f(e, j), 0) != 1)
              for (m = 0; m < M; m++)
                sten_p.append_line(M * (!aux_only * ne_tot + f) + m, M * (ne_tot + fb) + m);
          }
      else if (fcl(f, 0) == 1)
        for (m = 0; m < M; m++) sten_p.append_line(M * (!aux_only * ne_tot + f) + m, M * (ne_tot + f) + m); //Neumann
      else for (d = 0; d < D; d++)
          for (n = 0, m = 0; n < N; n++, m += (M > 1))
            sten_v.append_line(M * (!aux_only * ne_tot + f) + m, N * (D * f + d) + n); //Dirichlet

  tableau_trier_retirer_doublons(sten_v), tableau_trier_retirer_doublons(sten_p);
  if (!aux_only) Matrix_tools::allocate_morse_matrix(ne_tot + nf_tot, N * ne_tot, sten_a, *matrices.at("alpha"));
  Matrix_tools::allocate_morse_matrix(M * (!aux_only * ne_tot + nf_tot), M * (ne_tot + nf_tot), sten_p, *matrices.at("pression"));
  Matrix_tools::allocate_morse_matrix(M * (!aux_only * ne_tot + nf_tot), equation().inconnue()->valeurs().size_totale(), sten_v, *matrices.at("vitesse"));
}

void Assembleur_P_PolyVEF_P0P1NC::assembler_continuite(matrices_t matrices, DoubleTab& secmem, int aux_only) const
{
  const Domaine_PolyVEF_P0P1NC& domaine = ref_cast(Domaine_PolyVEF_P0P1NC, le_dom_PolyMAC.valeur());
  const Pb_Multiphase* pbm = sub_type(Pb_Multiphase, equation().probleme()) ? &ref_cast(Pb_Multiphase, equation().probleme()) : nullptr;
  const Conds_lim& cls = le_dom_Cl_PolyMAC->les_conditions_limites();
  const DoubleTab *alpha = pbm ? &pbm->equation_masse().inconnue()->valeurs() : nullptr, &press = equation().probleme().get_champ("pression").valeurs(),
                   &vit = equation().inconnue()->valeurs(), *alpha_rho = pbm ? &pbm->equation_masse().champ_conserve().passe() : nullptr, &nf = domaine.face_normales();
  const IntTab& fcl = ref_cast(Champ_Face_PolyVEF, mon_equation->inconnue().valeur()).fcl(), &e_f = domaine.elem_faces();
  const DoubleVect& ve = domaine.volumes(), &pe = equation().milieu().porosite_elem(), &fs = domaine.face_surfaces(), &vf = domaine.volumes_entrelaces();
  int i, j, e, f, fb, d, D = dimension, n, N = vit.line_size() / D, m, M = press.line_size(), ne_tot = domaine.nb_elem_tot();
  Matrice_Morse *mat_a = alpha ? matrices.at("alpha") : nullptr, &mat_p = *matrices.at("pression"), &mat_v = *matrices.at("vitesse");
  DoubleTrav w2, fac(N);
  double ar_tot = 1, acc, dt = equation().schema_temps().pas_de_temps(), fac2;
  secmem = 0, fac = 1;

  /* equations sum alpha_k = 1 */
  if (!aux_only)
    for (e = 0; e < domaine.nb_elem(); e++)
      {
        if (alpha_rho)
          for (ar_tot = 0, n = 0; n < N; n++)
            ar_tot += (*alpha_rho)(e, n);
        for (fac2 = ve(e) * pe(e) * ar_tot / dt, secmem(e) = -fac2, n = 0; n < N; n++) secmem(e) += fac2 * (*alpha)(e, n);
        for (n = 0; n < N; n++) (*mat_a)(e, N * e + n) = -fac2;
      }

  /* equations sur les p_f : continuite du gradient si interne, p = p_f si Neumann, sum_k alpha_k v_k = sum_k alpha_k v_k,imp si Dirichlet */
  for (mat_p.get_set_coeff() = 0, mat_v.get_set_coeff() = 0, e = 0; e < ne_tot; e++)
    for (domaine.W2(nullptr, e, w2), i = 0; i < w2.dimension(0); i++)
      if ((f = e_f(e, i)) >= domaine.nb_faces()) continue; //faces virtuelles
      else if (!fcl(f, 0)) //face interne
        {
          for (acc = 0, j = 0; j < w2.dimension(1); acc+= pe(e) * vf(f) * w2(i, j, 0), j++)
            for (m = 0; m < M; m++) //second membre
              secmem(!aux_only * ne_tot + f, m) -= pe(e) * vf(f) * w2(i, j, 0) * (press(ne_tot + e_f(e, j), m) - press(e, m));
          for (m = 0; m < M; m++) mat_p(M * (!aux_only * ne_tot + f) + m, M * e + m) -= acc;
          for (j = 0; j < w2.dimension(1); j++) //matrice (sauf bords de Meumann)
            if (w2(i, j, 0) && fcl(fb = e_f(e, j), 0) != 1)
              for (m = 0; m <M; m++)
                mat_p(M * (!aux_only * ne_tot + f) + m, M * (ne_tot + fb) + m) += pe(e) * vf(f) * w2(i, j, 0);
        }
      else if (fcl(f, 0) == 1) //Neumann -> egalites p_f = p_imp
        {
          for (m = 0; m < M; m++) secmem(M * (!aux_only * ne_tot + f) + m) = fs(f) * (ref_cast(Neumann, cls[fcl(f, 1)].valeur()).flux_impose(fcl(f, 2), m) - press(ne_tot + f, m));
          for (m = 0; m < M; m++) mat_p(M * (!aux_only * ne_tot + f) + m, M * (ne_tot + f) + m) = fs(f);
        }
      else  //Dirichlet -> egalite flux_tot_imp - flux_tot = 0
        {
          for (d = 0; d < D; d++)
            for (n = 0, m = 0; n < N; n++, m += (M > 1))
              secmem(!aux_only * ne_tot + f, m) += (alpha_rho ? (*alpha_rho)(e, n) : 1) * nf(f, d) * vit(f, N * d + n);
          if (fcl(f, 0) == 3)
            for (d = 0; d < D; d++)
              for (n = 0, m = 0; n < N; n++, m += (M > 1)) //contrib de la valeur imposee: Dirichlet non homogene seulement
                secmem(!aux_only * ne_tot + f, m) -= (alpha_rho ? (*alpha_rho)(e, n) : 1) * nf(f, d) * ref_cast(Dirichlet, cls[fcl(f, 1)].valeur()).val_imp(fcl(f, 2), N * d + n);
          for (d = 0; d < D; d++)
            for (n = 0, m = 0; n < N; n++, m += (M > 1))
              mat_v(M * (!aux_only * ne_tot + f) + m, N * (D * f + d) + n) -= (alpha_rho ? (*alpha_rho)(e, n) : 1) * nf(f, d);
        }
}

void Assembleur_P_PolyVEF_P0P1NC::modifier_secmem_pour_incr_p(const DoubleTab& press, const double fac, DoubleTab& secmem) const
{
  const Domaine_PolyVEF_P0P1NC& domaine = ref_cast(Domaine_PolyVEF_P0P1NC, le_dom_PolyMAC.valeur());
  const Champ_Face_PolyVEF& ch = ref_cast(Champ_Face_PolyVEF, mon_equation->inconnue().valeur());
  const Conds_lim& cls = le_dom_Cl_PolyMAC->les_conditions_limites();
  const IntTab& fcl = ch.fcl();
  int f, ne_tot = domaine.nb_elem_tot(), m, M = equation().probleme().get_champ("pression").valeurs().line_size();
  for (f = 0; f < domaine.premiere_face_int(); f++)
    if (fcl(f, 0) == 1)
      for (m = 0; m < M; m++)
        secmem(ne_tot + f, m) = (ref_cast(Neumann_sortie_libre, cls[fcl(f, 1)].valeur()).flux_impose(fcl(f, 2), m) - press(ne_tot + f, m)) / fac;
}

/* norme pour assembler_continuite */
DoubleTab Assembleur_P_PolyVEF_P0P1NC::norme_continuite() const
{
  const DoubleVect& pe= equation().milieu().porosite_elem(), &ve = le_dom_PolyMAC->volumes();
  DoubleTab norm(le_dom_PolyMAC->nb_elem());
  for (int e = 0; e < le_dom_PolyMAC->nb_elem(); e++) norm(e) = pe(e) * ve(e);
  return norm;
}

int Assembleur_P_PolyVEF_P0P1NC::modifier_solution(DoubleTab& pression)
{
  Debog::verifier("pression dans modifier solution in",pression);
  if(!has_P_ref) pression -= mp_min_vect(pression);
  return 1;
}