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

#include <Multiplicateur_diphasique_base.h>
#include <Fluide_Incompressible.h>
#include <Perte_Charge_PolyVEF.h>
#include <Champ_Face_PolyVEF.h>
#include <Domaine_PolyVEF.h>
#include <Domaine_Cl_PolyMAC.h>
#include <Domaine_PolyVEF.h>
#include <Schema_Temps_base.h>
#include <Milieu_composite.h>
#include <Champ_Uniforme.h>
#include <QDM_Multiphase.h>
#include <Pb_Multiphase.h>
#include <Probleme_base.h>
#include <Equation_base.h>
#include <Sous_Domaine.h>
#include <Param.h>
#include <Synonyme_info.h>

Implemente_base(Perte_Charge_PolyVEF, "Perte_Charge_PolyVEF_P0|Perte_Charge_PolyVEF_P0P1", Perte_Charge_PolyMAC_P0P1NC);
Add_synonym(Perte_Charge_PolyVEF, "Perte_Charge_PolyVEF_P0P1NC");

Sortie& Perte_Charge_PolyVEF::printOn(Sortie& s) const { return s << que_suis_je() << endl; }

Entree& Perte_Charge_PolyVEF::readOn(Entree& is) { return Perte_Charge_PolyMAC_P0P1NC::readOn(is); }

void Perte_Charge_PolyVEF::ajouter_blocs(matrices_t matrices, DoubleTab& secmem, const tabs_t& semi_impl) const
{
  assert(has_interface_blocs());
  const Domaine_Poly_base& domaine = ref_cast(Domaine_Poly_base, equation().domaine_dis().valeur());
  const Pb_Multiphase *pbm = sub_type(Pb_Multiphase, equation().probleme()) ? &ref_cast(Pb_Multiphase, equation().probleme()) : nullptr;
  const Champ_Face_PolyVEF& ch = ref_cast(Champ_Face_PolyVEF, equation().inconnue().valeur());
  const Champ_Don& dh = diam_hydr;

  const DoubleTab& xp = domaine.xp(), &vit = la_vitesse->valeurs(), &pvit = la_vitesse->passe(),
                   &nu = le_fluide->viscosite_cinematique()->valeurs(), &vfd = domaine.volumes_entrelaces_dir(),
                    &mu = le_fluide->viscosite_dynamique()->valeurs(), &rho = le_fluide->masse_volumique()->passe(),
                     *alp = pbm ? &pbm->equation_masse().inconnue()->passe() : nullptr;

  const DoubleVect& pf = le_fluide->porosite_face();
  const Multiplicateur_diphasique_base *fmult = pbm && pbm->has_correlation("multiplicateur_diphasique") ?
                                                &ref_cast(Multiplicateur_diphasique_base, pbm->get_correlation("multiplicateur_diphasique").valeur()) : nullptr;

  const Sous_Domaine *pssz = sous_domaine ? &le_sous_domaine.valeur() : nullptr;
  const IntTab& e_f = domaine.elem_faces(), &f_e = domaine.face_voisins();
  Matrice_Morse *mat = matrices.count(ch.le_nom().getString()) ? matrices.at(ch.le_nom().getString()) : nullptr;

  int i, j, k, f, d, D = dimension, cN = nu.dimension(0) == 1, cM = mu.dimension(0) == 1, cR = rho.dimension(0) == 1,
                     C_dh = sub_type(Champ_Uniforme, diam_hydr.valeur()), m, n, N = vit.line_size() / D;

  double t = equation().schema_temps().temps_courant(), v_min = 0.1, Gm, Fm, nvm, arm, C_dir, C_iso, v_dir;

  DoubleTrav pos(D), v(N, D), vm(D), v_ph(D), dir(D), C(N), nv(N), Cf(N), Cf_t(N), Fk(N), G(N), mult(N, 2), Ts_tab, Sigma_tab;

  for (n = 0; n < N; n++)
    mult(n, 0) = 1, mult(n, 1) = 0; //valeur par defaut de mult
  if (fmult) //si multiplicateur -> calcul de sigma
    {
      const Milieu_composite& milc = ref_cast(Milieu_composite, equation().milieu());
      // Et pour les methodes span de la classe Saturation
      const int ne_tot = domaine.nb_elem_tot(), nb_max_sat =  N * (N-1) /2; // oui !! suite arithmetique !!
      Ts_tab.resize(ne_tot, nb_max_sat), Sigma_tab.resize(ne_tot, nb_max_sat);
      for (k = 0; k < N; k++)
        for (int l = k + 1; l < N; l++)
          if (milc.has_saturation(k, l))
            {
              const Saturation_base& z_sat = milc.get_saturation(k, l);
              const int ind_trav = (k*(N-1)-(k-1)*(k)/2) + (l-k-1); // Et oui ! matrice triang sup !
              // recuperer Tsat et sigma ...
              const DoubleTab& sig = z_sat.get_sigma_tab();
              const DoubleTab& tsat = z_sat.get_Tsat_tab();

              // fill in the good case
              for (int ii = 0; ii < ne_tot; ii++)
                Sigma_tab(ii, ind_trav) = sig(ii), Ts_tab(ii, ind_trav) = tsat(ii);
            }
    }

  /* contribution de chaque element ou on applique la perte de charge */
  for (i = 0; i < (pssz ? pssz->nb_elem_tot() : domaine.nb_elem_tot()); i++)
    {
      int e = pssz ? (*pssz)[i] : i;
      for (d = 0; d < D; d++) pos(d) = xp(e, d);

      /* contributions aux faces de e */
      for (j = 0; j < e_f.dimension(1) && (f = e_f(e, j)) >= 0; j++)
        if (f < domaine.nb_faces())
          {
            /* vecteur vitesse en e */
            double dh_e = C_dh ? dh->valeurs()(0, 0) : dh->valeur_a_compo(pos, 0);
            for (d = 0; d < D; d++)
              for (n = 0; n < N; n++)
                v(n, d) = pvit(f, N * d + n); //vitesse par variable auxiliaire

            /* norme de v (avec seuil), debit surfacique par phase et total */
            for (n = 0, Gm = 0; n < N; Gm += G(n), n++)
              nv(n) = std::max(v_min, sqrt(domaine.dot(&v(n, 0), &v(n, 0)))), G(n) = (alp ? (*alp)(e, n) : 1) * rho(!cR * e, n) * nv(n);
            for (arm = 0, n = 0; n < N; n++)
              for (arm += (alp ? (*alp)(e, n) : 1) * rho(!cR * e, n), d = 0; d < D; d++)
                vm(d) += (alp ? (*alp)(e, n) : 1) * rho(!cR * e, n) * v(n, d);
            for (d = 0; d < D; d++)
              vm(d) /= arm;
            nvm = std::max(v_min, sqrt(domaine.dot(&vm(0), &vm(0))));

            /* coefficients de frottements par composante : Cf(n) (phase seule), Cf_t(n) (le debit total dans cette phase) */
            for (n = 0; n < N; n++)
              {
                double Re = dh_e * std::max(G(n), 1e-10) / mu(!cM * e, n), Re_m = dh_e * Gm / mu(!cM * e, n);
                for (d = 0; d < D; d++)
                  v_ph(d) = v(n, d);
                /* phase seule */
                coeffs_perte_charge(v_ph, pos, t, nv(n), dh_e, nu(!cN * e, n), Re, C_iso, C_dir, v_dir, dir);
                Cf(n) = (C_iso + (C_dir - C_iso) * (nv(n) > 1e-8 ? std::pow(domaine.dot(&v(n, 0), &dir(0)), 2) / (nv(n) * nv(n)) : 0)) * 2 * dh_e / std::max(nv(n), 1e-10);
                /* tout le melange dans la phase */
                coeffs_perte_charge(vm, pos, t, nvm, dh_e, nu(!cN * e, n), Re_m, C_iso, C_dir, v_dir, dir);
                Cf_t(n) = (C_iso + (C_dir - C_iso) * (nvm > 1e-8 ? std::pow(domaine.dot(&vm(0), &dir(0)), 2) / (nvm * nvm) : 0)) * 2 * dh_e / std::max(nvm, 1e-10);
                Fk(n) = Cf(n) * G(n) * G(n) / rho(!cR * e, n) / 2.0 / dh_e; //force
              }
            Fm = Cf_t(0) * Gm * Gm / rho(!cR * e, 0) / 2.0 / dh_e; //force paroi "melange" (debit total, mais proprietes physiques du liquide)

            /* appel du multiplicateur diphasique (si il existe) */
            if (fmult)
              fmult->coefficient(&(*alp)(e, 0), &rho(!cR * e, 0), &nv(0), &Cf_t(0), &mu(!cM * e, 0), dh_e, Sigma_tab(e,0), &Fk(0), Fm, mult);

            for (n = 0; n < N; n++)
              {
                double fac = pf(f) * vfd(f, e != f_e(f, 0)) * 0.5 / dh_e, fac_n = fac * mult(n, 0) * Cf(n) * G(n) / rho(!cR * e, n), fac_m = fac * mult(n, 1) * Cf_t(n) * Gm / rho(!cR * e, n);
                for (d = 0; d < D; d++)
                  for (m = 0; m < N; m++)
                    secmem(f, N * d + n) -= ((m == n) * fac_n + fac_m) * (alp ? (*alp)(e, m) : 1) * (pbm ? rho(!cR * e, m) : 1) * vit(f, N * d + m);
                if (mat)
                  for (d = 0; d < D; d++)
                    for (m = 0; m < N; m++)
                      (*mat)(N * (D * f + d) + n, N * (D * f + d) + m) += ((m == n) * fac_n + fac_m) * (alp ? (*alp)(e, m) : 1) * (pbm ? rho(!cR * e, m) : 1);
              }
          }
    }
}