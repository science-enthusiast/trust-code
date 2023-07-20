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

#include <Neumann_sortie_libre.h>
#include <Check_espace_virtuel.h>
#include <Discretisation_base.h>
#include <Op_Dift_VEF_Face.h>
#include <Neumann_homogene.h>
#include <Porosites_champ.h>
#include <Champ_Uniforme.h>
#include <Neumann_paroi.h>
#include <Milieu_base.h>
#include <Champ_P1NC.h>
#include <TRUSTTrav.h>
#include <Symetrie.h>
#include <Debog.h>
#include <Champ.h>

Implemente_instanciable_sans_constructeur(Op_Dift_VEF_Face, "Op_Dift_VEF_P1NC", Op_Dift_VEF_base);

Op_Dift_VEF_Face::Op_Dift_VEF_Face() : grad_(0) { }

Sortie& Op_Dift_VEF_Face::printOn(Sortie& s) const { return s << que_suis_je(); }

Entree& Op_Dift_VEF_Face::readOn(Entree& s) { return s; }

// La diffusivite est constante par elements donc il faut calculer dt_diff pour chaque element et
//  dt_stab=Min(dt_diff (K) = h(K)*h(K)/(2*dimension*diffu2_(K)))
// ou diffu2_ est la somme des 2 diffusivite laminaire et turbulente

//GF : alpha_dt_stab=(alpha+alpha_t)*alpha_dt_stab/alpha
// alpha_dt_stab=(nu+diff_nu_turb)*valeurs_diffusivite_dt/nu
double Op_Dift_VEF_Face::calculer_dt_stab() const
{
  remplir_nu(nu_); // On remplit le tableau nu contenant la diffusivite en chaque elem

  const Domaine_VEF& le_dom_VEF = le_dom_vef.valeur();
  DoubleVect diffu_turb(diffusivite_turbulente()->valeurs());
  // DoubleTab diffu(nu_);
  DoubleTrav diffu;
  diffu = nu_; // XXX : Elie Saikali : Attention pas pareil que DoubleTrav diffu(nu_) !!!!!!!!!

  if (equation().que_suis_je().debute_par("Convection_Diffusion_Temp"))
    {
      double rhocp = mon_equation->milieu().capacite_calorifique().valeurs()(0, 0) * mon_equation->milieu().masse_volumique().valeurs()(0, 0);
      diffu_turb /= rhocp;
      diffu /= rhocp;
    }

  const int le_dom_nb_elem = le_dom_VEF.domaine().nb_elem();
  double dt_stab = 1.e30, alpha = -123., coef = -123.;

  if (has_champ_masse_volumique())
    {
      const DoubleTab& rho_elem = get_champ_masse_volumique().valeurs();
      assert(rho_elem.size_array() == le_dom_VEF.nb_elem_tot());
      for (int num_elem = 0; num_elem < le_dom_nb_elem; num_elem++)
        {
          alpha = diffu[num_elem] + diffu_turb[num_elem]; // PQ : 06/03
          alpha /= rho_elem[num_elem];
          coef = le_dom_VEF.carre_pas_maille(num_elem) / (2. * dimension * (alpha + DMINFLOAT));
          if (coef < dt_stab) dt_stab = coef;
        }
    }
  else
    {
      const Champ_base& champ_diffusivite = diffusivite_pour_pas_de_temps();
      const DoubleTab& valeurs_diffusivite = champ_diffusivite.valeurs();
      const int nb_comp = valeurs_diffusivite.line_size(), cD = (valeurs_diffusivite.dimension(0) == 1); // uniforme ou pas ?
      for (int nc = 0; nc < nb_comp; nc++)
        for (int num_elem = 0; num_elem < le_dom_nb_elem; num_elem++)
          {
            alpha = diffu(num_elem, nc) + diffu_turb[num_elem];
            const double valeurs_diffusivite_dt = valeurs_diffusivite(!cD * num_elem, nc);
            alpha *= valeurs_diffusivite_dt / (diffu(num_elem, nc) + DMINFLOAT);
            coef = le_dom_VEF.carre_pas_maille(num_elem) / (2. * dimension * (alpha + DMINFLOAT));
            if (coef < dt_stab) dt_stab = coef;
          }
    }

  dt_stab = Process::mp_min(dt_stab);
  return dt_stab;
}

// cf Op_Dift_VEF_Face::calculer_dt_stab() pour choix de calcul de dt_stab
void Op_Dift_VEF_Face::calculer_pour_post(Champ& espace_stockage, const Nom& option, int comp) const
{
  if (Motcle(option) == "stabilite")
    {
      DoubleTab& es_valeurs = espace_stockage->valeurs();

      if (le_dom_vef.non_nul())
        {
          remplir_nu(nu_); // On remplit le tableau nu contenant la diffusivite en chaque elem

          const Domaine_VEF& le_dom_VEF = le_dom_vef.valeur();
          const Domaine& le_dom = le_dom_VEF.domaine();
          const DoubleVect& diffu_turb = diffusivite_turbulente()->valeurs();
          double alpha = -123., coef = -123.;

          int le_dom_nb_elem = le_dom.nb_elem();
          if (has_champ_masse_volumique())
            {
              const DoubleTab& rho_elem = get_champ_masse_volumique().valeurs();
              assert(rho_elem.size_array() == le_dom_VEF.nb_elem_tot());
              for (int num_elem = 0; num_elem < le_dom_nb_elem; num_elem++)
                {
                  alpha = nu_[num_elem] + diffu_turb[num_elem]; // PQ : 06/03
                  alpha /= rho_elem[num_elem];
                  coef = le_dom_VEF.carre_pas_maille(num_elem) / (2. * dimension * alpha);
                  es_valeurs(num_elem) = coef;
                }
            }
          else
            {
              const Champ_base& champ_diffusivite = diffusivite_pour_pas_de_temps();
              const DoubleTab& valeurs_diffusivite = champ_diffusivite.valeurs();
              const int cD = (valeurs_diffusivite.dimension(0) == 1); // uniforme ou pas ?
              for (int num_elem = 0; num_elem < le_dom_nb_elem; num_elem++)
                {
                  const double valeurs_diffusivite_dt = valeurs_diffusivite(!cD * num_elem);
                  alpha = nu_[num_elem] + diffu_turb[num_elem]; // PQ : 06/03
                  alpha *= valeurs_diffusivite_dt / nu_[num_elem];
                  coef = le_dom_VEF.carre_pas_maille(num_elem) / (2. * dimension * alpha);
                  es_valeurs(num_elem) = coef;
                }
            }

          assert(est_egal(mp_min_vect(es_valeurs), calculer_dt_stab()));
        }
    }
  else
    Op_Dift_VEF_base::calculer_pour_post(espace_stockage, option, comp);
}

void Op_Dift_VEF_Face::ajouter_cas_vectoriel(const DoubleTab& inconnue, DoubleTab& resu, DoubleTab& tab_flux_bords, const DoubleTab& nu, const DoubleTab& nu_turb, const Domaine_Cl_VEF& domaine_Cl_VEF,
                                             const Domaine_VEF& domaine_VEF, int nbr_comp) const
{
  assert(nbr_comp > 1);
  const IntTab& face_voisins = domaine_VEF.face_voisins();
  const DoubleTab& face_normale = domaine_VEF.face_normales();
  const int nb_faces = domaine_VEF.nb_faces(), nb_elem = domaine_VEF.nb_elem();

  // On dimensionne et initialise le tableau des bilans de flux:
  tab_flux_bords.resize(domaine_VEF.nb_faces_bord(), nbr_comp);
  tab_flux_bords = 0.;

  // Construction du tableau grad_ si necessaire
  if (!grad_.get_md_vector().non_nul())
    {
      grad_.resize(0, Objet_U::dimension, Objet_U::dimension);
      domaine_VEF.domaine().creer_tableau_elements(grad_);
    }
  grad_ = 0.;

  Champ_P1NC::calcul_gradient(inconnue, grad_, domaine_Cl_VEF);
  if (le_modele_turbulence.valeur().utiliser_loi_paroi())
    Champ_P1NC::calcul_duidxj_paroi(grad_, nu, nu_turb, tau_tan_, domaine_Cl_VEF);

  grad_.echange_espace_virtuel();

  DoubleTab Re;
  Re.resize(0, Objet_U::dimension, Objet_U::dimension);
  domaine_VEF.domaine().creer_tableau_elements(Re);
  Re = 0.;

  if (le_modele_turbulence.valeur().calcul_tenseur_Re(nu_turb, grad_, Re))
    {
      Cerr << "On utilise une diffusion turbulente non linaire dans NS" << finl;
      for (int elem = 0; elem < nb_elem; elem++)
        for (int i = 0; i < nbr_comp; i++)
          for (int j = 0; j < nbr_comp; j++)
            Re(elem, i, j) *= nu_turb[elem];
    }
  else
    for (int elem = 0; elem < nb_elem; elem++)
      for (int i = 0; i < nbr_comp; i++)
        for (int j = 0; j < nbr_comp; j++)
          Re(elem, i, j) = nu_turb[elem] * (grad_(elem, i, j) + grad_(elem, j, i));

  Re.echange_espace_virtuel();

  // boucle sur les CL
  const Conds_lim& les_cl = domaine_Cl_VEF.les_conditions_limites();
  const int nb_cl = les_cl.size();
  for (int n_bord = 0; n_bord < nb_cl; n_bord++)
    {
      const Cond_lim& la_cl = domaine_Cl_VEF.les_conditions_limites(n_bord);
      const Front_VF& le_bord = ref_cast(Front_VF, la_cl.frontiere_dis());
      const int ndeb = le_bord.num_premiere_face(), nfin = ndeb + le_bord.nb_faces();

      if (sub_type(Periodique, la_cl.valeur()))
        for (int num_face = ndeb; num_face < nfin; num_face++)
          for (int kk = 0; kk < 2; kk++)
            {
              const int elem = face_voisins(num_face, kk), ori = 1 - 2 * kk;
              for (int i = 0; i < nbr_comp; i++)
                for (int j = 0; j < nbr_comp; j++)
                  resu(num_face, i) -= ori * face_normale(num_face, j) * (nu[elem] * grad_(elem, i, j) + Re(elem, i, j));
            }
      else // CL pas periodique
        for (int num_face = ndeb; num_face < nfin; num_face++)
          {
            const int elem = face_voisins(num_face, 0);
            for (int i = 0; i < nbr_comp; i++)
              for (int j = 0; j < nbr_comp; j++)
                {
                  double flux = face_normale(num_face, j) * (nu[elem] * grad_(elem, i, j) + Re(elem, i, j));
                  resu(num_face, i) -= flux;
                  tab_flux_bords(num_face, i) -= flux;
                }

            // Correction tab_flux_bords si symetrie
            if (sub_type(Symetrie, la_cl.valeur()))
              tab_flux_bords(num_face, 0) = 0.;
          }
    }

  // Boucle sur les faces internes
  const int nint = domaine_VEF.premiere_face_int();
  for (int num_face = nint; num_face < nb_faces; num_face++)
    for (int kk = 0; kk < 2; kk++)
      {
        const int elem = face_voisins(num_face, kk), ori = 1 - 2 * kk;
        for (int i = 0; i < nbr_comp; i++)
          for (int j = 0; j < nbr_comp; j++)
            resu(num_face, i) -= ori * face_normale(num_face, j) * (nu[elem] * grad_(elem, i, j) + Re(elem, i, j));
      }
}

void Op_Dift_VEF_Face::ajouter_cas_scalaire(const DoubleTab& inconnue, DoubleTab& resu, DoubleTab& tab_flux_bords, const DoubleTab& nu, const DoubleVect& nu_turb,
                                            const Domaine_Cl_VEF& domaine_Cl_VEF, const Domaine_VEF& domaine_VEF, int nb_comp) const
{
  const IntTab& elem_faces = domaine_VEF.elem_faces(), &face_voisins = domaine_VEF.face_voisins();
  const int nb_faces = domaine_VEF.nb_faces(), nb_faces_elem = domaine_VEF.domaine().nb_faces_elem(), nb_front = domaine_VEF.nb_front_Cl();
  int j;

  // modif pour imprimer les flux sur les bords
  const int size_flux_bords = domaine_VEF.nb_faces_bord();
  tab_flux_bords.resize(size_flux_bords, nb_comp);
  tab_flux_bords = 0.;

  // contient -1 si la face n'est pas periodique et numero face_assso sinon
  ArrOfInt marq(domaine_VEF.nb_faces_tot());
  marq = -1;

  const int premiere_face_int = domaine_VEF.premiere_face_int();

  // On traite les faces bord
  for (int n_bord = 0; n_bord < nb_front; n_bord++)
    {
      const Cond_lim& la_cl = domaine_Cl_VEF.les_conditions_limites(n_bord);
      const Front_VF& le_bord = ref_cast(Front_VF, la_cl.frontiere_dis());
      int num1 = 0, num2 = le_bord.nb_faces_tot();
      int nb_faces_bord_reel = le_bord.nb_faces();

      if (sub_type(Periodique, la_cl.valeur()))
        {
          const Periodique& la_cl_perio = ref_cast(Periodique, la_cl.valeur());
          for (int ind_face = num1; ind_face < num2; ind_face++)
            {
              int fac_asso = la_cl_perio.face_associee(ind_face);
              fac_asso = le_bord.num_face(fac_asso);
              int num_face = le_bord.num_face(ind_face);
              marq[num_face] = fac_asso;
              for (int kk = 0; kk < 2; kk++)
                {
                  int elem = face_voisins(num_face, kk);
                  for (int i = 0; i < nb_faces_elem; i++)
                    if (((j = elem_faces(elem, i)) > num_face) && (j != fac_asso))
                      for (int nc = 0; nc < nb_comp; nc++)
                        {
                          const double d_nu = nu(elem, nc) + nu_turb(elem);
                          const double valA = viscA(num_face, j, elem, d_nu);
                          const double flux = valA * inconnue(j, nc) - valA * inconnue(num_face, nc);
                          resu(num_face, nc) += flux;
                          if (j < nb_faces) // face reelle
                            resu(j, nc) -= 0.5 * flux;
                        }
                }
            }
        }
      else // CL pas periodique
        {
          // on traite une equation scalaire (pas la vitesse) on a pas a utiliser le tau tangentiel (les lois de paroi thermiques ne calculent pas d'echange turbulent a la paroi pour l'instant
          const DoubleTab& face_normale = domaine_VEF.face_normales();
          const DoubleVect& vol = domaine_VEF.volumes();
          const Equation_base& my_eqn = domaine_Cl_VEF.equation();
          const RefObjU& modele_turbulence = my_eqn.get_modele(TURBULENCE);

          // Cas Equation Convection_Diffusion_Turbulente
          if (sub_type(Modele_turbulence_scal_base, modele_turbulence.valeur()))
            {
              const Modele_turbulence_scal_base& mod_turb_scal = ref_cast(Modele_turbulence_scal_base, modele_turbulence.valeur());
              const Turbulence_paroi_scal& loiparth = mod_turb_scal.loi_paroi();

              if (loiparth->use_equivalent_distance())
                {
                  const DoubleVect& d_equiv = loiparth->equivalent_distance(n_bord);
                  // d_equiv contient la distance equivalente pour le bord
                  // Dans d_equiv, pour les faces qui ne sont pas paroi_fixe (eg periodique, symetrie, etc...)
                  // il y a la distance geometrique grace a l'initialisation du tableau dans la loi de paroi.

                  // Les lois de parois ne s'appliquent qu'aux cas ou la CL est de type temperature imposee, car dans les autres cas
                  // (flux impose et adiabatique) le flux a la paroi est connu et fixe.
                  int nb_dim_pb = Objet_U::dimension;
                  const Cond_lim_base& cl_base = la_cl.valeur();

                  int ldp_appli = 0;

                  if (sub_type(Scalaire_impose_paroi, cl_base)) ldp_appli = 1;
                  else if (loiparth->get_flag_calcul_ldp_en_flux_impose())
                    if ((sub_type(Neumann_paroi, cl_base)) || (sub_type(Neumann_homogene, cl_base))) ldp_appli = 1;

                  if (ldp_appli)
                    {
                      DoubleVect le_mauvais_gradient(nb_dim_pb);
                      for (int ind_face = num1; ind_face < num2; ind_face++)
                        for (int nc = 0; nc < nb_comp; nc++)
                          {
                            // Tf est la temperature fluide moyenne dans le premier element
                            // sans tenir compte de la temperature de paroi.
                            double Tf = 0.;
                            double bon_gradient = 0.; // c'est la norme du gradient de temperature normal a la paroi calculee a l'aide de la loi de paroi.
                            le_mauvais_gradient = 0.;
                            int num_face = le_bord.num_face(ind_face);
                            int elem1 = face_voisins(num_face, 0);

                            if (elem1 == -1) elem1 = face_voisins(num_face, 1);

                            double surface_face = domaine_VEF.face_surfaces(num_face);

                            for (int i = 0; i < nb_faces_elem; i++)
                              {
                                if ((j = elem_faces(elem1, i)) != num_face)
                                  {
                                    double surface_pond = 0.;
                                    for (int kk = 0; kk < nb_dim_pb; kk++)
                                      surface_pond -= (face_normale(j, kk) * domaine_VEF.oriente_normale(j, elem1) * face_normale(num_face, kk) * domaine_VEF.oriente_normale(num_face, elem1)) / (surface_face * surface_face);

                                    Tf += inconnue(j, nc) * surface_pond;
                                  }

                                for (int kk = 0; kk < nb_dim_pb; kk++)
                                  le_mauvais_gradient(kk) += inconnue(j, nc) * face_normale(j, kk) * domaine_VEF.oriente_normale(j, elem1);
                              }
                            le_mauvais_gradient /= vol(elem1);
                            // mauvais_gradient = le_mauvais_gradient.n
                            double mauvais_gradient = 0;
                            for (int kk = 0; kk < nb_dim_pb; kk++)
                              mauvais_gradient += le_mauvais_gradient(kk) * face_normale(num_face, kk) / surface_face;

                            // inconnue(num_face) est la temperature de paroi : Tw.
                            // On se fiche du signe de bon gradient car c'est la norme du gradient de temperature dans l'element.
                            // Ensuite ce sera multiplie par le vecteur normal a la face de paroi qui lui a les bons signes.
                            bon_gradient = (Tf - inconnue(num_face, nc)) / d_equiv(ind_face) * (-domaine_VEF.oriente_normale(num_face, elem1));

                            double nutotal = nu(elem1, nc) + nu_turb(elem1);
                            for (int i = 0; i < nb_faces_elem; i++)
                              {
                                j = elem_faces(elem1, i);
                                double correction = 0.;
                                for (int kk = 0; kk < nb_dim_pb; kk++)
                                  {
                                    double resu2 = nutotal * (bon_gradient - mauvais_gradient) * face_normale(num_face, kk) * face_normale(j, kk) * (-domaine_VEF.oriente_normale(j, elem1)) / surface_face;
                                    correction += resu2;
                                  }

                                resu(j, nc) += correction;

                                if (marq[j] != -1)
                                  resu(marq[j], nc) += correction;

                                // la face num_face n'est pas periodique  mar(num_face)==-1 flux_bord n'est necessaire que sur les faces reelles
                                if (j == num_face && j < size_flux_bords)
                                  tab_flux_bords(j, nc) -= correction;
                              }
                          }
                    }
                } // loi de paroi use_equivalent_distance
            }
          // Fin de la correction du gradient dans la premiere maille par la loi de paroi thermique.

          for (int ind_face = num1; ind_face < num2; ind_face++)
            {
              const int num_face = le_bord.num_face(ind_face);
              int elem1 = face_voisins(num_face, 0);
              for (int i = 0; i < nb_faces_elem; i++)
                if (((j = elem_faces(elem1, i)) > num_face) || (ind_face >= nb_faces_bord_reel))
                  for (int nc = 0; nc < nb_comp; nc++)
                    {
                      const double d_nu = nu(elem1, nc) + nu_turb(elem1);
                      const double valA = viscA(num_face, j, elem1, d_nu);
                      if (ind_face < nb_faces_bord_reel)
                        {
                          double flux = valA * (inconnue(j,nc) - inconnue(num_face,nc));
                          resu(num_face, nc) += flux;
                          tab_flux_bords(num_face, nc) -= flux;
                        }
                      if (j < nb_faces) // face reelle
                        {
                          double flux = valA * (inconnue(num_face, nc) - inconnue(j, nc));
                          resu(j, nc) += flux;
                          if (j < premiere_face_int) tab_flux_bords(j, nc) -= flux;
                        }
                    }
            }
        }

      // EN FINI PAR LES CLS Neumann
      const int ndeb = le_bord.num_premiere_face(), nfin = ndeb + le_bord.nb_faces();
      if (sub_type(Neumann_paroi, la_cl.valeur()))
        {
          const Neumann_paroi& la_cl_paroi = ref_cast(Neumann_paroi, la_cl.valeur());
          for (int face = ndeb; face < nfin; face++)
            for (int nc = 0; nc < nb_comp; nc++)
              {
                resu(face, nc) += la_cl_paroi.flux_impose(face - ndeb, nc) * domaine_VEF.face_surfaces(face);
                tab_flux_bords(face, nc) = la_cl_paroi.flux_impose(face - ndeb, nc) * domaine_VEF.face_surfaces(face);
              }
        }
      else if (sub_type(Echange_externe_impose, la_cl.valeur()))
        {
          const Echange_externe_impose& la_cl_paroi = ref_cast(Echange_externe_impose, la_cl.valeur());
          for (int face = ndeb; face < nfin; face++)
            for (int nc = 0; nc < nb_comp; nc++)
              {
                resu(face, nc) += la_cl_paroi.h_imp(face - ndeb, nc) * (la_cl_paroi.T_ext(face - ndeb, nc) - inconnue(face, nc)) * domaine_VEF.face_surfaces(face);
                tab_flux_bords(face, nc) = la_cl_paroi.h_imp(face - ndeb, nc) * (la_cl_paroi.T_ext(face - ndeb, nc) - inconnue(face, nc)) * domaine_VEF.face_surfaces(face);
              }
        }
      else if (sub_type(Neumann_homogene,la_cl.valeur()) || sub_type(Symetrie, la_cl.valeur()) || sub_type(Neumann_sortie_libre, la_cl.valeur()))
        for (int face = ndeb; face < nfin; face++)
          for (int nc = 0; nc < nb_comp; nc++)
            tab_flux_bords(face, nc) = 0.;
    }

  // Faces internes
  for (int num_face = premiere_face_int; num_face < nb_faces; num_face++)
    for (int kk = 0; kk < 2; kk++)
      {
        int elem = face_voisins(num_face, kk);
        for (int i = 0; i < nb_faces_elem; i++)
          if ((j = elem_faces(elem, i)) > num_face)
            {
              int contrib = 1;
              if (j >= nb_faces) // C'est une face virtuelle
                {
                  const int el1 = face_voisins(j, 0), el2 = face_voisins(j, 1);
                  if ((el1 == -1) || (el2 == -1)) contrib = 0;
                }
              if (contrib)
                for (int nc = 0; nc < nb_comp; nc++)
                  {
                    const double d_nu = nu(elem, nc) + nu_turb(elem);
                    const double valA = viscA(num_face, j, elem, d_nu);
                    const double flux = valA * inconnue(j, nc) - valA * inconnue(num_face, nc);
                    resu(num_face, nc) += flux;
                    if (j < nb_faces) // On traite les faces reelles
                      resu(j, nc) -= flux;
                  }
            }
      }
}

DoubleTab& Op_Dift_VEF_Face::ajouter(const DoubleTab& inconnue_org, DoubleTab& resu) const
{
  remplir_nu(nu_);
  const Domaine_Cl_VEF& domaine_Cl_VEF = la_zcl_vef.valeur();
  const Domaine_VEF& domaine_VEF = le_dom_vef.valeur();
  const DoubleTab& nu_turb = diffusivite_turbulente()->valeurs();
  const int nb_comp = resu.line_size();

  DoubleTab nu, nu_turb_m;
  DoubleTab tab_inconnue;
  int marq = phi_psi_diffuse(equation());
  const DoubleVect& porosite_face = equation().milieu().porosite_face();
  const DoubleVect& porosite_elem = equation().milieu().porosite_elem();
  // soit on a div(phi nu grad inco)
  // soit on a div(nu grad phi inco)
  // cela depend si on diffuse phi_psi ou psi
  modif_par_porosite_si_flag(nu_, nu, !marq, porosite_elem);
  modif_par_porosite_si_flag(nu_turb, nu_turb_m, !marq, porosite_elem);
  const DoubleTab& inconnue = modif_par_porosite_si_flag(inconnue_org, tab_inconnue, marq, porosite_face);

  const Champ_base& inco = equation().inconnue().valeur();
  const Nature_du_champ nature_champ = inco.nature_du_champ();

  assert_espace_virtuel_vect(nu);
  assert_espace_virtuel_vect(inconnue);
  assert_espace_virtuel_vect(nu_turb_m);
  Debog::verifier("Op_Dift_VEF_Face::ajouter nu", nu);
  Debog::verifier("Op_Dift_VEF_Face::ajouter nu_turb", nu_turb_m);
  Debog::verifier("Op_Dift_VEF_Face::ajouter inconnue_org", inconnue_org);
  Debog::verifier("Op_Dift_VEF_Face::ajouter inconnue", inconnue);

  if (nature_champ == vectoriel)
    ajouter_cas_vectoriel(inconnue, resu, flux_bords_, nu, nu_turb_m, domaine_Cl_VEF, domaine_VEF, nb_comp);
  else
    ajouter_cas_scalaire(inconnue, resu, flux_bords_, nu, nu_turb_m, domaine_Cl_VEF, domaine_VEF, nb_comp);

  modifier_flux(*this);

  return resu;
}

void Op_Dift_VEF_Face::contribuer_a_avec(const DoubleTab& inco, Matrice_Morse& matrice) const
{
  modifier_matrice_pour_periodique_avant_contribuer(matrice, equation());

  // On remplit le tableau nu car l'assemblage d'une matrice avec ajouter_contribution peut se faire avant le premier pas de temps
  remplir_nu(nu_);

  const DoubleTab& nu_turb_ = diffusivite_turbulente()->valeurs();
  DoubleVect n(dimension);

  DoubleTab nu, nu_turb;
  int marq = phi_psi_diffuse(equation());
  const DoubleVect& porosite_elem = equation().milieu().porosite_elem();

  // soit on a div(phi nu grad inco) OU on a div(nu grad phi inco) : cela depend si on diffuse phi_psi ou psi
  modif_par_porosite_si_flag(nu_, nu, !marq, porosite_elem);
  modif_par_porosite_si_flag(nu_turb_, nu_turb, !marq, porosite_elem);

  DoubleVect porosite_eventuelle(equation().milieu().porosite_face());
  if (!marq) porosite_eventuelle = 1;

  if (equation().inconnue()->nature_du_champ() == vectoriel)
    {
      ajouter_contribution_bord_gen<Type_Champ::VECTORIEL>(inco, matrice, nu, nu_turb, porosite_eventuelle);
      ajouter_contribution_interne_gen<Type_Champ::VECTORIEL>(inco, matrice, nu, nu_turb, porosite_eventuelle);
    }
  else
    {
      ajouter_contribution_bord_gen<Type_Champ::SCALAIRE>(inco, matrice, nu, nu_turb, porosite_eventuelle);
      ajouter_contribution_interne_gen<Type_Champ::SCALAIRE>(inco, matrice, nu, nu_turb, porosite_eventuelle);
    }

  modifier_matrice_pour_periodique_apres_contribuer(matrice, equation());
}

void Op_Dift_VEF_Face::contribuer_au_second_membre(DoubleTab& resu) const
{
  const Domaine_Cl_VEF& domaine_Cl_VEF = la_zcl_vef.valeur();
  const Domaine_VEF& domaine_VEF = le_dom_vef.valeur();
  const IntTab& face_voisins = domaine_VEF.face_voisins();
  int n_bord;
  int nb_faces = domaine_VEF.nb_faces();
  int nb_comp = resu.line_size();
  const DoubleTab& face_normale = domaine_VEF.face_normales();

  // On traite les faces bord
  //  if (nb_comp!=1)
  if (equation().inconnue()->nature_du_champ() == vectoriel)
    {
      //  const Domaine_Cl_VEF& domaine_Cl_VEF = la_zcl_vef.valeur();
      //const Domaine_VEF& domaine_VEF = le_dom_vef.valeur();
      const DoubleTab& nu_turb = diffusivite_turbulente()->valeurs();
      const DoubleTab& inconnue_org = equation().inconnue().valeurs();
      DoubleTab nu, nu_turb_m;
      DoubleTab tab_inconnue;
      int marq = phi_psi_diffuse(equation());
      const DoubleVect& porosite_face = equation().milieu().porosite_face();
      const DoubleVect& porosite_elem = equation().milieu().porosite_elem();
      // soit on a div(phi nu grad inco)
      // soit on a div(nu grad phi inco)
      // cela depend si on diffuse phi_psi ou psi
      modif_par_porosite_si_flag(nu_, nu, !marq, porosite_elem);
      modif_par_porosite_si_flag(nu_turb, nu_turb_m, !marq, porosite_elem);
      const DoubleTab& inconnue = modif_par_porosite_si_flag(inconnue_org, tab_inconnue, marq, porosite_face);

      DoubleTab& grad = grad_;
      grad = 0.;

      //      const Conds_lim& les_cl = domaine_Cl_VEF.les_conditions_limites();
      //      int nb_cl=les_cl.size();

      Champ_P1NC::calcul_gradient(inconnue, grad, domaine_Cl_VEF);
      DoubleTab gradsa(grad);
      if (le_modele_turbulence.valeur().utiliser_loi_paroi())
        Champ_P1NC::calcul_duidxj_paroi(grad, nu, nu_turb, tau_tan_, domaine_Cl_VEF);
      grad -= gradsa;
      grad.echange_espace_virtuel();
      for (int num_face = 0; num_face < nb_faces; num_face++)
        {
          //double d_nu;
          for (int kk = 0; kk < 2; kk++)
            {
              int elem = face_voisins(num_face, kk);
              if (elem != -1)
                {
                  //d_nu = nu[elem]+nu_turb[elem];
                  int ori = 1 - 2 * kk;
                  for (int i = 0; i < nb_comp; i++)
                    for (int j = 0; j < nb_comp; j++)
                      {
                        resu(num_face, i) -= ori * face_normale(num_face, j) * ((nu[elem] + nu_turb[elem]) * grad(elem, i, j) + (nu_turb[elem]) * grad(elem, j, i) /* grad transpose */);
                      }
                }
            }
        }
    }

  for (n_bord = 0; n_bord < domaine_VEF.nb_front_Cl(); n_bord++)
    {
      const Cond_lim& la_cl = domaine_Cl_VEF.les_conditions_limites(n_bord);
      if (sub_type(Neumann_paroi, la_cl.valeur()))
        {
          const Neumann_paroi& la_cl_paroi = ref_cast(Neumann_paroi, la_cl.valeur());
          const Front_VF& le_bord = ref_cast(Front_VF, la_cl.frontiere_dis());
          int ndeb = le_bord.num_premiere_face();
          int nfin = ndeb + le_bord.nb_faces();
          for (int face = ndeb; face < nfin; face++)
            for (int comp = 0; comp < nb_comp; comp++)
              resu(face, comp) += la_cl_paroi.flux_impose(face - ndeb, comp) * domaine_VEF.face_surfaces(face);
        }
      else if (sub_type(Echange_externe_impose, la_cl.valeur()))
        {
          if (resu.line_size() == 1)
            {
              const Echange_externe_impose& la_cl_paroi = ref_cast(Echange_externe_impose, la_cl.valeur());
              const Front_VF& le_bord = ref_cast(Front_VF, la_cl.frontiere_dis());
              int ndeb = le_bord.num_premiere_face();
              int nfin = ndeb + le_bord.nb_faces();
              for (int face = ndeb; face < nfin; face++)
                {
                  resu[face] += la_cl_paroi.h_imp(face - ndeb) * (la_cl_paroi.T_ext(face - ndeb)) * domaine_VEF.face_surfaces(face);
                }
            }
          else
            {

              Cerr << "Non code pour Echange_externe_impose" << finl;
              assert(0);
            }
        }
    }
}