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
//////////////////////////////////////////////////////////////////////////////
//
// File:        Op_Conv_PolyMAC_old_base.cpp
// Directory:   $TRUST_ROOT/src/PolyMAC_old/Operateurs
// Version:     /main/31
//
//////////////////////////////////////////////////////////////////////////////

#include <Op_Conv_PolyMAC_old_base.h>

#include <Discretisation_base.h>
#include <Champ.h>

#include <Domaine_PolyMAC_old.h>
#include <Domaine_Cl_PolyMAC_old.h>


Implemente_base(Op_Conv_PolyMAC_old_base,"Op_Conv_PolyMAC_old_base",Operateur_Conv_base);

//// printOn
//

Sortie& Op_Conv_PolyMAC_old_base::printOn(Sortie& s ) const
{
  return s << que_suis_je() ;
}

//// readOn
//

Entree& Op_Conv_PolyMAC_old_base::readOn(Entree& s )
{
  return s ;
}



///////////////////////////////////////////////////////////////////////////////////
//
//    Implementation de fonctions de la classe Op_Conv_PolyMAC_old_base
//
///////////////////////////////////////////////////////////////////////////////////

double Op_Conv_PolyMAC_old_base::calculer_dt_stab() const
{
  return 1e8;
  /*
    const Domaine_VDF& domaine_VDF = iter.domaine();
    const Domaine_Cl_VDF& domaine_Cl_VDF = iter.domaine_Cl();
    const IntTab& face_voisins = domaine_VDF.face_voisins();
    const DoubleVect& volumes = domaine_VDF.volumes();
    const DoubleVect& face_surfaces = domaine_VDF.face_surfaces();
    const DoubleVect& vit_associe = vitesse().valeurs();
    const DoubleVect& vit= (vitesse_pour_pas_de_temps_.non_nul()?vitesse_pour_pas_de_temps_.valeur().valeurs(): vit_associe);
    DoubleTab fluent;
    // fluent est initialise a zero par defaut:
    domaine_VDF.domaine().creer_tableau_elements(fluent);

    // Remplissage du tableau fluent
    double psc;
    int num1,num2,face;
    int elem1;

    // On traite les bords
    for (int n_bord=0; n_bord<domaine_VDF.nb_front_Cl(); n_bord++)
      {

        const Cond_lim& la_cl = domaine_Cl_VDF.les_conditions_limites(n_bord);

        if ( sub_type(Dirichlet_entree_fluide,la_cl.valeur())
             || sub_type(Neumann_sortie_libre,la_cl.valeur())  )

          {
            const Front_VF& le_bord = ref_cast(Front_VF,la_cl.frontiere_dis());
            num1 = le_bord.num_premiere_face();
            num2 = num1 + le_bord.nb_faces();
            for (face=num1; face<num2; face++)
              {
                psc = vit[face]*face_surfaces(face);
                if ( (elem1 = face_voisins(face,0)) != -1)
                  {
                    if (psc < 0)
                      fluent[elem1] -= psc;
                  }
                else // (elem2 != -1)
                  if (psc > 0)
                    fluent[face_voisins(face,1)] += psc;
              }
          }
      }

    // Boucle sur les faces internes pour remplir fluent
    int domaine_VDF_nb_faces=domaine_VDF.nb_faces();
    int premiere_face=domaine_VDF.premiere_face_int();
    for (face=premiere_face; face<domaine_VDF_nb_faces; face++)
      {
        psc = vit[face]*face_surfaces(face);
        eval_fluent(psc,face_voisins(face,0),face_voisins(face,1),fluent);
      }

    // Calcul du pas de temps de stabilite a partir du tableau fluent
    if (vitesse().le_nom()=="rho_u")
      diviser_par_rho_si_dilatable(fluent,equation().milieu());
    double dt_stab = 1.e30;
    int domaine_VDF_nb_elem=domaine_VDF.nb_elem();
    // dt_stab = min ( 1 / ( |U|/dx + |V|/dy + |W|/dz ) )
    for (int num_poly=0; num_poly<domaine_VDF_nb_elem; num_poly++)
      {
        double dt_elem = volumes(num_poly)/(fluent[num_poly]+DMINFLOAT);
        if (dt_elem<dt_stab)
          dt_stab = dt_elem;
      }
    dt_stab = Process::mp_min(dt_stab);

    // astuce pour contourner le type const de la methode
    Op_Conv_PolyMAC_old_base& op =ref_cast_non_const(Op_Conv_PolyMAC_old_base, *this);
    op.fixer_dt_stab_conv(dt_stab);
    return dt_stab;
  */
}
/*
// cf Op_Conv_PolyMAC_old_base::calculer_dt_stab() pour choix de calcul de dt_stab
void Op_Conv_PolyMAC_old_base::calculer_pour_post(Champ& espace_stockage,const Nom& option,int comp) const
{
  if (Motcle(option)=="stabilite")
    {
      DoubleTab& es_valeurs = espace_stockage->valeurs();
      es_valeurs = 1.e30;

      const Domaine_VDF& domaine_VDF = iter.domaine();
      const Domaine_Cl_VDF& domaine_Cl_VDF = iter.domaine_Cl();
      const IntTab& face_voisins = domaine_VDF.face_voisins();
      const DoubleVect& volumes = domaine_VDF.volumes();
      const DoubleVect& face_surfaces = domaine_VDF.face_surfaces();
      const DoubleVect& vit = vitesse().valeurs();
      DoubleTrav fluent(domaine_VDF.domaine().nb_elem_tot());

      // Remplissage du tableau fluent

      fluent = 0;
      double psc;
      int num1,num2,face;
      int elem1;

      // On traite les bords

      for (int n_bord=0; n_bord<domaine_VDF.nb_front_Cl(); n_bord++)
        {

          const Cond_lim& la_cl = domaine_Cl_VDF.les_conditions_limites(n_bord);

          if ( sub_type(Dirichlet_entree_fluide,la_cl.valeur())
               || sub_type(Neumann_sortie_libre,la_cl.valeur())  )

            {
              const Front_VF& le_bord = ref_cast(Front_VF,la_cl.frontiere_dis());
              num1 = le_bord.num_premiere_face();
              num2 = num1 + le_bord.nb_faces();
              for (face=num1; face<num2; face++)
                {
                  psc = vit[face]*face_surfaces(face);
                  if ( (elem1 = face_voisins(face,0)) != -1)
                    {
                      if (psc < 0)
                        fluent[elem1] -= psc;
                    }
                  else // (elem2 != -1)
                    if (psc > 0)
                      fluent[face_voisins(face,1)] += psc;
                }
            }
        }



      // Boucle sur les faces internes pour remplir fluent
      int domaine_VDF_nb_faces=domaine_VDF.nb_faces();
      for (face=domaine_VDF.premiere_face_int(); face<domaine_VDF_nb_faces; face++)
        {
          psc = vit[face]*face_surfaces(face);
          eval_fluent(psc,face_voisins(face,0),face_voisins(face,1),fluent);
        }
      //fluent.echange_espace_virtuel();
      if (vitesse().le_nom()=="rho_u")
        diviser_par_rho_si_dilatable(fluent,equation().milieu());

      int domaine_VDF_nb_elem=domaine_VDF.nb_elem();
      for (int num_poly=0; num_poly<domaine_VDF_nb_elem; num_poly++)
        {
          es_valeurs(num_poly) = volumes(num_poly)/(fluent[num_poly]+1.e-30);
        }

      //double dt_min = mp_min_vect(es_valeurs);
      //assert(dt_min==calculer_dt_stab());
    }
  else
    Operateur_Conv_base::calculer_pour_post(espace_stockage,option,comp);
}

Motcle Op_Conv_PolyMAC_old_base::get_localisation_pour_post(const Nom& option) const
{
  Motcle loc;
  if (Motcle(option)=="stabilite")
    loc = "elem";
  else
    return Operateur_Conv_base::get_localisation_pour_post(option);
  return loc;
}
*/
void Op_Conv_PolyMAC_old_base::completer()
{
  Operateur_base::completer();
}

void Op_Conv_PolyMAC_old_base::associer_domaine_cl_dis(const Domaine_Cl_dis_base& zcl)
{
  la_zcl_poly_ = ref_cast(Domaine_Cl_PolyMAC_old,zcl);
}

void Op_Conv_PolyMAC_old_base::associer(const Domaine_dis& domaine_dis, const Domaine_Cl_dis& zcl,const Champ_Inc& )
{
  le_dom_poly_ = ref_cast(Domaine_PolyMAC_old,domaine_dis.valeur());
  la_zcl_poly_ = ref_cast(Domaine_Cl_PolyMAC_old,zcl.valeur());

}
int Op_Conv_PolyMAC_old_base::impr(Sortie& os) const
{
  return 0;
}



void Op_Conv_PolyMAC_old_base::associer_vitesse(const Champ_base& ch )
{
  vitesse_=ch;
}
