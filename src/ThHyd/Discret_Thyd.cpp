/****************************************************************************
* Copyright (c) 2015, CEA
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
// File:        Discret_Thyd.cpp
// Directory:   $TRUST_ROOT/src/ThHyd
// Version:     /main/29
//
//////////////////////////////////////////////////////////////////////////////

#include <Discret_Thyd.h>
#include <Schema_Temps_base.h>
#include <Zone_dis.h>
#include <Zone_VF.h>
#include <Champ_Fonc.h>
#include <Champ_Inc.h>

Implemente_base(Discret_Thyd,"Discret_Thyd",Discret_Thermique);


// Description:
//    NE FAIT RIEN
// Precondition:
// Parametre: Sortie& s
//    Signification: un flot de sortie
//    Valeurs par defaut:
//    Contraintes:
//    Acces:
// Retour: Sortie&
//    Signification: le flot de sortie
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition: la methode ne modifie pas l'objet
Sortie& Discret_Thyd::printOn(Sortie& s) const
{
  return s;
}


// Description:
//    NE FAIT RIEN
// Precondition:
// Parametre: Entree& s
//    Signification: un flot d'entree
//    Valeurs par defaut:
//    Contraintes:
//    Acces:
// Retour: Entree&
//    Signification: le flot d'entree
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition:
Entree& Discret_Thyd::readOn(Entree& s)
{
  return s ;
}
void Discret_Thyd::vitesse(const Schema_Temps_base& sch,
                           Zone_dis& z,
                           Champ_Inc& ch) const
{
  Cerr << "Discretisation de la vitesse" << finl;
  discretiser_champ("vitesse",z.valeur(),"vitesse","m/s",dimension,sch.nb_valeurs_temporelles(),
                    sch.temps_courant(),ch);


}

void Discret_Thyd::translation(const Schema_Temps_base& sch,
                               Zone_dis& z,
                               Champ_Fonc& ch) const
{
  Cerr << "Discretisation de translation" << finl;
  discretiser_champ("vitesse",z.valeur(),"translation","m/s",dimension,
                    sch.temps_courant(),ch);


}
void Discret_Thyd::entcor(const Schema_Temps_base& sch,
                          Zone_dis& z,
                          Champ_Fonc& ch) const
{
  Cerr << "Discretisation de entcor" << finl;
  discretiser_champ("vitesse",z.valeur(),"entcor","m/s",dimension,
                    sch.temps_courant(),ch);


}
void Discret_Thyd::pression(const Schema_Temps_base& sch,
                            Zone_dis& z,
                            Champ_Inc& ch) const
{
  Cerr << "Discretisation de la pression" << finl;
  discretiser_champ("pression",z.valeur(),"pression","Pa.m3/kg",1,1,
                    sch.temps_courant(),ch);

}
void Discret_Thyd::pression_en_pa(const Schema_Temps_base& sch,
                                  Zone_dis& z,
                                  Champ_Inc& ch) const
{
  Cerr << "Discretisation de la pression en pa" << finl;
  discretiser_champ("pression",z.valeur(),"pression_pa","Pa",1,1,
                    sch.temps_courant(),ch);

}

void Discret_Thyd::divergence_U(const Schema_Temps_base& sch,
                                Zone_dis& z,
                                Champ_Inc& ch) const
{
  Cerr << "Discretisation de la divergence de la vitesse" << finl;
  discretiser_champ("divergence_vitesse",z.valeur(), "divergence_U", "m3/s", 1,1,
                    sch.temps_courant(),ch);
}


void Discret_Thyd::gradient_P(const Schema_Temps_base& sch,
                              Zone_dis& z,
                              Champ_Inc& ch) const
{
  Cerr << "Discretisation du gradient de pression" << finl;
  discretiser_champ("gradient_pression",z.valeur(), "gradient_pression", "m/s2", dimension,1,
                    sch.temps_courant(),ch);
}
// Description:
//    NE FAIT RIEN, provoque une erreur
//    A surcharger dans les classes derivees.
// Precondition:
// Parametre: Schema_Temps_base&
//    Signification:
//    Valeurs par defaut:
//    Contraintes: reference constante
//    Acces: NON ACCEDE
// Parametre: Champ_Inc&
//    Signification:
//    Valeurs par defaut:
//    Contraintes: reference constante
//    Acces: NON ACCEDE
// Parametre: Champ_Fonc&
//    Signification:
//    Valeurs par defaut:
//    Contraintes:
//    Acces: NON ACCEDE
// Retour:
//    Signification:
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition: la methode ne modifie pas l'objet
void Discret_Thyd::creer_champ_vorticite(const Schema_Temps_base&, const Champ_Inc& ,
                                         Champ_Fonc& ) const
{
  Cerr << "Discret_Thyd::creer_champ_vorticite() ne fait rien" << finl;
  Cerr <<  que_suis_je() << "doit la surcharger !" << finl;
  exit();
}

// Description:
//    NE FAIT RIEN, provoque une erreur
//    A surcharger dans les classes derivees.
// Precondition:
// Parametre: Schema_Temps_base&
//    Signification:
//    Valeurs par defaut:
//    Contraintes: reference constante
//    Acces: NON ACCEDE
// Parametre: Champ_Inc&
//    Signification:
//    Valeurs par defaut:
//    Contraintes: reference constante
//    Acces: NON ACCEDE
// Parametre: Champ_Fonc&
//    Signification:
//    Valeurs par defaut:
//    Contraintes:
//    Acces: NON ACCEDE
// Retour:
//    Signification:
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition: la methode ne modifie pas l'objet
void Discret_Thyd::critere_Q(const Zone_dis& ,const Zone_Cl_dis& ,const Champ_Inc& , Champ_Fonc& ) const
{
  // pour le VDF, on a besoin de la Zone_Cl_dis, mais pas pour le VEF
  // -->> on passe quand meme l argument mais on n en fait rien!!!
  Cerr << "Discret_Thyd::critere_Q() ne fait rien" << finl;
  Cerr <<  que_suis_je() << "doit la surcharger !" << finl;
  exit();
}

// Description:
//    NE FAIT RIEN, provoque une erreur
//    A surcharger dans les classes derivees.
// Precondition:
// Parametre: Schema_Temps_base&
//    Signification:
//    Valeurs par defaut:
//    Contraintes: reference constante
//    Acces: NON ACCEDE
// Parametre: Champ_Inc&
//    Signification:
//    Valeurs par defaut:
//    Contraintes: reference constante
//    Acces: NON ACCEDE
// Parametre: Champ_Fonc&
//    Signification:
//    Valeurs par defaut:
//    Contraintes:
//    Acces: NON ACCEDE
// Retour:
//    Signification:
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition: la methode ne modifie pas l'objet
void Discret_Thyd::porosite_volumique(const Zone_dis& z, const Schema_Temps_base& sch , Champ_Fonc& ch ) const
{
  Cerr << "Discretisation du champ porosite_volumique" << finl;
  const Zone_VF& zone_VF=ref_cast(Zone_VF, z.valeur());
  discretiser_champ("champ_elem",zone_VF,"porosite_volumique",".",1,sch.temps_courant(),ch);
  Champ_Fonc_base& ch_fonc = ref_cast(Champ_Fonc_base,ch.valeur());
  DoubleVect& tab=ch_fonc.valeurs();
  tab = zone_VF.porosite_elem();
}


void Discret_Thyd::y_plus(const Zone_dis& ,const Zone_Cl_dis& ,const Champ_Inc& , Champ_Fonc& ) const
{
  // pour le VDF, on a besoin de la Zone_Cl_dis, mais pas pour le VEF
  // -->> on passe quand meme l argument mais on n en fait rien!!!
  Cerr << "Discret_Thyd::y_plus() ne fait rien" << finl;
  Cerr <<  que_suis_je() << "doit la surcharger !" << finl;
  exit();
}

void Discret_Thyd::grad_T(const Zone_dis& z,const Zone_Cl_dis& zcl, const Champ_Inc& eqn,Champ_Fonc& ch) const
{
  Cerr << "Discret_Thyd::grad_T() ne fait rien" << finl;
  Cerr <<  que_suis_je() << "doit la surcharger !" << finl;
  exit();
}

void Discret_Thyd::h_conv(const Zone_dis& z,const Zone_Cl_dis& zcl, const Champ_Inc& eqn,Champ_Fonc& ch, Motcle& nom,int temp_ref) const
{
  Cerr << "Discret_Thyd::h_conv() ne fait rien" << finl;
  Cerr <<  que_suis_je() << "doit la surcharger !" << finl;
  exit();
}



// Description:
//    NE FAIT RIEN, provoque une erreur
//    A surcharger dans les classes derivees.
// Precondition:
// Parametre: Zone_dis&
//    Signification:
//    Valeurs par defaut:
//    Contraintes: reference constante
//    Acces: NON ACCEDE
// Parametre: Fluide_Ostwald&
//    Signification:
//    Valeurs par defaut:
//    Contraintes:
//    Acces: NON ACCEDE
// Parametre: Champ_Inc&
//    Signification:
//    Valeurs par defaut:
//    Contraintes: reference constante
//    Acces: NON ACCEDE
// Parametre: Champ_Inc&
//    Signification:
//    Valeurs par defaut:
//    Contraintes:
//    Acces: NON ACCEDE
// Retour:
//    Signification:
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition: la methode ne modifie pas l'objet
void Discret_Thyd::proprietes_physiques_fluide_Ostwald(const Zone_dis& , Fluide_Ostwald& ,
                                                       const Navier_Stokes_std& ,
                                                       const Champ_Inc& ) const
{
  Cerr << "Discret_Thyd::proprietes_physiques_fluide_Ostwald() ne fait rien" << finl;
  Cerr <<  que_suis_je() << "doit la surcharger !" << finl;
  exit();
}


void Discret_Thyd::concentration(const Schema_Temps_base& sch,
                                 Zone_dis& z, Champ_Inc& ch,
                                 int nb_constituants) const
{
  Cerr << "Discretisation de la concentration" << finl;
  discretiser_champ("temperature",z.valeur(),"concentration","%",nb_constituants,sch.nb_valeurs_temporelles(),sch.temps_courant(),ch);
  ch->nommer("concentration");
  if (nb_constituants > 1)
    {
      ch->fixer_nature_du_champ(multi_scalaire);
      Noms noms(nb_constituants);
      for (int i=0; i<nb_constituants; i++)
        {
          noms[i] = "concentration";
          Nom param(i);
          noms[i] += param;
          ch->fixer_nom_compo(i, noms[i]);
        }
    }
}
