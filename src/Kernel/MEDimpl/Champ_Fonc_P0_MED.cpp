/****************************************************************************
* Copyright (c) 2015 - 2016, CEA
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
// File:        Champ_Fonc_P0_MED.cpp
// Directory:   $TRUST_ROOT/src/Kernel/MEDimpl
// Version:     /main/10
//
//////////////////////////////////////////////////////////////////////////////

#include <Champ_Fonc_P0_MED.h>
Implemente_instanciable(Champ_Fonc_P0_MED,"Champ_Fonc_P0_MED",Champ_Fonc_P0_base);


// Description:
//    Ecrit le type et le nom du Champ sur un flot de sortie
// Precondition:
// Parametre: Sortie& s
//    Signification: un flot de sortie
//    Valeurs par defaut:
//    Contraintes:
//    Acces: entree/sortie
// Retour: Sortie&
//    Signification: le flot de sortie modifie
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition: la methode ne modifie pas l'objet
Sortie& Champ_Fonc_P0_MED::printOn(Sortie& s ) const
{
  return s << que_suis_je() << " " << le_nom();
}


// Description:
//    NE FAIT RIEN
// Precondition:
// Parametre: Entree& is
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
Entree& Champ_Fonc_P0_MED::readOn(Entree& s )
{
  return s ;
}
void Champ_Fonc_P0_MED::mettre_a_jour(double t)
{
  Cerr<<"Champ_Fonc_P0_MED::mettre_a_jour must be overloaded"<<finl;
  exit();
}
const Zone_dis_base& Champ_Fonc_P0_MED::zone_dis_base() const
{
  return zonebidon.valeur();
}
void Champ_Fonc_P0_MED::associer_zone_dis_base(const Zone_dis_base& la_zone_dis_base)
{
  zonebidon=la_zone_dis_base;
}
