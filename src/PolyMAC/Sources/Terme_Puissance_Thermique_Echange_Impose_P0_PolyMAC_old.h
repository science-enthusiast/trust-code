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
// File:        Terme_Puissance_Thermique_Echange_Impose_P0_PolyMAC_old.h
// Directory:   $TRUST_ROOT/src/PolyMAC_old/Sources
// Version:     /main/9
//
//////////////////////////////////////////////////////////////////////////////


#ifndef Terme_Puissance_Thermique_Echange_Impose_P0_PolyMAC_old_included
#define Terme_Puissance_Thermique_Echange_Impose_P0_PolyMAC_old_included

#include <Source_base.h>
#include <TRUST_Ref.h>
class Domaine_Cl_PolyMAC_old;
class Domaine_PolyMAC_old;
#include <Champ_Don.h>

class Probleme_base;

//
// .DESCRIPTION class Terme_Puissance_Thermique_Echange_Impose_P0_PolyMAC_old
//

//
// .SECTION voir aussi Source_base
//
//

class Terme_Puissance_Thermique_Echange_Impose_P0_PolyMAC_old :  public Source_base


{

  Declare_instanciable_sans_constructeur(Terme_Puissance_Thermique_Echange_Impose_P0_PolyMAC_old);

public:

  DoubleTab& ajouter(DoubleTab& )  const override ;
  DoubleTab& calculer(DoubleTab& ) const override ;
  void associer_pb(const Probleme_base& ) override { };
  void mettre_a_jour(double ) override;

  void contribuer_a_avec(const DoubleTab&, Matrice_Morse&) const override ;

protected:

  REF(Domaine_PolyMAC_old) le_dom_PolyMAC_old;
  REF(Domaine_Cl_PolyMAC_old) le_dom_Cl_PolyMAC_old;
  Champ_Don himp_,Text_;
  void associer_domaines(const Domaine_dis& ,const Domaine_Cl_dis& ) override;

};

#endif
