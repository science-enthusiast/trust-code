/****************************************************************************
* Copyright (c) 2022, CEA
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
// File:        Op_Div_PolyMAC_old.h
// Directory:   $TRUST_ROOT/src/PolyMAC_old/Operateurs
// Version:     /main/7
//
//////////////////////////////////////////////////////////////////////////////


#ifndef Op_Div_PolyMAC_old_included
#define Op_Div_PolyMAC_old_included

#include <Operateur_Div.h>
#include <Domaine_PolyMAC_old.h>
#include <TRUST_Ref.h>
class Domaine_Cl_PolyMAC_old;
//
// .DESCRIPTION class Op_Div_PolyMAC_old
//
//  Cette classe represente l'operateur de divergence
//  La discretisation est PolyMAC_old
//  On calcule la divergence d'un champ_P1NC (la vitesse)
//


//
// .SECTION voir aussi
// Operateur_Div_base
//


//////////////////////////////////////////////////////////////////////////////
//
// CLASS: Op_Div_PolyMAC_old
//
//////////////////////////////////////////////////////////////////////////////
class MAtrice_Morse;
class Op_Div_PolyMAC_old : public Operateur_Div_base
{

  Declare_instanciable(Op_Div_PolyMAC_old);

public:

  void associer(const Domaine_dis& , const Domaine_Cl_dis&,const Champ_Inc&) override;
  DoubleTab& ajouter(const DoubleTab& ,  DoubleTab& ) const override;
  DoubleTab& calculer(const DoubleTab& , DoubleTab& ) const override;
  int impr(Sortie& os) const override;
  void volumique(DoubleTab& ) const override;

  void dimensionner(Matrice_Morse& matrice) const override;
  void contribuer_a_avec(const DoubleTab&,Matrice_Morse& matrice) const override;

protected:

  REF(Domaine_PolyMAC_old) la_domaine_PolyMAC_old;
  REF(Domaine_Cl_PolyMAC_old) la_zcl_PolyMAC_old;

  DoubleVect porosite_face;
};

#endif
