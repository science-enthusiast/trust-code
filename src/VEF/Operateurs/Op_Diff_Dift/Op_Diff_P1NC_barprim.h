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

#ifndef Op_Diff_P1NC_barprim_included
#define Op_Diff_P1NC_barprim_included

#include <Operateur_Diff_base.h>
#include <Domaine_Cl_VEF.h>
#include <Equation_base.h>
#include <Domaine_VEF.h>
#include <Op_VEF_Face.h>
#include <TRUST_Ref.h>

class Champ_Uniforme;
class Matrice_Morse;

class Op_Diff_P1NC_barprim: public Operateur_Diff_base, public Op_VEF_Face
{
  Declare_instanciable(Op_Diff_P1NC_barprim);
public:
  double calculer_dt_stab() const override;
  void associer(const Domaine_dis_base&, const Domaine_Cl_dis_base&, const Champ_Inc_base&) override;
  void associer_diffusivite(const Champ_base& ) override;
  void completer() override;
  const Champ_base& diffusivite() const override;

  DoubleTab& ajouter(const DoubleTab&, DoubleTab&) const override;
  DoubleTab& calculer(const DoubleTab&, DoubleTab&) const override;

  void dimensionner(Matrice_Morse&) const override { }
  void modifier_pour_Cl(Matrice_Morse&, DoubleTab&) const override { }
  void contribuer_a_avec(const DoubleTab&, Matrice_Morse&) const override { }
  void contribuer_au_second_membre(DoubleTab&) const override { }

protected:
  void calculer_divergence(const DoubleTab&, const DoubleVect&, DoubleTab&) const;
  OBS_PTR(Domaine_VEF) le_dom_vef;
  OBS_PTR(Domaine_Cl_VEF) la_zcl_vef;
  OBS_PTR(Champ_Uniforme) diffusivite_;
};

#endif /* Op_Diff_P1NC_barprim_included */
