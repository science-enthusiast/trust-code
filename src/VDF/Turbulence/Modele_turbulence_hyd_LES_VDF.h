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

#ifndef Modele_turbulence_hyd_LES_VDF_included
#define Modele_turbulence_hyd_LES_VDF_included

#include <Modele_turbulence_hyd_LES_VDF_base.h>

/*! @brief classe Modele_turbulence_hyd_LES_VDF Cette classe correspond a la mise en oeuvre du modele sous
 *
 *  maille fonction de structure en VDF
 *
 *  .SECTION  voir aussi
 *  Modele_turbulence_hyd_LES_base
 *
 */
class Modele_turbulence_hyd_LES_VDF: public Modele_turbulence_hyd_LES_VDF_base
{
  Declare_instanciable(Modele_turbulence_hyd_LES_VDF);
public:
  void set_param(Param& param) override;
  int lire_motcle_non_standard(const Motcle&, Entree&) override;

protected:
  DoubleVect F2_;
  int nb_points_ = 6;    // nb_points=4 ou 6 selon que nous utilisons la formulation de la FST en 4 ou 6 points!!
  int dir1_ = -123, dir2_ = -123; // direction du plan dans lequel on veut calculer la FST en 4 points
  int dir3_ = -123; // 3eme direction!!
  double Csm1_ = CSM1; // constante du modele (differente d'une classe fille a l'autre)
  double Csm2_ = CSM2; // constante pour calcul de l'energie ( idem )

  Champ_Fonc& calculer_viscosite_turbulente() override;
  void calculer_energie_cinetique_turb() override;
  virtual void calculer_fonction_structure();
};

#endif /* Modele_turbulence_hyd_LES_VDF_included */
