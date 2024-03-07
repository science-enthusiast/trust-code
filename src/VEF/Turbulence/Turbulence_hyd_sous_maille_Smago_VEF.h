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
// File:        Turbulence_hyd_sous_maille_Smago_VEF.h
// Directory:   $TURBULENCE_ROOT/src/Specializations/VEF/Modeles_Turbulence/LES/Hydr
//
//////////////////////////////////////////////////////////////////////////////

#ifndef Turbulence_hyd_sous_maille_Smago_VEF_included
#define Turbulence_hyd_sous_maille_Smago_VEF_included




#include <Mod_turb_hyd_ss_maille_VEF.h>


/*! @brief classe Turbulence_hyd_sous_maille_Smago_VEF Cette classe correspond a la mise en oeuvre du modele sous
 *
 *  maille smagorinsky en VEF
 *
 *  .SECTION  voir aussi
 *  Mod_turb_hyd_ss_maille
 *
 */
class Turbulence_hyd_sous_maille_Smago_VEF : public Mod_turb_hyd_ss_maille_VEF
{

  Declare_instanciable_sans_constructeur(Turbulence_hyd_sous_maille_Smago_VEF);

public:

  Turbulence_hyd_sous_maille_Smago_VEF();
  void set_param(Param& param) override;

protected :

  Champ_Fonc& calculer_viscosite_turbulente() override;
  virtual void calculer_S_barre();

  double cs;
  DoubleVect SMA_barre;
};

#endif
