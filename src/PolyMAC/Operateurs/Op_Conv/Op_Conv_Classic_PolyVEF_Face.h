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

#ifndef Op_Conv_Classic_PolyVEF_Face_included
#define Op_Conv_Classic_PolyVEF_Face_included

#include <Op_Conv_EF_Stab_PolyMAC_P0_Face.h>

/*! @brief : class Op_Conv_Classic_PolyVEF_Face
 *
 *  <Description of class Op_Conv_Classic_PolyVEF_Face>
 *
 *
 *
 */

class Op_Conv_Classic_PolyVEF_Face : public Op_Conv_EF_Stab_PolyMAC_P0_Face
{

  Declare_instanciable( Op_Conv_Classic_PolyVEF_Face ) ;

public :
  void completer() override;
  double calculer_dt_stab() const override;

  /* interface ajouter_blocs */
  void dimensionner_blocs(matrices_t matrices, const tabs_t& semi_impl = {}) const override;
  void ajouter_blocs(matrices_t matrices, DoubleTab& secmem, const tabs_t& semi_impl = {}) const override;

  void set_incompressible(const int flag) override { } //always compressible

protected :
  double alpha = -1e8; //alpha = 0 -> centre, alpha = 1 -> amont
  IntTab e_fa_d, e_fa_f; //paires de faces reliees par un sommet (2D) / arete (3D) dans chaque element : e_fa_f([e_fa_d(e), e_fa_d(e) + 1[, 0/1)
  DoubleTab e_fa_s;      //surface * normale de la facette de f0 vers f1, stockee a partir de e_fa_d(e)
};

class Op_Conv_Classic_Amont_PolyVEF_Face : public Op_Conv_Classic_PolyVEF_Face
{
  Declare_instanciable( Op_Conv_Classic_Amont_PolyVEF_Face ) ;
};

class Op_Conv_Classic_Centre_PolyVEF_Face : public Op_Conv_Classic_PolyVEF_Face
{
  Declare_instanciable( Op_Conv_Classic_Centre_PolyVEF_Face ) ;
};

#endif /* Op_Conv_Classic_PolyVEF_Face_included */