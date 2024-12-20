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

#ifndef Op_Div_VDF_Elem_included
#define Op_Div_VDF_Elem_included

#include <Iterateur_VDF_Elem.h>
#include <Eval_Div_VDF_Elem.h>
#include <Op_Div_VDF_base.h>

/*! @brief class Op_Div_VDF_Elem Cette classe represente l'operateur de divergence
 *
 *   La discretisation est VDF. On calcule la divergence d'un champ de type Champ_Face_VDF
 *   L'iterateur associe est de type Iterateur_VDF_Elem. L'evaluateur associe est de type Eval_Div_VDF_Elem
 *
 */
class Op_Div_VDF_Elem : public Op_Div_VDF_base
{
  Declare_instanciable_sans_constructeur(Op_Div_VDF_Elem);
public:
  Op_Div_VDF_Elem();
  void associer(const Domaine_dis_base& , const Domaine_Cl_dis_base& , const Champ_Inc_base& ) override;
  void volumique(DoubleTab& ) const override;

protected:
  OBS_PTR(Domaine_VDF) le_dom_vdf;
  OBS_PTR(Domaine_Cl_VDF) la_zcl_vdf;
};

#endif /* Op_Div_VDF_Elem_included */
