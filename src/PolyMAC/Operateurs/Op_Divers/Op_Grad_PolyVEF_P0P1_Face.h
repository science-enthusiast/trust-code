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

#ifndef Op_Grad_PolyVEF_P0P1_Face_included
#define Op_Grad_PolyVEF_P0P1_Face_included

#include <Op_Grad_PolyMAC_P0_Face.h>

class Champ_Face_PolyVEF;

/*! @brief class Op_Grad_PolyVEF_P0P1_Face
 *
 *   Cette classe represente l'operateur de gradient
 *   La discretisation est PolyVEF
 *   On calcule le gradient d'un champ_Elem_PolyVEF (la pression)
 *
 *
 * @sa Operateur_Grad_base
 */
class Op_Grad_PolyVEF_P0P1_Face : public Op_Grad_PolyMAC_P0_Face
{
  Declare_instanciable(Op_Grad_PolyVEF_P0P1_Face);
public:
  void completer() override;

  /* interface {dimensionner,ajouter}_blocs -> cf Equation_base.h */
  int has_interface_blocs() const override { return 1; };
  /* fonctions etendues : permet de remmlir les lignes des faces virtuelles en reglant virt = 1 */
  void dimensionner_blocs_ext(matrices_t matrices, int virt, const tabs_t& semi_impl = {}) const;
  void ajouter_blocs_ext(matrices_t matrices, DoubleTab& secmem, int virt, const tabs_t& semi_impl = {}) const;
  /* interface standard -> avec virt = 0 */
  void dimensionner_blocs(matrices_t matrices, const tabs_t& semi_impl = {}) const override
  {
    dimensionner_blocs_ext(matrices, 0, semi_impl);
  }
  void ajouter_blocs(matrices_t matrices, DoubleTab& secmem, const tabs_t& semi_impl = {}) const override
  {
    ajouter_blocs_ext(matrices, secmem, 0, semi_impl);
  }

  void check_multiphase_compatibility() const override { }; //ok

  const DoubleTab& alpha_es() const;
private:
  mutable DoubleTab alpha_es_;
};

#endif /* Op_Grad_PolyVEF_P0P1_Face_included */
