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

#ifndef Terme_Boussinesq_PolyMAC_Face_included
#define Terme_Boussinesq_PolyMAC_Face_included

#include <Terme_Boussinesq_base.h>
#include <TRUST_Ref.h>

class Convection_Diffusion_std;
class Domaine_Cl_PolyMAC;
class Domaine_PolyMAC;

/*! @brief class Terme_Boussinesq_scalaire_PolyMAC_P0P1NC_Face
 *
 *  Terme Source de Boussinesq pour une dicretisation PolyMAC_P0P1NC
 *
 */
class Terme_Boussinesq_PolyMAC_Face : public Terme_Boussinesq_base
{
  Declare_instanciable(Terme_Boussinesq_PolyMAC_Face);
public:
  int has_interface_blocs() const override { return 1; }
  void dimensionner_blocs(matrices_t matrices, const tabs_t& semi_impl = {}) const override { } //rien
  void ajouter_blocs(matrices_t matrices, DoubleTab& secmem, const tabs_t& semi_impl = {}) const override;
  void check_multiphase_compatibility() const override { }

protected:
  REF(Domaine_PolyMAC) le_dom_PolyMAC;
  REF(Domaine_Cl_PolyMAC) le_dom_Cl_PolyMAC;
  void associer_domaines(const Domaine_dis& ,const Domaine_Cl_dis& ) override;
};

class Terme_Boussinesq_PolyVEF_P0_Face : public Terme_Boussinesq_PolyMAC_Face
{
  Declare_instanciable(Terme_Boussinesq_PolyVEF_P0_Face);
public:
  void ajouter_blocs(matrices_t matrices, DoubleTab& secmem, const tabs_t& semi_impl = {}) const override;
};

#endif /* Terme_Boussinesq_PolyMAC_Face_included */
