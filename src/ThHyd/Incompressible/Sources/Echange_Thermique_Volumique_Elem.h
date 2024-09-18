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

#ifndef Echange_Thermique_Volumique_Elem_included
#define Echange_Thermique_Volumique_Elem_included

#include <Source_base.h>
#include <TRUST_Ref.h>
#include <Champ_Don.h>
#include <Parser_U.h>
#include <Correlation_base.h>

class Domaine_Cl_dis_base;
class Probleme_base;

class Echange_Thermique_Volumique_Elem: public Source_base
{
  Declare_instanciable(Echange_Thermique_Volumique_Elem);
public:
  int has_interface_blocs() const override { return 1; }
  int lire_motcle_non_standard(const Motcle& mot, Entree& is) override;
  void dimensionner_blocs(matrices_t matrices, const tabs_t& semi_impl = {}) const override;
  void ajouter_blocs(matrices_t matrices, DoubleTab& secmem, const tabs_t& semi_impl = {}) const override;
  void associer_pb(const Probleme_base& ) override { }
  void mettre_a_jour(double ) override;
  int initialiser(double temps) override;
  void associer_domaines(const Domaine_dis&, const Domaine_Cl_dis&) override {}

protected:
  Nom tag_; //pour trouver le terme source en face
  REF(Echange_Thermique_Volumique_Elem) o_ech_; //autre terme source
  Champ_Don Ai_, ep_cond_, cond_; //aire interfaciale, epaisseur pour la conduction, conductivite

  Correlation flux_par_;//correlation de flux parietal
};

#endif
