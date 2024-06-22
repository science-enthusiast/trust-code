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

#include <Op_Diff_Turbulent_PolyVEF_P0P1_Elem.h>
#include <Op_Diff_Turbulent_PolyMAC_P0_Face.h>
#include <Op_Diff_Turbulent_PolyVEF_Face.h>
#include <PolyMAC_P0_discretisation.h>
#include <Viscosite_turbulente_base.h>
#include <Transport_turbulent_base.h>
#include <Champ_Elem_PolyMAC_P0.h>
#include <Pb_Multiphase.h>
#include <Synonyme_info.h>

Implemente_instanciable( Op_Diff_Turbulent_PolyVEF_P0P1_Elem, "Op_Diff_Turbulent_PolyVEF_P0P1_Elem|Op_Diff_Turbulente_PolyVEF_P0P1_Elem", Op_Diff_PolyVEF_P0P1_Elem);

Sortie& Op_Diff_Turbulent_PolyVEF_P0P1_Elem::printOn(Sortie& os) const { return Op_Diff_PolyVEF_P0P1_Elem::printOn(os); }

Entree& Op_Diff_Turbulent_PolyVEF_P0P1_Elem::readOn(Entree& is)
{
  //lecture de la correlation de diffusivite turbulente
  corr_.typer_lire(equation().probleme(), "transport_turbulent", is);
  associer_proto(equation().probleme(), champs_compris_);
  ajout_champs_proto_elem();
  return is;
}

void Op_Diff_Turbulent_PolyVEF_P0P1_Elem::get_noms_champs_postraitables(Noms& nom,Option opt) const
{
  Op_Diff_PolyVEF_P0P1_Elem::get_noms_champs_postraitables(nom,opt);
  get_noms_champs_postraitables_proto(que_suis_je(), nom, opt);
}

void Op_Diff_Turbulent_PolyVEF_P0P1_Elem::creer_champ(const Motcle& motlu)
{
  Op_Diff_PolyVEF_P0P1_Elem::creer_champ(motlu);
  creer_champ_proto_elem(motlu);
}

void Op_Diff_Turbulent_PolyVEF_P0P1_Elem::mettre_a_jour(double temps)
{
  Op_Diff_PolyVEF_P0P1_Elem::mettre_a_jour(temps);
  mettre_a_jour_proto_elem(temps);
}

void Op_Diff_Turbulent_PolyVEF_P0P1_Elem::completer()
{
  Op_Diff_PolyVEF_P0P1_Elem::completer();
  completer_proto_elem(*this);
  nu_constant_ = 0; // if pb_hydraulique with mu champ_uniforme : fgrad is not the same!
}

void Op_Diff_Turbulent_PolyVEF_P0P1_Elem::modifier_mu(DoubleTab& mu) const
{
  if (corr_.est_nul()) return; //rien a faire

  const Operateur_base& op_qdm = equation().probleme().equation(0).operateur(0).l_op_base();
  if (! (sub_type(Op_Diff_Turbulent_PolyVEF_Face, op_qdm)))
    {
      Cerr << "Error in " << que_suis_je() << ": no turbulent momentum diffusion found!" << finl;
      Process::exit();
    }

  const Correlation& corr_visc = ref_cast(Op_Diff_Turbulent_PolyVEF_Face, op_qdm).correlation();
  if (corr_.est_nul() || !sub_type(Viscosite_turbulente_base, corr_visc.valeur()))
    {
      Cerr << "Error in " << que_suis_je() << ": no turbulent viscosity correlation found!" << finl;
      Process::exit();
    }

  // on calcule d_t_
  DoubleTab& diff_turb = ref_cast_non_const(DoubleTab, nu_ou_lambda_turb_);
  assert (diff_turb.dimension_tot(0) == mu.dimension_tot(0) && diff_turb.line_size() == mu.line_size());
  diff_turb = 0.; // XXX : pour postraitement et pour n'a pas avoir la partie laminaire

  for (int i = 0; i < diff_turb.dimension_tot(0); i++)
    for (int j = 0; j < diff_turb.line_size(); j++)
      diff_turb(i,j) -= mu(i,j);

  // remplissage par la correlation : ICI c'est LAMBDA_T ET PAS ALPHA_T => W/mK et pas m2/s
  ref_cast(Transport_turbulent_base, corr_.valeur()).modifier_mu(ref_cast(Convection_Diffusion_std, equation()),
                                                                 ref_cast(Viscosite_turbulente_base, corr_visc.valeur()),
                                                                 mu);
  mu.echange_espace_virtuel();

  for (int i = 0; i < diff_turb.dimension_tot(0); i++)
    for (int j = 0; j < diff_turb.line_size(); j++)
      diff_turb(i,j) += mu(i,j);
}
