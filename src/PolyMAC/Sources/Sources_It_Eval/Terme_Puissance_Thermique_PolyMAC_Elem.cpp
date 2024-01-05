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

#include <Terme_Puissance_Thermique_PolyMAC_Elem.h>
#include <Discretisation_base.h>
#include <Probleme_base.h>
#include <Synonyme_info.h>
#include <Milieu_base.h>

Implemente_instanciable_sans_constructeur(Terme_Puissance_Thermique_PolyMAC_Elem, "Puissance_Thermique_Elem_PolyMAC|Puissance_Thermique_Elem_PolyMAC_P0P1NC", Terme_Puissance_Thermique_PolyMAC_base);
Add_synonym(Terme_Puissance_Thermique_PolyMAC_Elem, "Puissance_Thermique_Elem_PolyMAC_P0");
Add_synonym(Terme_Puissance_Thermique_PolyMAC_Elem, "Puissance_Thermique_Elem_PolyVEF_P0");
Add_synonym(Terme_Puissance_Thermique_PolyMAC_Elem, "Puissance_Thermique_Elem_PolyVEF_P0P1");

Sortie& Terme_Puissance_Thermique_PolyMAC_Elem::printOn(Sortie& s) const { return s << que_suis_je(); }
Entree& Terme_Puissance_Thermique_PolyMAC_Elem::readOn(Entree& s) { return Terme_Puissance_Thermique_PolyMAC_base::readOn(s); }

void Terme_Puissance_Thermique_PolyMAC_Elem::associer_domaines(const Domaine_dis& domaine_dis, const Domaine_Cl_dis& domaine_cl_dis)
{
  Terme_Puissance_Thermique_PolyMAC_base::associer_domaines(domaine_dis, domaine_cl_dis);
  Eval_Puiss_Th_PolyMAC_Elem& eval_puis = dynamic_cast<Eval_Puiss_Th_PolyMAC_Elem&> (iter->evaluateur());
  eval_puis.associer_domaines(domaine_dis.valeur(), domaine_cl_dis.valeur());
}

void Terme_Puissance_Thermique_PolyMAC_Elem::associer_pb(const Probleme_base& pb)
{
  Eval_Puiss_Th_PolyMAC_Elem& eval_puis = dynamic_cast<Eval_Puiss_Th_PolyMAC_Elem&> (iter->evaluateur());
  eval_puis.associer_champs(la_puissance);
}
