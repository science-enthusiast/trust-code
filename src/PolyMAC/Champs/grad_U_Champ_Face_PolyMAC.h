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

#ifndef grad_U_Champ_Face_PolyMAC_included
#define grad_U_Champ_Face_PolyMAC_included

#include <Champ_Fonc_Elem_PolyMAC.h>
#include <Champ_Face_PolyMAC.h>
#include <TRUST_Ref.h>
class Domaine_Cl_PolyMAC;

class grad_U_Champ_Face_PolyMAC: public Champ_Fonc_Elem_PolyMAC
{
  Declare_instanciable(grad_U_Champ_Face_PolyMAC);

public:

  inline void mettre_a_jour(double) override;
  void associer_champ(const Champ_Face_PolyMAC&);
  void me_calculer(double);

  inline void associer_domaine_Cl_dis_base(const Domaine_Cl_dis_base&);

protected:
  OBS_PTR(Domaine_Cl_PolyMAC) le_dom_Cl_PolyMAC;
  OBS_PTR(Champ_Face_PolyMAC) vitesse_;
};

inline void grad_U_Champ_Face_PolyMAC::mettre_a_jour(double tps)
{
  me_calculer(tps);
  changer_temps(tps);
  Champ_Fonc_base::mettre_a_jour(tps);
}

inline void grad_U_Champ_Face_PolyMAC::associer_domaine_Cl_dis_base(const Domaine_Cl_dis_base& le_dom_Cl_dis_base)
{
  le_dom_Cl_PolyMAC = (const Domaine_Cl_PolyMAC&) le_dom_Cl_dis_base;
}

#endif
