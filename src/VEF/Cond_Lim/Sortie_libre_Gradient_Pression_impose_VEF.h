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

#ifndef Sortie_libre_Gradient_Pression_impose_VEF_included
#define Sortie_libre_Gradient_Pression_impose_VEF_included

#include <Neumann_sortie_libre.h>
#include <TRUST_Ref.h>

class Champ_P0_VEF;
class Domaine_VEF;

/*! @brief Sortie_libre_Gradient_Pression_impose_VEF
 *
 *  Cette classe derive de la classe Neumann_sortie_libre
 *
 *     Elle represente une frontiere ouverte avec condition de gradient de pression impose.
 *     L'objet de type OWN_PTR(Champ_front_base) le_champ_front contient le gradient impose. La fonction flux_impose() renvoie une valeur de pression
 *     a l'exterieur calculee a partir du gradient de pression impose et de la pression a l'interieur du domaine.
 *
 */
class Sortie_libre_Gradient_Pression_impose_VEF: public Neumann_sortie_libre
{
  Declare_instanciable(Sortie_libre_Gradient_Pression_impose_VEF);
public:
  double flux_impose(int) const override;
  double flux_impose(int, int) const override;

protected:
  OBS_PTR(Domaine_VEF) le_dom_VEF;
  OBS_PTR(Champ_P0_VEF) pression_interne;
  DoubleVect trace_pression_int;   // pression interne sur les mailles de bord
  DoubleVect coeff;
};

#endif
