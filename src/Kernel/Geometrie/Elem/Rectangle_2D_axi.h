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

#ifndef Rectangle_2D_axi_included
#define Rectangle_2D_axi_included

#include <Rectangle.h>

/*! @brief Classe Rectangle_axi Cette classe represente la deformee du rectangle dans
 *
 *     le cas d'un maillage en coordonnees cylindriques
 *     Les methodes de la classe Rectangle dont Rectangle_axi derive sont
 *     surchargees pour effectuer les calculs en coordonnees cylindrique.
 *
 * @sa Rectangle Elem_geom_base Elem_geom
 */
template <typename _SIZE_>
class Rectangle_2D_axi_32_64 : public Rectangle_32_64<_SIZE_>
{

  Declare_instanciable_32_64(Rectangle_2D_axi_32_64);

public :
  using int_t = _SIZE_;
  using DoubleVect_t = DoubleVect_T<_SIZE_>;
  using DoubleTab_t = DoubleTab_T<_SIZE_>;
  using Domaine_t = Domaine_32_64<_SIZE_>;


  const Nom& nom_lml() const override;
  inline Type_Face type_face(int=0) const override;
  void calculer_centres_gravite(DoubleTab_t& ) const override ;
  void calculer_volumes(DoubleVect_t& ) const override;

protected:
  // Members herited from top classes:
  using Objet_U::dimension;
  using Elem_geom_base_32_64<_SIZE_>::mon_dom;

};


/*! @brief Renvoie le i-ieme type de face.
 *
 * Un rectangle (2D_Axi) n'a qu'un seul type de face.
 *
 * @param (int i) le rang du type de face a renvoyer
 * @return (Type_Face) un type de face
 */
template <typename _SIZE_>
inline Type_Face Rectangle_2D_axi_32_64<_SIZE_>::type_face(int i) const
{
  assert(i==0);
  return Type_Face::quadrilatere_2D_axi;
}

using Rectangle_2D_axi = Rectangle_2D_axi_32_64<int>;
using Rectangle_2D_axi_64 = Rectangle_2D_axi_32_64<trustIdType>;


#endif
