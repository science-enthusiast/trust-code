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

#ifndef Tetraedre_included
#define Tetraedre_included

#include <Elem_geom_base.h>
#include <Linear_algebra_tools.h>

/*! @brief Classe Tetraedre Cette classe represente l'element geometrique Tetraedre.
 *
 *     Un tetraedre est un polyedre qui a 4 faces, 4 sommets et
 *     un seul type de face ayant 3 sommets par face.
 *
 * @sa Elem_geom_base Elem_geom, C'est un element utilisable en 3D
 */
template <typename _SIZE_>
class Tetraedre_32_64 : public Elem_geom_base_32_64<_SIZE_>
{

  Declare_instanciable_32_64(Tetraedre_32_64);

public :
  using int_t = _SIZE_;
  using IntTab_t = IntTab_T<_SIZE_>;
  using SmallArrOfTID_t = SmallArrOfTID_T<_SIZE_>;
  using DoubleVect_t = DoubleVect_T<_SIZE_>;
  using DoubleTab_t = DoubleTab_T<_SIZE_>;
  using Domaine_t = Domaine_32_64<_SIZE_>;


  inline int face_sommet(int i, int j) const override;
  inline int face_sommet0(int i) const;
  inline int face_sommet1(int i) const;
  inline int face_sommet2(int i) const;
  inline int face_sommet3(int i) const;
  inline int nb_som() const override { return 4; }
  inline int nb_faces(int=0) const override;
  inline int nb_som_face(int=0) const override;
  inline bool est_structure() const override { return false; }
  const Nom& nom_lml() const override;
  int contient(const ArrOfDouble& pos, int_t elem) const override;
  int contient(const SmallArrOfTID_t& soms, int_t elem) const override;
  inline Type_Face type_face(int=0) const override;
  void calculer_volumes(DoubleVect_t& vols) const override;
  void calculer_normales(const IntTab_t& faces_sommets , DoubleTab_t& face_normales) const override;

  int get_tab_faces_sommets_locaux(IntTab& faces_som_local) const override;
  void get_tab_aretes_sommets_locaux(IntTab& aretes_som_local) const override;

  static inline void matrice_base_tetraedre(const IntTab& polys,
                                            const DoubleTab& coords,
                                            int num_elem,
                                            Matrice33& m,
                                            Vecteur3& origine);
  static inline void matrice_base_tetraedre(const IntTab& polys,
                                            const DoubleTab& coords,
                                            int num_elem,
                                            Matrice33& m);
//  static double coord_bary(const IntTab& polys,
//                           const DoubleTab& coords,
//                           const Vecteur3& point,
//                           int le_poly,
//                           Vecteur3& coord_bary,
//                           double epsilon = 0.);

protected:
  // Members herited from top classes:
  using Objet_U::dimension;
  using Elem_geom_base_32_64<_SIZE_>::mon_dom;

};


/*! @brief Renvoie le numero du j-ieme sommet de la i-ieme face de l'element.
 *
 * @param (int i) un numero de face
 * @param (int j) un numero de sommet
 * @return (int) le numero du j-ieme sommet de la i-ieme face
 */
template <typename _SIZE_>
inline int Tetraedre_32_64<_SIZE_>::face_sommet(int i, int j) const
{
  assert(i<4);
  switch(i)
    {
    case 0:
      return face_sommet0(j);
    case 1:
      return face_sommet1(j);
    case 2:
      return face_sommet2(j);
    case 3:
      return face_sommet3(j);
    default :
      return -1;
    }
}


/*! @brief Renvoie le nombre de faces du type specifie que possede l'element geometrique.
 *
 * @param (int i) le type de face
 * @return (int) le nombre de faces de type i
 */
template <typename _SIZE_>
inline int Tetraedre_32_64<_SIZE_>::nb_faces(int i) const
{
  assert(i==0);
  return 4;
}


/*! @brief Renvoie le nombre de sommets des faces du type specifie.
 *
 * @param (int i) le type de face
 * @return (int) le nombre de sommets des faces de type i
 */
template <typename _SIZE_>
inline int Tetraedre_32_64<_SIZE_>::nb_som_face(int i) const
{
  assert(i==0);
  return 3;
}


/*! @brief Renvoie le numero du i-ieme sommet de la face 0
 *
 * @param (int i) le numero du sommet a renvoyer
 * @return (int) le numero du i-ieme sommet de la face 0
 */
template <typename _SIZE_>
inline int Tetraedre_32_64<_SIZE_>::face_sommet0(int i) const
{
  //face_sommet0(0)=1;
  //face_sommet0(1)=2;
  //face_sommet0(2)=3;
  assert(i>=0);
  assert(i<3);
  return (i+1);
}


/*! @brief Renvoie le numero du i-ieme sommet de la face 1
 *
 * @param (int i) le numero du sommet a renvoyer
 * @return (int) le numero du i-ieme sommet de la face 1
 */
template <typename _SIZE_>
inline int Tetraedre_32_64<_SIZE_>::face_sommet1(int i) const
{
  //face_sommet0(0)=2;
  //face_sommet1(1)=3;
  //face_sommet2(2)=0;
  assert(i>=0);
  assert(i<3);
  return (i+2)%4;
}


/*! @brief Renvoie le numero du i-ieme sommet de la face 2
 *
 * @param (int i) le numero du sommet a renvoyer
 * @return (int) le numero du i-ieme sommet de la face 2
 */
template <typename _SIZE_>
inline int Tetraedre_32_64<_SIZE_>::face_sommet2(int i) const
{
  //face_sommet2(0)=3;
  //face_sommet2(1)=0;
  //face_sommet2(2)=1;
  assert(i>=0);
  assert(i<3);
  return (i+3)%4;
}


/*! @brief Renvoie le numero du i-ieme sommet de la face 3
 *
 * @param (int i) le numero du sommet a renvoyer
 * @return (int) le numero du i-ieme sommet de la face 3
 */
template <typename _SIZE_>
inline int Tetraedre_32_64<_SIZE_>::face_sommet3(int i) const
{
  //face_sommet2(0)=0;
  //face_sommet2(1)=1;
  //face_sommet2(2)=2;
  assert(i>=0);
  assert(i<3);
  return (i);
}


/*! @brief Renvoie le i-ieme type de face.
 *
 * Un tetraedre n'a qu'un seul type de face.
 *
 * @param (int i) le rang du type de face a renvoyer
 * @return (Type_Face) un type de face
 */
template <typename _SIZE_>
inline Type_Face Tetraedre_32_64<_SIZE_>::type_face(int i) const
{
  assert(i==0);
  return Type_Face::triangle_3D;
}

/*! @brief remplit la matrice m avec les trois vecteurs de base du tetraedre demande (le premier sommet du tetra est pris comme origine).
 *
 *   polys est le tableau des elements du domaine, coords le tableau des coordonnees
 *   num_elem le numero du tetraedre a calculer.
 *   m est rempli avec la matrice m(i,j) = coord(polys(num_elem, j+1), i) - coord(polys(num_elem, 0), i)
 *
 */
template <typename _SIZE_>
inline void Tetraedre_32_64<_SIZE_>::matrice_base_tetraedre(const IntTab& polys,
                                                            const DoubleTab& coords,
                                                            int num_elem,
                                                            Matrice33& m)
{
  const int som0 = polys(num_elem, 0);
  const int som1 = polys(num_elem, 1);
  const int som2 = polys(num_elem, 2);
  const int som3 = polys(num_elem, 3);
  const double x0 = coords(som0, 0);
  const double y0 = coords(som0, 1);
  const double z0 = coords(som0, 2);
  // Matrice des trois vecteurs de base du tetra (origine au sommet 0)
  m(0,0) = coords(som1,0) - x0;
  m(1,0) = coords(som1,1) - y0;
  m(2,0) = coords(som1,2) - z0;
  m(0,1) = coords(som2,0) - x0;
  m(1,1) = coords(som2,1) - y0;
  m(2,1) = coords(som2,2) - z0;
  m(0,2) = coords(som3,0) - x0;
  m(1,2) = coords(som3,1) - y0;
  m(2,2) = coords(som3,2) - z0;
}

/*! @brief Idem que la precedente, mais remplit en plus "origine" avec les coordonnees du premier sommet
 *
 */
template <typename _SIZE_>
inline void Tetraedre_32_64<_SIZE_>::matrice_base_tetraedre(const IntTab& polys,
                                                            const DoubleTab& coords,
                                                            int num_elem,
                                                            Matrice33& m,
                                                            Vecteur3& origine)
{
  const int som0 = polys(num_elem, 0);
  const int som1 = polys(num_elem, 1);
  const int som2 = polys(num_elem, 2);
  const int som3 = polys(num_elem, 3);
  const double x0 = coords(som0, 0);
  const double y0 = coords(som0, 1);
  const double z0 = coords(som0, 2);
  // Matrice des trois vecteurs de base du tetra (origine au sommet 0)
  m(0,0) = coords(som1,0) - x0;
  m(1,0) = coords(som1,1) - y0;
  m(2,0) = coords(som1,2) - z0;
  m(0,1) = coords(som2,0) - x0;
  m(1,1) = coords(som2,1) - y0;
  m(2,1) = coords(som2,2) - z0;
  m(0,2) = coords(som3,0) - x0;
  m(1,2) = coords(som3,1) - y0;
  m(2,2) = coords(som3,2) - z0;
  origine[0] = x0;
  origine[1] = y0;
  origine[2] = z0;
}


using Tetraedre = Tetraedre_32_64<int>;
using Tetraedre_64 = Tetraedre_32_64<trustIdType>;

#endif
