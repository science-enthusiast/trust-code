/****************************************************************************
* Copyright (c) 2015 - 2016, CEA
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
//////////////////////////////////////////////////////////////////////////////
//
// File:        IntListElem.cpp
// Directory:   $TRUST_ROOT/src/Kernel/Math
// Version:     /main/10
//
//////////////////////////////////////////////////////////////////////////////
//
// WARNING: DO NOT EDIT THIS FILE! Only edit the template file IntListElem.cpp.P
//

#include <IntList.h>
#include <Nom.h>

// Description:
//    Constructeur par copie
// Precondition:
// Parametre: const IntListElem& list
//    Signification: la liste a copier
//    Valeurs par defaut:
//    Contraintes:
//    Acces:
// Retour:
//    Signification:
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition:
IntListElem::IntListElem(const IntListElem& list)
{
  if(list.est_vide() )
    {
      suivant_=this;
    }
  else
    {
      data=list.data;
      if(list.suivant_)
        {
          IntListElem* next = new IntListElem(*list.suivant_); //Recursif !!
          suivant_ = next;
        }
      else
        suivant_ =0;
    }
}

// Description:
//    destructeur
// Precondition:
// Parametre:
//    Signification:
//    Valeurs par defaut:
//    Contraintes:
//    Acces:
// Retour:
//    Signification:
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition:
IntListElem:: ~IntListElem()
{
  // Cout << "Destruction de IntList" << finl;
  if (est_vide())
    suivant_=0;
  if(suivant_)
    {
      // On ne garde pas la version delete suivant_ car sinon on est limite par le nombre d'appel recursif possible
      // delete suivant_;
      IntListElem *poignee, *pr;
      pr=suivant_;
      suivant_=0;
      while(pr)
        {
          poignee=pr->suivant_;
          pr->suivant_=0;
          delete pr ;
          pr=poignee;
        }
    }

}

// Description:
//    insertion en queue
// Precondition:
// Parametre: int int_to_add
//    Signification: element a ajouter
//    Valeurs par defaut:
//    Contraintes:
//    Acces:
// Retour: Int_ListElem&
//    Signification: *this
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition:
IntListElem& IntListElem::add(int int_to_add)
{
  assert(est_dernier());
  IntListElem* next=new IntListElem(int_to_add);
  suivant_ = next;
  return *this;
}

// Description:
//     Operateur de comparaison de deux listes
// Precondition:
// Parametre: const IntList& list1
//    Signification: premiere liste a comparer
//    Valeurs par defaut:
//    Contraintes:
//    Acces:
// Parametre: const IntList& list2
//    Signification: seconde liste a comparer
//    Valeurs par defaut:
//    Contraintes:
//    Acces:
// Retour: int
//    Signification: 1 si les listes sont egales, 0 sinon
//    Contraintes:
// Exception:
// Effets de bord:
// Postcondition:
int operator ==(const IntListElem& list1 , const IntListElem& list2)
{
  int retour=1;
  if(list1.data != list2.data)
    retour= 0;
  if( (!list1.est_dernier()) && (list2.est_dernier()) )
    retour= 0;
  if( (list1.est_dernier()) && (!list2.est_dernier()) )
    retour= 0;
  if( (!list1.est_dernier()) && (!list2.est_dernier()) )
    retour= (*list1.suivant_ == *list2.suivant_);
  return retour;
}


