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
// File:        ProblemTrio.h
// Directory:   $TRUST_ROOT/src/Kernel/ICoCo
// Version:     /main/15
//
//////////////////////////////////////////////////////////////////////////////

#ifndef ProblemTrio_included
#define ProblemTrio_included

#include <Problem.h>
#include <vector>
#include <string>

class mon_main;
class Objet_U;

class Probleme_U;
//using std::string;
//using std::vector;

#undef OLD_MEDCOUPLING

#ifdef OLD_MEDCOUPLING
namespace ParaMEDMEM
{
class MEDCouplingFieldDouble;
}
#endif

namespace ICoCo
{

class TrioField;
class MEDField;
class Init_Params;

//////////////////////////////////////////////////////////////////////////////
//
// .DESCRIPTION
//     class ProblemTrio
//
//     This class defines the API a problem has to implement in order to be coupled.
//     For precise specifications and possible use of this API, See NT
//     SSTH/LMDL_2006_001
//     Implements IterativeUnsteadyProblem and FieldIO
//
// .SECTION voir aussi
//////////////////////////////////////////////////////////////////////////////
class ProblemTrio : public Problem
{

public :
  // interface specifique
  bool initialize_pb(Probleme_U& pb);

  // interface Problem
  ProblemTrio();
  virtual void setDataFile(const std::string& datafile);
  virtual void setMPIComm(void*);


  virtual ~ProblemTrio()  ;
  virtual bool initialize();

  virtual void terminate();

  // interface UnsteadyProblem

  virtual double presentTime() const;
  virtual double computeTimeStep(bool& stop) const;
  virtual bool initTimeStep(double dt);
  virtual bool solveTimeStep();
  virtual void validateTimeStep();
  virtual bool isStationary() const;
  virtual void abortTimeStep();

  // interface IterativeUnsteadyProblem

  virtual bool iterateTimeStep(bool& converged);

  // interface FieldIO

  virtual std::vector<std::string> getInputFieldsNames() const;
  virtual void getInputFieldTemplate(const std::string& name, TrioField& afield) const;
  virtual void setInputField(const std::string& name, const TrioField& afield);
  virtual std::vector<std::string> getOutputFieldsNames() const;
  virtual void getOutputField(const std::string& name, TrioField& afield) const;

#ifdef OLD_MEDCOUPLING
  // interface salome
  ParaMEDMEM::MEDCouplingFieldDouble* getOutputMEDField(const std::string& name) const ;
  ParaMEDMEM::MEDCouplingFieldDouble* getInputMEDFieldTemplate(const std::string& name) const;
  void setInputMEDField(const std::string& name, const ParaMEDMEM::MEDCouplingFieldDouble* afield);
#endif
  void getInputFieldTemplate(const std::string& name, MEDField& afield) const;
  void setInputField(const std::string& name, const MEDField& afield);
  void getOutputField(const std::string& name, MEDField& afield) const;


protected :

  Init_Params* my_params;
  Probleme_U* pb;
  mon_main* p;

};
} // namespace ICoCo
Objet_U& get_obj(const char *) ;
#endif

