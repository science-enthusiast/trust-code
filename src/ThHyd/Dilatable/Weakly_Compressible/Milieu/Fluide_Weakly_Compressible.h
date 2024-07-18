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

#ifndef Fluide_Weakly_Compressible_included
#define Fluide_Weakly_Compressible_included

#include <Fluide_Dilatable_base.h>

/*! @brief classe Fluide_Weakly_Compressible Cette classe represente un d'un fluide faiblement compressible
 *
 *     c'est a dire heritant de Fluide_Dilatable_base
 *
 * @sa Milieu_base Fluide_Dilatable_base
 */

class Fluide_Weakly_Compressible : public Fluide_Dilatable_base
{
  Declare_instanciable(Fluide_Weakly_Compressible);

public :
  void set_param(Param& param) override;
  void completer(const Probleme_base&) override;
  void checkTraitementPth(const Domaine_Cl_dis&) override;
  void discretiser(const Probleme_base& pb, const  Discretisation_base& dis) override;
  void abortTimeStep() override;
  void calculer_pression_tot() override;
  void update_pressure_fields(double ) override;
  void write_mean_edo(double) override;
  void write_header_edo() override;

  // Methodes inlines
  inline void Resoudre_EDO_PT() override;
  inline const Champ_Don& pression_hydro() const { return pression_hydro_; }
  inline Champ_Don& pression_hydro() { return pression_hydro_; }
  inline const Champ_Don& pression_eos() const { return pression_eos_; }
  inline Champ_Don& pression_eos() { return pression_eos_; }
  inline const Champ_Don& fraction_massique_nonresolue() const { return unsolved_species_; }
  inline Champ_Don& fraction_massique_nonresolue() { return unsolved_species_; }
  inline const DoubleTab& pression_th_tab() const { return Pth_tab_; } // Tab Pression thermodynamique
  inline DoubleTab& pression_th_tab() { return Pth_tab_; } // Tab Pression thermodynamique
  inline const DoubleTab& pression_thn_tab() const { return Pth_n_tab_; } // Tab Pression thermodynamique a l'etape precedente
  inline DoubleTab& pression_thn_tab() { return Pth_n_tab_; } // Tab Pression thermodynamique a l'etape precedente

  inline void secmembre_divU_Z(DoubleTab& tab_W) const override { eos_tools_->secmembre_divU_Z(tab_W); }
  inline void set_pression_th_tab(DoubleTab& Pth_tab) { Pth_n_tab_ = Pth_tab_ = Pth_tab; }
  inline void set_resume_flag() { sim_resumed_ = 1; }
  inline bool use_total_pressure() { return use_total_pressure_; }
  inline bool use_hydrostatic_pressure() { return use_hydrostatic_pressure_; }
  inline bool use_pth_xyz() { return Pth_xyz_.non_nul(); }
  inline bool use_total_hydro_pressure() { return (use_total_pressure_||use_hydrostatic_pressure_); }
  inline bool use_saved_data() { return sim_resumed_; }
  inline bool use_grad_pression_eos() { return use_grad_pression_eos_; }

protected:
  Champ_Don Pth_xyz_, pression_hydro_, pression_eos_, unsolved_species_;
  DoubleTab Pth_tab_, Pth_n_tab_, P_NS_elem_;
  int use_total_pressure_ = 0, use_hydrostatic_pressure_ = 0, use_grad_pression_eos_ = 1, sim_resumed_ = 0;
  double time_activate_ptot_ = -1.;

private:
  void calculer_pression_hydro();
  void remplir_champ_pression_tot(int n, const DoubleTab& PHydro, DoubleTab& PTot) override;
  void remplir_champ_pression_for_EOS();
};

inline void Fluide_Weakly_Compressible::Resoudre_EDO_PT()
{
  if (Pth_ > -1. && !use_total_pressure() && !use_hydrostatic_pressure())
    {
      Pth_n = Pth_;
      Pth_n_tab_ = Pth_tab_;

      if (traitement_PTh != 2)
        {
          Pth_ = EDO_Pth_->resoudre(Pth_);

          if (Pth_ <= 0)
            {
              Cerr << "Error : the pressure calculated by Resoudre_EDO_PT method is negative : " << Pth_ << finl;
              abort();
            }

          const int n = Pth_tab_.dimension_tot(0);
          for (int i=0; i<n; i++) Pth_tab_(i) = Pth_;
        }
    }
  else
    {
      Pth_n_tab_ = Pth_tab_;

      if (traitement_PTh != 2)
        {
          EDO_Pth_->resoudre(Pth_tab_);
          if (mp_min_vect(Pth_tab_) <= 0)
            {
              Cerr << "Error : the pressure calculated by Resoudre_EDO_PT method is negative : " << mp_min_vect(Pth_tab_) << finl;
              abort();
            }
        }
    }
}

#endif /* Fluide_Weakly_Compressible_included */
