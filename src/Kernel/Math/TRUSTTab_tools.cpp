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

#include <TRUSTTab.h>
#include <TRUSTTab_tools.tpp>
#include <limits>

// ToDo OpenMP: porter boucle mais mp_norme_tab semble pas utilise
template <typename _TYPE_, typename _SIZE_>
void local_carre_norme_tab(const TRUSTTab<_TYPE_,_SIZE_>& tableau, TRUSTArray<_TYPE_,_SIZE_>& norme_colonne)
{
  norme_colonne = 0.;
  const TRUSTArray<int,_SIZE_>& blocs = tableau.get_md_vector()->get_items_to_sum();
  const _SIZE_ nblocs = blocs.size_array() >> 1;
  const TRUSTVect<_TYPE_,_SIZE_>& vect = tableau;
  const _SIZE_ lsize = vect.line_size();
  assert(lsize == norme_colonne.size_array());
  for (_SIZE_ ibloc = 0; ibloc < nblocs; ibloc++)
    {
      const _SIZE_ begin_bloc = blocs[ibloc], end_bloc = blocs[ibloc+1];
      for (_SIZE_ i = begin_bloc; i < end_bloc; i++)
        {
          _SIZE_ k = i * lsize;
          for (_SIZE_ j = 0; j < lsize; j++)
            {
              const _TYPE_ x = vect[k++];
              norme_colonne[j] += x*x;
            }
        }
    }
}

template <typename ExecSpace, typename tableauViewType, typename max_colonneViewType, typename BlocType, typename _TYPE_, typename _SIZE_>
void parallel_max_abs_column(const tableauViewType& tableau_view, max_colonneViewType& max_colonne_view,
                             BlocType blocs, int lsize, bool kernelOnDevice)
{
  const _SIZE_ nblocs = blocs.size_array() >> 1;
  for (_SIZE_ ibloc = 0; ibloc < nblocs; ibloc++)
    {
      const _SIZE_ begin_bloc = blocs[ibloc], end_bloc = blocs[ibloc+1];
      // Define a Kokkos range policy based on the execution space
      Kokkos::RangePolicy<ExecSpace> policy(begin_bloc, end_bloc);
      // Parallel loop for any value of lsize, using atomic_max for thread safety
      Kokkos::parallel_for(start_gpu_timer(__KERNEL_NAME__), policy, KOKKOS_LAMBDA(const int i)
      {
        int k = i * lsize;
        for (int j = 0; j < lsize; j++)
          {
            const _TYPE_ x = Kokkos::fabs(tableau_view(k,j));
            Kokkos::atomic_max(&max_colonne_view(j), x);
          }
      });
    }
  end_gpu_timer(kernelOnDevice, __KERNEL_NAME__);
}

template <typename _TYPE_, typename _SIZE_>
void local_max_abs_tab(const TRUSTTab<_TYPE_,_SIZE_>& tableau, TRUSTArray<_TYPE_,_SIZE_>& max_colonne)
{
  max_colonne = std::numeric_limits<_TYPE_>::min();
  const TRUSTArray<int,_SIZE_>& blocs = tableau.get_md_vector()->get_items_to_compute();
  const _SIZE_ lsize = tableau.line_size();
  for (_SIZE_ j = 0; j < lsize; j++) max_colonne[j] = 0;
  assert(lsize == max_colonne.size_array());

  bool kernelOnDevice = tableau.isDataOnDevice();
  ToDo_Kokkos("!!! WARNING !!!: Function local_max_abs_tab was never tested, contact remi.bourgeois@cea.fr if wrong results");

  if (kernelOnDevice)
    {
      auto tableau_view = tableau.view_ro();
      auto max_colonne_view = max_colonne.view_rw();
      using ExecSpace = Kokkos::DefaultExecutionSpace; //Compute on the Device
      parallel_max_abs_column<ExecSpace, decltype(tableau_view), decltype(max_colonne_view), decltype(blocs), _TYPE_, _SIZE_>(tableau_view, max_colonne_view, blocs, lsize, kernelOnDevice);
    }
  else
    {
      auto tableau_view = Kokkos::View<const _TYPE_**, Kokkos::HostSpace, Kokkos::MemoryUnmanaged>(tableau.addr(), tableau.size_array());
      auto max_colonne_view = Kokkos::View<_TYPE_*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged>(max_colonne.addr(), max_colonne.size_array());
      using ExecSpace = Kokkos::DefaultHostExecutionSpace; //Compute on the Host
      parallel_max_abs_column<ExecSpace, decltype(tableau_view), decltype(max_colonne_view), decltype(blocs), _TYPE_, _SIZE_>(tableau_view, max_colonne_view, blocs, lsize, kernelOnDevice);
    }
  //Useful for host run ?
  //copyFromDevice(max_colonne, "max_colonne in local_max_abs_tab"); // ToDo OpenMP pourquoi necessaire ? Est ce a cause des ecritures put(addr[]) ?
}
template void local_carre_norme_tab<double,int>(const TRUSTTab<double,int>& tableau, TRUSTArray<double,int>& norme_colonne);
template void local_carre_norme_tab<float,int>(const TRUSTTab<float,int>& tableau, TRUSTArray<float,int>& norme_colonne);
template void local_max_abs_tab<double,int>(const TRUSTTab<double,int>& tableau, TRUSTArray<double,int>& max_colonne);
template void local_max_abs_tab<float,int>(const TRUSTTab<float,int>& tableau, TRUSTArray<float,int>& max_colonne);
