/*################################################################################
  ##
  ##   Copyright (C) 2011-2018 Keith O'Hara
  ##
  ##   This file is part of the StatsLib C++ library.
  ##
  ##   Licensed under the Apache License, Version 2.0 (the "License");
  ##   you may not use this file except in compliance with the License.
  ##   You may obtain a copy of the License at
  ##
  ##       http://www.apache.org/licenses/LICENSE-2.0
  ##
  ##   Unless required by applicable law or agreed to in writing, software
  ##   distributed under the License is distributed on an "AS IS" BASIS,
  ##   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  ##   See the License for the specific language governing permissions and
  ##   limitations under the License.
  ##
  ################################################################################*/

/* 
 * Sample from an inverse-Wishart distribution
 */

#ifndef _statslib_rinvwish_HPP
#define _statslib_rinvwish_HPP

#ifdef STATS_WITH_MATRIX_LIB

#ifdef STATS_USE_ARMA
template<typename mT, typename pT,
         typename std::enable_if<!(std::is_same<mT,arma::mat>::value)>::type* = nullptr>
#else
template<typename mT, typename pT>
#endif
statslib_inline
mT rinvwish(const mT& Psi_par, const pT nu_par, const bool pre_chol = false);

// specializations
#ifdef STATS_USE_ARMA
template<typename mT, typename eT, typename pT>
statslib_inline
mT rinvwish(const ArmaMat<eT>& Psi_par, const pT nu_par, const bool pre_chol = false);
#endif

#include "rinvwish.ipp"

#endif

#endif