//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
// Copyright 2014 joydeepb@ri.cmu.edu
// Robotics Institute, Carnegie Mellon University
//
// OpenMP helper functions and macros.

#ifndef OPENMP_UTILS_H
#define OPENMP_UTILS_H

#ifdef NDEBUG
  #ifdef _OPENMP
    #define OMP_PARALLEL_FOR _Pragma("omp parallel for")
    #define OMP_PARALLEL _Pragma("omp parallel")
    #define OMP_FOR _Pragma("omp for")
  #else
    #warning OpenMP not detected, but in Release mode.
    #define OMP_PARALLEL_FOR {}
    #define OMP_PARALLEL {}
    #define OMP_FOR {}
  #endif  // _OPENMP
#else
  #define OMP_PARALLEL_FOR {}
#endif  // NDEBUG

#endif  // OPENMP_UTILS_H
