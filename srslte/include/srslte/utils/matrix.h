/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2014 The srsLTE Developers. See the
 * COPYRIGHT file at the top-level directory of this distribution.
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * A copy of the GNU Lesser General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#ifndef MATRIX_
#define MATRIX_

#include <stdio.h>
#include "srslte/config.h"

typedef _Complex float cf_t;

SRSLTE_API int matrix_init(void ***m, int sz_x, int sz_y, int elem_sz);
SRSLTE_API void matrix_free(void **q, int sz_x);
SRSLTE_API void matrix_bzero(void **q, int sz_x, int sz_y, int elem_sz);
SRSLTE_API void matrix_fprintf_cf(FILE *f, cf_t **q, int sz_x, int sz_y);
SRSLTE_API void matrix_fprintf_f(FILE *f, float **q, int sz_x, int sz_y);
SRSLTE_API void matrix_copy(void **dst, void **src, int sz_x, int sz_y, int elem_sz);
SRSLTE_API void matrix_dotprod_cf(cf_t **x, cf_t **y, cf_t **out, int sz_x, int sz_y);
SRSLTE_API void matrix_dotprod_float(float **x, float **y, float **out, int sz_x, int sz_y);
SRSLTE_API void matrix_dotprod_int(int **x, int **y, int **out, int sz_x, int sz_y);

#endif // MATRIX_
