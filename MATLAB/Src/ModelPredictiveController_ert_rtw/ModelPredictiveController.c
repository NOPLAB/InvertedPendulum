/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ModelPredictiveController.c
 *
 * Code generated for Simulink model 'ModelPredictiveController'.
 *
 * Model version                  : 1.41
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Mon Jun 16 15:38:04 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "ModelPredictiveController.h"
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#include "math.h"

/* Named constants for MATLAB Function: '<S22>/optimizer' */
#define degrees                        (3)

/* Block signals and states (default storage) */
DW rtDW;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_hypotd_snf(real_T u0, real_T u1);
static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);

/* Forward declaration for local functions */
static real_T norm(const real_T x[3]);
static real_T maximum(const real_T x[3]);
static real_T xnrm2(int32_T n, const real_T x[9], int32_T ix0);
static void xgemv(int32_T b_m, int32_T n, const real_T b_A[9], int32_T ia0,
                  const real_T x[9], int32_T ix0, real_T y[3]);
static void xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0, const
                  real_T y[3], real_T b_A[9], int32_T ia0);
static real_T KWIKfactor(const real_T b_Ac[12], const int32_T iC[4], int32_T nA,
  const real_T b_Linv[9], real_T RLinv[9], real_T D[9], real_T b_H[9], int32_T n);
static void DropConstraint(int32_T kDrop, boolean_T iA[4], int32_T *nA, int32_T
  iC[4]);
static void qpkwik(const real_T b_Linv[9], const real_T b_Hinv[9], const real_T
                   f[3], const real_T b_Ac[12], const real_T b[4], boolean_T iA
                   [4], int32_T maxiter, real_T FeasTol, real_T x[3], real_T
                   lambda[4], int32_T *status);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);
extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
real_T rtNaN = -(real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = -(real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      /* do nothing */
#else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

/* Return rtNaN needed by the generated code. */
static real_T rtGetNaN(void)
{
  return rtNaN;
}

/* Return rtNaNF needed by the generated code. */
static real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/* Function for MATLAB Function: '<S22>/optimizer' */
static real_T norm(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S22>/optimizer' */
static real_T maximum(const real_T x[3])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 4; k++) {
      real_T x_0;
      x_0 = x[k - 1];
      if (ex < x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S22>/optimizer' */
static real_T xnrm2(int32_T n, const real_T x[9], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (k = ix0; k < kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else if (rtIsNaN(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S22>/optimizer' */
static void xgemv(int32_T b_m, int32_T n, const real_T b_A[9], int32_T ia0,
                  const real_T x[9], int32_T ix0, real_T y[3])
{
  int32_T b_iy;
  int32_T ia;
  if ((b_m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)n * sizeof(real_T));
    }

    b = (n - 1) * 3 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 3) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = (b_iy + b_m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        c += x[((ix0 + ia) - b_iy) - 1] * b_A[ia - 1];
      }

      ia = div_nde_s32_floor(b_iy - ia0, 3);
      y[ia] += c;
    }
  }
}

/* Function for MATLAB Function: '<S22>/optimizer' */
static void xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0, const
                  real_T y[3], real_T b_A[9], int32_T ia0)
{
  int32_T ijA;
  int32_T j;
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = (b_m + jA) - 1;
        for (ijA = jA; ijA <= b; ijA++) {
          b_A[ijA - 1] += b_A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 3;
    }
  }
}

/* Function for MATLAB Function: '<S22>/optimizer' */
static real_T KWIKfactor(const real_T b_Ac[12], const int32_T iC[4], int32_T nA,
  const real_T b_Linv[9], real_T RLinv[9], real_T D[9], real_T b_H[9], int32_T n)
{
  real_T Q[9];
  real_T R[9];
  real_T TL[9];
  real_T b_A[9];
  real_T tau[3];
  real_T work[3];
  real_T Status;
  real_T atmp;
  real_T b_A_0;
  real_T beta1;
  int32_T b_coltop;
  int32_T b_lastv;
  int32_T coltop;
  int32_T exitg1;
  int32_T ii;
  int32_T k_i;
  int32_T knt;
  boolean_T exitg2;
  Status = 1.0;
  memset(&RLinv[0], 0, 9U * sizeof(real_T));
  for (k_i = 0; k_i < nA; k_i++) {
    b_lastv = iC[k_i];
    for (b_coltop = 0; b_coltop < 3; b_coltop++) {
      RLinv[b_coltop + 3 * k_i] = (b_Ac[b_lastv - 1] * b_Linv[b_coltop] +
        b_Linv[b_coltop + 3] * b_Ac[b_lastv + 3]) + b_Linv[b_coltop + 6] *
        b_Ac[b_lastv + 7];
    }
  }

  memcpy(&b_A[0], &RLinv[0], 9U * sizeof(real_T));
  tau[0] = 0.0;
  work[0] = 0.0;
  tau[1] = 0.0;
  work[1] = 0.0;
  tau[2] = 0.0;
  work[2] = 0.0;
  for (k_i = 0; k_i < 3; k_i++) {
    ii = k_i * 3 + k_i;
    if (k_i + 1 < 3) {
      atmp = b_A[ii];
      b_lastv = ii + 2;
      tau[k_i] = 0.0;
      beta1 = xnrm2(2 - k_i, b_A, ii + 2);
      if (beta1 != 0.0) {
        b_A_0 = b_A[ii];
        beta1 = rt_hypotd_snf(b_A_0, beta1);
        if (b_A_0 >= 0.0) {
          beta1 = -beta1;
        }

        if (fabs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          coltop = (ii - k_i) + 3;
          do {
            knt++;
            for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
              b_A[b_coltop - 1] *= 9.9792015476736E+291;
            }

            beta1 *= 9.9792015476736E+291;
            atmp *= 9.9792015476736E+291;
          } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

          beta1 = rt_hypotd_snf(atmp, xnrm2(2 - k_i, b_A, ii + 2));
          if (atmp >= 0.0) {
            beta1 = -beta1;
          }

          tau[k_i] = (beta1 - atmp) / beta1;
          atmp = 1.0 / (atmp - beta1);
          for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
            b_A[b_coltop - 1] *= atmp;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            beta1 *= 1.0020841800044864E-292;
          }

          atmp = beta1;
        } else {
          tau[k_i] = (beta1 - b_A_0) / beta1;
          atmp = 1.0 / (b_A_0 - beta1);
          b_coltop = (ii - k_i) + 3;
          for (knt = b_lastv; knt <= b_coltop; knt++) {
            b_A[knt - 1] *= atmp;
          }

          atmp = beta1;
        }
      }

      b_A[ii] = 1.0;
      if (tau[k_i] != 0.0) {
        b_lastv = 3 - k_i;
        knt = (ii - k_i) + 2;
        while ((b_lastv > 0) && (b_A[knt] == 0.0)) {
          b_lastv--;
          knt--;
        }

        knt = 2 - k_i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          b_coltop = ((knt - 1) * 3 + ii) + 3;
          coltop = b_coltop;
          do {
            exitg1 = 0;
            if (coltop + 1 <= b_coltop + b_lastv) {
              if (b_A[coltop] != 0.0) {
                exitg1 = 1;
              } else {
                coltop++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        knt = 0;
      }

      if (b_lastv > 0) {
        xgemv(b_lastv, knt, b_A, ii + 4, b_A, ii + 1, work);
        xgerc(b_lastv, knt, -tau[k_i], ii + 1, work, b_A, ii + 4);
      }

      b_A[ii] = atmp;
    } else {
      tau[2] = 0.0;
    }
  }

  for (k_i = 0; k_i < 3; k_i++) {
    for (ii = 0; ii <= k_i; ii++) {
      R[ii + 3 * k_i] = b_A[3 * k_i + ii];
    }

    for (ii = k_i + 2; ii < 4; ii++) {
      R[(ii + 3 * k_i) - 1] = 0.0;
    }

    work[k_i] = 0.0;
  }

  for (k_i = 2; k_i >= 0; k_i--) {
    b_lastv = (k_i * 3 + k_i) + 4;
    if (k_i + 1 < 3) {
      b_A[b_lastv - 4] = 1.0;
      if (tau[k_i] != 0.0) {
        knt = 3 - k_i;
        b_coltop = b_lastv - k_i;
        while ((knt > 0) && (b_A[b_coltop - 2] == 0.0)) {
          knt--;
          b_coltop--;
        }

        b_coltop = 2 - k_i;
        exitg2 = false;
        while ((!exitg2) && (b_coltop > 0)) {
          coltop = (b_coltop - 1) * 3 + b_lastv;
          ii = coltop;
          do {
            exitg1 = 0;
            if (ii <= (coltop + knt) - 1) {
              if (b_A[ii - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ii++;
              }
            } else {
              b_coltop--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        knt = 0;
        b_coltop = 0;
      }

      if (knt > 0) {
        xgemv(knt, b_coltop, b_A, b_lastv, b_A, b_lastv - 3, work);
        xgerc(knt, b_coltop, -tau[k_i], b_lastv - 3, work, b_A, b_lastv);
      }

      b_coltop = b_lastv - k_i;
      for (knt = b_lastv - 2; knt < b_coltop; knt++) {
        b_A[knt - 1] *= -tau[k_i];
      }
    }

    b_A[b_lastv - 4] = 1.0 - tau[k_i];
    for (knt = 0; knt < k_i; knt++) {
      b_A[(b_lastv - knt) - 5] = 0.0;
    }
  }

  for (k_i = 0; k_i < 3; k_i++) {
    Q[3 * k_i] = b_A[3 * k_i];
    b_lastv = 3 * k_i + 1;
    Q[b_lastv] = b_A[b_lastv];
    b_lastv = 3 * k_i + 2;
    Q[b_lastv] = b_A[b_lastv];
  }

  k_i = 0;
  do {
    exitg1 = 0;
    if (k_i <= nA - 1) {
      if (fabs(R[3 * k_i + k_i]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        k_i++;
      }
    } else {
      for (k_i = 0; k_i < n; k_i++) {
        for (ii = 0; ii < n; ii++) {
          TL[k_i + 3 * ii] = (b_Linv[3 * k_i + 1] * Q[3 * ii + 1] + b_Linv[3 *
                              k_i] * Q[3 * ii]) + b_Linv[3 * k_i + 2] * Q[3 * ii
            + 2];
        }
      }

      memset(&RLinv[0], 0, 9U * sizeof(real_T));
      for (k_i = nA; k_i >= 1; k_i--) {
        b_coltop = (k_i - 1) * 3;
        knt = (k_i + b_coltop) - 1;
        RLinv[knt] = 1.0;
        for (ii = k_i; ii <= nA; ii++) {
          coltop = ((ii - 1) * 3 + k_i) - 1;
          RLinv[coltop] /= R[knt];
        }

        if (k_i > 1) {
          for (ii = 0; ii <= k_i - 2; ii++) {
            for (b_lastv = k_i; b_lastv <= nA; b_lastv++) {
              knt = (b_lastv - 1) * 3;
              coltop = knt + ii;
              RLinv[coltop] -= RLinv[(knt + k_i) - 1] * R[b_coltop + ii];
            }
          }
        }
      }

      for (k_i = 0; k_i < n; k_i++) {
        for (ii = k_i + 1; ii <= n; ii++) {
          b_coltop = (ii - 1) * 3 + k_i;
          b_H[b_coltop] = 0.0;
          for (b_lastv = nA + 1; b_lastv <= n; b_lastv++) {
            knt = (b_lastv - 1) * 3;
            b_H[b_coltop] -= TL[(knt + ii) - 1] * TL[knt + k_i];
          }

          b_H[(ii + 3 * k_i) - 1] = b_H[b_coltop];
        }
      }

      for (k_i = 0; k_i < nA; k_i++) {
        for (ii = 0; ii < n; ii++) {
          b_coltop = 3 * k_i + ii;
          D[b_coltop] = 0.0;
          for (b_lastv = k_i + 1; b_lastv <= nA; b_lastv++) {
            knt = (b_lastv - 1) * 3;
            D[b_coltop] += TL[knt + ii] * RLinv[knt + k_i];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

/* Function for MATLAB Function: '<S22>/optimizer' */
static void DropConstraint(int32_T kDrop, boolean_T iA[4], int32_T *nA, int32_T
  iC[4])
{
  int32_T i;
  if (kDrop > 0) {
    iA[iC[kDrop - 1] - 1] = false;
    if (kDrop < *nA) {
      for (i = kDrop; i < *nA; i++) {
        iC[i - 1] = iC[i];
      }
    }

    iC[*nA - 1] = 0;
    (*nA)--;
  }
}

/* Function for MATLAB Function: '<S22>/optimizer' */
static void qpkwik(const real_T b_Linv[9], const real_T b_Hinv[9], const real_T
                   f[3], const real_T b_Ac[12], const real_T b[4], boolean_T iA
                   [4], int32_T maxiter, real_T FeasTol, real_T x[3], real_T
                   lambda[4], int32_T *status)
{
  real_T D[9];
  real_T RLinv[9];
  real_T U[9];
  real_T b_H[9];
  real_T Opt[6];
  real_T Rhs[6];
  real_T cTol[4];
  real_T r[3];
  real_T z[3];
  real_T Xnorm0;
  real_T cMin;
  real_T cVal;
  real_T cVal_tmp;
  real_T rMin;
  real_T t;
  int32_T iC[4];
  int32_T b_exponent;
  int32_T exitg1;
  int32_T exitg3;
  int32_T exponent;
  int32_T iC_0;
  int32_T iSave;
  int32_T kDrop;
  int32_T kNext;
  int32_T nA;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1;
  boolean_T guard2;
  x[0] = 0.0;
  x[1] = 0.0;
  x[2] = 0.0;
  lambda[0] = 0.0;
  lambda[1] = 0.0;
  lambda[2] = 0.0;
  lambda[3] = 0.0;
  *status = 1;
  r[0] = 0.0;
  r[1] = 0.0;
  r[2] = 0.0;
  rMin = 0.0;
  cTolComputed = false;
  cTol[0] = 1.0;
  iC[0] = 0;
  cTol[1] = 1.0;
  iC[1] = 0;
  cTol[2] = 1.0;
  iC[2] = 0;
  cTol[3] = 1.0;
  iC[3] = 0;
  nA = 0;
  if (iA[0]) {
    nA = 1;
    iC[0] = 1;
  }

  if (iA[1]) {
    nA++;
    iC[nA - 1] = 2;
  }

  if (iA[2]) {
    nA++;
    iC[nA - 1] = 3;
  }

  if (iA[3]) {
    nA++;
    iC[nA - 1] = 4;
  }

  guard1 = false;
  if (nA > 0) {
    for (kNext = 0; kNext < 6; kNext++) {
      Opt[kNext] = 0.0;
    }

    Rhs[0] = f[0];
    Rhs[3] = 0.0;
    Rhs[1] = f[1];
    Rhs[4] = 0.0;
    Rhs[2] = f[2];
    Rhs[5] = 0.0;
    DualFeasible = false;
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && (*status <= maxiter)) {
        Xnorm0 = KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, D, b_H, degrees);
        if (Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            nA = 0;
            iA[0] = false;
            iC[0] = 0;
            iA[1] = false;
            iC[1] = 0;
            iA[2] = false;
            iC[2] = 0;
            iA[3] = false;
            iC[3] = 0;
            ColdReset = true;
          }
        } else {
          for (kNext = 0; kNext < nA; kNext++) {
            Rhs[kNext + 3] = b[iC[kNext] - 1];
            for (kDrop = kNext + 1; kDrop <= nA; kDrop++) {
              iC_0 = (3 * kNext + kDrop) - 1;
              U[iC_0] = 0.0;
              for (iSave = 0; iSave < nA; iSave++) {
                U[iC_0] += RLinv[(3 * iSave + kDrop) - 1] * RLinv[3 * iSave +
                  kNext];
              }

              U[kNext + 3 * (kDrop - 1)] = U[iC_0];
            }
          }

          for (kNext = 0; kNext < 3; kNext++) {
            Opt[kNext] = (b_H[kNext + 3] * Rhs[1] + b_H[kNext] * Rhs[0]) +
              b_H[kNext + 6] * Rhs[2];
            for (kDrop = 0; kDrop < nA; kDrop++) {
              Opt[kNext] += D[3 * kDrop + kNext] * Rhs[kDrop + 3];
            }
          }

          for (kNext = 0; kNext < nA; kNext++) {
            Opt[kNext + 3] = (D[3 * kNext + 1] * Rhs[1] + D[3 * kNext] * Rhs[0])
              + D[3 * kNext + 2] * Rhs[2];
            for (kDrop = 0; kDrop < nA; kDrop++) {
              Opt[kNext + 3] += U[3 * kDrop + kNext] * Rhs[kDrop + 3];
            }
          }

          Xnorm0 = -1.0E-12;
          kDrop = -1;
          for (kNext = 0; kNext < nA; kNext++) {
            cMin = Opt[kNext + 3];
            lambda[iC[kNext] - 1] = cMin;
            if ((cMin < Xnorm0) && (kNext + 1 <= nA)) {
              kDrop = kNext;
              Xnorm0 = cMin;
            }
          }

          if (kDrop + 1 <= 0) {
            DualFeasible = true;
            x[0] = Opt[0];
            x[1] = Opt[1];
            x[2] = Opt[2];
          } else {
            (*status)++;
            if (*status > 5) {
              nA = 0;
              iA[0] = false;
              iC[0] = 0;
              iA[1] = false;
              iC[1] = 0;
              iA[2] = false;
              iC[2] = 0;
              iA[3] = false;
              iC[3] = 0;
              ColdReset = true;
            } else {
              lambda[iC[kDrop] - 1] = 0.0;
              DropConstraint(kDrop + 1, iA, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          lambda[0] = 0.0;
          lambda[1] = 0.0;
          lambda[2] = 0.0;
          lambda[3] = 0.0;
          Xnorm0 = f[1];
          cMin = f[0];
          cVal = f[2];
          for (kNext = 0; kNext < 3; kNext++) {
            x[kNext] = (-b_Hinv[kNext + 3] * Xnorm0 + -b_Hinv[kNext] * cMin) +
              -b_Hinv[kNext + 6] * cVal;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    Xnorm0 = f[1];
    cMin = f[0];
    cVal = f[2];
    for (kNext = 0; kNext < 3; kNext++) {
      x[kNext] = (-b_Hinv[kNext + 3] * Xnorm0 + -b_Hinv[kNext] * cMin) +
        -b_Hinv[kNext + 6] * cVal;
    }

    guard1 = true;
  }

  if (guard1) {
    Xnorm0 = norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= maxiter)) {
      cMin = -FeasTol;
      kNext = -1;
      for (kDrop = 0; kDrop < 4; kDrop++) {
        if (!cTolComputed) {
          z[0] = fabs(b_Ac[kDrop] * x[0]);
          z[1] = fabs(b_Ac[kDrop + 4] * x[1]);
          z[2] = fabs(b_Ac[kDrop + 8] * x[2]);
          cTol[kDrop] = fmax(cTol[kDrop], maximum(z));
        }

        if (!iA[kDrop]) {
          cVal = (((b_Ac[kDrop + 4] * x[1] + b_Ac[kDrop] * x[0]) + b_Ac[kDrop +
                   8] * x[2]) - b[kDrop]) / cTol[kDrop];
          if (cVal < cMin) {
            cMin = cVal;
            kNext = kDrop;
          }
        }
      }

      cTolComputed = true;
      if (kNext + 1 <= 0) {
        exitg2 = true;
      } else if (*status == maxiter) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((kNext + 1 > 0) && (*status <= maxiter)) {
            guard2 = false;
            if (nA == 0) {
              for (iC_0 = 0; iC_0 < 3; iC_0++) {
                z[iC_0] = (b_Hinv[iC_0 + 3] * b_Ac[kNext + 4] + b_Hinv[iC_0] *
                           b_Ac[kNext]) + b_Hinv[iC_0 + 6] * b_Ac[kNext + 8];
              }

              guard2 = true;
            } else {
              cMin = KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, D, b_H, degrees);
              if (cMin <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (iC_0 = 0; iC_0 < 9; iC_0++) {
                  U[iC_0] = -b_H[iC_0];
                }

                for (iC_0 = 0; iC_0 < 3; iC_0++) {
                  z[iC_0] = (U[iC_0 + 3] * b_Ac[kNext + 4] + U[iC_0] *
                             b_Ac[kNext]) + U[iC_0 + 6] * b_Ac[kNext + 8];
                }

                for (kDrop = 0; kDrop < nA; kDrop++) {
                  r[kDrop] = (D[3 * kDrop + 1] * b_Ac[kNext + 4] + D[3 * kDrop] *
                              b_Ac[kNext]) + D[3 * kDrop + 2] * b_Ac[kNext + 8];
                }

                guard2 = true;
              }
            }

            if (guard2) {
              kDrop = 0;
              cMin = 0.0;
              DualFeasible = true;
              ColdReset = true;
              if (nA > 0) {
                iSave = 0;
                exitg4 = false;
                while ((!exitg4) && (iSave <= nA - 1)) {
                  if (r[iSave] >= 1.0E-12) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    iSave++;
                  }
                }
              }

              if ((nA != 0) && (!ColdReset)) {
                for (iSave = 0; iSave < nA; iSave++) {
                  cVal = r[iSave];
                  if (cVal > 1.0E-12) {
                    cVal = lambda[iC[iSave] - 1] / cVal;
                    if ((kDrop == 0) || (cVal < rMin)) {
                      rMin = cVal;
                      kDrop = iSave + 1;
                    }
                  }
                }

                if (kDrop > 0) {
                  cMin = rMin;
                  DualFeasible = false;
                }
              }

              t = b_Ac[kNext + 4];
              cVal_tmp = b_Ac[kNext + 8];
              cVal = (t * z[1] + z[0] * b_Ac[kNext]) + cVal_tmp * z[2];
              if (cVal <= 0.0) {
                cVal = 0.0;
                ColdReset = true;
              } else {
                cVal = (b[kNext] - ((t * x[1] + b_Ac[kNext] * x[0]) + cVal_tmp *
                                    x[2])) / cVal;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (ColdReset) {
                  t = cMin;
                } else if (DualFeasible) {
                  t = cVal;
                } else if (cMin < cVal) {
                  t = cMin;
                } else {
                  t = cVal;
                }

                for (iSave = 0; iSave < nA; iSave++) {
                  iC_0 = iC[iSave];
                  lambda[iC_0 - 1] -= t * r[iSave];
                  if ((iC_0 <= 4) && (lambda[iC_0 - 1] < 0.0)) {
                    lambda[iC_0 - 1] = 0.0;
                  }
                }

                lambda[kNext] += t;
                frexp(1.0, &exponent);
                if (fabs(t - cMin) < 2.2204460492503131E-16) {
                  DropConstraint(kDrop, iA, &nA, iC);
                }

                if (!ColdReset) {
                  x[0] += t * z[0];
                  x[1] += t * z[1];
                  x[2] += t * z[2];
                  frexp(1.0, &b_exponent);
                  if (fabs(t - cVal) < 2.2204460492503131E-16) {
                    if (nA == degrees) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      nA++;
                      iC[nA - 1] = kNext + 1;
                      kDrop = nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (kDrop + 1 > 1)) {
                        iC_0 = iC[kDrop - 1];
                        if (iC[kDrop] > iC_0) {
                          exitg4 = true;
                        } else {
                          iSave = iC[kDrop];
                          iC[kDrop] = iC_0;
                          iC[kDrop - 1] = iSave;
                          kDrop--;
                        }
                      }

                      iA[kNext] = true;
                      kNext = -1;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            cMin = norm(x);
            if (fabs(cMin - Xnorm0) > 0.001) {
              Xnorm0 = cMin;
              cTol[0] = fmax(fabs(b[0]), 1.0);
              cTol[1] = fmax(fabs(b[1]), 1.0);
              cTol[2] = fmax(fabs(b[2]), 1.0);
              cTol[3] = fmax(fabs(b[3]), 1.0);
              cTolComputed = false;
            }

            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
}

/* Model step function */
real_T ModelPredictiveController_step(real_T *arg_ref, real_T arg_x, real_T
  arg_dx, real_T arg_theta, real_T arg_dtheta)
{
  real_T rtb_xest[7];
  real_T xk[7];
  real_T a__1[4];
  real_T tmp[4];
  real_T y_innov[4];
  real_T f[3];
  real_T zopt[3];
  real_T a;
  real_T y_innov_0;
  real_T y_innov_1;
  real_T y_innov_2;
  int32_T i;
  int32_T i_0;
  static const int8_T a_0[28] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
    0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0 };

  static const real_T b_a[28] = { 3.8216564810691809E-5, 7.3045701397469064E-8,
    -2.9188075776671982E-9, -1.5700282421903682E-8, 5.8322643193407636E-9,
    -2.847761185841298E-9, -1.3382585580074114E-7, -6.0780154379213165E-8,
    3.4164723109823269E-7, 1.2272160771363625E-8, 1.8822780722139534E-6,
    3.1412222675992588E-9, -2.4739274326428087E-8, 0.0099501232900803885,
    2.9134567541383008E-9, 1.8723191914699987E-8, 5.5342108305959936E-6,
    5.5323354314012817E-6, 0.00995009579928689, -5.530277040458033E-6,
    -3.309800762807951E-9, -1.8548043484901437E-8, 1.8514733020387006E-6,
    -5.5274332998538079E-6, 0.00028067620111692058, 5.5294916868287928E-6,
    0.0099487200083071253, 6.0654993696662708E-9 };

  static const real_T b_Linv[9] = { 0.24389033721716669, -0.6560687205146627,
    0.0, 0.0, 0.706574699803851, 0.0, 0.0, 0.0, 0.003162277660168379 };

  static const real_T b_Hinv[9] = { 0.48990866262564986, -0.46356155924834441,
    0.0, -0.46356155924834441, 0.49924780640290217, 0.0, 0.0, 0.0,
    9.9999999999999974E-6 };

  static const real_T b_Ac[12] = { -1.0, -1.0, 1.0, 1.0, -0.0, -1.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0 };

  static const real_T b_Kx[14] = { 13.473105993596526, -4125.9487292592385,
    -7401.1970606133791, 974.03620630880937, 163.89619993655441,
    -36.318562194360069, 0.31389594809949184, 13.161119720347298,
    -4073.3749549759868, -7717.0852014450811, 880.46521407266732,
    165.90041402888835, -64.296070164508421, 0.31004100993127065 };

  static const int8_T c_a[4] = { -1, -1, 1, 1 };

  static const real_T f_a[28] = { 3.8217275393145913E-5, 6.9107386616759161E-8,
    -3.1678585191041653E-9, -3.3906543487176324E-8, 5.8322643195477573E-9,
    -2.84776118378547E-9, -1.3382585580409253E-7, -5.7457142221757482E-8,
    3.2311713389477969E-7, 3.0582364013038774E-8, 1.7790753281173934E-6,
    3.1412223216929815E-9, -2.4739274321047343E-8, 0.0099501232900803763,
    3.0094997082367956E-9, 6.3120580545315866E-10, 5.5754661067069624E-6,
    2.7160759904518629E-6, 0.0099500957992868788, -5.5302770404740131E-6,
    -3.3098007588928381E-9, -4.67117189575776E-10, 1.7640386637463027E-6,
    -2.7115829166252593E-6, 0.00028226029419759386, 5.5294916868493249E-6,
    0.0099487200083071357, 6.0654993598254047E-9 };

  static const real_T e_a[7] = { 1.0426118049872042E-6, 0.0002066065832933287,
    5.210912774339763E-6, 0.001032176856249029, 0.0, 0.0, 0.0 };

  static const real_T d_a[49] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0097272854612053088, 0.94595819958562755, -0.001363011302150832,
    -0.26998508357572637, 0.0, 0.0, 0.0, -1.5502774437351163E-5,
    -0.0030707873408820164, 0.99747102034417734, -0.50544032027220132, 0.0, 0.0,
    0.0, -5.1923137608766757E-8, -1.5502774437351163E-5, 0.0099915674529621017,
    0.99747102034417789, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  /* specified return value */
  real_T arg_out;
  UNUSED_PARAMETER(arg_ref);

  /* MATLAB Function: '<S22>/optimizer' incorporates:
   *  Memory: '<S2>/last_x'
   */
  for (i = 0; i < 7; i++) {
    xk[i] = rtDW.last_x_PreviousInput[i];
  }

  /* SignalConversion generated from: '<S23>/ SFunction ' incorporates:
   *  Inport: '<Root>/dtheta'
   *  Inport: '<Root>/dx'
   *  Inport: '<Root>/theta'
   *  Inport: '<Root>/x'
   *  MATLAB Function: '<S22>/optimizer'
   */
  tmp[0] = arg_x;
  tmp[1] = arg_dx;
  tmp[2] = arg_theta;
  tmp[3] = arg_dtheta;

  /* MATLAB Function: '<S22>/optimizer' incorporates:
   *  UnitDelay: '<S2>/last_mv'
   */
  for (i_0 = 0; i_0 < 4; i_0++) {
    a = 0.0;
    for (i = 0; i < 7; i++) {
      a += (real_T)a_0[(i << 2) + i_0] * xk[i];
    }

    y_innov[i_0] = tmp[i_0] - a;
  }

  a = y_innov[1];
  y_innov_0 = y_innov[0];
  y_innov_1 = y_innov[2];
  y_innov_2 = y_innov[3];
  for (i_0 = 0; i_0 < 7; i_0++) {
    rtb_xest[i_0] = (((b_a[i_0 + 7] * a + b_a[i_0] * y_innov_0) + b_a[i_0 + 14] *
                      y_innov_1) + b_a[i_0 + 21] * y_innov_2) + xk[i_0];
  }

  f[0] = 0.0;
  f[1] = 0.0;
  f[2] = 0.0;
  for (i = 0; i < 2; i++) {
    a = 0.0;
    for (i_0 = 0; i_0 < 7; i_0++) {
      a += b_Kx[7 * i + i_0] * rtb_xest[i_0];
    }

    f[i] = (-0.20169851717922249 * (real_T)i + 15.811668261472512) *
      rtDW.last_mv_DSTATE + a;
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    a = 0.0;
    for (i = 0; i < 7; i++) {
      a += 0.0 * rtb_xest[i];
    }

    tmp[i_0] = -((a + 1.0) + (real_T)c_a[i_0] * rtDW.last_mv_DSTATE);
  }

  /* Update for Memory: '<S2>/Memory' incorporates:
   *  MATLAB Function: '<S22>/optimizer'
   */
  qpkwik(b_Linv, b_Hinv, f, b_Ac, tmp, rtDW.Memory_PreviousInput, 120, 1.0E-6,
         zopt, a__1, &i);

  /* MATLAB Function: '<S22>/optimizer' incorporates:
   *  UnitDelay: '<S2>/last_mv'
   */
  if ((i < 0) || (i == 0)) {
    zopt[0] = 0.0;
  }

  y_innov_0 = rtDW.last_mv_DSTATE + zopt[0];

  /* Outport: '<Root>/out' incorporates:
   *  Gain: '<Root>/Gain'
   *  MATLAB Function: '<S22>/optimizer'
   */
  arg_out = -y_innov_0;
  for (i_0 = 0; i_0 < 7; i_0++) {
    /* MATLAB Function: '<S22>/optimizer' */
    a = 0.0;
    for (i = 0; i < 7; i++) {
      a += d_a[7 * i + i_0] * xk[i];
    }

    /* Update for Memory: '<S2>/last_x' incorporates:
     *  MATLAB Function: '<S22>/optimizer'
     */
    rtDW.last_x_PreviousInput[i_0] = (((f_a[i_0 + 7] * y_innov[1] + f_a[i_0] *
      y_innov[0]) + f_a[i_0 + 14] * y_innov[2]) + f_a[i_0 + 21] * y_innov[3]) +
      (e_a[i_0] * y_innov_0 + a);
  }

  /* Update for UnitDelay: '<S2>/last_mv' incorporates:
   *  MATLAB Function: '<S22>/optimizer'
   */
  rtDW.last_mv_DSTATE = y_innov_0;
  return arg_out;
}

/* Model initialize function */
void ModelPredictiveController_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
