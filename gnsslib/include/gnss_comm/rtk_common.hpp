/**
* This file is part of gnss_comm.
*
* Copyright (C) 2021 Aerial Robotics Group, Hong Kong University of Science and Technology
* Author: CAO Shaozu (shaozu.cao@gmail.com)
*
* gnss_comm is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* gnss_comm is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with gnss_comm. If not, see <http://www.gnu.org/licenses/>.
*
* As many of the utility functions are adapted from RTKLIB, 
* the license for those part of code is claimed as follows:
* 
* The RTKLIB software package is distributed under the following BSD 2-clause
* license (http://opensource.org/licenses/BSD-2-Clause) and additional two
* exclusive clauses. Users are permitted to develop, produce or sell their own
* non-commercial or commercial products utilizing, linking or including RTKLIB as
* long as they comply with the license.
* 
*         Copyright (c) 2007-2020, T. Takasu, All rights reserved.
*/

#ifndef GNSS_RTK_COMMON_HPP_
#define GNSS_RTK_COMMON_HPP_

#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <cstdint>
#include "gnss_constant.hpp"

// RTKLIB export macro (same as in rtklib.h)
#ifndef EXPORT
#ifdef WIN_DLL
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif
#endif

namespace gnss_comm
{
    // Forward declarations for RTKLIB types (only for those not defined in gnss_constant.hpp)
    struct rtksvr_t;
    struct obsd_t;
    // gtime_t is defined in gnss_constant.hpp
    
    // Matrix operations (compatible with RTKLIB column-major order)
    // Note: RTKLIB uses column-major (Fortran) order, Eigen defaults to row-major.
    // We will store matrices in column-major to match RTKLIB.
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> Matrix;

    /* multiply matrix (wrapper of Eigen) -------------------------------------
    * multiply matrix by matrix (C=alpha*A*B+beta*C)
    * args   : char   *tr       I  transpose flags ("N":normal,"T":transpose)
    *          int    n,k,m     I  size of (transposed) matrix A,B
    *          double alpha     I  alpha
    *          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
    *          double beta      I  beta
    *          double *C        IO matrix C (n x k)
    * return : none
    *-----------------------------------------------------------------------------*/
    void matmul(const char *tr, int n, int k, int m, double alpha,
                const double *A, const double *B, double beta, double *C);

    /* inverse of matrix -----------------------------------------------------------
    * inverse of matrix (A=A^-1)
    * args   : double *A        IO  matrix (n x n)
    *          int    n         I   size of matrix A
    * return : status (0:ok,0>:error)
    *-----------------------------------------------------------------------------*/
    int matinv(double *A, int n);

    /* solve linear equation -------------------------------------------------------
    * solve linear equation (X=A\Y or X=A'\Y)
    * args   : char   *tr       I   transpose flag ("N":normal,"T":transpose)
    *          double *A        I   input matrix A (n x n)
    *          double *Y        I   input matrix Y (n x m)
    *          int    n,m       I   size of matrix A,Y
    *          double *X        O   X=A\Y or X=A'\Y (n x m)
    * return : status (0:ok,0>:error)
    * notes  : matirix stored by column-major order (fortran convention)
    *          X can be same as Y
    *-----------------------------------------------------------------------------*/
    int solve(const char *tr, const double *A, const double *Y, int n,
              int m, double *X);

    /* least square estimation -----------------------------------------------------
    * least square estimation by solving normal equation (x=(A*A')^-1*A*y)
    * args   : double *A        I   transpose of (weighted) design matrix (n x m)
    *          double *y        I   (weighted) measurements (m x 1)
    *          int    n,m       I   number of parameters and measurements (n<=m)
    *          double *x        O   estmated parameters (n x 1)
    *          double *Q        O   esimated parameters covariance matrix (n x n)
    * return : status (0:ok,0>:error)
    * notes  : for weighted least square, replace A and y by A*w and w*y (w=W^(1/2))
    *          matirix stored by column-major order (fortran convention)
    *-----------------------------------------------------------------------------*/
    int lsq(const double *A, const double *y, int n, int m, double *x,
            double *Q);

    /* kalman filter ---------------------------------------------------------------
    * kalman filter state update as follows:
    *
    *   K=P*H*(H'*P*H+R)^-1, xp=x+K*v, Pp=(I-K*H')*P
    *
    * args   : double *x        I   states vector (n x 1)
    *          double *P        I   covariance matrix of states (n x n)
    *          double *H        I   transpose of design matrix (n x m)
    *          double *v        I   innovation (measurement - model) (m x 1)
    *          double *R        I   covariance matrix of measurement error (m x m)
    *          int    n,m       I   number of states and measurements
    *          double *xp       O   states vector after update (n x 1)
    *          double *Pp       O   covariance matrix of states after update (n x n)
    * return : status (0:ok,<0:error)
    * notes  : matirix stored by column-major order (fortran convention)
    *          if state x[i]==0.0, not updates state x[i]/P[i+i*n]
    *-----------------------------------------------------------------------------*/
    int filter_(const double *x, const double *P, const double *H,
                const double *v, const double *R, int n, int m,
                double *xp, double *Pp);

    int filter(double *x, double *P, const double *H, const double *v,
               const double *R, int n, int m);

    /* smoother --------------------------------------------------------------------
    * combine forward and backward filters by fixed-interval smoother as follows:
    *
    *   xs=Qs*(Qf^-1*xf+Qb^-1*xb), Qs=(Qf^-1+Qb^-1)^-1)
    *
    * args   : double *xf       I   forward solutions (n x 1)
    * args   : double *Qf       I   forward solutions covariance matrix (n x n)
    *          double *xb       I   backward solutions (n x 1)
    *          double *Qb       I   backward solutions covariance matrix (n x n)
    *          int    n         I   number of solutions
    *          double *xs       O   smoothed solutions (n x 1)
    *          double *Qs       O   smoothed solutions covariance matrix (n x n)
    * return : status (0:ok,0>:error)
    * notes  : see reference [4] 5.2
    *          matirix stored by column-major order (fortran convention)
    *-----------------------------------------------------------------------------*/
    int smoother(const double *xf, const double *Qf, const double *xb,
                 const double *Qb, int n, double *xs, double *Qs);

    /* geometric distance ----------------------------------------------------------
    * compute geometric distance and receiver-to-satellite unit vector
    * args   : double *rs       I   satellilte position (ecef at transmission) (m)
    *          double *rr       I   receiver position (ecef at reception) (m)
    *          double *e        O   line-of-sight vector (ecef)
    * return : geometric distance (m) (0>:error/no satellite position)
    * notes  : distance includes sagnac effect correction
    *-----------------------------------------------------------------------------*/
    double geodist(const double *rs, const double *rr, double *e);

    /* satellite azimuth/elevation angle -------------------------------------------
    * compute satellite azimuth/elevation angle
    * args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
    *          double *e        I   receiver-to-satellilte unit vevtor (ecef)
    *          double *azel     IO  azimuth/elevation {az,el} (rad)
    *                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
    * return : elevation angle (rad)
    *-----------------------------------------------------------------------------*/
    double satazel(const double *pos, const double *e, double *azel);

    /* troposphere model -----------------------------------------------------------
    * compute tropospheric delay by standard atmosphere and saastamoinen model
    * args   : gtime_t time     I   time
    *          double *pos      I   receiver position {lat,lon,h} (rad,m)
    *          double *azel     I   azimuth/elevation angle {az,el} (rad)
    *          double humi      I   relative humidity
    * return : tropospheric delay (m)
    *-----------------------------------------------------------------------------*/
    double tropmodel(gtime_t time, const double *pos, const double *azel,
                     double humi);

    /* troposphere mapping function ------------------------------------------------
    * compute tropospheric mapping function by NMF
    * args   : gtime_t t        I   time
    *          double *pos      I   receiver position {lat,lon,h} (rad,m)
    *          double *azel     I   azimuth/elevation angle {az,el} (rad)
    *          double *mapfw    IO  wet mapping function (NULL: not output)
    * return : dry mapping function
    * note   : see ref [5] (NMF) and [9] (GMF)
    *          original JGR paper of [5] has bugs in eq.(4) and (5). the corrected
    *          paper is obtained from:
    *          ftp://web.haystack.edu/pub/aen/nmf/NMF_JGR.pdf
    *-----------------------------------------------------------------------------*/
    double tropmapf(gtime_t time, const double pos[], const double azel[],
                    double *mapfw);

    /* ionosphere model ------------------------------------------------------------
    * compute ionospheric delay by broadcast ionosphere model (klobuchar model)
    * args   : gtime_t t        I   time (gpst)
    *          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
    *          double *pos      I   receiver position {lat,lon,h} (rad,m)
    *          double *azel     I   azimuth/elevation angle {az,el} (rad)
    * return : ionospheric delay (L1) (m)
    *-----------------------------------------------------------------------------*/
    double ionmodel(gtime_t t, const double *ion, const double *pos,
                    const double *azel);

    /* antenna phase center correction ---------------------------------------------
    * compute antenna offset by antenna phase center parameters
    * args   : pcv_t *pcv       I   antenna phase center parameters
    *          double *del      I   antenna delta {e,n,u} (m)
    *          double *azel     I   azimuth/elevation for receiver {az,el} (rad)
    *          int     opt      I   option (0:only offset,1:offset+pcv)
    *          double *dant     O   range offsets for each frequency (m)
    * return : none
    * notes  : current version does not support azimuth dependent terms
    *-----------------------------------------------------------------------------*/
    void antmodel(const pcv_t *pcv, const double *del, const double *azel,
                  int opt, double *dant);

    /* satellite antenna model ------------------------------------------------------
    * compute satellite antenna phase center parameters
    * args   : pcv_t *pcv       I   antenna phase center parameters
    *          double nadir     I   nadir angle for satellite (rad)
    *          double *dant     O   range offsets for each frequency (m)
    * return : none
    *-----------------------------------------------------------------------------*/
    void antmodel_s(const pcv_t *pcv, double nadir, double *dant);

    /* test excluded satellite -----------------------------------------------------
    * test excluded satellite
    * args   : int    sat       I   satellite number
    *          double var       I   variance of ephemeris (m^2)
    *          int    svh       I   sv health flag
    *          prcopt_t *opt    I   processing options (NULL: not used)
    * return : status (1:excluded,0:not excluded)
    *-----------------------------------------------------------------------------*/
    int satexclude(int sat, double var, int svh, const prcopt_t *opt);

    /* test SNR mask ---------------------------------------------------------------
    * test SNR mask
    * args   : int    base      I   rover or base-station (0:rover,1:base station)
    *          int    idx       I   frequency index (0:L1,1:L2,2:L3,...)
    *          double el        I   elevation angle (rad)
    *          double snr       I   C/N0 (dBHz)
    *          snrmask_t *mask  I   SNR mask
    * return : status (1:masked,0:unmasked)
    *-----------------------------------------------------------------------------*/
    int testsnr(int base, int idx, double el, double snr,
                const snrmask_t *mask);

    /* system and obs code to frequency --------------------------------------------
    * convert system and obs code to carrier frequency
    * args   : int    sys       I   satellite system (SYS_???)
    *          uint8_t code     I   obs code (CODE_???)
    *          int    fcn       I   frequency channel number for GLONASS
    * return : carrier frequency (Hz) (0.0: error)
    *-----------------------------------------------------------------------------*/
    double code2freq(int sys, uint8_t code, int fcn);

    /* satellite and obs code to frequency -----------------------------------------
    * convert satellite and obs code to carrier frequency
    * args   : int    sat       I   satellite number
    *          uint8_t code     I   obs code (CODE_???)
    *          nav_t  *nav_t    I   navigation data for GLONASS (NULL: not used)
    * return : carrier frequency (Hz) (0.0: error)
    *-----------------------------------------------------------------------------*/
    double sat2freq(int sat, uint8_t code, const nav_t *nav);

    /* system and obs code to frequency index --------------------------------------
    * convert system and obs code to frequency index
    * args   : int    sys       I   satellite system (SYS_???)
    *          uint8_t code     I   obs code (CODE_???)
    * return : frequency index (-1: error)
    *                       0     1     2     3     4 
    *           --------------------------------------
    *            GPS       L1    L2    L5     -     - 
    *            GLONASS   G1    G2    G3     -     -  (G1=G1,G1a,G2=G2,G2a)
    *            Galileo   E1    E5b   E5a   E6   E5ab
    *            QZSS      L1    L2    L5    L6     - 
    *            SBAS      L1     -    L5     -     -
    *            BDS       B1    B2    B2a   B3   B2ab (B1=B1I,B1C,B2=B2I,B2b)
    *            NavIC     L5     S     -     -     - 
    *-----------------------------------------------------------------------------*/
    int code2idx(int sys, uint8_t code);

    /* extract unsigned/signed bits ------------------------------------------------
    * extract unsigned/signed bits from byte data
    * args   : uint8_t *buff    I   byte data
    *          int    pos       I   bit position from start of data (bits)
    *          int    len       I   bit length (bits) (len<=32)
    * return : extracted unsigned/signed bits
    *-----------------------------------------------------------------------------*/
    uint32_t getbitu(const uint8_t *buff, int pos, int len);
    int32_t getbits(const uint8_t *buff, int pos, int len);

    /* set unsigned/signed bits ----------------------------------------------------
    * set unsigned/signed bits to byte data
    * args   : uint8_t *buff IO byte data
    *          int    pos       I   bit position from start of data (bits)
    *          int    len       I   bit length (bits) (len<=32)
    *          [u]int32_t data  I   unsigned/signed data
    * return : none
    *-----------------------------------------------------------------------------*/
    void setbitu(uint8_t *buff, int pos, int len, uint32_t data);
    void setbits(uint8_t *buff, int pos, int len, int32_t data);

    /* Standard positioning functions ---------------------------------------------*/
    EXPORT int  pntpos(const obsd_t *obs, int n, const nav_t *nav,
                       const prcopt_t *opt, sol_t *sol, double *azel, 
                       void *ssat, char *msg);
    EXPORT void satposs(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
                        int ephopt, double *rs, double *dts, double *var, int *svh);
    
    /* RTK control functions -------------------------------------------------------*/
    EXPORT void rtkinit(rtk_t *rtk, const prcopt_t *opt);
    EXPORT void rtkfree(rtk_t *rtk);
    EXPORT int  rtkpos (rtk_t *rtk, const obsd_t *obs, int nobs, const nav_t *nav);
    EXPORT int  rtkopenstat(const char *file, int level);
    EXPORT void rtkclosestat(void);
    EXPORT int  rtkoutstat(rtk_t *rtk, char *buff);
    EXPORT int  input_rnxctr(rnxctr_t *rnx, FILE *fp);
    EXPORT int  init_rnxctr(rnxctr_t *rnx);
    EXPORT void free_rnxctr(rnxctr_t *rnx);
    EXPORT int  open_rnxctr(rnxctr_t *rnx, FILE *fp);

    /* RTK server functions -------------------------------------------------------*/
    EXPORT int  rtksvrinit  (rtksvr_t *svr);
    EXPORT void rtksvrfree  (rtksvr_t *svr);
    EXPORT int  rtksvrstart (rtksvr_t *svr, int cycle, int buffsize, int *strs, char *paths[], int *formats, int n, char *rcvopts);
    EXPORT void rtksvrstop  (rtksvr_t *svr, char **cmds);
    EXPORT int  rtksvropenstr(rtksvr_t *svr, int index, int str, const char *path, int format, const char *rcvopt);
    EXPORT void rtksvrclosestr(rtksvr_t *svr, int index);
    EXPORT void rtksvrlock  (rtksvr_t *svr);
    EXPORT void rtksvrunlock(rtksvr_t *svr);
    EXPORT int  rtksvrostat (rtksvr_t *svr, int type, gtime_t *time, int *sat, int *vsat, double *azel, int *snr, int *ns);
    EXPORT void rtksvrsstat (rtksvr_t *svr, int *sstat, char *msg);
    EXPORT int  rtksvrmark(rtksvr_t *svr, const char *name, const char *comment);

}   // namespace gnss_comm

#endif
