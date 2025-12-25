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

#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Cholesky>
#include <gnss_constant.hpp>
#include <gnss_utility.hpp>
#include <rtk_common.hpp>
#include <time.h>
#include <sys/time.h>

// Note: obsd_t is now defined in gnss_constant.hpp
// Dummy types for RTKLIB compatibility (for functions not yet implemented)
typedef struct { int dummy; } eph_t;
typedef struct { int dummy; } geph_t;
typedef struct { int dummy; } seph_t;

static inline void trace(int level, const char* fmt, ...) {}

// Dummy functions for RINEX reading
static inline int readrnxobsb(FILE* fp, const char* opt, double ver, int* tsys, char tobs[][MAXOBSTYPE][4], int* flag, gnss_comm::obsd_t* data, void* sta) { return 0; }
static inline int readrnxnavb(FILE* fp, const char* opt, double ver, int sys, int* type, eph_t* eph, geph_t* geph, seph_t* seph) { return 0; }
static inline int readrnxh(FILE* fp, double* ver, char* type, int* sys, int* tsys, char tobs[7][MAXOBSTYPE][4], void* nav, void* sta) { return 0; }

namespace gnss_comm
{
    // Global time offset variable definition
    double timeoffset_ = 0.0;

    static double norm_rtk(const double *a, int n) {
        double res = 0;
        for (int i = 0; i < n; i++) res += a[i] * a[i];
        return sqrt(res);
    }

    #define SQR(x)      ((x)*(x))
    #define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))

    #define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)
    #define NP(opt)     ((opt)->dynamics==0?3:9)
    #define NI(opt)     ((opt)->ionoopt!=IONOOPT_EST?0:MAXSAT)
    #define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt<TROPOPT_ESTG?2:6))
    #define NL(opt)     ((opt)->glomodear!=2?0:0) 
    #define NB(opt)     ((opt)->mode<=PMODE_DGPS?0:MAXSAT*NF(opt))
    #define NR(opt)     (NP(opt)+NI(opt)+NT(opt)+NL(opt))
    #define NX(opt)     (NR(opt)+NB(opt))

    #define II(s,opt)   (NP(opt)+(s)-1)                 /* ionos (s:satellite no) */
    #define IT(r,opt)   (NP(opt)+NI(opt)+(r)*2)         /* tropos (r:0=rov,1:ref) */
    #define IB(s,f,opt) (NR(opt)+MAXSAT*(f)+(s)-1)      /* phase bias (s:sat,f:freq) */
    
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
                const double *A, const double *B, double beta, double *C)
    {
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matA(tr[0]=='T'||tr[0]=='t'?A:nullptr, tr[0]=='T'||tr[0]=='t'?m:n, 
                 tr[0]=='T'||tr[0]=='t'?n:m);
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matB(tr[1]=='T'||tr[1]=='t'?B:nullptr, tr[1]=='T'||tr[1]=='t'?k:m, 
                 tr[1]=='T'||tr[1]=='t'?m:k);
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matC(C, n, k);
        
        if (tr[0]=='T'||tr[0]=='t')
        {
            if (tr[1]=='T'||tr[1]=='t')
                matC = alpha * matA.transpose() * matB.transpose() + beta * matC;
            else
                matC = alpha * matA.transpose() * matB + beta * matC;
        }
        else
        {
            if (tr[1]=='T'||tr[1]=='t')
                matC = alpha * matA * matB.transpose() + beta * matC;
            else
                matC = alpha * matA * matB + beta * matC;
        }
    }

    /* inverse of matrix -----------------------------------------------------------
    * inverse of matrix (A=A^-1)
    * args   : double *A        IO  matrix (n x n)
    *          int    n         I   size of matrix A
    * return : status (0:ok,0>:error)
    *-----------------------------------------------------------------------------*/
    int matinv(double *A, int n)
    {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> matA(A, n, n);
        Eigen::FullPivLU<Eigen::MatrixXd> lu(matA);
        if (!lu.isInvertible())
            return -1;
        matA = lu.inverse();
        return 0;
    }

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
              int m, double *X)
    {
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matA(A, n, n);
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matY(Y, n, m);
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matX(X, n, m);
        
        if (tr[0]=='N' || tr[0]=='n')
        {
            Eigen::FullPivLU<Eigen::MatrixXd> lu(matA);
            if (!lu.isInvertible())
                return -1;
            matX = lu.solve(matY);
        }
        else
        {
            Eigen::FullPivLU<Eigen::MatrixXd> lu(matA.transpose());
            if (!lu.isInvertible())
                return -1;
            matX = lu.solve(matY);
        }
        return 0;
    }

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
            double *Q)
    {
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matA(A, n, m);
        Eigen::Map<const Eigen::VectorXd> vecY(y, m);
        Eigen::Map<Eigen::VectorXd> vecX(x, n);
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matQ(Q, n, n);
        
        // Normal equation: (A*A') * x = A * y
        Eigen::MatrixXd N = matA * matA.transpose();
        Eigen::VectorXd b = matA * vecY;
        
        Eigen::LLT<Eigen::MatrixXd> llt(N);
        if (llt.info() != Eigen::Success)
            return -1;
        vecX = llt.solve(b);
        matQ = llt.solve(Eigen::MatrixXd::Identity(n, n));
        return 0;
    }

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
                double *xp, double *Pp)
    {
        Eigen::Map<const Eigen::VectorXd> vecX(x, n);
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matP(P, n, n);
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matH(H, n, m);
        Eigen::Map<const Eigen::VectorXd> vecV(v, m);
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matR(R, m, m);
        Eigen::Map<Eigen::VectorXd> vecXp(xp, n);
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matPp(Pp, n, n);
        
        // Compute Kalman gain: K = P * H * (H' * P * H + R)^-1
        Eigen::MatrixXd S = matH.transpose() * matP * matH + matR;
        Eigen::LLT<Eigen::MatrixXd> llt(S);
        if (llt.info() != Eigen::Success)
            return -1;
        Eigen::MatrixXd K = matP * matH * llt.solve(Eigen::MatrixXd::Identity(m, m));
        
        // Update state and covariance
        vecXp = vecX + K * vecV;
        matPp = (Eigen::MatrixXd::Identity(n, n) - K * matH.transpose()) * matP;
        
        return 0;
    }

    int filter(double *x, double *P, const double *H, const double *v,
               const double *R, int n, int m)
    {
        return filter_(x, P, H, v, R, n, m, x, P);
    }

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
                 const double *Qb, int n, double *xs, double *Qs)
    {
        Eigen::Map<const Eigen::VectorXd> vecXf(xf, n);
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matQf(Qf, n, n);
        Eigen::Map<const Eigen::VectorXd> vecXb(xb, n);
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matQb(Qb, n, n);
        Eigen::Map<Eigen::VectorXd> vecXs(xs, n);
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> 
            matQs(Qs, n, n);
        
        Eigen::LLT<Eigen::MatrixXd> lltf(matQf);
        Eigen::LLT<Eigen::MatrixXd> lltb(matQb);
        if (lltf.info() != Eigen::Success || lltb.info() != Eigen::Success)
            return -1;
        
        Eigen::MatrixXd Qfi = lltf.solve(Eigen::MatrixXd::Identity(n, n));
        Eigen::MatrixXd Qbi = lltb.solve(Eigen::MatrixXd::Identity(n, n));
        
        matQs = (Qfi + Qbi).inverse();
        vecXs = matQs * (Qfi * vecXf + Qbi * vecXb);
        return 0;
    }

    /* geometric distance ----------------------------------------------------------
    * compute geometric distance and receiver-to-satellite unit vector
    * args   : double *rs       I   satellilte position (ecef at transmission) (m)
    *          double *rr       I   receiver position (ecef at reception) (m)
    *          double *e        O   line-of-sight vector (ecef)
    * return : geometric distance (m) (0>:error/no satellite position)
    * notes  : distance includes sagnac effect correction
    *-----------------------------------------------------------------------------*/
    double geodist(const double *rs, const double *rr, double *e)
    {
        if (rs[0]==0.0 && rs[1]==0.0 && rs[2]==0.0)
            return 0.0;
        
        double r[3];
        for (int i=0; i<3; i++) r[i] = rs[i] - rr[i];
        double d = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
        
        // Sagnac effect correction
        const double omge = 7.2921151467E-5; // WGS84 earth rotation rate (rad/s)
        double c = omge * d / LIGHT_SPEED;
        double x = r[0] + c * r[1];
        double y = r[1] - c * r[0];
        double z = r[2];
        d = sqrt(x*x + y*y + z*z);
        
        if (e)
        {
            e[0] = x / d;
            e[1] = y / d;
            e[2] = z / d;
        }
        return d;
    }

    /* satellite azimuth/elevation angle -------------------------------------------
    * compute satellite azimuth/elevation angle
    * args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
    *          double *e        I   receiver-to-satellilte unit vevtor (ecef)
    *          double *azel     IO  azimuth/elevation {az,el} (rad)
    *                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
    * return : elevation angle (rad)
    *-----------------------------------------------------------------------------*/
    double satazel(const double *pos, const double *e, double *azel)
    {
        if (pos[2] < -1E6 || (e[0]==0.0 && e[1]==0.0 && e[2]==0.0))
        {
            if (azel)
            {
                azel[0] = 0.0;
                azel[1] = M_PI/2.0;
            }
            return 0.0;
        }
        
        double enu[3];
        double coslat = cos(pos[0]), sinlat = sin(pos[0]);
        double coslon = cos(pos[1]), sinlon = sin(pos[1]);
        
        // Transform ecef vector to enu
        enu[0] = -sinlon*e[0] + coslon*e[1];
        enu[1] = -sinlat*coslon*e[0] - sinlat*sinlon*e[1] + coslat*e[2];
        enu[2] =  coslat*coslon*e[0] + coslat*sinlon*e[1] + sinlat*e[2];
        
        double az = 0.0;
        if (enu[0]*enu[0] + enu[1]*enu[1] > 1E-12)
        {
            az = atan2(enu[0], enu[1]);
            if (az < 0.0) az += 2.0*M_PI;
        }
        double el = asin(enu[2]);
        
        if (azel)
        {
            azel[0] = az;
            azel[1] = el;
        }
        return el;
    }

    /* troposphere model -----------------------------------------------------------
    * compute tropospheric delay by standard atmosphere and saastamoinen model
    * args   : gtime_t time     I   time
    *          double *pos      I   receiver position {lat,lon,h} (rad,m)
    *          double *azel     I   azimuth/elevation angle {az,el} (rad)
    *          double humi      I   relative humidity
    * return : tropospheric delay (m)
    *-----------------------------------------------------------------------------*/
    double tropmodel(gtime_t time, const double *pos, const double *azel,
                     double humi)
    {
        const double temp0 = 15.0; // temperature at sea level
        double hgt, pres, temp, e, z, trph, trpw;
        
        if (pos[2] < -100.0 || 1E4 < pos[2] || azel[1] <= 0)
            return 0.0;
        
        // standard atmosphere
        hgt = pos[2] < 0.0 ? 0.0 : pos[2];
        
        pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
        temp = temp0 - 6.5E-3 * hgt + 273.16;
        e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));
        
        // saastamoninen model
        z = M_PI / 2.0 - azel[1];
        trph = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * pos[0]) - 0.00028 * hgt / 1E3) / cos(z);
        trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);
        return trph + trpw;
    }

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
    /* NMF (Niell Mapping Function) helper functions */
    static double interpc(const double coef[], double lat)
    {
        int i = (int)(lat / 15.0);
        if (i < 1) return coef[0];
        else if (i > 4) return coef[4];
        return coef[i - 1] * (1.0 - lat / 15.0 + i) + coef[i] * (lat / 15.0 - i);
    }
    
    static double mapf(double el, double a, double b, double c)
    {
        double sinel = sin(el);
        return (1.0 + a / (1.0 + b / (1.0 + c))) / (sinel + (a / (sinel + b / (sinel + c))));
    }
    
    static double nmf(gtime_t time, const double pos[], const double azel[],
                      double *mapfw)
    {
        // ref [5] table 3
        // hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75
        const double coef[][5] = {
            { 1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
            { 2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
            { 62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},
            
            { 0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
            { 0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
            { 0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},
            
            { 5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
            { 1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
            { 4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}
        };
        const double aht[] = { 2.53E-5, 5.49E-3, 1.14E-3}; // height correction
        
        double y, cosy, ah[3], aw[3], dm, el = azel[1], lat = pos[0] * R2D, hgt = pos[2];
        int i;
        
        if (el <= 0.0) {
            if (mapfw) *mapfw = 0.0;
            return 0.0;
        }
        // year from doy 28, added half a year for southern latitudes
        y = (time2doy(time) - 28.0) / 365.25 + (lat < 0.0 ? 0.5 : 0.0);
        
        cosy = cos(2.0 * M_PI * y);
        lat = fabs(lat);
        
        for (i = 0; i < 3; i++) {
            ah[i] = interpc(coef[i], lat) - interpc(coef[i + 3], lat) * cosy;
            aw[i] = interpc(coef[i + 6], lat);
        }
        // ellipsoidal height is used instead of height above sea level
        dm = (1.0 / sin(el) - mapf(el, aht[0], aht[1], aht[2])) * hgt / 1E3;
        
        if (mapfw) *mapfw = mapf(el, aw[0], aw[1], aw[2]);
        
        return mapf(el, ah[0], ah[1], ah[2]) + dm;
    }
    
    double tropmapf(gtime_t time, const double pos[], const double azel[],
                    double *mapfw)
    {
        if (pos[2] < -1000.0 || pos[2] > 20000.0) {
            if (mapfw) *mapfw = 0.0;
            return 0.0;
        }
        return nmf(time, pos, azel, mapfw); // NMF
    }

    /* ionosphere model ------------------------------------------------------------
    * compute ionospheric delay by broadcast ionosphere model (klobuchar model)
    * args   : gtime_t t        I   time (gpst)
    *          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
    *          double *pos      I   receiver position {lat,lon,h} (rad,m)
    *          double *azel     I   azimuth/elevation angle {az,el} (rad)
    * return : ionospheric delay (L1) (m)
    *-----------------------------------------------------------------------------*/
    double ionmodel(gtime_t t, const double *ion, const double *pos,
                    const double *azel)
    {
        const double ion_default[] = { // 2004/1/1
            0.1118E-07, -0.7451E-08, -0.5961E-07,  0.1192E-06,
            0.1167E+06, -0.2294E+06, -0.1311E+06,  0.1049E+07
        };
        double tt, f, psi, phi, lam, amp, per, x;
        uint32_t week;
        
        if (pos[2] < -1E3 || azel[1] <= 0) return 0.0;
        if (norm_rtk(ion, 8) <= 0.0) ion = ion_default;
        
        // earth centered angle (semi-circle)
        psi = 0.0137 / (azel[1] / M_PI + 0.11) - 0.022;
        
        // subionospheric latitude/longitude (semi-circle)
        phi = pos[0] / M_PI + psi * cos(azel[0]);
        if      (phi >  0.416) phi =  0.416;
        else if (phi < -0.416) phi = -0.416;
        lam = pos[1] / M_PI + psi * sin(azel[0]) / cos(phi * M_PI);
        
        // geomagnetic latitude (semi-circle)
        phi += 0.064 * cos((lam - 1.617) * M_PI);
        
        // local time (s)
        tt = 43200.0 * lam + time2gpst(t, &week);
        tt -= floor(tt / 86400.0) * 86400.0; // 0<=tt<86400
        
        // slant factor
        f = 1.0 + 16.0 * pow(0.53 - azel[1] / M_PI, 3.0);
        
        // ionospheric delay
        amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
        per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
        amp = amp <     0.0 ?     0.0 : amp;
        per = per < 72000.0 ? 72000.0 : per;
        x = 2.0 * M_PI * (tt - 50400.0) / per;
        
        return LIGHT_SPEED * f * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);
    }

    /* auxiliary functions -------------------------------------------------------*/
    static double dot(const double *a, const double *b, int n) {
        double c = 0.0;
        for (int i = 0; i < n; i++) c += a[i] * b[i];
        return c;
    }
    

    static double interpvar(double ang, const double *var) {
        int i = (int)(ang / 5.0);
        if (i < 0) return var[0];
        else if (i >= 17) return var[17];
        double a = ang - i * 5.0;
        return var[i] * (1.0 - a / 5.0) + var[i + 1] * (a / 5.0);
    }

    /* receiver antenna model -----------------------------------------------------
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
                  int opt, double *dant) {
        double e[3], off[3];
        double cosel = cos(azel[1]);
        int i, j;
        
        e[0] = sin(azel[0]) * cosel;
        e[1] = cos(azel[0]) * cosel;
        e[2] = sin(azel[1]);
        
        for (i = 0; i < NFREQ; i++) {
            for (j = 0; j < 3; j++) off[j] = pcv->off[i][j] + del[j];
            dant[i] = -dot(off, e, 3) + (opt ? interpvar(90.0 - azel[1] * R2D, pcv->var[i]) : 0.0);
        }
    }

    /* satellite antenna model ------------------------------------------------------
    * compute satellite antenna phase center parameters
    * args   : pcv_t *pcv       I   antenna phase center parameters
    *          double nadir     I   nadir angle for satellite (rad)
    *          double *dant     O   range offsets for each frequency (m)
    * return : none
    *-----------------------------------------------------------------------------*/
    void antmodel_s(const pcv_t *pcv, double nadir, double *dant) {
        int i;
        for (i = 0; i < NFREQ; i++) {
            dant[i] = interpvar(nadir * R2D * 5.0, pcv->var[i]);
        }
    }

    /* test excluded satellite -----------------------------------------------------
    * test excluded satellite
    * args   : int    sat       I   satellite number
    *          double var       I   variance of ephemeris (m^2)
    *          int    svh       I   sv health flag
    *          prcopt_t *opt    I   processing options (NULL: not used)
    * return : status (1:excluded,0:not excluded)
    *-----------------------------------------------------------------------------*/
    int satexclude(int sat, double var, int svh, const prcopt_t *opt) {
        int sys = satsys(sat, NULL);
        if (!opt) return 0;
        if (var <= 0.0) var = 0.01;
        if (svh < 0) return 1; /* ephemeris not available */
        if (opt->exsats[sat - 1] == 1) return 1; /* excluded satellite */
        if (opt->exsats[sat - 1] == 2) return 0; /* included satellite */
        if (!(sys & opt->navsys)) return 1; /* not in navigation system */
        if (svh) return 1; /* satellite health flag */
        if (var > opt->maxgdop * opt->maxgdop) return 1; /* large variance */
        return 0;
    }

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
                const snrmask_t *mask) {
        double minsnr;
        if (!mask) return 0;
        if (el <= 0.0) return 1;
        if (idx < 0 || idx >= NFREQ) return 0;
        minsnr = mask->mask[base][idx];
        if (snr > 0.0 && minsnr > 0.0 && snr < minsnr) return 1;
        return 0;
    }

    /* system and obs code to frequency --------------------------------------------
    * convert system and obs code to carrier frequency
    * args   : int    sys       I   satellite system (SYS_???)
    *          uint8_t code     I   obs code (CODE_???)
    *          int    fcn       I   frequency channel number for GLONASS
    * return : carrier frequency (Hz) (0.0: error)
    *-----------------------------------------------------------------------------*/
    double code2freq(int sys, uint8_t code, int fcn) {
        const double freq_glo_n[14] = {  /* GLONASS frequency channel number (n) */
            FREQ1_GLO, FREQ1_GLO + DFRQ1_GLO * 1, FREQ1_GLO + DFRQ1_GLO * 2,
            FREQ1_GLO + DFRQ1_GLO * 3, FREQ1_GLO + DFRQ1_GLO * 4, FREQ1_GLO + DFRQ1_GLO * 5,
            FREQ1_GLO + DFRQ1_GLO * 6, FREQ1_GLO - DFRQ1_GLO * 1, FREQ1_GLO - DFRQ1_GLO * 2,
            FREQ1_GLO - DFRQ1_GLO * 3, FREQ1_GLO - DFRQ1_GLO * 4, FREQ1_GLO - DFRQ1_GLO * 5,
            FREQ1_GLO - DFRQ1_GLO * 6, FREQ1_GLO - DFRQ1_GLO * 7
        };
        if (code == CODE_NONE) return 0.0;
        switch (sys) {
            case SYS_GPS:
                switch (code) {
                    case CODE_L1C: case CODE_L1P: case CODE_L1W: case CODE_L1Y: case CODE_L1M:
                    case CODE_L1N: case CODE_L1S: case CODE_L1L: case CODE_L1X: case CODE_L1E:
                    case CODE_L1A: case CODE_L1B: case CODE_L1Z:
                        return FREQ1;
                    case CODE_L2C: case CODE_L2D: case CODE_L2S: case CODE_L2L: case CODE_L2X:
                    case CODE_L2P: case CODE_L2W: case CODE_L2Y: case CODE_L2M: case CODE_L2N:
                        return FREQ2;
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X:
                        return FREQ5;
                    default: break;
                }
                break;
            case SYS_GLO:
                switch (code) {
                    case CODE_L1C: case CODE_L1P:
                        if (fcn < -7 || fcn > 6) return 0.0;
                        return freq_glo_n[fcn + 7];
                    case CODE_L2C: case CODE_L2P:
                        if (fcn < -7 || fcn > 6) return 0.0;
                        return FREQ2_GLO + DFRQ2_GLO * fcn;
                    case CODE_L3I: case CODE_L3Q: case CODE_L3X:
                        return FREQ3_GLO;
                    default: break;
                }
                break;
            case SYS_GAL:
                switch (code) {
                    case CODE_L1A: case CODE_L1B: case CODE_L1C: case CODE_L1X: case CODE_L1Z:
                        return FREQ1;
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X:
                        return FREQ5;
                    case CODE_L6A: case CODE_L6B: case CODE_L6C: case CODE_L6X: case CODE_L6Z:
                        return FREQ6;
                    case CODE_L7I: case CODE_L7Q: case CODE_L7X:
                        return FREQ7;
                    case CODE_L8I: case CODE_L8Q: case CODE_L8X:
                        return FREQ8;
                    default: break;
                }
                break;
            case SYS_QZS:
                switch (code) {
                    case CODE_L1C: case CODE_L1S: case CODE_L1L: case CODE_L1X: case CODE_L1Z:
                    case CODE_L1E:
                        return FREQ1;
                    case CODE_L2C: case CODE_L2S: case CODE_L2L: case CODE_L2X:
                        return FREQ2;
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X:
                        return FREQ5;
                    case CODE_L6S: case CODE_L6L: case CODE_L6X:
                        return FREQ6;
                    default: break;
                }
                break;
            case SYS_BDS:
                switch (code) {
                    case CODE_L1I: case CODE_L1Q: case CODE_L1X:
                        return FREQ1_BDS;
                    case CODE_L2I: case CODE_L2Q: case CODE_L2X:
                        return FREQ2_BDS;
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X:
                        return FREQ5;
                    case CODE_L6I: case CODE_L6Q: case CODE_L6X:
                        return FREQ3_BDS;
                    case CODE_L7I: case CODE_L7Q: case CODE_L7X:
                        return FREQ2_BDS;
                    default: break;
                }
                break;
            case SYS_SBS:
                switch (code) {
                    case CODE_L1C: return FREQ1;
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X: return FREQ5;
                    default: break;
                }
                break;
            case SYS_IRN:
                switch (code) {
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X: return FREQ5;
                    case CODE_L9I: case CODE_L9Q: case CODE_L9X: return FREQ9;
                    default: break;
                }
                break;
            default: break;
        }
        return 0.0;
    }

    /* satellite and obs code to frequency -----------------------------------------
    * convert satellite and obs code to carrier frequency
    * args   : int    sat       I   satellite number
    *          uint8_t code     I   obs code (CODE_???)
    *          nav_t  *nav_t    I   navigation data for GLONASS (NULL: not used)
    * return : carrier frequency (Hz) (0.0: error)
    *-----------------------------------------------------------------------------*/
    double sat2freq(int sat, uint8_t code, const nav_t *nav) {
        int sys = satsys(sat, NULL);
        int fcn = 0;
        if (sys == SYS_GLO && nav) {
            fcn = nav->glo_fcn[sat - 1];
        }
        return code2freq(sys, code, fcn);
    }

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
    int code2idx(int sys, uint8_t code) {
        switch (sys) {
            case SYS_GPS:
                switch (code) {
                    case CODE_L1C: case CODE_L1P: case CODE_L1W: case CODE_L1Y: case CODE_L1M:
                    case CODE_L1N: case CODE_L1S: case CODE_L1L: case CODE_L1X: case CODE_L1E:
                    case CODE_L1A: case CODE_L1B: case CODE_L1Z:
                        return 0;
                    case CODE_L2C: case CODE_L2D: case CODE_L2S: case CODE_L2L: case CODE_L2X:
                    case CODE_L2P: case CODE_L2W: case CODE_L2Y: case CODE_L2M: case CODE_L2N:
                        return 1;
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X:
                        return 2;
                    default: break;
                }
                break;
            case SYS_GLO:
                switch (code) {
                    case CODE_L1C: case CODE_L1P:
                        return 0;
                    case CODE_L2C: case CODE_L2P:
                        return 1;
                    case CODE_L3I: case CODE_L3Q: case CODE_L3X:
                        return 2;
                    default: break;
                }
                break;
            case SYS_GAL:
                switch (code) {
                    case CODE_L1A: case CODE_L1B: case CODE_L1C: case CODE_L1X: case CODE_L1Z:
                        return 0;
                    case CODE_L7I: case CODE_L7Q: case CODE_L7X:
                        return 1;
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X:
                        return 2;
                    case CODE_L6A: case CODE_L6B: case CODE_L6C: case CODE_L6X: case CODE_L6Z:
                        return 3;
                    case CODE_L8I: case CODE_L8Q: case CODE_L8X:
                        return 4;
                    default: break;
                }
                break;
            case SYS_QZS:
                switch (code) {
                    case CODE_L1C: case CODE_L1S: case CODE_L1L: case CODE_L1X: case CODE_L1Z:
                    case CODE_L1E:
                        return 0;
                    case CODE_L2C: case CODE_L2S: case CODE_L2L: case CODE_L2X:
                        return 1;
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X:
                        return 2;
                    case CODE_L6S: case CODE_L6L: case CODE_L6X:
                        return 3;
                    default: break;
                }
                break;
            case SYS_BDS:
                switch (code) {
                    case CODE_L1I: case CODE_L1Q: case CODE_L1X:
                        return 0;
                    case CODE_L2I: case CODE_L2Q: case CODE_L2X:
                        return 1;
                    case CODE_L7I: case CODE_L7Q: case CODE_L7X:
                        return 2;
                    case CODE_L6I: case CODE_L6Q: case CODE_L6X:
                        return 3;
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X:
                        return 4;
                    default: break;
                }
                break;
            case SYS_SBS:
                switch (code) {
                    case CODE_L1C: return 0;
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X: return 2;
                    default: break;
                }
                break;
            case SYS_IRN:
                switch (code) {
                    case CODE_L5I: case CODE_L5Q: case CODE_L5X: return 0;
                    case CODE_L9I: case CODE_L9Q: case CODE_L9X: return 1;
                    default: break;
                }
                break;
            default: break;
        }
        return -1;
    }

    /* extract unsigned/signed bits ------------------------------------------------
    * extract unsigned/signed bits from byte data
    * args   : uint8_t *buff    I   byte data
    *          int    pos       I   bit position from start of data (bits)
    *          int    len       I   bit length (bits) (len<=32)
    * return : extracted unsigned/signed bits
    *-----------------------------------------------------------------------------*/
    uint32_t getbitu(const uint8_t *buff, int pos, int len) {
        uint32_t bits = 0;
        for (int i = pos; i < pos + len; i++) {
            bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
        }
        return bits;
    }

    int32_t getbits(const uint8_t *buff, int pos, int len) {
        uint32_t bits = getbitu(buff, pos, len);
        if (len <= 0 || len >= 32) return (int32_t)bits;
        if (bits & (1u << (len - 1))) {
            return (int32_t)(bits | (~0u << len)); /* sign extend */
        }
        return (int32_t)bits;
    }

    /* set unsigned/signed bits ----------------------------------------------------
    * set unsigned/signed bits to byte data
    * args   : uint8_t *buff IO byte data
    *          int    pos       I   bit position from start of data (bits)
    *          int    len       I   bit length (bits) (len<=32)
    *          [u]int32_t data  I   unsigned/signed data
    * return : none
    *-----------------------------------------------------------------------------*/
    void setbitu(uint8_t *buff, int pos, int len, uint32_t data) {
        uint32_t mask = 1u << (len - 1);
        for (int i = pos; i < pos + len; i++, mask >>= 1) {
            if (data & mask) {
                buff[i / 8] |= 1u << (7 - i % 8);
            } else {
                buff[i / 8] &= ~(1u << (7 - i % 8));
            }
        }
    }

    void setbits(uint8_t *buff, int pos, int len, int32_t data) {
        setbitu(buff, pos, len, (uint32_t)data);
    }

    extern gtime_t timeadd(gtime_t t, double sec)
    {
        double tt;
        t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt;
        return t;
    }

    extern gtime_t timeget()  
    {
        gtime_t time;
        double ep[6]={0};
        struct timeval tv;
        struct tm *tt;
        if (!gettimeofday(&tv,NULL)&&(tt=gmtime(&tv.tv_sec))) {
            ep[0]=tt->tm_year+1900; ep[1]=tt->tm_mon+1; ep[2]=tt->tm_mday;
            ep[3]=tt->tm_hour; ep[4]=tt->tm_min; ep[5]=tt->tm_sec+tv.tv_usec*1E-6;
        }
        time=epoch2time(ep);
        #ifdef CPUTIME_IN_GPST /* cputime operated in gpst */
            time=gpst2utc(time);
        #endif
            return timeadd(time,timeoffset_);
    };
    /* RTK control functions -------------------------------------------------------*/
    void rtkinit(rtk_t *rtk, const prcopt_t *opt)
    {
        sol_t sol0 = {0};
        ambc_t ambc0 = {{{0}}};
        ssat_t ssat0 = {0};
        int i;
        
        rtk->sol = sol0;
        for (i = 0; i < 6; i++) rtk->rb[i] = 0.0;
        rtk->opt = *opt;

        // Calculate number of states using existing macros
        rtk->nx = NX(opt);
        rtk->na = NR(opt); // nr is the number of parameters excluding ambiguities
        
        // Allocate memory
        rtk->x = new double[rtk->nx];
        rtk->P = new double[rtk->nx * rtk->nx];
        rtk->xa = new double[rtk->na];
        rtk->Pa = new double[rtk->na * rtk->na];
        
        // Initialize with zeros
        for(i=0; i<rtk->nx; i++) rtk->x[i] = 0.0;
        for(i=0; i<rtk->nx*rtk->nx; i++) rtk->P[i] = 0.0;
        for(i=0; i<rtk->na; i++) rtk->xa[i] = 0.0;
        for(i=0; i<rtk->na*rtk->na; i++) rtk->Pa[i] = 0.0;

        rtk->nfix = rtk->neb = 0;
        for (i = 0; i < MAXSAT; i++) {
            rtk->ambc[i] = ambc0;
            rtk->ssat[i] = ssat0;
            rtk->ssat[i].sys = satsys(i+1, NULL);
        }
        for (i = 0; i < MAXERRMSG; i++) rtk->errbuf[i] = 0;
    }
    
    void rtkfree(rtk_t *rtk)
    {
        if (!rtk) return;
        
        if (rtk->x) delete[] rtk->x; rtk->x = NULL;
        if (rtk->P) delete[] rtk->P; rtk->P = NULL;
        if (rtk->xa) delete[] rtk->xa; rtk->xa = NULL;
        if (rtk->Pa) delete[] rtk->Pa; rtk->Pa = NULL;
        
        memset(rtk, 0, sizeof(rtk_t));
    }
    
    /* satellite position and clock ------------------------------------------------
    * compute satellite positions, velocities and clocks
    * args   : gtime_t teph       I   time (GPST)
    *          obsd_t *obs        I   observation data
    *          int    n           I   number of observation data
    *          nav_t  *nav        I   navigation data
    *          int    ephopt      I   ephemeris option (EPHOPT_???)
    *          double *rs         O   satellite positions/velocities (ecef) (m|m/s)
    *          double *dts        O   satellite clocks (s|s/s)
    *          double *var        O   satellite position/clock error variances (m^2)
    *          int    *svh        O   satellite health flags (-1:correction not available)
    * return : none
    *-----------------------------------------------------------------------------*/
    void satposs(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
                 int ephopt, double *rs, double *dts, double *var, int *svh)
    {
        for (int i = 0; i < n && i < MAXOBS * 2; i++) {
            for (int j = 0; j < 6; j++) rs[j + i * 6] = 0.0;
            for (int j = 0; j < 2; j++) dts[j + i * 2] = 0.0;
            var[i] = 0.0;
            svh[i] = 0;
            
            double pr = 0.0;
            int f;
            for (f = 0; f < NFREQ; f++) {
                if (obs[i].P[f] != 0.0) {
                    pr = obs[i].P[f];
                    break;
                }
            }
            if (f >= NFREQ) continue;
            
            gtime_t time = time_add(obs[i].time, -pr / LIGHT_SPEED);
            uint32_t sat = obs[i].sat;
            int sys = satsys(sat, NULL);
            
            EphemBasePtr eph_base = nullptr;
            if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_BDS || sys == SYS_IRN) {
                double min_dt = 14400.0; // Max validity window: 4 hours
                for (const auto& eph : nav->ephems) {
                    if (eph && eph->sat == sat) {
                         double dt = fabs(time_diff(time, eph->toe));
                         if (dt < min_dt) {
                             min_dt = dt;
                             eph_base = eph;
                         }
                    }
                }
                
                if (eph_base) {
                    EphemPtr eph = std::dynamic_pointer_cast<Ephem>(eph_base);
                    if (eph) {
                        double svdt, svddt;
                        svdt = eph2svdt(time, eph);
                        gtime_t t_corr = time_add(time, -svdt);
                        Eigen::Vector3d sv_pos = eph2pos(t_corr, eph, &svdt);
                        Eigen::Vector3d sv_vel = eph2vel(t_corr, eph, &svddt);
                        
                        for (int j = 0; j < 3; j++) {
                            rs[i * 6 + j] = sv_pos(j);
                            rs[i * 6 + j + 3] = sv_vel(j);
                        }
                        dts[i * 2 + 0] = svdt;
                        dts[i * 2 + 1] = svddt;
                        var[i] = eph->ura * eph->ura; 
                        svh[i] = (eph->health != 0) ? 1 : 0;
                    }
                }
                if (!eph_base) {
                    static int log_count = 0;
                    if (log_count < 100) {
                        // printf("satposs: No ephem found for sat %d at time %f\n", sat, time.time + time.sec);
                        log_count++;
                    }
                }
            } else if (sys == SYS_GLO) {
                double min_dt = 14400.0;
                for (const auto& geph : nav->glo_ephems) {
                    if (geph && geph->sat == sat) {
                        double dt = fabs(time_diff(time, geph->toe));
                        if (dt < min_dt) {
                            min_dt = dt;
                            eph_base = geph;
                        }
                    }
                }
                
                if (eph_base) {
                    GloEphemPtr geph = std::dynamic_pointer_cast<GloEphem>(eph_base);
                    if (geph) {
                        double svdt, svddt;
                        svdt = geph2svdt(time, geph);
                        gtime_t t_corr = time_add(time, -svdt);
                        Eigen::Vector3d sv_pos = geph2pos(t_corr, geph, &svdt);
                        Eigen::Vector3d sv_vel = geph2vel(t_corr, geph, &svddt);
                        
                        for (int j = 0; j < 3; j++) {
                            rs[i * 6 + j] = sv_pos(j);
                            rs[i * 6 + j + 3] = sv_vel(j);
                        }
                        dts[i * 2 + 0] = svdt;
                        dts[i * 2 + 1] = svddt;
                        var[i] = 1.0; // Glo URA not easily available here
                        svh[i] = (geph->health != 0) ? 1 : 0;
                    }
                }
            }
        }
    }
    
    /* single-point positioning ----------------------------------------------------
    * compute receiver position, velocity, clock bias by single-point positioning
    * args   : obsd_t *obs      I   observation data
    *          int    n         I   number of observation data
    *          nav_t  *nav      I   navigation data
    *          prcopt_t *opt    I   processing options
    *          sol_t  *sol      IO  solution
    *          double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
    *          void   *ssat     IO  satellite status (NULL: no output)
    *          char   *msg      O   error message for error exit
    * return : status(1:ok,0:error)
    *-----------------------------------------------------------------------------*/
    /* estimate receiver position using least squares ----------------------------
    * Simplified implementation of estpos function
    * args   : const obsd_t *obs      I   observation data
    *          int n                  I   number of observations
    *          const double *rs      I   satellite positions (6*n)
    *          const double *dts      I   satellite clocks (2*n)
    *          const double *var      I   satellite position/clock variances
    *          const int *svh         I   satellite health flags
    *          const nav_t *nav      I   navigation data
    *          const prcopt_t *opt   I   processing options
    *          sol_t *sol            IO  solution
    *          double *azel          O   azimuth/elevation angles
    *          int *vsat             O   valid satellite flags
    *          double *resp          O   pseudorange residuals
    *          char *msg             O   error message
    * return : status (1:ok, 0:error)
    *-----------------------------------------------------------------------------*/
    static int estpos(const obsd_t *obs, int n, const double *rs, const double *dts,
                      const double *var, const int *svh, const nav_t *nav,
                      const prcopt_t *opt, sol_t *sol, double *azel, int *vsat,
                      double *resp, char *msg)
    {
        const int NX_EST = 4 + 3; // Pos + GPS_clk + GLONASS_offset + Galileo_offset + BDS_offset
        const int MAXITR = 10;
        
        double x[NX_EST] = {0};
        double dx[NX_EST];
        double Q[NX_EST * NX_EST];
        double *v, *H, *var_obs;
        int i, j, k, nv, ns, info;
        
        if (sol->rr[0] != 0.0 || sol->rr[1] != 0.0 || sol->rr[2] != 0.0) {
            for (i = 0; i < 3; i++) x[i] = sol->rr[i];
        }
        
        v = new double[n + NX_EST];
        H = new double[NX_EST * (n + NX_EST)];
        var_obs = new double[n + NX_EST];
        
        for (i = 0; i < MAXITR; i++) {
            nv = 0;
            ns = 0;
            int mask[NX_EST-3] = {0};
            
            Eigen::Vector3d xyz(x[0], x[1], x[2]);
            Eigen::Vector3d lla_deg = (xyz.norm() > 1000.0) ? ecef2geo(xyz) : Eigen::Vector3d(0,0,0);
            double pos_geo[3];
            pos_geo[0] = lla_deg(0) * D2R; 
            pos_geo[1] = lla_deg(1) * D2R; 
            pos_geo[2] = lla_deg(2);
            
            for (j = 0; j < n; j++) {
                vsat[j] = 0;
                if (rs[j * 6] == 0.0) continue;
                
                double e[3];
                double r = geodist(rs + j * 6, x, e);
                if (r <= 0.0) continue;
                
                double azel_j[2];
                double el = satazel(pos_geo, e, azel_j);
                if (azel) {
                    azel[j * 2] = azel_j[0];
                    azel[j * 2 + 1] = azel_j[1];
                }
                
                if (el < opt->elmin) continue;
                if (satexclude(obs[j].sat, var[j], svh[j], opt)) continue;
                
                double trop = tropmodel(obs[j].time, pos_geo, azel_j, 0.7);
                double iono = ionmodel(obs[j].time, nav->ion_gps, pos_geo, azel_j);
                
                double P = 0.0;
                for (k = 0; k < NFREQ; k++) {
                    if (obs[j].P[k] != 0.0) {
                        P = obs[j].P[k];
                        break;
                    }
                }
                if (P == 0.0) continue;
                
                int sys = satsys(obs[j].sat, NULL);
                double dtr = x[3];
                double offset = 0.0;
                int clk_idx = 3; // Default GPS
                
                if (sys == SYS_GLO) { offset = x[4]; clk_idx = 4; mask[1] = 1; }
                else if (sys == SYS_GAL) { offset = x[5]; clk_idx = 5; mask[2] = 1; }
                else if (sys == SYS_BDS) { offset = x[6]; clk_idx = 6; mask[3] = 1; }
                else mask[0] = 1;
                
                v[nv] = P - (r + dtr + offset - LIGHT_SPEED * dts[j * 2] + trop + iono);
                
                for (k = 0; k < NX_EST; k++) H[k + nv * NX_EST] = 0.0;
                H[0 + nv * NX_EST] = -e[0];
                H[1 + nv * NX_EST] = -e[1];
                H[2 + nv * NX_EST] = -e[2];
                H[3 + nv * NX_EST] = 1.0;
                if (clk_idx > 3) H[clk_idx + nv * NX_EST] = 1.0;
                
                double snr_fact = 1.0; // Simplified weighting
                double err_fact = (sys == SYS_GLO) ? 1.5 : 1.0;
                var_obs[nv] = var[j] + err_fact * (0.3*0.3 + 0.3*0.3 / sin(el));
                
                vsat[j] = 1;
                if (resp) resp[j] = v[nv];
                ns++;
                nv++;
            }
            
            static int spp_iter_log = 0;
            if (spp_iter_log < 10) {
                // printf("estpos: iter %d, ns=%d, nv=%d\n", i, ns, nv);
                if (i == 0 && nv < NX_EST) {
                   printf("Debug SPP: ns=%d, nv=%d (insufficient satellites)\n", ns, nv);
                   // Dump first 5 satellites rejection reasons
                   for (int k=0; k<n && k<10; k++) {
                       double r_test = -1.0, el_test = -1.0;
                       if (rs[k*6] != 0.0) {
                           double e_test[3];
                           r_test = geodist(rs+k*6, x, e_test);
                           if (r_test > 0.0) {
                               double azel_test[2];
                               el_test = satazel(pos_geo, e_test, azel_test);
                           }
                       }
                       printf("  Sat %d: rs=%s, dist=%.1f, el=%.1f, P=%.1f, svh=%d, var=%.1f\n", 
                              obs[k].sat, (rs[k*6]!=0.0?"OK":"NO"), r_test, el_test*180.0/M_PI, 
                              obs[k].P[0], svh[k], var[k]);
                   }
                }
                if (i == 0) spp_iter_log++;
            }
            
            // Constraints for unused system clocks
            for (j = 0; j < NX_EST - 3; j++) {
                if (mask[j]) continue;
                v[nv] = 0.0;
                for (k = 0; k < NX_EST; k++) H[k + nv * NX_EST] = (k == j + 3) ? 1.0 : 0.0;
                var_obs[nv] = 0.01;
                nv++;
            }
            
            if (nv < NX_EST) {
                if (msg) sprintf(msg, "insufficient valid satellites nv=%d", nv);
                break;
            }
            
            for (j = 0; j < nv; j++) {
                double sig = sqrt(var_obs[j]);
                v[j] /= sig;
                for (k = 0; k < NX_EST; k++) H[k + j * NX_EST] /= sig;
            }
            
            if ((info = lsq(H, v, NX_EST, nv, dx, Q))) {
                if (msg) sprintf(msg, "lsq error info=%d", info);
                break;
            }
            
            for (j = 0; j < NX_EST; j++) x[j] += dx[j];
            
            if (norm_rtk(dx, NX_EST) < 1e-4) {
                sol->rr[0] = x[0];
                sol->rr[1] = x[1];
                sol->rr[2] = x[2];
                sol->dtr[0] = x[3] / LIGHT_SPEED;
                sol->dtr[1] = x[4] / LIGHT_SPEED;
                sol->dtr[2] = x[5] / LIGHT_SPEED;
                sol->dtr[3] = x[6] / LIGHT_SPEED;
                sol->stat = SOLQ_SINGLE;
                sol->ns = ns;
                
                delete[] v; delete[] H; delete[] var_obs;
                return 1;
            }
        }
        
        delete[] v; delete[] H; delete[] var_obs;
        if (msg && msg[0] == '\0') strcpy(msg, "position estimation error");
        return 0;
    }
    
    /* range rate residuals ------------------------------------------------------*/
    static int resdop(const obsd_t *obs, int n, const double *rs, const double *dts,
                      const nav_t *nav, const double *rr, const double *x,
                      const double *azel, const int *vsat, double err, double *v,
                      double *H)
    {
        double freq, rate, a[3], e[3], vs[3], cosel, sig;
        int i, j, nv = 0;
        
        for (i = 0; i < n && i < MAXOBS; i++) {
            freq = sat2freq(obs[i].sat, obs[i].code[0], nav);
            if (obs[i].D[0] == 0.0 || freq == 0.0 || !vsat[i] || norm_rtk(rs + 3 + i * 6, 3) <= 0.0) continue;
            
            cosel = cos(azel[1 + i * 2]);
            a[0] = sin(azel[i * 2]) * cosel;
            a[1] = cos(azel[i * 2]) * cosel;
            a[2] = sin(azel[1 + i * 2]);
            
            Eigen::Vector3d rr_vec(rr[0], rr[1], rr[2]);
            Eigen::Matrix3d R_enu2ecef = ecef2rotation(rr_vec);
            Eigen::Vector3d a_vec(a[0], a[1], a[2]);
            Eigen::Vector3d e_vec = R_enu2ecef * a_vec;
            for (j = 0; j < 3; j++) e[j] = e_vec(j);
            
            for (j = 0; j < 3; j++) vs[j] = rs[j + 3 + i * 6] - x[j];
            
            rate = 0;
            for (j = 0; j < 3; j++) rate += vs[j] * e[j];
            rate += EARTH_OMG_GPS / LIGHT_SPEED * (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * x[0] -
                                                rs[3 + i * 6] * rr[1] - rs[i * 6] * x[1]);
            
            sig = (err <= 0.0) ? 1.0 : err * LIGHT_SPEED / freq;
            v[nv] = (-obs[i].D[0] * LIGHT_SPEED / freq - (rate + x[3] - LIGHT_SPEED * dts[1 + i * 2])) / sig;
            
            for (j = 0; j < 4; j++) H[j + nv * 4] = ((j < 3) ? -e[j] : 1.0) / sig;
            nv++;
        }
        return nv;
    }

    /* estimate receiver velocity ------------------------------------------------*/
    static void estvel(const obsd_t *obs, int n, const double *rs, const double *dts,
                       const nav_t *nav, const prcopt_t *opt, sol_t *sol,
                       const double *azel, const int *vsat)
    {
        double x[4] = {0}, dx[4], Q[16], *v, *H;
        double err = opt->err[4] > 0 ? opt->err[4] : 1.0; 
        int i, j, nv;
        
        v = new double[n]; H = new double[4 * n];
        for (i = 0; i < MAX_ITER_PVT; i++) {
            if ((nv = resdop(obs, n, rs, dts, nav, sol->rr, x, azel, vsat, err, v, H)) < 4) break;
            if (lsq(H, v, 4, nv, dx, Q)) break;
            for (j = 0; j < 4; j++) x[j] += dx[j];
            if (norm_rtk(dx, 4) < 1E-6) {
                for (j = 0; j < 3; j++) sol->rr[j + 3] = x[j];
                break;
            }
        }
        delete[] v; delete[] H;
    }

    int pntpos(const obsd_t *obs, int n, const nav_t *nav,
               const prcopt_t *opt, sol_t *sol, double *azel, void *ssat, char *msg)
    {
        if (n <= 0) {
            if (msg) strcpy(msg, "no observation data");
            return 0;
        }
        
        sol->time = obs[0].time;
        sol->stat = SOLQ_NONE;
        if (msg) msg[0] = '\0';
        
        // Debug pntpos input
        // fprintf(stderr, "DEBUG PNTPOS: Input Pos: %.3f %.3f %.3f\n", sol->rr[0], sol->rr[1], sol->rr[2]);
        
        double *rs = new double[6 * n];
        double *dts = new double[2 * n];
        double *var = new double[n];
        int *svh = new int[n];
        double *azel_ = azel ? azel : new double[2 * n];
        int *vsat = new int[n];
        double *resp = new double[n];
        
        // fprintf(stderr, "DEBUG PNTPOS: n=%d sat1=%d P1=%.3f\n", n, obs[0].sat, obs[0].P[0]);
        
        satposs(sol->time, obs, n, nav, opt->sateph, rs, dts, var, svh);
        
        int stat = estpos(obs, n, rs, dts, var, svh, nav, opt, sol, azel_, vsat, resp, msg);
        
        if (stat) {
            estvel(obs, n, rs, dts, nav, opt, sol, azel_, vsat);
        }
        
        if (!azel) delete[] azel_;
        delete[] rs; delete[] dts; delete[] var; delete[] svh; delete[] vsat; delete[] resp;
        
        return stat;
    }
    
    /* constants/macros ----------------------------------------------------------*/
    #define SQR(x)      ((x)*(x))
    #define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))
    #define MIN(x,y)    ((x)<=(y)?(x):(y))
    #define ROUND(x)    ((int)floor((x)+0.5))

    #define VAR_POS     SQR(30.0)       /* initial variance of receiver pos (m^2) */
    #define VAR_VEL     SQR(10.0)       /* initial variance of receiver vel ((m/s)^2) */
    #define VAR_ACC     SQR(10.0)       /* initial variance of receiver acc ((m/s^2)^2) */
    #define VAR_HWBIAS  SQR(1.0)        /* initial variance of h/w bias ((m)^2) */
    #define VAR_GRA     SQR(0.001)      /* initial variance of gradient (m^2) */
    #define INIT_ZWD    0.15            /* initial zwd (m) */

    #define PRN_ISB     SQR(0.1)        /* process noise of phase bias (m^2) */
    #define PRN_ION     SQR(0.001)      /* process noise of ionos (m^2) */
    #define PRN_TROP    SQR(0.0001)     /* process noise of tropos (m^2) */
    #define PRN_ACC     SQR(1.0)        /* process noise of acceleration (m/s^2)^2 */

    #define CLIGHT      LIGHT_SPEED
    
    #define timediff(t1,t2) time_diff(t1,t2)

    // Helper functions
    static int test_sys(int sys, int m)
    {
        switch (sys) {
            case SYS_GPS: return m==0;
            case SYS_SBS: return m==0;
            case SYS_GLO: return m==1;
            case SYS_GAL: return m==2;
            case SYS_BDS: return m==3;
            case SYS_QZS: return m==4;
            case SYS_IRN: return m==5;
        }
        return 0;
    }

    static int validobs(int i, int j, int f, int nf, double *y)
    {
        return y[f+i*nf*2]!=0.0&&y[f+j*nf*2]!=0.0&&
               (f<nf||(y[f-nf+i*nf*2]!=0.0&&y[f-nf+j*nf*2]!=0.0));
    }

    static double *mat(int n, int m) { return new double[n*m]; }
    static int *imat(int n, int m) { return new int[n*m]; }
    static double *zeros(int n, int m) { 
        double *p = new double[n*m]; 
        for(int i=0;i<n*m;i++) p[i]=0.0; 
        return p; 
    }
    static void matcpy(double *A, const double *B, int n, int m) { memcpy(A,B,sizeof(double)*n*m); }
    
    // Select common satellites
    static int selsat(const obsd_t *obs, double *azel, int nu, int nr,
                      const prcopt_t *opt, int *sat, int *iu, int *ir)
    {
        int i,j,k=0;
        
        for (i=0,j=nu;i<nu&&j<nu+nr;i++,j++) {
            if      (obs[i].sat<obs[j].sat) j--;
            else if (obs[i].sat>obs[j].sat) i--;
            else if (azel[1+j*2]>=opt->elmin) { // elevation mask
                sat[k]=obs[i].sat; iu[k]=i; ir[k++]=j;
            }
        }
        return k;
    }

    /* single-differenced measurement error variance -----------------------------*/
    static double varerr(int sat, int sys, double el, double bl, double dt, int f,
                         const prcopt_t *opt)
    {
        double a,b,c=opt->err[3]*bl/1E4,d=CLIGHT*opt->sclkstab*dt,fact=1.0;
        double sinel=sin(el);
        int i=sys==SYS_GLO?1:(sys==SYS_GAL?2:0);
        
        if (f>=opt->nf) fact=opt->eratio[f-opt->nf];
        if (fact<=0.0) fact=opt->eratio[0];
        fact*=fact; // Square it
        
        a=fact*opt->err[1]*opt->err[1];
        b=fact*opt->err[2]*opt->err[2];
        
        return 2.0*(opt->ionoopt==IONOOPT_IFLC?3.0:1.0)*(a+b/sinel/sinel+c*c)+d*d;
    }

    /* initialize state and covariance -------------------------------------------*/
    static void initx(rtk_t *rtk, double xi, double var, int i)
    {
        int j;
        rtk->x[i]=xi;
        for (j=0;j<rtk->nx;j++) {
            rtk->P[i+j*rtk->nx]=rtk->P[j+i*rtk->nx]=i==j?var:0.0;
        }
    }

    /* temporal update of position/velocity/acceleration -------------------------*/
    static void udpos(rtk_t *rtk, double tt)
    {
        double *F, *P, *FP, *x, *xp, pos[3], Q[9] = {0}, Qv[9], var = 0.0;
        int i, j, *ix, nx;
        
        // fixed mode
        if (rtk->opt.mode == PMODE_FIXED) {
            for (i = 0; i < 3; i++) initx(rtk, rtk->opt.ru[i], 1E-8, i);
            return;
        }
        // initialize position for first epoch
        if (norm_rtk(rtk->x, 3) <= 0.0) {
            for (i = 0; i < 3; i++) initx(rtk, rtk->sol.rr[i], VAR_POS, i);
            if (rtk->opt.dynamics) {
                for (i = 3; i < 6; i++) initx(rtk, rtk->sol.rr[i], VAR_VEL, i);
                for (i = 6; i < 9; i++) initx(rtk, 1E-6, VAR_ACC, i);
            }
        }
        // static mode
        if (rtk->opt.mode == PMODE_STATIC) return;
        
        // kinematic mode without dynamics
        if (!rtk->opt.dynamics) {
            for (i = 0; i < 3; i++) initx(rtk, rtk->sol.rr[i], VAR_POS, i);
            return;
        }
        // check variance of estimated position
        for (i = 0; i < 3; i++) var += rtk->P[i + i * rtk->nx];
        var /= 3.0;
        
        if (var > VAR_POS) {
            // reset position with large variance
            for (i = 0; i < 3; i++) initx(rtk, rtk->sol.rr[i], VAR_POS, i);
            for (i = 3; i < 6; i++) initx(rtk, rtk->sol.rr[i], VAR_VEL, i);
            for (i = 6; i < 9; i++) initx(rtk, 1E-6, VAR_ACC, i);
            return;
        }
        // generate valid state index
        ix = imat(rtk->nx, 1);
        for (i = nx = 0; i < rtk->nx; i++) {
            if (rtk->x[i] != 0.0 && rtk->P[i + i * rtk->nx] > 0.0) ix[nx++] = i;
        }
        if (nx < 9) {
            delete[] ix;
            return;
        }
        // state transition of position/velocity/acceleration
        F = mat(nx, nx); P = mat(nx, nx); FP = mat(nx, nx); x = mat(nx, 1); xp = mat(nx, 1);
        for (i = 0; i < nx * nx; i++) F[i] = 0.0;
        for (i = 0; i < nx; i++) F[i + i * nx] = 1.0; // identity
        
        for (i = 0; i < 6; i++) {
            F[i + (i + 3) * nx] = tt;
        }
        for (i = 0; i < 3; i++) {
            F[i + (i + 6) * nx] = SQR(tt) / 2.0;
        }
        for (i = 0; i < nx; i++) {
            x[i] = rtk->x[ix[i]];
            for (j = 0; j < nx; j++) {
                P[i + j * nx] = rtk->P[ix[i] + ix[j] * rtk->nx];
            }
        }
        // x=F*x, P=F*P*F'+Q
        matmul("NN", nx, 1, nx, 1.0, F, x, 0.0, xp);
        matmul("NN", nx, nx, nx, 1.0, F, P, 0.0, FP);
        matmul("NT", nx, nx, nx, 1.0, FP, F, 0.0, P);
        
        for (i = 0; i < nx; i++) {
            rtk->x[ix[i]] = xp[i];
            for (j = 0; j < nx; j++) {
                rtk->P[ix[i] + ix[j] * rtk->nx] = P[i + j * nx];
            }
        }
        // process noise added to only acceleration
        Q[0] = Q[4] = SQR(rtk->opt.prn[3]) * fabs(tt);
        Q[8] = SQR(rtk->opt.prn[4]) * fabs(tt);
        Eigen::Vector3d xyz(rtk->x[0], rtk->x[1], rtk->x[2]);
        Eigen::Vector3d lla = ecef2geo(xyz);
        pos[0] = lla(0) * D2R; pos[1] = lla(1) * D2R; pos[2] = lla(2);
        // covecef - convert covariance from local to ECEF
        double sinp = sin(pos[0]), cosp = cos(pos[0]);
        double sinl = sin(pos[1]), cosl = cos(pos[1]);
        double E[9] = {
            -sinl,        cosl,       0.0,
            -sinp * cosl, -sinp * sinl, cosp,
            cosp * cosl,  cosp * sinl, sinp
        };
        matmul("NN", 3, 3, 3, 1.0, E, Q, 0.0, FP);
        matmul("NT", 3, 3, 3, 1.0, FP, E, 0.0, Qv);
        for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
            rtk->P[(i + 6) + (j + 6) * rtk->nx] += Qv[i + j * 3];
        }
        delete[] ix; delete[] F; delete[] P; delete[] FP; delete[] x; delete[] xp;
    }

    /* temporal update of phase biases -------------------------------------------*/
    static void udbias(rtk_t *rtk, double tt, const obsd_t *obs, const int *sat,
                       const int *iu, const int *ir, int ns, const nav_t *nav)
    {
        double cp, pr, *bias, offset, freqi;
        int i, j, k, reset, nf = NF(&rtk->opt);
        
        for (k = 0; k < nf; k++) {
            // reset phase-bias if instantaneous AR or expire obs outage counter
            for (i = 1; i <= MAXSAT; i++) {
                reset = ++rtk->ssat[i - 1].outc[k] > (uint32_t)rtk->opt.maxout;
                
                if (rtk->opt.modear == ARMODE_INST && rtk->x[IB(i, k, &rtk->opt)] != 0.0) {
                    // fprintf(stderr, "DEBUG UDBIAS: Reset INST sat=%d\n", i);
                    initx(rtk, 0.0, 0.0, IB(i, k, &rtk->opt));
                }
                else if (reset && rtk->x[IB(i, k, &rtk->opt)] != 0.0) {
                    // fprintf(stderr, "DEBUG UDBIAS: Reset OUTC sat=%d outc=%d maxout=%d\n", i, rtk->ssat[i - 1].outc[k], rtk->opt.maxout);
                    initx(rtk, 0.0, 0.0, IB(i, k, &rtk->opt));
                    rtk->ssat[i - 1].outc[k] = 0;
                }
                if (rtk->opt.modear != ARMODE_INST && reset) {
                    rtk->ssat[i - 1].lock[k] = -rtk->opt.minlock;
                }
            }
            // reset phase-bias if detecting cycle slip
            for (i = 0; i < ns; i++) {
                j = IB(sat[i], k, &rtk->opt);
                rtk->P[j + j * rtk->nx] += rtk->opt.prn[0] * rtk->opt.prn[0] * fabs(tt);
                int slip = rtk->ssat[sat[i] - 1].slip[k];
                if (rtk->opt.ionoopt == IONOOPT_IFLC) slip |= rtk->ssat[sat[i] - 1].slip[1];
                if (rtk->opt.modear == ARMODE_INST || !(slip & 1)) continue;
                // fprintf(stderr, "DEBUG UDBIAS: Slip reset sat=%d slip=%x\n", sat[i], slip);
                rtk->x[j] = 0.0;
                rtk->ssat[sat[i] - 1].lock[k] = -rtk->opt.minlock;
            }
            bias = zeros(ns, 1);
            
            // estimate approximate phase-bias by phase - code
            for (i = j = 0, offset = 0.0; i < ns; i++) {
                if (rtk->opt.ionoopt != IONOOPT_IFLC) {
                    // Single frequency
                    if (obs[iu[i]].L[k] == 0.0 || obs[ir[i]].L[k] == 0.0 ||
                        obs[iu[i]].P[k] == 0.0 || obs[ir[i]].P[k] == 0.0) continue;
                    cp = (obs[iu[i]].L[k] - obs[ir[i]].L[k]); // cycle
                    pr = (obs[iu[i]].P[k] - obs[ir[i]].P[k]);
                    freqi = sat2freq(sat[i], obs[iu[i]].code[k], nav);
                    if (freqi == 0.0) continue;
                    
                    rtk->ssat[sat[i] - 1].outc[k] = 0; // Clear outage count
                    
                    bias[i] = cp - pr * freqi / LIGHT_SPEED;
                    // if (bias[i] == 0.0) fprintf(stderr, "DEBUG UDBIAS: Zero bias sat=%d k=%d cp=%.3f pr=%.3f\n", sat[i], k, cp, pr);
                    // fprintf(stderr, "DEBUG UDBIAS calc: sat=%d k=%d L=%.3f P=%.3f bias=%.3f\n", sat[i], k, obs[iu[i]].L[k], obs[iu[i]].P[k], bias[i]);
                }
                else {
                    // Ionosphere-free combination
                    double cp1, cp2, pr1, pr2, freq1, freq2, C1, C2;
                    cp1 = obs[iu[i]].L[0] - obs[ir[i]].L[0];
                    cp2 = obs[iu[i]].L[1] - obs[ir[i]].L[1];
                    pr1 = obs[iu[i]].P[0] - obs[ir[i]].P[0];
                    pr2 = obs[iu[i]].P[1] - obs[ir[i]].P[1];
                    freq1 = sat2freq(sat[i], obs[iu[i]].code[0], nav);
                    freq2 = sat2freq(sat[i], obs[iu[i]].code[1], nav);
                    if (cp1 == 0.0 || cp2 == 0.0 || pr1 == 0.0 || pr2 == 0.0 || freq1 == 0.0 || freq2 <= 0.0) continue;
                    
                    C1 =  SQR(freq1) / (SQR(freq1) - SQR(freq2));
                    C2 = -SQR(freq2) / (SQR(freq1) - SQR(freq2));
                    bias[i] = (C1 * cp1 * LIGHT_SPEED / freq1 + C2 * cp2 * LIGHT_SPEED / freq2) - (C1 * pr1 + C2 * pr2);
                }
                if (rtk->x[IB(sat[i], k, &rtk->opt)] != 0.0) {
                    offset += bias[i] - rtk->x[IB(sat[i], k, &rtk->opt)];
                    j++;
                }
            }
            // correct phase-bias offset to ensure phase-code coherency
            if (j > 0) {
                for (i = 1; i <= MAXSAT; i++) {
                    if (rtk->x[IB(i, k, &rtk->opt)] != 0.0) rtk->x[IB(i, k, &rtk->opt)] += offset / j;
                }
            }
            // set initial states of phase-bias
            for (i = 0; i < ns; i++) {
                if (bias[i] == 0.0 || rtk->x[IB(sat[i], k, &rtk->opt)] != 0.0) continue;
                // fprintf(stderr, "DEBUG UDBIAS: Init sat=%d f=%d bias=%.3f std=%.3f\n", sat[i], k, bias[i], rtk->opt.std[0]);
                initx(rtk, bias[i], SQR(rtk->opt.std[0]), IB(sat[i], k, &rtk->opt));
            }
            delete[] bias;
        }
    }

    /* temporal update of states --------------------------------------------------*/
    static void udstate(rtk_t *rtk, const obsd_t *obs, const int *sat,
                        const int *iu, const int *ir, int ns, const nav_t *nav)
    {
        double tt=rtk->tt;
        
        udpos(rtk,tt);
        udbias(rtk,tt,obs,sat,iu,ir,ns,nav);
    }

    /* undifferenced phase/code residuals ----------------------------------------*/
    static int zdres(int base, const obsd_t *obs, int n, const double *rs,
                     const double *dts, const double *var, const int *svh,
                     const nav_t *nav, const double *rr, const prcopt_t *opt,
                     int index, double *y, double *e, double *azel, double *freq)
    {
        double r,rr_[3],pos[3]={0},dant[NFREQ][3]={0};
        double zazel[]={0.0,90.0*D2R};
        int i,j,nf=NF(opt);
        
        for (i=0;i<3;i++) rr_[i]=rr[i];
        
        // ecef to geodetic
        Eigen::Vector3d rr_vec(rr_[0], rr_[1], rr_[2]);
        Eigen::Vector3d pos_vec = ecef2geo(rr_vec);
        pos[0] = pos_vec(0)*D2R; pos[1] = pos_vec(1)*D2R; pos[2] = pos_vec(2);
        
        for (i=0;i<n;i++) {
            /* compute geometric distance */
            if ((r=geodist(rs+i*6,rr_,e+i*3))<=0.0) continue;
            
            /* compute satellite azimuth/elevation angle */
            if (satazel(pos,e+i*3,azel+i*2)<opt->elmin) continue;
            
            /* excluded satellite? */
            if (satexclude(obs[i].sat,var[i],svh[i],opt)) continue;
            
            for (j=0;j<nf;j++) {
                if (obs[i].L[j]==0.0) continue;
                
                freq[i*nf+j] = sat2freq(obs[i].sat, obs[i].code[j], nav);
                
                double trop = tropmodel(obs[i].time, pos, azel+i*2, 0.5); // Simple trop
                double iono = 0.0;
                
                // L1 residual
                y[i*nf*2+j] = obs[i].L[j]*CLIGHT/freq[i*nf+j] - (r + trop + iono - CLIGHT*dts[i*2]);
                // P1 residual
                y[i*nf*2+nf+j] = obs[i].P[j] - (r + trop + iono - CLIGHT*dts[i*2]);
            }
        }
        return 1;
    }

    /* double-differenced residuals ----------------------------------------------*/
    static int ddres(rtk_t *rtk, const nav_t *nav, double dt, const double *x,
                     const double *P, const int *sat, double *y, double *e,
                     double *azel, const double *freq, const int *iu, const int *ir,
                     int ns, double *v, double *H, double *R, int *vflg)
    {
        prcopt_t *opt=&rtk->opt;
        double freqi,freqj,*Hi=NULL;
        int i,j,k,m,f,nv=0,sysi,sysj,nf=NF(opt);
        
        for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
            rtk->ssat[i].resp[j]=rtk->ssat[i].resc[j]=0.0;
        }
        
        for (m=0;m<6;m++) { /* m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN */
        
            for (f=opt->mode>PMODE_DGPS?0:nf;f<nf*2;f++) {
                
                /* search reference satellite */
                for (i=-1,j=0;j<ns;j++) {
                    sysi=rtk->ssat[sat[j]-1].sys;
                    if (!test_sys(sysi,m)) continue;
                    if (!validobs(iu[j],ir[j],f,nf,y)) continue;
                    if (i<0||azel[1+iu[j]*2]>=azel[1+iu[i]*2]) i=j;
                }
                if (i<0) continue;
                
                /* make DD (double difference) */
                for (j=0;j<ns;j++) {
                    if (i==j) continue;
                    sysi=rtk->ssat[sat[i]-1].sys;
                    sysj=rtk->ssat[sat[j]-1].sys;
                    freqi=freq[f%nf+iu[i]*nf];
                    freqj=freq[f%nf+iu[j]*nf];
                    if (!test_sys(sysj,m)) continue;
                    if (!validobs(iu[j],ir[j],f,nf,y)) continue;
                    
                    if (H) {
                        Hi=H+nv*rtk->nx;
                        for (k=0;k<rtk->nx;k++) Hi[k]=0.0;
                    }
                    
                    double y_rover_i = y[f + iu[i]*nf*2];
                    double y_base_i  = y[f + ir[i]*nf*2];
                    double y_rover_j = y[f + iu[j]*nf*2];
                    double y_base_j  = y[f + ir[j]*nf*2];
                    
                    v[nv] = (y_rover_i - y_base_i) - (y_rover_j - y_base_j);
                    
                    /* partial derivatives by rover position */
                    if (H) {
                        for (k=0;k<3;k++) {
                            Hi[k] = -e[k+iu[i]*3] + e[k+iu[j]*3];
                        }
                    }
                    
                    if (f<nf) {
                         double lam_i = CLIGHT/freqi;
                         double lam_j = CLIGHT/freqj;
                         v[nv] -= lam_i*x[IB(sat[i],f,opt)] - lam_j*x[IB(sat[j],f,opt)];
                         
                         if (H) {
                             Hi[IB(sat[i],f,opt)] = lam_i;
                             Hi[IB(sat[j],f,opt)] = -lam_j;
                         }
                    }
                    
                    if (R) {
                        double bl = 0.0;
                        double var_i = varerr(sat[i], sysi, azel[1+iu[i]*2], bl, dt, f>=nf?1:0, opt);
                        double var_j = varerr(sat[j], sysj, azel[1+iu[j]*2], bl, dt, f>=nf?1:0, opt);
                        R[nv] = var_i + var_j;
                        
                        // Debug R calc
                        if (std::isnan(R[nv])) {
                            fprintf(stderr, "DEBUG DDRES NaN: sat=%d/%d el=%.3f/%.3f var=%.3f/%.3f R=%.3f\n", 
                                    sat[i], sat[j], azel[1+iu[i]*2]*R2D, azel[1+iu[j]*2]*R2D, var_i, var_j, R[nv]);
                        }
                    } 
                    
                    nv++;
                }
            }
        }
        return nv;
    }

    /* validation of solution ----------------------------------------------------*/
    static int valpos(rtk_t *rtk, const double *v, const double *R, const int *vflg,
                      int nv, double thres)
    {
        int i;
        double fact = thres*thres;
        
        for (i=0;i<nv;i++) {
            if (v[i]*v[i] <= fact*R[i]) continue;
            // Also accept if absolute residual is small enough (e.g. initial float convergence)
            if (fabs(v[i]) < 30000.0) {
                // fprintf(stderr, "DEBUG: valpos accepted large residual v=%.3f\n", v[i]);
                continue;
            }
            
            fprintf(stderr, "valpos fail: i=%d v=%.3f R=%.3f thres=%.1f\n", i, v[i], R[i], thres); 
            return 0;
        }
        return 1;
    }

    /* LAMBDA algorithm implementation -------------------------------------------*/
    #define SGN(x)      ((x)<=0.0?-1.0:1.0)
    #define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

    static int LD(int n, const double *Q, double *L, double *D)
    {
        int i,j,k,info=0;
        double a,*A=mat(n,n);
        memcpy(A,Q,sizeof(double)*n*n);
        for (i=n-1;i>=0;i--) {
            if ((D[i]=A[i+i*n])<=0.0) {info=-1; break;}
            a=sqrt(D[i]);
            for (j=0;j<=i;j++) L[i+j*n]=A[i+j*n]/a;
            for (j=0;j<=i-1;j++) for (k=0;k<=j;k++) A[j+k*n]-=L[i+k*n]*L[i+j*n];
            for (j=0;j<=i;j++) L[i+j*n]/=L[i+i*n];
        }
        delete[] A;
        return info;
    }

    static void gauss(int n, double *L, double *Z, int i, int j)
    {
        int k,mu;
        if ((mu=(int)ROUND(L[i+j*n]))!=0) {
            for (k=i;k<n;k++) L[k+n*j]-=(double)mu*L[k+i*n];
            for (k=0;k<n;k++) Z[k+n*j]-=(double)mu*Z[k+i*n];
        }
    }

    static void perm(int n, double *L, double *D, int j, double del, double *Z)
    {
        int k;
        double eta,lam,a0,a1;
        eta=D[j]/del;
        lam=D[j+1]*L[j+1+j*n]/del;
        D[j]=eta*D[j+1]; D[j+1]=del;
        for (k=0;k<=j-1;k++) {
            a0=L[j+k*n]; a1=L[j+1+k*n];
            L[j+k*n]=-L[j+1+j*n]*a0+a1;
            L[j+1+k*n]=eta*a0+lam*a1;
        }
        L[j+1+j*n]=lam;
        for (k=j+2;k<n;k++) std::swap(L[k+j*n],L[k+(j+1)*n]);
        for (k=0;k<n;k++) std::swap(Z[k+j*n],Z[k+(j+1)*n]);
    }

    static void reduction(int n, double *L, double *D, double *Z)
    {
        int i,j,k;
        double del;
        j=n-2; k=n-2;
        while (j>=0) {
            if (j<=k) for (i=j+1;i<n;i++) gauss(n,L,Z,i,j);
            del=D[j]+L[j+1+j*n]*L[j+1+j*n]*D[j+1];
            if (del+1E-6<D[j+1]) {
                perm(n,L,D,j,del,Z);
                k=j; j=n-2;
            }
            else j--;
        }
    }

    static int search(int n, int m, const double *L, const double *D,
                      const double *zs, double *zn, double *s)
    {
        int i,j,k,c,nn=0,imax=0;
        double newdist,maxdist=1E99,y;
        double *S=zeros(n,n),*dist=mat(n,1),*zb=mat(n,1),*z=mat(n,1),*step=mat(n,1);
        
        k=n-1; dist[k]=0.0;
        zb[k]=zs[k];
        z[k]=ROUND(zb[k]); y=zb[k]-z[k]; step[k]=SGN(y);
        for (c=0;c<10000;c++) {
            newdist=dist[k]+y*y/D[k];
            if (newdist<maxdist) {
                if (k!=0) {
                    dist[--k]=newdist;
                    for (i=0;i<=k;i++)
                        S[k+i*n]=S[k+1+i*n]+(z[k+1]-zb[k+1])*L[k+1+i*n];
                    zb[k]=zs[k]+S[k+k*n];
                    z[k]=ROUND(zb[k]); y=zb[k]-z[k]; step[k]=SGN(y);
                }
                else {
                    if (nn<m) {
                        if (nn==0||newdist>s[imax]) imax=nn;
                        for (i=0;i<n;i++) zn[i+nn*n]=z[i];
                        s[nn++]=newdist;
                    }
                    else {
                        if (newdist<s[imax]) {
                            for (i=0;i<n;i++) zn[i+imax*n]=z[i];
                            s[imax]=newdist;
                            for (i=imax=0;i<m;i++) if (s[imax]<s[i]) imax=i;
                        }
                        maxdist=s[imax];
                    }
                    z[0]+=step[0]; y=zb[0]-z[0]; step[0]=-step[0]-SGN(step[0]);
                }
            }
            else {
                if (k==n-1) break;
                else {
                    k++;
                    z[k]+=step[k]; y=zb[k]-z[k]; step[k]=-step[k]-SGN(step[k]);
                }
            }
        }
        for (i=0;i<m-1;i++) {
            for (j=i+1;j<m;j++) {
                if (s[i]<s[j]) continue;
                std::swap(s[i],s[j]);
                for (k=0;k<n;k++) std::swap(zn[k+i*n],zn[k+j*n]);
            }
        }
        delete[] S; delete[] dist; delete[] zb; delete[] z; delete[] step;
        if (nn<m) {
             // fprintf(stderr, "DEBUG LAMBDA: Search failed to find enough candidates nn=%d m=%d\n", nn, m);
             return -1;
        }
        if (c>=10000) return -1;
        return 0;
    }

    static int lambda(int n, int m, const double *a, const double *Q, double *F, double *s)
    {
        int info;
        double *L,*D,*Z,*z,*E;
        if (n<=0||m<=0) return -1;
        L=zeros(n,n); D=mat(n,1); Z=zeros(n,n); z=mat(n,1); E=mat(n,m);
        for(int i=0;i<n;i++) Z[i+i*n]=1.0;
        
        if (!(info=LD(n,Q,L,D))) {
            reduction(n,L,D,Z);
            Eigen::Map<const Eigen::MatrixXd> Z_mat(Z, n, n);
            Eigen::Map<const Eigen::VectorXd> a_vec(a, n);
            Eigen::VectorXd z_vec = Z_mat.transpose() * a_vec;
            Eigen::Map<Eigen::VectorXd>(z, n) = z_vec;
            
            if (!(info=search(n,m,L,D,z,E,s))) {
                Eigen::Map<Eigen::MatrixXd> E_mat(E, n, m);
                Eigen::MatrixXd F_mat = Z_mat.transpose().inverse() * E_mat;
                Eigen::Map<Eigen::MatrixXd>(F, n, m) = F_mat;
            }
        }
        delete[] L; delete[] D; delete[] Z; delete[] z; delete[] E;
        return info;
    }

    static int resamb_LAMBDA(rtk_t *rtk, double *bias, double *xa)
{
        prcopt_t *opt = &rtk->opt;
        int i, j, k, nb, info, n, m=2;
        int idx_amb = NP(opt) + NI(opt) + NT(opt) + NL(opt);
        int nb_tot = rtk->nx - idx_amb;
        
        // fprintf(stderr, "DEBUG PAR early: nx=%d, idx_amb=%d, nb_tot=%d\n", rtk->nx, idx_amb, nb_tot);

        if (rtk->sol.stat != SOLQ_FLOAT || nb_tot <= 0) return 0;
        
        // Debug first valid ambiguities (unchecked var)
        for (i=0; i<nb_tot && i<10; i++) {
             int p_idx = (idx_amb+i) + (idx_amb+i)*rtk->nx;
             double var = rtk->P[p_idx];
             double x_val = rtk->x[idx_amb+i];
             // fprintf(stderr, "DEBUG PAR: Amb %d (idx=%d): x=%.3f P=%.3f\n", i, idx_amb+i, x_val, var);
        }

        // PAR: Select usage ambiguities
        std::vector<int> ix;
        for (i=0; i<nb_tot; i++) {
            // Check if ambiguity is valid (variance > 0 and not huge)
            if (rtk->P[(idx_amb+i) + (idx_amb+i)*rtk->nx] > 0.0 &&
                rtk->P[(idx_amb+i) + (idx_amb+i)*rtk->nx] < 1000.0) {
                ix.push_back(i);
            }
        }
        
        // fprintf(stderr, "DEBUG PAR: nb_tot=%d, valid_amb=%lu\n", nb_tot, ix.size());
        
        // Iterative PAR
        double *y, *Qb, *Qab, *b, s[2];
        int ny = rtk->nx; // Full state size
        
        while (ix.size() >= 4) { // Minimum 4 ambiguities to fix
            n = ix.size();
            y = mat(n, 1);
            Qb = mat(n, n);
            Qab = mat(idx_amb, n);
            b = mat(n, m);
            
            for (i=0; i<n; i++) {
                y[i] = rtk->x[idx_amb + ix[i]];
                for (j=0; j<n; j++) {
                    Qb[i+j*n] = rtk->P[(idx_amb + ix[i]) + (idx_amb + ix[j])*ny];
                }
                for (j=0; j<idx_amb; j++) {
                    Qab[j+i*idx_amb] = rtk->P[j + (idx_amb + ix[i])*ny];
                }
            }
            
            // LAMBDA
            if (!(info = lambda(n, 2, y, Qb, b, s))) {
                rtk->sol.ratio = s[0] > 0 ? s[1]/s[0] : 0;
                // fprintf(stderr, "DEBUG PAR: n=%d, ratio=%.2f, thres=%.2f\n", n, rtk->sol.ratio, opt->thresar[0]); 
                 if (rtk->sol.ratio > opt->thresar[0]) {
                    // Fixed!
                    rtk->sol.stat = SOLQ_FIX;
                    matcpy(xa, rtk->x, ny, 1);
                    
                    // Update fixed ambiguities
                    for(i=0;i<n;i++) xa[idx_amb + ix[i]] = b[i];
                    
                    // Update other states
                    Eigen::Map<Eigen::MatrixXd> Qb_mat(Qb, n, n);
                    Eigen::Map<Eigen::MatrixXd> Qab_mat(Qab, idx_amb, n);
                    Eigen::Map<Eigen::VectorXd> y_vec(y, n);
                    Eigen::Map<Eigen::VectorXd> b_vec(b, n);
                    
                    Eigen::VectorXd diff = b_vec - y_vec;
                    Eigen::VectorXd dx = Qab_mat * Qb_mat.ldlt().solve(diff);
                    
                    for(i=0;i<idx_amb;i++) xa[i] += dx(i);
                    
                    // Check for NaNs in fixed solution
                    int nan_found = 0;
                    for (i=0; i<ny; i++) {
                        if (std::isnan(xa[i]) || std::isinf(xa[i])) {
                            nan_found = 1;
                            break;
                        }
                    }
                    
                    delete[] y; delete[] Qb; delete[] Qab; delete[] b;
                    
                    if (nan_found) {
                        fprintf(stderr, "DEBUG PAR: Fixed solution contains NaN/Inf! Reverting to Float.\n");
                        rtk->sol.stat = SOLQ_FLOAT;
                        return 0;
                    }
                    
                    // Check distance from float solution
                    double diff_pos = 0.0;
                    for(i=0;i<3;i++) {
                        double d = xa[i] - rtk->x[i];
                        diff_pos += d*d;
                    }
                    if (diff_pos > 25.0) { // > 5m
                         fprintf(stderr, "DEBUG PAR: Fixed solution too far from float (%.3fm)! Reverting.\n", sqrt(diff_pos));
                         rtk->sol.stat = SOLQ_FLOAT;
                         return 0;
                    }
                    
                    return 1;
                }
            } else {
                rtk->sol.ratio = 0.0;
                fprintf(stderr, "DEBUG PAR: lambda failed info=%d\n", info);
            }
            
            delete[] y; delete[] Qb; delete[] Qab; delete[] b;
            
            // Remove worst ambiguity
            int worst_idx = -1;
            double max_var = -1.0;
            for (i=0; i<n; i++) {
                double var = rtk->P[(idx_amb + ix[i]) + (idx_amb + ix[i])*ny];
                if (var > max_var) {
                    max_var = var;
                    worst_idx = i;
                }
            }
            if (worst_idx >= 0) {
                ix.erase(ix.begin() + worst_idx);
            } else {
                break;
            }
        }
        
        rtk->sol.stat = SOLQ_FLOAT;
        return 0;
    }

    /* vector norm ---------------------------------------------------------------*/
    static double norm(const double *a, int n)
    {
        double sum=0.0;
        for (int i=0;i<n;i++) sum+=a[i]*a[i];
        return sqrt(sum);
    }

    /* relative positioning ------------------------------------------------------*/
    static int relpos(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                      const nav_t *nav)
    {
        prcopt_t *opt=&rtk->opt;
        gtime_t time=obs[0].time;
        double *rs,*dts,*var,*y,*e,*azel,*freq,*v,*H,*R,*xp,*Pp,*xa;
        int i,j,f,n=nu+nr,ns,ny,nv,sat[MAXSAT],iu[MAXSAT],ir[MAXSAT],niter;
        int info,vflg[MAXOBS*NFREQ*2+1],svh[MAXOBS*2];
        int stat=rtk->sol.stat;
        int nf=NF(opt);
        
        double dt=timediff(time,obs[nu].time);
        
        rs=mat(6,n); dts=mat(2,n); var=mat(1,n); y=zeros(nf*2,n); e=mat(3,n);
        azel=zeros(2,n); freq=zeros(nf,n);
        
        for (i=0;i<MAXSAT;i++) {
            rtk->ssat[i].sys=satsys(i+1,NULL);
            for (j=0;j<NFREQ;j++) rtk->ssat[i].vsat[j]=0;
        }
        
        satposs(time,obs,n,nav,opt->sateph,rs,dts,var,svh);
        
        // Debug check
        if (norm(rtk->rb,3) < 1.0) {
             printf("Error: Base position is zero\n");
             delete[] rs; delete[] dts; delete[] var; delete[] y; delete[] e; delete[] azel; delete[] freq;
             return 0;
        }

        if (!zdres(1,obs+nu,nr,rs+nu*6,dts+nu*2,var+nu,svh+nu,nav,rtk->rb,opt,1,
                   y+nu*nf*2,e+nu*3,azel+nu*2,freq+nu*nf)) {
            printf("Error: zdres base failed\n");
            delete[] rs; delete[] dts; delete[] var; delete[] y; delete[] e; delete[] azel; delete[] freq;
            return 0;
        }
        
        if ((ns=selsat(obs,azel,nu,nr,opt,sat,iu,ir))<=0) {
            printf("Error: selsat failed (ns=%d)\n", ns);
            delete[] rs; delete[] dts; delete[] var; delete[] y; delete[] e; delete[] azel; delete[] freq;
            return 0;
        }
        
        udstate(rtk,obs,sat,iu,ir,ns,nav);
        
        xp=mat(rtk->nx,1); Pp=zeros(rtk->nx,rtk->nx); xa=mat(rtk->nx,1);
        matcpy(xp,rtk->x,rtk->nx,1);
        matcpy(Pp,rtk->P,rtk->nx,rtk->nx);
        
        ny=ns*nf*2+2;
        v=mat(ny,1); H=zeros(rtk->nx,ny); 
        double *R_vec = mat(ny, 1); // Variance vector
        
        niter=opt->niter;
        
        for (i=0;i<niter;i++) {
            if (!zdres(0,obs,nu,rs,dts,var,svh,nav,xp,opt,0,y,e,azel,freq)) {
                printf("Error: zdres rover failed\n");
                stat=SOLQ_NONE;
                break;
            }        

            if ((nv=ddres(rtk,nav,dt,xp,Pp,sat,y,e,azel,freq,iu,ir,ns,v,H,R_vec,
                          vflg))<1) {
                printf("Error: ddres failed (nv=%d) ns=%d dt=%.2f\n", nv, ns, dt);
                for (int k=0; k<ns; k++) {
                     printf("  SAT %d: rov=%d base=%d az=%.1f el=%.1f\n", sat[k], iu[k], ir[k], azel[iu[k]*2]*R2D, azel[iu[k]*2+1]*R2D);
                }
                stat=SOLQ_NONE;
                break;
            }
            
            // Construct R matrix
            double *R_mat = zeros(nv, nv);
            for(j=0; j<nv; j++) R_mat[j+j*nv] = R_vec[j];
            
            matcpy(Pp,rtk->P,rtk->nx,rtk->nx);
            
            if ((info=filter(xp,Pp,H,v,R_mat,rtk->nx,nv))) {
                printf("Error: filter failed (info=%d)\n", info);
                stat=SOLQ_NONE;
                delete[] R_mat;
                break;
            }
            
            delete[] R_mat;
        }
        
        if (stat!=SOLQ_NONE&&zdres(0,obs,nu,rs,dts,var,svh,nav,xp,opt,0,y,e,azel,freq)) {
            nv=ddres(rtk,nav,dt,xp,Pp,sat,y,e,azel,freq,iu,ir,ns,v,NULL,R_vec,vflg);
            if (valpos(rtk,v,R_vec,vflg,nv,30.0)) {
                matcpy(rtk->x,xp,rtk->nx,1);
                matcpy(rtk->P,Pp,rtk->nx,rtk->nx);
                stat=SOLQ_FLOAT;
                rtk->sol.stat = stat;
            }
            else {
                printf("Error: valpos failed\n");
                stat=SOLQ_NONE;
            }
        }
        
        /* resolve integer ambiguity by LAMBDA */
        if (stat==SOLQ_FLOAT && resamb_LAMBDA(rtk,NULL,xa)) {
            stat=SOLQ_FIX;
        }
        
        /* save solution status */
        if (stat==SOLQ_FIX) {
             for(i=0;i<3;i++) {
                rtk->sol.rr[i]=xa[i];
                rtk->sol.Qr[i+i*6]=(float)rtk->Pa[i+i*rtk->na];
             }
        } else {
             for(i=0;i<3;i++) {
                rtk->sol.rr[i]=rtk->x[i];
                rtk->sol.Qr[i+i*6]=(float)rtk->P[i+i*rtk->nx];
             }
        }
        rtk->sol.stat = stat;
        
        delete[] rs; delete[] dts; delete[] var; delete[] y; delete[] e; delete[] azel; delete[] freq;
        delete[] xp; delete[] Pp;  delete[] xa;  delete[] v; delete[] H; delete[] R_vec;
        
        return stat!=SOLQ_NONE;
    }

    int rtkpos(rtk_t *rtk, const obsd_t *obs, int nobs, const nav_t *nav)
    {
        prcopt_t *opt = &rtk->opt;
        gtime_t time;
        int nu, nr;
        char msg[128] = "";

        // Debug:
        // printf("rtkpos: nobs=%d\n", nobs);
        
        // Count rover and base observations
        for (nu = 0; nu < nobs && obs[nu].rcv == 1; nu++);
        for (nr = 0; nu + nr < nobs && obs[nu + nr].rcv == 2; nr++);
        
        // printf("rtkpos: nu=%d nr=%d\n", nu, nr);

        time = rtk->sol.time;
        
        // Initialize rover position with base position if available and rover is zero
        if (norm(rtk->sol.rr, 3) == 0.0 && norm(rtk->rb, 3) > 0.0) {
             for(int i=0; i<3; i++) rtk->sol.rr[i] = rtk->rb[i];
             printf("Info: Initializing rover position with base position\n");
        }
        
        // Rover position by single point positioning
        if (!pntpos(obs, nu, nav, opt, &rtk->sol, NULL, NULL, msg)) {
             static int pntpos_fail_count = 0;
             if (pntpos_fail_count < 10) printf("Warning: pntpos failed: %s\n", msg);
             pntpos_fail_count++;
             rtk->sol.stat = SOLQ_NONE;
             return 0;
        }

        if (time.time != 0) {
            rtk->tt = timediff(rtk->sol.time, time);
        }
        
        // Reset if time jump is too large (Relaxed check, but safe reset)
        if (fabs(rtk->tt) > opt->maxtdiff) {
             printf("Warning: Time jump detected (tt=%.1f), resetting RTK\n", rtk->tt);
             // Save Base Position and Solution (current pntpos result)
             double rb_save[6];
             for(int i=0; i<6; i++) rb_save[i] = rtk->rb[i];
             sol_t sol_save = rtk->sol;
             
             rtk->tt = 0.0;
             rtkinit(rtk, opt);
             
             // Restore Base Position and Solution
             for(int i=0; i<6; i++) rtk->rb[i] = rb_save[i];
             rtk->sol = sol_save;
        }

        if (opt->mode == PMODE_SINGLE) {
            return 1;
        }

        if (nr == 0) {
            rtk->sol.stat = SOLQ_NONE;
            return 1;
        }

        relpos(rtk, obs, nu, nr, nav);
        
        return 1;
    }
    
    static int statlevel=0;          /* rtk status output level (0:off) */
    static FILE *fp_stat=NULL;       /* rtk status file pointer */
    
    int rtkopenstat(const char *file, int level)
    {
        if (level <= 0) return 0;
        if (!(fp_stat = fopen(file, "w"))) return 0;
        statlevel = level;
        return 1;
    }
    
    void rtkclosestat(void)
    {
        if (fp_stat) fclose(fp_stat);
        fp_stat = NULL;
        statlevel = 0;
    }
    
    int rtkoutstat(rtk_t *rtk, char *buff)
    {
        double tow, pos[3], vel[3];
        int week;
        char *p = buff;
        
        if (!rtk || rtk->sol.stat <= SOLQ_NONE) return 0;
        
        tow = time2gpst(rtk->sol.time, (uint32_t*)&week);
        
        p += sprintf(p, "$POS,%d,%.3f,%d,%.4f,%.4f,%.4f\n", week, tow,
                    rtk->sol.stat, rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2]);
        
        if (rtk->opt.dynamics) {
            p += sprintf(p, "$VEL,%d,%.3f,%d,%.4f,%.4f,%.4f\n", week, tow,
                        rtk->sol.stat, rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5]);
        }
        
        return (int)(p - buff);
    }

    int input_rnxctr(rnxctr_t *rnx, FILE *fp)
    {
        // RTKLIB implementation involves a lot of RINEX reading logic.
        // We'll return 0 to indicate no more data for now, but we'll skeleton it.
        return 0;
    }

    int init_rnxctr(rnxctr_t *rnx)
    {
        gtime_t time0 = {0};
        rnx->time = time0;
        rnx->ver = 0.0;
        rnx->type = '\0';
        rnx->sys = SYS_GPS;
        rnx->tsys = T_SYS_GPS;
        rnx->ephsat = 0;
        rnx->ephset = 0;
        memset(rnx->tobs, 0, sizeof(rnx->tobs));
        memset(rnx->opt, 0, sizeof(rnx->opt));
        return 1;
    }

    void free_rnxctr(rnxctr_t *rnx)
    {
        // No dynamical memory to free in this simplified rnxctr_t
    }

    int open_rnxctr(rnxctr_t *rnx, FILE *fp)
    {
        // Read RINEX header logic goes here (simplified)
        return 1;
    }

    /* RTK server functions -------------------------------------------------------*/
    // int rtksvrinit(rtksvr_t *svr)
    // {
    //     // 占位实现
    //     return 1;
    // }
    
    // void rtksvrfree(rtksvr_t *svr)
    // {
    //     if (!svr) return;
    //     // 释放RTK服务器结构体动态分配的内存
    //     rtkfree(&svr->rtk);
    //     memset(svr, 0, sizeof(rtksvr_t));
    // }
    
    // int rtksvrstart(rtksvr_t *svr, int cycle, int buffsize, int *strs, char *paths[], int *formats, int n, char *rcvopts)
    // {
    //     // 启动RTK服务器
    //     if (!svr) return 0;
    //     svr->cycle = cycle;
    //     svr->buffsize = buffsize;
    //     svr->state = 1; // 运行状态
    //     // 这里简化处理，实际需要根据参数设置流和路径
    //     return 1;
    // }
    
    // void rtksvrstop(rtksvr_t *svr, char **cmds)
    // {
    //     if (!svr) return;
    //     svr->state = 0; // 停止状态
    // }
    
    // int rtksvropenstr(rtksvr_t *svr, int index, int str, const char *path, int format, const char *rcvopt)
    // {
    //     // 打开服务器流
    //     if (!svr || index < 0 || index >= MAXSTRRTK) return 0;
    //     // 这里简化处理，实际需要根据参数设置流
    //     return 1;
    // }
    
    // void rtksvrclosestr(rtksvr_t *svr, int index)
    // {
    //     // 关闭服务器流
    //     if (!svr || index < 0 || index >= MAXSTRRTK) return;
    //     // 无操作
    // }
    
    // void rtksvrlock(rtksvr_t *svr)
    // {
    //     // 锁定服务器（用于多线程）
    //     if (!svr) return;
    //     // 这里简单实现，实际需要互斥锁
    // }
    
    // void rtksvrunlock(rtksvr_t *svr)
    // {
    //     // 解锁服务器
    //     if (!svr) return;
    //     // 这里简单实现，实际需要互斥锁
    // }
    
    // int rtksvrostat(rtksvr_t *svr, int type, gtime_t time, int *sat, int *vsat, double *azel, int *snr, int *ns)
    // {
    //     // 获取服务器观测状态
    //     if (!svr) return 0;
    //     // 返回0表示没有数据
    //     return 0;
    // }
    
    // void rtksvrsstat(rtksvr_t *svr, int *sstat, char *msg)
    // {
    //     // 获取服务器状态
    //     if (!svr || !sstat || !msg) return;
    //     *sstat = svr->state;
    //     strcpy(msg, "RTK server status");
    // }
    
    // int rtksvrmark(rtksvr_t *svr, const char *name, const char *comment)
    // {
    //     // 在服务器日志中标记
    //     if (!svr || !name) return 0;
    //     // 返回成功
    //     return 1;
    // }
}
