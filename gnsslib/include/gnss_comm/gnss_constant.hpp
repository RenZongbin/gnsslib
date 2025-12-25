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
* As most of the GNSS-related constants are adapted from RTKLIB, 
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

#ifndef GNSS_CONSTANT_HPP_
#define GNSS_CONSTANT_HPP_

#include <vector>
#include <map>
#include <set>
#include <string>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <ctime>

namespace gnss_comm
{
    // some parameters are adapted from RTKLIB
    #define MAXFREQ     7                   /* max N_FREQ */

    #define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */
    #define FREQ2       1.22760E9           /* L2     frequency (Hz) */
    #define FREQ5       1.17645E9           /* L5/E5a frequency (Hz) */
    #define FREQ6       1.27875E9           /* E6/LEX frequency (Hz) */
    #define FREQ7       1.20714E9           /* E5b    frequency (Hz) */
    #define FREQ8       1.191795E9          /* E5a+b  frequency (Hz) */
    #define FREQ9       2.492028E9          /* S      frequency (Hz) */
    #define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz) */
    #define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
    #define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz) */
    #define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
    #define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz) */
    #define FREQ1_BDS   1.561098E9          /* BeiDou B1 frequency (Hz) */
    #define FREQ2_BDS   1.20714E9           /* BeiDou B2 frequency (Hz) */
    #define FREQ3_BDS   1.26852E9           /* BeiDou B3 frequency (Hz) */

    #define SYS_NONE    0x00                /* navigation system: none */
    #define SYS_GPS     0x01                /* navigation system: GPS */
    #define SYS_SBS     0x02                /* navigation system: SBAS */
    #define SYS_GLO     0x04                /* navigation system: GLONASS */
    #define SYS_GAL     0x08                /* navigation system: Galileo */
    #define SYS_QZS     0x10                /* navigation system: QZSS */
    #define SYS_BDS     0x20                /* navigation system: BeiDou */
    #define SYS_IRN     0x40                /* navigation system: IRNSS */
    #define SYS_LEO     0x80                /* navigation system: LEO */
    #define SYS_ALL     0xFF                /* navigation system: all */

    #define T_SYS_GPS       0               /* time system: GPS time */
    #define T_SYS_UTC       1               /* time system: UTC */
    #define T_SYS_GLO       2               /* time system: GLONASS time */
    #define T_SYS_GAL       3               /* time system: Galileo time */
    #define T_SYS_QZS       4               /* time system: QZSS time */
    #define T_SYS_BDS       5               /* time system: BeiDou time */

    #ifndef N_FREQ
    #define N_FREQ          3                   /* number of carrier frequencies */
    #endif
    #define N_FREQ_GLO      2                   /* number of carrier frequencies of GLONASS */

    #define MIN_PRN_GPS     1                   /* min satellite PRN number of GPS */
    #define MAX_PRN_GPS     32                  /* max satellite PRN number of GPS */
    #define N_SAT_GPS       (MAX_PRN_GPS-MIN_PRN_GPS+1) /* number of GPS satellites */
    #define N_SYS_GPS       1

    // #define MIN_PRN_GLO     38                   /* min satellite slot number of GLONASS */ For UM4B0
    // #define MAX_PRN_GLO     61                  /* max satellite slot number of GLONASS */  For UM4B0
    #define MIN_PRN_GLO     1                   /* min satellite slot number of GLONASS */
    #define MAX_PRN_GLO     27                  /* max satellite slot number of GLONASS */
    #define N_SAT_GLO       (MAX_PRN_GLO-MIN_PRN_GLO+1) /* number of GLONASS satellites */
    #define N_SYS_GLO       1

    #define MIN_PRN_GAL     1                   /* min satellite PRN number of Galileo */
    #define MAX_PRN_GAL     38                  /* max satellite PRN number of Galileo */
    #define N_SAT_GAL       (MAX_PRN_GAL-MIN_PRN_GAL+1) /* number of Galileo satellites */
    #define N_SYS_GAL       1

    #define MIN_PRN_BDS     1                   /* min satellite sat number of BeiDou */
    #define MAX_PRN_BDS     63                  /* max satellite sat number of BeiDou */
    #define N_SAT_BDS       (MAX_PRN_BDS-MIN_PRN_BDS+1) /* number of BeiDou satellites */
    #define N_SYS_BDS       1

    #define N_SYS           (N_SYS_GPS+N_SYS_GLO+N_SYS_GAL+N_SYS_BDS) /* number of systems */

    #define MAX_SAT         (N_SAT_GPS+N_SAT_GLO+N_SAT_GAL+N_SAT_BDS)

    #ifndef MAXOBS
    #define MAXOBS      128                 /* max number of obs in an epoch */
    #endif
    #define MAXSTRRTK   8                   /* max number of streams in RTK server */

    // solution type
    // #define SOL_COMPUTED            0
    // #define SOL_INSUFFICIENT_OBS    1
    // #define NO_CONVERGENCE          2
    // #define COV_TRACE               4

    // extend solution type
    // #define EXT_NONE                0x00
    // #define EXT_RTK                 0x01
    // #define EXT_IONO_BRDC           0x02
    // #define EXT_IONO_SBAS           0x04
    // #define EXT_IONO_IFLC           0x06
    // #define EXT_IONO_DIFF           0x08

    // position(velocity) type
    #define PV_NONE                     0               /* no solution */
    #define PV_FIXED_POS                1
    #define PV_FIXED_HEIGHT             2
    #define PV_DOPPLER_VELOCITY         8
    #define PV_SINGLE                   16
    #define PV_PSR_DIFF                 17
    #define PV_WASS                     18
    #define PV_L1_FLOAT                 32
    #define PV_IONO_FREE_FLOAT          33
    #define PV_NARROW_FLOW              34
    #define PV_L1_INT                   48
    #define PV_WIDE_INT                 49
    #define PV_NARROW_INT               50
    #define PV_INS                      52
    #define PV_INS_PSR_SP               53
    #define PV_INS_PSR_DIFF             54
    #define PV_INS_RTK_FLOAT            55
    #define PV_INS_RTK_FIXED            56

    #define CODE_NONE   0                   /* obs code: none or unknown */
    #define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
    #define CODE_L1P    2                   /* obs code: L1P,G1P    (GPS,GLO) */
    #define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
    #define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
    #define CODE_L1M    5                   /* obs code: L1M        (GPS) */
    #define CODE_L1N    6                   /* obs code: L1codeless (GPS) */
    #define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
    #define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
    #define CODE_L1E    9                   /* obs code: L1-SAIF    (QZS) */
    #define CODE_L1A    10                  /* obs code: E1A        (GAL) */
    #define CODE_L1B    11                  /* obs code: E1B        (GAL) */
    #define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P) (GAL,QZS) */
    #define CODE_L1Z    13                  /* obs code: E1A+B+C,L1SAIF (GAL,QZS) */
    #define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
    #define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
    #define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
    #define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
    #define CODE_L2X    18                  /* obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) */
    #define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
    #define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
    #define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
    #define CODE_L2M    22                  /* obs code: L2M        (GPS) */
    #define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
    #define CODE_L5I    24                  /* obs code: L5/E5aI    (GPS,GAL,QZS,SBS) */
    #define CODE_L5Q    25                  /* obs code: L5/E5aQ    (GPS,GAL,QZS,SBS) */
    #define CODE_L5X    26                  /* obs code: L5/E5aI+Q  (GPS,GAL,QZS,SBS) */
    #define CODE_L7I    27                  /* obs code: E5bI,B2I   (GAL,CMP) */
    #define CODE_L7Q    28                  /* obs code: E5bQ,B2Q   (GAL,CMP) */
    #define CODE_L7X    29                  /* obs code: E5bI+Q,B2I+Q (GAL,CMP) */
    #define CODE_L6A    30                  /* obs code: E6A        (GAL) */
    #define CODE_L6B    31                  /* obs code: E6B        (GAL) */
    #define CODE_L6C    32                  /* obs code: E6C        (GAL) */
    #define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) */
    #define CODE_L6Z    34                  /* obs code: E6A+B+C    (GAL) */
    #define CODE_L6S    35                  /* obs code: LEXS       (QZS) */
    #define CODE_L6L    36                  /* obs code: LEXL       (QZS) */
    #define CODE_L8I    37                  /* obs code: E5(a+b)I   (GAL) */
    #define CODE_L8Q    38                  /* obs code: E5(a+b)Q   (GAL) */
    #define CODE_L8X    39                  /* obs code: E5(a+b)I+Q (GAL) */
    #define CODE_L2I    40                  /* obs code: B1I        (CMP) */
    #define CODE_L2Q    41                  /* obs code: B1Q        (CMP) */
    #define CODE_L6I    42                  /* obs code: B3I        (CMP) */
    #define CODE_L6Q    43                  /* obs code: B3Q        (CMP) */
    #define CODE_L3I    44                  /* obs code: G3I        (GLO) */
    #define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
    #define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
    #define CODE_L1I    47                  /* obs code: B1I        (BDS) */
    #define CODE_L1Q    48                  /* obs code: B1Q        (BDS) */
    #define CODE_L9I    49                  /* obs code: L9I        (IRS) */
    #define CODE_L9Q    50                  /* obs code: L9Q        (IRS) */
    #define CODE_L9X    51                  /* obs code: L9X        (IRS) */
    #define CODE_L1D    56                  /* obs code: B1D        (BDS) */
    #define CODE_L5D    57                  /* obs code: L5D(L5S),B2aD (QZS,BDS) */
    #define CODE_L5P    58                  /* obs code: L5P(L5S),B2aP (QZS,BDS) */
    #define CODE_L5Z    59                  /* obs code: L5D+P(L5S) (QZS) */
    #define CODE_L6E    60                  /* obs code: L6E        (QZS) */
    #define CODE_L7D    61                  /* obs code: B2bD       (BDS) */
    #define CODE_L7P    62                  /* obs code: B2bP       (BDS) */
    #define CODE_L7Z    63                  /* obs code: B2bD+P     (BDS) */
    #define CODE_L8D    64                  /* obs code: B2abD      (BDS) */
    #define CODE_L8P    65                  /* obs code: B2abP      (BDS) */
    #define CODE_L4A    66                  /* obs code: G1aL1OCd   (GLO) */
    #define CODE_L4B    67                  /* obs code: G1aL1OCd   (GLO) */
    #define CODE_L4X    68                  /* obs code: G1al1OCd+p (GLO) */
    #ifndef MAXCODE
    #define MAXCODE     48                  /* max number of obs code */
    #endif

    #define MAXRCV      64                  /* max receiver number (1 to MAXRCV) */
    #define MAXSAT      (N_SAT_GPS+N_SAT_GLO+N_SAT_GAL+N_SAT_BDS) /* max satellite number */
    #define MAXANT      64                  /* max length of station name/antenna type */

    #define EARTH_ECCE_2            6.69437999014e-3    // WGS 84 (Earth eccentricity)^2 (m^2)
    #define EARTH_MEAN_RADIUS       6371009             // Mean R of ellipsoid(m) IU Gedosey& Geophysics
    #define EARTH_SEMI_MAJOR        6378137             // WGS 84 Earth semi-major axis (m)
    #define EARTH_SEMI_MAJOR_GLO    6378136.0           // radius of earth (m)
    #define EARTH_OMG_GLO           7.2921150000e-5     // GLO value of earth's rotation rate (rad/s)
    #define EARTH_OMG_GPS           7.2921151467e-5     // GPS/GAL value of earth's rotation rate (rad/s)
    #define EARTH_OMG_BDS           7.2921150000e-5     // BDS value of earth's rotation rate (rad/s)
    #define MU_GPS                  3.9860050000e14     // gravitational constant (GPS)      
    #define MU                      3.9860044180e14     // gravitational constant (GAL, BDS, GLO)    
    #define TSTEP                   60.0                // integration step glonass ephemeris (s)
    #define J2_GLO                  1.0826257E-3        // 2nd zonal harmonic of geopot
    #define LIGHT_SPEED             2.99792458e8        // WGS-84 Speed of light in a vacuum (m/s)
    #define GPS_EPHCH_JD            2444244.5           // GPS Epoch in Julian Days
    #define EPH_VALID_SECONDS       7200                // 2 hours ephemeris validity
    #define WEEK_SECONDS            604800              // Seconds within one week
    #define EPSILON_KEPLER          1e-14               // Kepler equation terminate condition
    #define MAX_ITER_KEPLER         30                  // Kepler equation maximum iteration number
    #define EPSILON_PVT             1e-8                // PVT terminate condition
    #define MAX_ITER_PVT            30                  // PVT maximum iteration number
    #define RANGE_FREQ              1                   // Range measurement frequency
    #ifndef R2D
    #define R2D                     (180.0/M_PI)        // radius to degree
    #endif
    #ifndef D2R
    #define D2R                     (M_PI/180.0)        // degree to radius
    #endif
    #define SC2RAD                  3.1415926535898     /* semi-circle to radian (IS-GPS) */
    #define SIN_N5                  -0.0871557427476582 // sin(-5.0 deg)
    #define COS_N5                   0.9961946980917456 // cos(-5.0 deg)

    #define INVALID_CLK             999999.999999

    #define MAXLEAPS                64

    // Define RTKLIB constants
    #ifndef NFREQ
    #define NFREQ 3                                    /* number of carrier frequencies */
    // #define N_FREQ 2                                   /* number of carrier frequencies of GLONASS */
    #endif
    #ifndef NEXOBS
    #define NEXOBS 0                                   /* number of extended obs codes */
    #endif
    const double leaps[MAXLEAPS+1][7]={ /* leap seconds (y,m,d,h,m,s,utc-gpst) */
        {2017,1,1,0,0,0,-18},
        {2015,7,1,0,0,0,-17},
        {2012,7,1,0,0,0,-16},
        {2009,1,1,0,0,0,-15},
        {2006,1,1,0,0,0,-14},
        {1999,1,1,0,0,0,-13},
        {1997,7,1,0,0,0,-12},
        {1996,1,1,0,0,0,-11},
        {1994,7,1,0,0,0,-10},
        {1993,7,1,0,0,0, -9},
        {1992,7,1,0,0,0, -8},
        {1991,1,1,0,0,0, -7},
        {1990,1,1,0,0,0, -6},
        {1988,1,1,0,0,0, -5},
        {1985,7,1,0,0,0, -4},
        {1983,7,1,0,0,0, -3},
        {1982,7,1,0,0,0, -2},
        {1981,7,1,0,0,0, -1},
        {0}
    };

    /* System identifier to system code */
    const std::map<uint8_t, uint32_t> char2sys = 
    {
        {'G', SYS_GPS},
        {'C', SYS_BDS},
        {'R', SYS_GLO},
        {'E', SYS_GAL}
    };

    /* System map to index */
    const std::map<uint32_t, uint32_t> sys2idx = 
    {
        {SYS_GPS, 0},
        {SYS_GLO, 1},
        {SYS_GAL, 2},
        {SYS_BDS, 3}
    };

    /* RINEX frequency encoding to frequency value */
    const std::map<std::string, double> type2freq = 
    {
        {"G1", FREQ1},
        {"G2", FREQ2},
        {"G5", FREQ5},
        {"R1", FREQ1_GLO},
        {"R2", FREQ2_GLO},
        {"R3", 1.202025E9},
        {"R4", 1.600995E9},
        {"R6", 1.248060E9},
        {"E1", FREQ1},
        {"E5", FREQ5},
        {"E6", FREQ6},
        {"E7", FREQ7},
        {"E8", FREQ8},
        {"C1", FREQ1},
        {"C2", FREQ1_BDS},
        {"C5", FREQ5},
        {"C6", FREQ3_BDS},
        {"C7", FREQ2_BDS},
        {"C8", FREQ8}
    };

    // gtime_t definition (replacing RTKLIB's gtime_t)
    struct gtime_t
    {
        time_t time;            /* time (s) expressed by standard time_t */
        double sec;             /* fraction of second under 1 s */
    };

    struct EphemBase
    {
        virtual ~EphemBase() = default;
        uint32_t sat;                   /* satellite number */
        gtime_t  ttr;                   /* transmission time in GPST */
        gtime_t  toe;                   /* ephemeris reference time in GPST */
        gtime_t  toc;
        uint32_t health;                /* satellite health */
        double   ura;                   /* satellite signal accuracy */
        uint32_t iode;
    };
    typedef std::shared_ptr<EphemBase> EphemBasePtr;

    struct GloEphem : EphemBase
    {
        int         freqo;              /* satellite frequency number */
        uint32_t    age;                /* satellite age */
        double      pos[3];             /* satellite position (ecef) (m) */
        double      vel[3];             /* satellite velocity (ecef) (m/s) */
        double      acc[3];             /* satellite acceleration (ecef) (m/s^2) */
        double      tau_n, gamma;       /* SV clock bias (s)/relative freq bias */
        double      delta_tau_n;        /* delay between L1 and L2 (s) */
    };
    typedef std::shared_ptr<GloEphem> GloEphemPtr;

    struct SEphem
    {        /* SBAS ephemeris type */
        int sat;            /* satellite number */
        gtime_t t0;         /* reference epoch time (GPST) */
        gtime_t tof;        /* time of message frame (GPST) */
        int sva;            /* SV accuracy (URA index) */
        int svh;            /* SV health (0:ok) */
        double pos[3];      /* satellite position (m) (ecef) */
        double vel[3];      /* satellite velocity (m/s) (ecef) */
        double acc[3];      /* satellite acceleration (m/s^2) (ecef) */
        double af0,af1;     /* satellite clock-offset/drift (s,s/s) */
    };
    typedef std::shared_ptr<SEphem> SEphemPtr;

    /* precise ephemeris type */
    struct PEphem
    {
        gtime_t time;       /* time (GPST) */
        int index;          /* ephemeris index for multiple files */
        double pos[MAXSAT][4]; /* satellite position/clock (ecef) (m|s) */
        float  std[MAXSAT][4]; /* satellite position/clock std (m|s) */
        double vel[MAXSAT][4]; /* satellite velocity/clk-rate (m/s|s/s) */
        float  vst[MAXSAT][4]; /* satellite velocity/clk-rate std (m/s|s/s) */
        float  cov[MAXSAT][3]; /* satellite position covariance (m^2) */
        float  vco[MAXSAT][3]; /* satellite velocity covariance (m^2) */
    };
    typedef std::shared_ptr<PEphem> PEphemPtr;

    /* almanac type */
    struct Alm
    {
        int sat;            /* satellite number */
        int svh;            /* sv health (0:ok) */
        int svconf;         /* as and sv config */
        int week;           /* GPS/QZS: gps week, GAL: galileo week */
        gtime_t toa;        /* Toa */
                            /* SV orbit parameters */
        double A,e,i0,OMG0,omg,M0,OMGd;
        double toas;        /* Toa (s) in week */
        double f0,f1;       /* SV clock parameters (af0,af1) */
    };
    typedef std::shared_ptr<Alm> AlmPtr;

    /* TEC grid type */
    struct TEC
    {
        gtime_t time;       /* epoch time (GPST) */
        int ndata[3];       /* TEC grid data size {nlat,nlon,nhgt} */
        double rb;          /* earth radius (km) */
        double lats[3];     /* latitude start/interval (deg) */
        double lons[3];     /* longitude start/interval (deg) */
        double hgts[3];     /* heights start/interval (km) */
        std::vector<double> data; /* TEC grid data (tecu) */
        std::vector<float> rms;   /* RMS values (tecu) */
    };
    typedef std::shared_ptr<TEC> TECPtr;

    struct Ephem : EphemBase
    {
        gtime_t     toc;                    /* clock correction reference time in GPST */
        double      toe_tow;                /* toe seconds within the week */
        uint32_t    week;
        uint32_t    iodc;
        uint32_t    code;
        double      A, e, i0, omg, OMG0, M0, delta_n, OMG_dot, i_dot;       /* SV orbit parameters */
        double      cuc, cus, crc, crs, cic, cis;
        double      af0, af1, af2;          /* SV clock parameters */
        double      tgd[2];                 /* group delay parameters */
                                            /* GPS    :tgd[0]=TGD */
                                            /* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
                                            /* BDS    :tgd[0]=BGD1,tgd[1]=BGD2 */
        double A_dot, n_dot;                /* Adot,ndot for CNAV */
    };
    typedef std::shared_ptr<Ephem> EphemPtr;

    struct Obs                                           /* observation data record */
    {
        gtime_t     time;                                /* receiver sampling time (GPST) */
        uint32_t    sat;                                 /* satellite number */
        std::vector<double> freqs;                       /* received satellite frequencies */
        std::vector<double> CN0;                         /* signal strength */
        std::vector<uint8_t> LLI;                        /* signal lost-lock indictor */
        std::vector<uint8_t> code;                       /* code indicator (CODE_???) */
        std::vector<double> psr;                         /* observation data pseudorange (m) */
        std::vector<double> psr_std;                     /* pseudorange std (m) */
        std::vector<double> cp;                          /* observation data carrier-phase (cycle) */
        std::vector<double> cp_std;                      /* carrier-phase std (cycle) */
        std::vector<double> dopp;                        /* observation data doppler frequency (Hz) */
        std::vector<double> dopp_std;                    /* doppler std (Hz) */
        std::vector<uint8_t> status;                     /* cycle slip valid flag. bit_0 (psr valid), bit_1(cp valid), bit_2(half cp valid, *ublox), bit_3(half cp subtracted, *ublox) */
    };
    typedef std::shared_ptr<Obs> ObsPtr;

    struct TimePulseInfo                /* time pulse(PPS) information */
    {
        gtime_t time;
        bool utc_based;
        uint32_t time_sys;
    };
    typedef std::shared_ptr<TimePulseInfo> TimePulseInfoPtr;

    struct BestSat
    {
        gtime_t     time;
        uint32_t    sat;
        uint8_t     status;
        std::set<double> freq_used;      /* frequency used in solution */
    };
    typedef std::shared_ptr<BestSat> BestSatPtr;

    struct BestPos
    {
        gtime_t     time;
        uint32_t    sol_status;
        uint32_t    pos_type;
        double      lat, lon, hgt;
        double      lat_sigma, lon_sigma, hgt_sigma;
        double      undulation;
        double      diff_age;
        double      sol_age;
        uint8_t     num_svs;
        uint8_t     num_soln_svs;
        uint8_t     ext_sol_stat;
        uint8_t     sig_mask;
    };
    typedef std::shared_ptr<BestPos> BestPosPtr;

    struct BestVel
    {
        gtime_t     time;
        uint32_t    sol_status;
        uint32_t    vel_type;
        double      latency;
        double      age;
        double      hor_speed, vel_speed;
        double      direction2north_degree;
    };
    typedef std::shared_ptr<BestVel> BestVelPtr;

    struct BestXYZ
    {
        gtime_t     time;
        uint32_t    pos_sol_status;
        uint32_t    pos_type;
        double      pos[3], pos_sigma[3];
        uint32_t    vel_sol_status;
        uint32_t    vel_type;
        double      vel[3], vel_sigma[3];
        double      vel_latency;
        double      diff_age;
        double      sol_age;
        uint8_t     num_svs;
        uint8_t     num_soln_svs;
        uint8_t     ext_sol_stat;
        uint8_t     sig_mask;
    };
    typedef std::shared_ptr<BestXYZ> BestXYZPtr;

    struct PVTSolution
    {
        gtime_t time;
        uint8_t fix_type;   // 0-no fix, 1-dead reckoning only, 2-2D, 3-3D, 4-GNSS+(1), 5-time only fix 
        bool valid_fix;
        bool diff_soln;     // whether differential applied
        uint8_t carr_soln;  // 0-no cp solution, 1-float, 2-fixed
        uint8_t num_sv;
        double lat, lon, hgt;
        double hgt_msl;     // height above mean sea level
        double h_acc, v_acc; // horizontal and vertical accuracy
        double p_dop;       // position DOP
        double vel_n, vel_e, vel_d; // velocity in NED frame
        double vel_acc;         // velocity accuracy
    };
    typedef std::shared_ptr<PVTSolution> PVTSolutionPtr;

    struct PsrPos
    {
        gtime_t     time;
        uint32_t    sol_status;
        uint32_t    pos_type;
        double      lat, lon, hgt;
        double      lat_sigma, lon_sigma, hgt_sigma;
        double      undulation;
        double      diff_age;
        double      sol_age;
        uint8_t     num_svs;
        uint8_t     num_soln_svs;
        uint8_t     ext_sol_stat;
        uint8_t     sig_mask;
    };
    typedef std::shared_ptr<PsrPos> PsrPosPtr;

    struct PsrVel
    {
        gtime_t     time;
        uint32_t    sol_status;
        uint32_t    vel_type;
        double      latency;
        double      age;                /* differential age */
        double      hor_spd;
        double      trk_gnd;
        double      vert_spd;
    };
    typedef std::shared_ptr<PsrVel> PsrVelPtr;

    struct SvInfo
    {
        gtime_t     time;
        uint32_t    sat;
        uint32_t    freqo;
        bool        health;
        double      elev_degree;
        double      az_degree;
        uint32_t    sig_mask;
    };
    typedef std::shared_ptr<SvInfo> SvInfoPtr;

    struct TECEpoch
    {
        gtime_t time;       // epoch time
        double rb;          // earth radius (km)
        double lla_start[3];
        double lla_end[3];
        double lla_interval[3];
        std::vector<double> data;  // num_hgt * num_lat * num_lon
        std::vector<double> rms;   // num_hgt * num_lat * num_lon
    };
    typedef std::shared_ptr<TECEpoch> TECEpochPtr;

    struct EphemerideEpoch
    {
        gtime_t time;
        std::map<uint32_t, Eigen::Vector3d> sat2pos;
        std::map<uint32_t, double> sat2clk;
    };
    typedef std::shared_ptr<EphemerideEpoch> EphemerideEpochPtr;

    struct SatState
    {
        uint32_t sat_id;
        gtime_t  ttx;
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        double dt;
        double ddt;
        double tgd;
    };
    typedef std::shared_ptr<SatState> SatStatePtr;

    // RINEX constants
    #define MAXOBSTYPE  64                  /* max number of obs type in RINEX */
    #define MAXCOMMENT  100                 /* max number of RINEX comments */
    #define RNX2VER     2.10                /* RINEX ver.2 default output version */
    #define RNX3VER     3.00                /* RINEX ver.3 default output version */

    // Stream formats
    #define STRFMT_RTCM2 0                  /* stream format: RTCM 2 */
    #define STRFMT_RTCM3 1                  /* stream format: RTCM 3 */
    #define STRFMT_RINEX 13                 /* stream format: RINEX */
    #define STRFMT_RNXCLK 15                /* stream format: RINEX CLK */

    // Positioning modes
    #define PMODE_SINGLE 0                  /* positioning mode: single */
    #define PMODE_DGPS   1                  /* positioning mode: DGPS/DGNSS */
    #define PMODE_KINEMA 2                  /* positioning mode: kinematic */
    #define PMODE_STATIC 3                  /* positioning mode: static */
    #define PMODE_MOVEB  4                  /* positioning mode: moving-base */
    #define PMODE_FIXED  5                  /* positioning mode: fixed */
    #define PMODE_PPP_KINEMA 6              /* positioning mode: PPP-kinemaric */
    #define PMODE_PPP_STATIC 7              /* positioning mode: PPP-static */
    #define PMODE_PPP_FIXED 8               /* positioning mode: PPP-fixed */

    // Ambiguity resolution modes
    #define ARMODE_OFF  0                   /* AR mode: off */
    #define ARMODE_CONT 1                   /* AR mode: continuous */
    #define ARMODE_INST 2                   /* AR mode: instantaneous */
    #define ARMODE_FIXHOLD 3                /* AR mode: fix and hold */

    // Ionosphere options
    #define IONOOPT_OFF 0                   /* ionosphere option: correction off */
    #define IONOOPT_BRDC 1                  /* ionosphere option: broadcast model */
    #define IONOOPT_SBAS 2                  /* ionosphere option: SBAS model */
    #define IONOOPT_IFLC 3                  /* ionosphere option: L1/L2 iono-free LC */
    #define IONOOPT_EST 4                   /* ionosphere option: estimation */
    #define IONOOPT_TEC 5                   /* ionosphere option: IONEX TEC model */
    #define IONOOPT_QZS 6                   /* ionosphere option: QZSS broadcast model */

    // Troposphere options
    #define TROPOPT_OFF 0                   /* troposphere option: correction off */
    #define TROPOPT_SAAS 1                  /* troposphere option: Saastamoinen model */
    #define TROPOPT_SBAS 2                  /* troposphere option: SBAS model */
    #define TROPOPT_EST 3                   /* troposphere option: ZTD estimation */
    #define TROPOPT_ESTG 4                  /* troposphere option: ZTD+grad estimation */
    #define TROPOPT_ZTD 5                   /* troposphere option: ZTD correction */

    // Ephemeris options
    #define EPHOPT_BRDC 0                   /* ephemeris option: broadcast ephemeris */
    #define EPHOPT_PREC 1                   /* ephemeris option: precise ephemeris */
    #define EPHOPT_SBAS 2                   /* ephemeris option: broadcast + SBAS */
    #define EPHOPT_SSRAPC 3                 /* ephemeris option: broadcast + SSR_APC */
    #define EPHOPT_SSRCOM 4                 /* ephemeris option: broadcast + SSR_COM */

    // Solution status
    #define SOLQ_NONE   0                   /* solution status: no solution */
    #define SOLQ_FIX    1                   /* solution status: fix */
    #define SOLQ_FLOAT  2                   /* solution status: float */
    #define SOLQ_SBAS   3                   /* solution status: SBAS */
    #define SOLQ_DGPS   4                   /* solution status: DGPS/DGNSS */
    #define SOLQ_SINGLE 5                   /* solution status: single */
    #define SOLQ_PPP    6                   /* solution status: PPP */

    struct snrmask_t {
        double mask[2][NFREQ];      /* SNR mask (0:rover,1:base) */
    };
    // Simplified navigation data container
    /* antenna parameter type */
    struct pcv_t
    {
        int sat;            /* satellite number (0:receiver) */
        char type[MAXANT];  /* antenna type */
        char code[MAXANT];  /* serial number or satellite code */
        gtime_t ts,te;      /* valid time start and end */
        double off[N_FREQ][3]; /* phase center offset e/n/u or x/y/z (m) */
        double var[N_FREQ][19]; /* phase center variation (m) */
                              /* el=90,85,...,0 or nadir=0,1,2,3,... (deg) */
    };
    // Simplified processing options structure
    struct PrcOpt
    {
        int mode;           /* positioning mode (PMODE_???) */
        int soltype;        /* solution type (0:forward,1:backward,2:combined) */
        int nf;             /* number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
        int navsys;         /* navigation system */
        double elmin;       /* elevation mask angle (rad) */
        snrmask_t snrmask;  /* SNR mask */
        int sateph;         /* satellite ephemeris/clock (EPHOPT_???) */
        int modear;         /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
        int glomodear;      /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
        int bdsmodear;      /* BeiDou AR mode (0:off,1:on) */
        int maxout;         /* obs outage count to reset bias */
        int minlock;        /* min lock count to fix ambiguity */
        int minfix;         /* min fix count to hold ambiguity */
        int armaxiter;      /* max iteration to resolve ambiguity */
        int ionoopt;        /* ionosphere option (IONOOPT_???) */
        int tropopt;        /* troposphere option (TROPOPT_???) */
        int dynamics;       /* dynamics model (0:none,1:velociy,2:accel) */
        int tidecorr;       /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
        int niter;          /* number of filter iteration */
        int codesmooth;     /* code smoothing window size (0:none) */
        int intpref;        /* interpolate reference obs (for post mission) */
        int sbascorr;       /* SBAS correction options */
        int sbassatsel;     /* SBAS satellite selection (0:all) */
        int rovpos;         /* rover position for fixed mode */
        int refpos;         /* base position for relative mode */
                            /* (0:pos in prcopt,  1:average of single pos, */
                            /*  2:read from file, 3:rinex header, 4:rtcm pos) */
        double eratio[NFREQ]; /* code/phase error ratio */
        double err[5];      /* measurement error factor */
                            /* [0]:reserved */
                            /* [1-3]:error factor a/b/c of phase (m) */
                            /* [4]:doppler frequency (hz) */
        double std[3];      /* initial-state std [0]bias,[1]iono [2]trop */
        double prn[6];      /* process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
        double sclkstab;    /* satellite clock stability (sec/sec) */
        double thresar[8];  /* AR validation threshold */
        double elmaskar;    /* elevation mask of AR for rising satellite (deg) */
        double elmaskhold;  /* elevation mask to hold ambiguity (deg) */
        double thresslip;   /* slip threshold of geometry-free phase (m) */
        double maxtdiff;    /* max difference of time (sec) */
        double maxinno;     /* reject threshold of innovation (m) */
        double maxgdop;     /* reject threshold of gdop */
        double baseline[2]; /* baseline length constraint {const,sigma} (m) */
        double ru[3];       /* rover position for fixed mode {x,y,z} (ecef) (m) */
        double rb[3];       /* base position for relative mode {x,y,z} (ecef) (m) */
        char anttype[2][MAXANT]; /* antenna types {rover,base} */
        double antdel[2][3]; /* antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
        pcv_t pcvr[2];      /* receiver antenna parameters {rov,base} */
        uint8_t exsats[MAXSAT]; /* excluded satellites (1:excluded,2:included) */
        int  maxaveep;      /* max averaging epoches */
        int  initrst;       /* initialize by restart */
        int  outsingle;     /* output single by dgps/float/fix/ppp outage */
        char rnxopt[2][256]; /* rinex options {rover,base} */
        int  posopt[6];     /* positioning options */
        int  syncsol;       /* solution sync mode (0:off,1:on) */
        double odisp[2][6*11]; /* ocean tide loading parameters {rov,base} */
        int  freqopt;       /* disable L2-AR */
        char pppopt[256];   /* ppp option */
    };
    typedef std::shared_ptr<PrcOpt> PrcOptPtr;

    struct NavData
    {
        int n,nmax;         /* number of broadcast ephemeris */
        int ng,ngmax;       /* number of glonass ephemeris */
        //int ns,nsmax;       /* number of sbas ephemeris */
        int ne,nemax;       /* number of precise ephemeris */
        int nc,ncmax;       /* number of precise clock */
        int na,namax;       /* number of almanac data */
        int nt,ntmax;       /* number of tec grid data */
        std::vector<EphemPtr> ephems;           /* GPS/QZS/GAL/BDS ephemeris */
        std::vector<GloEphemPtr> glo_ephems;    /* GLONASS ephemeris */
        std::map<uint32_t, double> sat_clk;     /* satellite clock corrections */
        std::map<uint32_t, Eigen::Vector3d> sat_pos; /* satellite positions */
        std::vector<SEphemPtr> seph;       /* SBAS ephemeris */
        std::vector<PEphemPtr> peph;       /* precise ephemeris */
        std::map<uint32_t, double> pclk;       /* precise clock */
        std::vector<AlmPtr> alm;        /* almanac data */
        std::vector<TECPtr> tec;         /* tec grid data */
        std::map<uint32_t, double> erp;         /* earth rotation parameters */
        double iono_params[8];                  /* ionospheric parameters */
        double utc_gps[8];                      /* GPS UTC parameters [0]=A0, [1]=A1, [2]=T, [3]=W, [4]=UTC ref week, [5]=UTC ref tow, [6]=delta tLS, [7]=delta tLSF */
        double utc_glo[8];  /* GLONASS UTC time parameters {tau_C,tau_GPS} */
        double utc_gal[8];  /* Galileo UTC parameters */
        double utc_qzs[8];  /* QZS UTC parameters */
        double utc_cmp[8];  /* BeiDou UTC parameters */
        double utc_irn[9];  /* IRNSS UTC parameters {A0,A1,Tot,...,dt_LSF,A2} */
        double utc_sbs[4];  /* SBAS UTC parameters */
        double ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        double ion_gal[4];  /* Galileo iono model parameters {ai0,ai1,ai2,0} */
        double ion_qzs[8];  /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        double ion_cmp[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        double ion_irn[8];  /* IRNSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        int glo_fcn[32];    /* GLONASS FCN + 8, index 0-31 correspond to PRN 1-32 */
        double cbias[MAXSAT][3]; /* satellite DCB (0:P1-P2,1:P1-C1,2:P2-C2) (m) */
        double rbias[MAXRCV][2][3]; /* receiver DCB (0:P1-P2,1:P1-C1,2:P2-C2) (m) */
        pcv_t pcvs[MAXSAT]; /* satellite antenna pcv */
        //sbssat_t sbssat;    /* SBAS satellite corrections */
        //sbsion_t sbsion[MAXBAND+1]; /* SBAS ionosphere corrections */
        //dgps_t dgps[MAXSAT]; /* DGPS corrections */
        //ssr_t ssr[MAXSAT];  /* SSR corrections */
    };
    typedef std::shared_ptr<NavData> NavDataPtr;

    // Station information structure
    struct StaInfo
    {
        std::string name;           /* marker name */
        std::string marker;         /* marker number */
        std::string antdes;         /* antenna descriptor */
        std::string antsno;         /* antenna serial number */
        std::string rectype;        /* receiver type descriptor */
        std::string recver;         /* receiver firmware version */
        std::string recsno;         /* receiver serial number */
        double pos[3];              /* station position (ecef) (m) */
        double del[3];              /* antenna delta (e/n/u) (m) */
        gtime_t time;               /* measurement time */
        double glo_cp_bias[4];      /* GLONASS code-phase biases {C1-P1,C1-P2,C2-P1,C2-P2} */
    };
    typedef std::shared_ptr<StaInfo> StaInfoPtr;

    // Observation data collection
    struct ObsData
    {
        std::vector<ObsPtr> data;   /* observation data records */
        gtime_t start_time;         /* start time of observations */
        gtime_t end_time;           /* end time of observations */
        int num_obs;                /* number of observations */
    };
    typedef std::shared_ptr<ObsData> ObsDataPtr;

    // General solution structure
    struct Solution
    {
        gtime_t time;               /* time (GPST) */
        uint8_t stat;               /* solution status (SOLQ_???) */
        uint8_t ns;                 /* number of valid satellites */
        double pos[3];              /* position (ecef) (m) */
        double vel[3];              /* velocity (ecef) (m/s) */
        float pos_var[3];           /* position variance (m^2) */
        float vel_var[3];           /* velocity variance (m^2/s^2) */
        double dtr[6];              /* receiver clock bias (s) */
    };
    typedef std::shared_ptr<Solution> SolutionPtr;

    // RTKLIB compatible structures
    struct sol_t {
        gtime_t time;               /* time (GPST) */
        uint8_t stat;               /* solution status (SOLQ_???) */
        double ratio;               /* ratio factor for AR validation */
        double thres;               /* threshold for AR validation */
        int ns;                     /* number of valid satellites */
        double rr[6];               /* position/velocity (ecef) (m|m/s) */
        double dtr[6];              /* receiver clock bias (s) */
        float Qr[36];              /* position/velocity covariance (m^2) */
        double age;                 /* age of differential (s) */
        // Add other fields as needed
    };
    
    // RTKLIB observation data record (compatible with RTKLIB)
    struct obsd_t {
        gtime_t time;               /* receiver sampling time (GPST) */
        uint8_t sat, rcv;           /* satellite/receiver number */
        uint16_t SNR[NFREQ+NEXOBS]; /* signal strength (0.001 dBHz) */
        uint8_t LLI[NFREQ+NEXOBS];  /* loss of lock indicator */
        uint8_t code[NFREQ+NEXOBS]; /* code indicator (CODE_???) */
        double L[NFREQ+NEXOBS];     /* observation data carrier-phase (cycle) */
        double P[NFREQ+NEXOBS];     /* observation data pseudorange (m) */
        float D[NFREQ+NEXOBS];      /* observation data doppler frequency (Hz) */
    };

    // Satellite status structure
    struct ssat_t {
        uint8_t sys;                /* navigation system */
        uint8_t vs;                 /* valid satellite flag */
        double azel[2];             /* azimuth/elevation angles {az,el} (rad) */
        double resp[NFREQ];         /* residuals of pseudorange (m) */
        double resc[NFREQ];         /* residuals of carrier-phase (m) */
        uint8_t vsat[NFREQ];        /* valid satellite flag */
        uint8_t snr[NFREQ];         /* signal strength (0.25 dBHz) */
        uint8_t fix[NFREQ];         /* ambiguity fix flag (1:float,2:fix,3:hold) */
        uint8_t slip[NFREQ];        /* cycle slip flag */
        uint8_t half[NFREQ];        /* half-cycle valid flag */
        int lock[NFREQ];            /* lock counter of phase */
        uint32_t outc[NFREQ];       /* obs outage counter */
        uint32_t slipc[NFREQ];      /* cycle slip counter */
        int rejc[NFREQ];            /* reject counter */
        double gf;                  /* geometry-free phase L1-L2 (m) */
        double gf2;                 /* geometry-free phase L1-L5 (m) */
        double ph[2][NFREQ];        /* phase windup correction {src,rcv} (cycle) */
        gtime_t pt[2][NFREQ];       /* previous carrier-phase time */
    };

    // Ambiguity control structure
    struct ambc_t {
        gtime_t epoch[4];           /* last epoch */
        int n[4];                   /* number of epochs */
        double LC[4];               /* linear combination average */
        double LCv[4];              /* linear combination variance */
        int fixcnt;                 /* fix count */
        char flags[MAXSAT];         /* fix flags */
    };

    // Use PrcOpt as prcopt_t
    typedef PrcOpt prcopt_t;

    // Use existing NavData as nav_t
    using nav_t = NavData;
    // Use existing Obs as obs_t
    using obs_t = Obs;
    // Use existing StaInfo as sta_t
    using sta_t = StaInfo;

    #ifndef MAXERRMSG
    #define MAXERRMSG 4096
    #endif

    struct rtk_t {
        sol_t sol;                  /* solution */
        double rb[6];               /* base position/velocity (ecef) (m|m/s) */
        int nx, na;                 /* number of float states/fixed states */
        double tt;                  /* time difference between current and previous (s) */
        double *x, *P;              /* float states and their covariance */
        double *xa, *Pa;            /* fixed states and their covariance */
        int nfix;                   /* number of continuous fixes of ambiguity */
        ambc_t ambc[MAXSAT];        /* ambiguity control */
        ssat_t ssat[MAXSAT];        /* satellite status */
        int neb;                    /* bytes in error message buffer */
        char errbuf[MAXERRMSG];     /* error message buffer */
        prcopt_t opt;               /* processing options */
    };

    // Forward declarations for RTKLIB types used in rtk_common.cpp
    struct obsd_t;                  /* observation data record */

    // RINEX control structure
    typedef struct {        /* RINEX control struct type */
        gtime_t time;       /* message time */
        double ver;         /* RINEX version */
        char   type;        /* RINEX file type ('O','N',...) */
        int    sys;         /* navigation system */
        int    tsys;        /* time system */
        char   tobs[8][MAXOBSTYPE][4]; /* rinex obs types */
        obs_t  obs;         /* observation data */
        nav_t  nav;         /* navigation data */
        sta_t  sta;         /* station info */
        int    ephsat;      /* input ephemeris satellite number */
        int    ephset;      /* input ephemeris set (0-1) */
        char   opt[256];    /* rinex dependent options */
    } rnxctr_t;

    // RINEX options structure
    typedef struct {
        gtime_t ts,te;      /* 开始/结束时间 */
        double tint;        /* 时间间隔 (秒) */
        double ttol;        /* 时间容差 (秒) */
        double tunit;       /* 多会话时间单位 (秒) */
        int rnxver;         /* RINEX版本 (x100) */
        int navsys;         /* 导航系统 */
        int obstype;        /* 观测类型 */
        int freqtype;       /* 频率类型 */
        char mask[7][64];   /* 代码掩码 {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
        char staid [32];    /* 站点ID用于rinex文件名 */
        char prog  [32];    /* 程序名 */
        char runby [32];    /* 运行者 */
        char marker[64];    /* 标记名称 */
        char markerno[32];  /* 标记编号 */
        char markertype[32]; /* 标记类型 (ver.3) */
        char name[2][32];   /* 观测者/机构 */
        char rec [3][32];   /* 接收机 #/类型/版本 */
        char ant [3][32];   /* 天线 #/类型 */
        double apppos[3];   /* 近似位置 x/y/z */
        double antdel[3];   /* 天线增量 h/e/n */
        double glo_cp_bias[4]; /* GLONASS码相位偏差 (米) */
        char comment[MAXCOMMENT][64]; /* 注释 */
        char rcvopt[256];   /* 接收机相关选项 */
        uint8_t exsats[MAXSAT]; /* 排除的卫星 */
        int glofcn[32];     /* glonass频点号+8 */
        int outiono;        /* 输出电离层校正 */
        int outtime;        /* 输出时间系统校正 */
        int outleaps;       /* 输出闰秒 */
        int autopos;        /* 自动近似位置 */
        int phshift;        /* 相位偏移校正 */
        int halfcyc;        /* 半周期校正 */
        int sep_nav;        /* 分离的导航文件 */
        gtime_t tstart;     /* 首次观测时间 */
        gtime_t tend;       /* 最后观测时间 */
        gtime_t trtcm;      /* RTCM的近似日志开始时间 */
        char tobs[7][MAXOBSTYPE][4]; /* 观测类型 {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
        double shift[7][MAXOBSTYPE]; /* 相位偏移 (周) */
        int nobs[7];        /* 观测类型数量 */
    } rnxopt_t;

    // Global time offset variable (declaration)
    extern double timeoffset_;

}   // namespace gnss_comm

#endif
