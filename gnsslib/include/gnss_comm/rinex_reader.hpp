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
* This file provides RINEX file reading utilities for batch processing,
* similar to RTKLIB's readobsnav function.
*/

#ifndef RINEX_READER_HPP_
#define RINEX_READER_HPP_

#include <vector>
#include <string>
#include <map>
#include "gnss_constant.hpp"
#include "gnss_utility.hpp"

namespace gnss_comm
{
    /* Use StaInfo from gnss_constant.hpp */

    /* RINEX file type enumeration -----------------------------------------------------------
    * based on RTKLIB's file types
    *---------------------------------------------------------------------------------------*/
    enum RinexFileType {
        RINEX_TYPE_OBS = 0,        /* observation data */
        RINEX_TYPE_NAV = 1,        /* navigation data */
        RINEX_TYPE_CLK = 2,        /* clock data */
        RINEX_TYPE_SP3 = 3,        /* precise orbit */
        RINEX_TYPE_UNKNOWN = -1    /* unknown type */
    };

    /* Read multiple RINEX observation and navigation files ------------------------------------
    * similar to RTKLIB's readobsnav() function
    * args   : gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const std::vector<std::string>& infiles  I   input file paths
    *          const std::vector<int>& index I   file index (0: rover, 1: base, 2: nav, etc.)
    *          const PrcOpt& prcopt         I   processing options
    *          std::vector<std::vector<ObsPtr>>& obs IO observation data (per epoch)
    *          std::map<uint32_t, std::vector<EphemBasePtr>>& nav IO navigation data
    *          std::map<uint32_t, StaInfo>& sta IO station information (key: rcv index)
    * return : bool - true if success, false otherwise
    * notes  : This function reads multiple RINEX files and merges the data according to
    *          the file index. It handles time window filtering, sorting, and duplicate removal.
    *---------------------------------------------------------------------------------------*/
    bool readobsnav(gtime_t ts, gtime_t te, double ti,
                    const std::vector<std::string>& infiles,
                    const std::vector<int>& index,
                    const PrcOpt& prcopt,
                    std::vector<std::vector<ObsPtr>>& obs,
                    std::map<uint32_t, std::vector<EphemBasePtr>>& nav,
                    std::map<uint32_t, StaInfo>& sta);

    /* Sort observation data by time -----------------------------------------------------------
    * similar to RTKLIB's sortobs() function
    * args   : std::vector<std::vector<ObsPtr>>& obs IO observation data
    * return : size_t - number of epochs after sorting
    *---------------------------------------------------------------------------------------*/
    size_t sortobs(std::vector<std::vector<ObsPtr>>& obs);

    /* Remove duplicate ephemeris --------------------------------------------------------------
    * similar to RTKLIB's uniqnav() function
    * args   : std::map<uint32_t, std::vector<EphemBasePtr>>& nav IO navigation data
    * return : void
    *---------------------------------------------------------------------------------------*/
    void uniqnav(std::map<uint32_t, std::vector<EphemBasePtr>>& nav);

    /* Read RINEX observation file with time window filtering ---------------------------------
    * similar to RTKLIB's readrnxt() function
    * args   : const std::string& file      I   RINEX observation file path
    *          int rcv                      I   receiver index (1: rover, 2: base, etc.)
    *          gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const PrcOpt& prcopt         I   processing options
    *          std::vector<std::vector<ObsPtr>>& obs IO observation data
    *          StaInfo* sta                 O   station information (optional)
    * return : bool - true if success, false otherwise
    *---------------------------------------------------------------------------------------*/
    bool readrnxt(const std::string& file, int rcv,
                  gtime_t ts, gtime_t te, double ti,
                  const PrcOpt& prcopt,
                  std::vector<std::vector<ObsPtr>>& obs,
                  StaInfo* sta = nullptr);

    /* Read RINEX file from file pointer -----------------------------------------------------
    * similar to RTKLIB's readrnxfp() function
    * args   : FILE* fp                     I   file pointer
    *          int rcv                      I   receiver index (1: rover, 2: base, etc.)
    *          gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const PrcOpt& prcopt         I   processing options
    *          std::vector<std::vector<ObsPtr>>& obs IO observation data
    *          std::map<uint32_t, std::vector<EphemBasePtr>>& nav IO navigation data
    *          StaInfo* sta                 O   station information (optional)
    * return : RinexFileType - type of file read, or RINEX_TYPE_UNKNOWN if error
    *---------------------------------------------------------------------------------------*/
    RinexFileType readrnxfp(FILE* fp, int rcv,
                           gtime_t ts, gtime_t te, double ti,
                           const PrcOpt& prcopt,
                           std::vector<std::vector<ObsPtr>>& obs,
                           std::map<uint32_t, std::vector<EphemBasePtr>>& nav,
                           StaInfo* sta = nullptr);

    /* Read RINEX file header ----------------------------------------------------------------
    * similar to RTKLIB's readrnxh() function
    * args   : FILE* fp                     I   file pointer
    *          double* ver                  O   RINEX version
    *          int* type                    O   file type (0:obs,1:nav,2:clk,3:sp3)
    *          int* sys                     O   satellite system
    *          StaInfo* sta                 O   station information
    *          char* opt                    O   options
    *          char tobs[][MAXOBSTYPE][4]   O   observation types (for observation files)
    * return : bool - true if success, false otherwise
    *---------------------------------------------------------------------------------------*/
    bool readrnxh(FILE* fp, double* ver, int* type, int* sys,
                  StaInfo* sta, char* opt,
                  char tobs[][MAXOBSTYPE][4] = nullptr);

    /* Read RINEX observation file body ------------------------------------------------------
    * similar to RTKLIB's readrnxobs() function
    * args   : FILE* fp                     I   file pointer
    *          double ver                   I   RINEX version
    *          int sys                      I   satellite system
    *          int rcv                      I   receiver index
    *          gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const PrcOpt& prcopt         I   processing options
    *          std::vector<std::vector<ObsPtr>>& obs IO observation data
    *          StaInfo* sta                 IO  station information
    *          const char* opt              I   options
    *          char tobs[][MAXOBSTYPE][4]   I   observation types (from header, optional)
    * return : bool - true if success, false otherwise
    *---------------------------------------------------------------------------------------*/
    bool readrnxobs(FILE* fp, double ver, int sys, int rcv,
                    gtime_t ts, gtime_t te, double ti,
                    const PrcOpt& prcopt,
                    std::vector<std::vector<ObsPtr>>& obs,
                    StaInfo* sta, const char* opt,
                    char tobs[][MAXOBSTYPE][4] = nullptr);

    /* Read RINEX navigation file body -------------------------------------------------------
    * similar to RTKLIB's readrnxnav() function
    * args   : FILE* fp                     I   file pointer
    *          double ver                   I   RINEX version
    *          int sys                      I   satellite system
    *          gtime_t ts,te                I   time start, end (0: all)
    *          std::map<uint32_t, std::vector<EphemBasePtr>>& nav IO navigation data
    *          const char* opt              I   options
    * return : bool - true if success, false otherwise
    *---------------------------------------------------------------------------------------*/
    bool readrnxnav(FILE* fp, double ver, int sys,
                    gtime_t ts, gtime_t te,
                    std::map<uint32_t, std::vector<EphemBasePtr>>& nav,
                    const char* opt);

    /* Expand file path with wildcards -------------------------------------------------------
    * similar to RTKLIB's expath() function
    * args   : const std::string& path      I   path with wildcards
    *          std::vector<std::string>& files O expanded file list
    * return : int - number of expanded files
    *---------------------------------------------------------------------------------------*/
    int expath(const std::string& path, std::vector<std::string>& files);

    /* Read RINEX file with path expansion and decompression ---------------------------------
    * similar to RTKLIB's readrnxfile() function
    * args   : const std::string& file      I   file path
    *          int rcv                      I   receiver index
    *          gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const PrcOpt& prcopt         I   processing options
    *          std::vector<std::vector<ObsPtr>>& obs IO observation data
    *          std::map<uint32_t, std::vector<EphemBasePtr>>& nav IO navigation data
    *          StaInfo* sta                 O   station information
    * return : bool - true if success, false otherwise
    *---------------------------------------------------------------------------------------*/
    bool readrnxfile(const std::string& file, int rcv,
                     gtime_t ts, gtime_t te, double ti,
                     const PrcOpt& prcopt,
                     std::vector<std::vector<ObsPtr>>& obs,
                     std::map<uint32_t, std::vector<EphemBasePtr>>& nav,
                     StaInfo* sta = nullptr);

    /* Read from stdin if file is empty ------------------------------------------------------
    * similar to RTKLIB's readmxfp() function
    * args   : int rcv                      I   receiver index
    *          gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const PrcOpt& prcopt         I   processing options
    *          std::vector<std::vector<ObsPtr>>& obs IO observation data
    *          std::map<uint32_t, std::vector<EphemBasePtr>>& nav IO navigation data
    *          StaInfo* sta                 O   station information
    * return : bool - true if success, false otherwise
    *---------------------------------------------------------------------------------------*/
    bool readmxfp(int rcv,
                  gtime_t ts, gtime_t te, double ti,
                  const PrcOpt& prcopt,
                  std::vector<std::vector<ObsPtr>>& obs,
                  std::map<uint32_t, std::vector<EphemBasePtr>>& nav,
                  StaInfo* sta = nullptr);

    /* Set time span for progress display -----------------------------------------------------
    * args   : gtime_t ts, te               I   start and end time
    * return : void
    *---------------------------------------------------------------------------------------*/
    void settspan(gtime_t ts, gtime_t te);

    /* Precise clock data structure -----------------------------------------------------------
    * based on RTKLIB's pclk_t structure
    *---------------------------------------------------------------------------------------*/
    struct PreciseClockEpoch
    {
        gtime_t time;                       /* time (GPST) */
        int index;                          /* file index */
        std::map<uint32_t, double> clk;     /* satellite clock offset (m) */
        std::map<uint32_t, double> std;     /* satellite clock std (m) */
    };
    typedef std::shared_ptr<PreciseClockEpoch> PreciseClockEpochPtr;

    /* Read RINEX clock file ----------------------------------------------------------------
    * similar to RTKLIB's readrnxclk() function
    * args   : FILE* fp                     I   file pointer
    *          const char* opt              I   options (optional)
    *          int index                    I   file index
    *          std::vector<PreciseClockEpochPtr>& clocks IO precise clock data
    * return : bool - true if success, false otherwise
    *---------------------------------------------------------------------------------------*/
    bool readrnxclk(FILE* fp, const char* opt, int index,
                    std::vector<PreciseClockEpochPtr>& clocks);

    /* Read RINEX clock file with path expansion ---------------------------------------------
    * similar to RTKLIB's readrnxc() function
    * args   : const std::string& file      I   file path (wild-card * expanded)
    *          std::vector<PreciseClockEpochPtr>& clocks IO precise clock data
    * return : int - number of precise clock epochs read
    *---------------------------------------------------------------------------------------*/
    int readrnxc(const std::string& file, std::vector<PreciseClockEpochPtr>& clocks);

}   // namespace gnss_comm

#endif
