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
* Implementation of RINEX file reading utilities for batch processing,
* following RTKLIB's complete reading flow.
*/

#include "rinex_reader.hpp"
#include <algorithm>
#include <set>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <glob.h>
#include <glog/logging.h>

static const char *obscodes="CLDS";  /* observation code: C=carrier phase, L=psuedorange, D=doppler, S=cn0 */

// Helper functions to replace RTKLIB functions
static int str2time(const char *s, int i, int n, gnss_comm::gtime_t *t) {
    // Simple implementation: parse YYYY MM DD HH MM SS
    char buffer[64];
    if (i < 0 || n > 63) return -1;
    strncpy(buffer, s + i, n);
    buffer[n] = '\0';
    
    // Trim leading and trailing spaces
    char *start = buffer;
    char *end = buffer + strlen(buffer) - 1;
    while (*start == ' ') start++;
    while (end > start && *end == ' ') end--;
    *(end + 1) = '\0';
    
    double ep[6] = {0};
    // Try to parse the time string that may have variable spaces
    // We'll use sscanf to parse the numbers, ignoring the exact spacing
    int matched = sscanf(start, "%lf %lf %lf %lf %lf %lf", 
                         &ep[0], &ep[1], &ep[2], &ep[3], &ep[4], &ep[5]);
    if (matched == 6) {
        // If year is two-digit, convert to four-digit
        if (ep[0] < 100) {
            if (ep[0] < 80) ep[0] += 2000;
            else ep[0] += 1900;
        }
        *t = gnss_comm::epoch2time(ep);
        return 0;
    }
    return -1;
}

static int satid2no(const char *satid) {
    // Convert satellite ID string to satellite number
    // Format: G01, R01, C01, E01, etc.
    if (strlen(satid) < 3) return 0;
    
    int sys_char = satid[0];
    int prn = atoi(satid + 1);
    
    switch (sys_char) {
        case 'G': return gnss_comm::sat_no(SYS_GPS, prn);
        case 'R': return gnss_comm::sat_no(SYS_GLO, prn);
        case 'C': return gnss_comm::sat_no(SYS_BDS, prn);
        case 'E': return gnss_comm::sat_no(SYS_GAL, prn);
        case 'J': return gnss_comm::sat_no(SYS_QZS, prn);
        case 'S': return gnss_comm::sat_no(SYS_SBS, prn);
        default: return 0;
    }
}

namespace gnss_comm
{
    // Global variables for progress display
    static gtime_t progress_ts = {0, 0};
    static gtime_t progress_te = {0, 0};
    
    /* Set time span for progress display -----------------------------------------------------
    * args   : gtime_t ts, te               I   start and end time
    * return : void
    *---------------------------------------------------------------------------------------*/
    void settspan(gtime_t ts, gtime_t te)
    {
        progress_ts = ts;
        progress_te = te;
    }
    
    /* Sort observation data by time, receiver, satellite --------------------------------------
    * similar to RTKLIB's sortobs() function
    * args   : std::vector<std::vector<ObsPtr>>& obs IO observation data
    * return : size_t - number of epochs after sorting
    *---------------------------------------------------------------------------------------*/
    size_t sortobs(std::vector<std::vector<ObsPtr>>& obs)
    {
        if (obs.empty()) {
            return 0;
        }
        
        // First, flatten all observations into a single vector with time, rcv, sat info
        // Since we don't have rcv field in Obs structure, we'll assume rcv=1 for all
        // In a more complete implementation, we would need to add rcv field to Obs
        
        // For now, sort epochs by time, and within each epoch sort by satellite
        for (auto& epoch : obs) {
            std::sort(epoch.begin(), epoch.end(), 
                     [](const ObsPtr& a, const ObsPtr& b) { return a->sat < b->sat; });
        }
        
        // Sort epochs by time
        std::sort(obs.begin(), obs.end(),
                 [](const std::vector<ObsPtr>& a, const std::vector<ObsPtr>& b) {
                     if (a.empty() || b.empty()) return false;
                     return a[0]->time.time < b[0]->time.time ||
                           (a[0]->time.time == b[0]->time.time && a[0]->time.sec < b[0]->time.sec);
                 });
        
        return obs.size();
    }
    
    /* Remove duplicate ephemeris --------------------------------------------------------------
    * similar to RTKLIB's uniqnav() function
    * args   : std::map<uint32_t, std::vector<EphemBasePtr>>& nav IO navigation data
    * return : void
    *---------------------------------------------------------------------------------------*/
    void uniqnav(std::map<uint32_t, std::vector<EphemBasePtr>>& nav)
    {
        for (auto& entry : nav) {
            auto& eph_list = entry.second;
            if (eph_list.size() <= 1) {
                continue;
            }
            
            // Sort by time of ephemeris (toe)
            std::sort(eph_list.begin(), eph_list.end(),
                     [](const EphemBasePtr& a, const EphemBasePtr& b) {
                         return a->toe.time < b->toe.time ||
                               (a->toe.time == b->toe.time && a->toe.sec < b->toe.sec);
                     });
            
            // Remove duplicates (same toe)
            auto last = std::unique(eph_list.begin(), eph_list.end(),
                                   [](const EphemBasePtr& a, const EphemBasePtr& b) {
                                       return a->toe.time == b->toe.time && a->toe.sec == b->toe.sec;
                                   });
            eph_list.erase(last, eph_list.end());
        }
    }
    
    /* Expand file path with wildcards -------------------------------------------------------
    * similar to RTKLIB's expath() function
    * args   : const std::string& path      I   path with wildcards
    *          std::vector<std::string>& files O expanded file list
    * return : int - number of expanded files
    *---------------------------------------------------------------------------------------*/
    int expath(const std::string& path, std::vector<std::string>& files)
    {
        files.clear();
        
        // Check if path contains wildcards
        if (path.find('*') == std::string::npos && path.find('?') == std::string::npos) {
            // No wildcards, just add the path
            files.push_back(path);
            return 1;
        }
        
        // Use glob to expand wildcards
        glob_t glob_result;
        memset(&glob_result, 0, sizeof(glob_result));
        
        int ret = glob(path.c_str(), GLOB_TILDE | GLOB_BRACE, NULL, &glob_result);
        if (ret != 0) {
            globfree(&glob_result);
            return 0;
        }
        
        // Add expanded files
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            files.push_back(glob_result.gl_pathv[i]);
        }
        
        globfree(&glob_result);
        return files.size();
    }
    
    // Define MAXRNXLEN if not already defined
    #ifndef MAXRNXLEN
    #define MAXRNXLEN 1024
    #endif
    
    /* Helper function to copy string with length -------------------------------------------
    * similar to RTKLIB's setstr() function
    * args   : char* dest                  O   destination string
    *          const char* src             I   source string
    *          int n                       I   number of characters to copy
    * return : void
    *---------------------------------------------------------------------------------------*/
    static void setstr(char* dest, const char* src, int n)
    {
        strncpy(dest, src, n);
        dest[n] = '\0';
        // Remove trailing spaces
        for (int i = n - 1; i >= 0 && dest[i] == ' '; --i) {
            dest[i] = '\0';
        }
    }
    
    /* String to number conversion ----------------------------------------------------------
    * args   : const char *s              I   string
    *          int i                      I   start position (0-index)
    *          int n                      I   number of characters
    * return : double - converted number
    *---------------------------------------------------------------------------------------*/
    static double str2num(const char *s, int i, int n)
    {
        double value;
        char str[256],*p=str;
        
        if (i<0||(int)strlen(s)<i||(int)sizeof(str)-1<n) return 0.0;
        for (s+=i;*s&&--n>=0;s++) *p++=*s=='d'||*s=='D'?'E':*s;
        *p='\0';
        return sscanf(str,"%lf",&value)==1?value:0.0;
    }
    
    /* Adjust time considering week handover -------------------------------------
    * args   : gtime_t t                  I   time to adjust
    *          gtime_t t0                 I   reference time
    * return : gtime_t - adjusted time
    *---------------------------------------------------------------------------------------*/
    static gtime_t adjweek(gtime_t t, gtime_t t0)
    {
        double tt = time_diff(t, t0);
        if (tt < -302400.0) return time_add(t, 604800.0);
        if (tt >  302400.0) return time_add(t, -604800.0);
        return t;
    }
    
    /* URA value (m) to URA index ------------------------------------------------
    * args   : double value               I   URA value in meters
    * return : int - URA index (0-15)
    *---------------------------------------------------------------------------------------*/
    static int uraindex(double value)
    {
        static const double ura_eph[] = {
            2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0,
            384.0, 768.0, 1536.0, 3072.0, 6144.0, 0.0
        };
        int i;
        for (i = 0; i < 15; i++) {
            if (ura_eph[i] >= value) break;
        }
        return i;
    }
    
    /* Square function -----------------------------------------------------------
    * args   : double x                   I   value
    * return : double - x squared
    *---------------------------------------------------------------------------------------*/
    static double SQR(double x) { return x * x; }
    
    /* Convert RINEX 2.x observation code to RINEX 3.x format --------------------------------
    * args   : double ver                 I   RINEX version
    *          int sys                    I   satellite system
    *          const char *str            I   RINEX 2.x observation code (2 characters)
    *          char *type                 O   RINEX 3.x observation code (3 characters)
    * return : void
    *---------------------------------------------------------------------------------------*/
    static void convcode(double ver, int sys, const char *str, char *type)
    {
        strcpy(type,"   ");
        
        if      (!strcmp(str,"P1")) { /* ver.2.11 GPS L1PY,GLO L2P */
            if      (sys==SYS_GPS) sprintf(type,"%c1W",'C');
            else if (sys==SYS_GLO) sprintf(type,"%c1P",'C');
        }
        else if (!strcmp(str,"P2")) { /* ver.2.11 GPS L2PY,GLO L2P */
            if      (sys==SYS_GPS) sprintf(type,"%c2W",'C');
            else if (sys==SYS_GLO) sprintf(type,"%c2P",'C');
        }
        else if (!strcmp(str,"C1")) { /* ver.2.11 GPS L1C,GLO L1C/A */
            if      (ver>=2.12) ; /* reject C1 for 2.12 */
            else if (sys==SYS_GPS) sprintf(type,"%c1C",'C');
            else if (sys==SYS_GLO) sprintf(type,"%c1C",'C');
            else if (sys==SYS_GAL) sprintf(type,"%c1X",'C'); /* ver.2.12 */
            else if (sys==SYS_QZS) sprintf(type,"%c1C",'C');
            else if (sys==SYS_SBS) sprintf(type,"%c1C",'C');
        }
        else if (!strcmp(str,"C2")) {
            if (sys==SYS_GPS) {
                if (ver>=2.12) sprintf(type,"%c2W",'C'); /* L2P(Y) */
                else           sprintf(type,"%c2X",'C'); /* L2C */
            }
            else if (sys==SYS_GLO) sprintf(type,"%c2C",'C');
            else if (sys==SYS_QZS) sprintf(type,"%c2X",'C');
            else if (sys==SYS_BDS) sprintf(type,"%c2X",'C'); /* ver.2.12 B1_2 */
        }
        else if (ver>=2.12&&str[1]=='A') { /* ver.2.12 L1C/A */
            if      (sys==SYS_GPS) sprintf(type,"%c1C",str[0]);
            else if (sys==SYS_GLO) sprintf(type,"%c1C",str[0]);
            else if (sys==SYS_QZS) sprintf(type,"%c1C",str[0]);
            else if (sys==SYS_SBS) sprintf(type,"%c1C",str[0]);
        }
        else if (ver>=2.12&&str[1]=='B') { /* ver.2.12 GPS L1C */
            if      (sys==SYS_GPS) sprintf(type,"%c1X",str[0]);
            else if (sys==SYS_QZS) sprintf(type,"%c1X",str[0]);
        }
        else if (ver>=2.12&&str[1]=='C') { /* ver.2.12 GPS L2C */
            if      (sys==SYS_GPS) sprintf(type,"%c2X",str[0]);
            else if (sys==SYS_QZS) sprintf(type,"%c2X",str[0]);
        }
        else if (ver>=2.12&&str[1]=='D') { /* ver.2.12 GLO L2C/A */
            if      (sys==SYS_GLO) sprintf(type,"%c2C",str[0]);
        }
        else if (ver>=2.12&&str[1]=='1') { /* ver.2.12 GPS L1PY,GLO L1P */
            if      (sys==SYS_GPS) sprintf(type,"%c1W",str[0]);
            else if (sys==SYS_GLO) sprintf(type,"%c1P",str[0]);
            else if (sys==SYS_GAL) sprintf(type,"%c1X",str[0]); /* tentative */
            else if (sys==SYS_BDS) sprintf(type,"%c2X",str[0]); /* extension */
        }
        else if (ver<2.12&&str[1]=='1') {
            if      (sys==SYS_GPS) sprintf(type,"%c1C",str[0]);
            else if (sys==SYS_GLO) sprintf(type,"%c1C",str[0]);
            else if (sys==SYS_GAL) sprintf(type,"%c1X",str[0]); /* tentative */
            else if (sys==SYS_QZS) sprintf(type,"%c1C",str[0]);
            else if (sys==SYS_SBS) sprintf(type,"%c1C",str[0]);
        }
        else if (str[1]=='2') {
            if      (sys==SYS_GPS) sprintf(type,"%c2W",str[0]);
            else if (sys==SYS_GLO) sprintf(type,"%c2P",str[0]);
            else if (sys==SYS_QZS) sprintf(type,"%c2X",str[0]);
            else if (sys==SYS_BDS) sprintf(type,"%c2X",str[0]); /* ver.2.12 B1_2 */
        }
        else if (str[1]=='5') {
            if      (sys==SYS_GPS) sprintf(type,"%c5X",str[0]);
            else if (sys==SYS_GAL) sprintf(type,"%c5X",str[0]);
            else if (sys==SYS_QZS) sprintf(type,"%c5X",str[0]);
            else if (sys==SYS_SBS) sprintf(type,"%c5X",str[0]);
        }
        else if (str[1]=='6') {
            if      (sys==SYS_GAL) sprintf(type,"%c6X",str[0]);
            else if (sys==SYS_QZS) sprintf(type,"%c6X",str[0]);
            else if (sys==SYS_BDS) sprintf(type,"%c6X",str[0]); /* ver.2.12 B3 */
        }
        else if (str[1]=='7') {
            if      (sys==SYS_GAL) sprintf(type,"%c7X",str[0]);
            else if (sys==SYS_BDS) sprintf(type,"%c7X",str[0]); /* ver.2.12 B2b */
        }
        else if (str[1]=='8') {
            if      (sys==SYS_GAL) sprintf(type,"%c8X",str[0]);
        }
        LOG(INFO) << "convcode: ver=" << ver << " sys=" << sys << " type= " << str << " -> " << type;
    }
    
    /* obs code string to obs code ----------------------------------------------------------
    * args   : const char *obs            I   obs code string ("L1C","L2P",...)
    * return : int - obs code (CODE_???)
    *---------------------------------------------------------------------------------------*/
static int obs2code(const char *obs)
{
    // Map the two-character observation code (frequency + attribute) to integer code
    // The input is the last two characters of RINEX observation code (e.g., "1C" from "L1C" or "C1C")
    if (obs[0] == '\0') return 0;
    
    // Common GPS codes
    if (!strcmp(obs, "1C")) return CODE_L1C;
    if (!strcmp(obs, "1P")) return CODE_L1P;
    if (!strcmp(obs, "1W")) return CODE_L1W;
    if (!strcmp(obs, "1Y")) return CODE_L1Y;
    if (!strcmp(obs, "1M")) return CODE_L1M;
    if (!strcmp(obs, "1N")) return CODE_L1N;
    if (!strcmp(obs, "1S")) return CODE_L1S;
    if (!strcmp(obs, "1L")) return CODE_L1L;
    if (!strcmp(obs, "1E")) return CODE_L1E;
    if (!strcmp(obs, "1A")) return CODE_L1A;
    if (!strcmp(obs, "1B")) return CODE_L1B;
    if (!strcmp(obs, "1X")) return CODE_L1X;
    if (!strcmp(obs, "1Z")) return CODE_L1Z;
    
    if (!strcmp(obs, "2C")) return CODE_L2C;
    if (!strcmp(obs, "2D")) return CODE_L2D;
    if (!strcmp(obs, "2S")) return CODE_L2S;
    if (!strcmp(obs, "2L")) return CODE_L2L;
    if (!strcmp(obs, "2X")) return CODE_L2X;
    if (!strcmp(obs, "2P")) return CODE_L2P;
    if (!strcmp(obs, "2W")) return CODE_L2W;
    if (!strcmp(obs, "2Y")) return CODE_L2Y;
    if (!strcmp(obs, "2M")) return CODE_L2M;
    if (!strcmp(obs, "2N")) return CODE_L2N;
    
    if (!strcmp(obs, "5I")) return CODE_L5I;
    if (!strcmp(obs, "5Q")) return CODE_L5Q;
    if (!strcmp(obs, "5X")) return CODE_L5X;
    
    if (!strcmp(obs, "7I")) return CODE_L7I;
    if (!strcmp(obs, "7Q")) return CODE_L7Q;
    if (!strcmp(obs, "7X")) return CODE_L7X;
    
    if (!strcmp(obs, "6A")) return CODE_L6A;
    if (!strcmp(obs, "6B")) return CODE_L6B;
    if (!strcmp(obs, "6C")) return CODE_L6C;
    if (!strcmp(obs, "6X")) return CODE_L6X;
    if (!strcmp(obs, "6Z")) return CODE_L6Z;
    if (!strcmp(obs, "6S")) return CODE_L6S;
    if (!strcmp(obs, "6L")) return CODE_L6L;
    
    if (!strcmp(obs, "8I")) return CODE_L8I;
    if (!strcmp(obs, "8Q")) return CODE_L8Q;
    if (!strcmp(obs, "8X")) return CODE_L8X;
    
    // GLONASS codes
    if (!strcmp(obs, "3I")) return CODE_L3I;
    if (!strcmp(obs, "3Q")) return CODE_L3Q;
    if (!strcmp(obs, "3X")) return CODE_L3X;
    
    // BeiDou codes
    if (!strcmp(obs, "1I")) return CODE_L1I;
    if (!strcmp(obs, "1Q")) return CODE_L1Q;
    if (!strcmp(obs, "2I")) return CODE_L2I;
    if (!strcmp(obs, "2Q")) return CODE_L2Q;
    if (!strcmp(obs, "6I")) return CODE_L6I;
    if (!strcmp(obs, "6Q")) return CODE_L6Q;
    
    // IRNSS codes
    if (!strcmp(obs, "9I")) return CODE_L9I;
    if (!strcmp(obs, "9Q")) return CODE_L9Q;
    if (!strcmp(obs, "9X")) return CODE_L9X;
    
    return 0;
}
    
    /* obs code to obs code string ----------------------------------------------------------
    * args   : int code                   I   obs code (CODE_???)
    * return : const char* - obs code string ("L1C","L2P",...)
    *---------------------------------------------------------------------------------------*/
    static const char* code2obs(int code)
    {
        switch (code) {
            case CODE_L1C: return "L1C";
            case CODE_L1P: return "L1P";
            case CODE_L1W: return "L1W";
            case CODE_L1Y: return "L1Y";
            case CODE_L1M: return "L1M";
            case CODE_L1N: return "L1N";
            case CODE_L1S: return "L1S";
            case CODE_L1L: return "L1L";
            case CODE_L1E: return "L1E";
            case CODE_L1A: return "L1A";
            case CODE_L1B: return "L1B";
            case CODE_L1X: return "L1X";
            case CODE_L1Z: return "L1Z";
            case CODE_L2C: return "L2C";
            case CODE_L2D: return "L2D";
            case CODE_L2S: return "L2S";
            case CODE_L2L: return "L2L";
            case CODE_L2X: return "L2X";
            case CODE_L2P: return "L2P";
            case CODE_L2W: return "L2W";
            case CODE_L2Y: return "L2Y";
            case CODE_L2M: return "L2M";
            case CODE_L2N: return "L2N";
            case CODE_L5I: return "L5I";
            case CODE_L5Q: return "L5Q";
            case CODE_L5X: return "L5X";
            case CODE_L7I: return "L7I";
            case CODE_L7Q: return "L7Q";
            case CODE_L7X: return "L7X";
            case CODE_L6A: return "L6A";
            case CODE_L6B: return "L6B";
            case CODE_L6C: return "L6C";
            case CODE_L6X: return "L6X";
            case CODE_L6Z: return "L6Z";
            case CODE_L6S: return "L6S";
            case CODE_L6L: return "L6L";
            case CODE_L8I: return "L8I";
            case CODE_L8Q: return "L8Q";
            case CODE_L8X: return "L8X";
            case CODE_L3I: return "L3I";
            case CODE_L3Q: return "L3Q";
            case CODE_L3X: return "L3X";
            case CODE_L1I: return "L1I";
            case CODE_L1Q: return "L1Q";
            case CODE_L2I: return "L2I";
            case CODE_L2Q: return "L2Q";
            case CODE_L6I: return "L6I";
            case CODE_L6Q: return "L6Q";
            case CODE_L9I: return "L9I";
            case CODE_L9Q: return "L9Q";
            case CODE_L9X: return "L9X";
            default: return "";
        }
    }
    
    /* obs code to frequency index ----------------------------------------------------------
    * args   : int sys                    I   satellite system
    *          int code                   I   obs code (CODE_???)
    * return : int - frequency index (0:L1,1:L2,...)
    *---------------------------------------------------------------------------------------*/
static int code2idx(int sys, int code)
{
    // BeiDou frequencies - must be checked before general frequency ranges
    if (sys == SYS_BDS) {
        if (code == CODE_L1I || code == CODE_L1Q ||
            code == CODE_L2I || code == CODE_L2Q) return 0;  // BDS B1
        if (code == CODE_L7I || code == CODE_L7Q) return 1;  // BDS B2b (C7I/C7Q)
        if (code == CODE_L6I || code == CODE_L6Q) return 2;  // BDS B3
    }
    
    // Check code ranges for different frequencies
    if (code >= CODE_L1C && code <= CODE_L1Z) return 0;  // L1 frequencies
    if (code >= CODE_L2C && code <= CODE_L2N) return 1;  // L2 frequencies
    if (code >= CODE_L5I && code <= CODE_L5X) return 2;  // L5/E5a frequencies
    if (code >= CODE_L6A && code <= CODE_L6L) return 3;  // L6/E6 frequencies
    if (code >= CODE_L7I && code <= CODE_L7X) return 4;  // L7/E5b frequencies
    if (code >= CODE_L8I && code <= CODE_L8X) return 5;  // L8/E5a+b frequencies
    if (code >= CODE_L9I && code <= CODE_L9X) return 6;  // L9/S frequencies
    
    // GLONASS frequencies
    if (code >= CODE_L3I && code <= CODE_L3X) return 2;  // GLONASS G3 (around 1.2 GHz)
    
    return 0;  // Default to L1 if unknown
}
    
    /* get observation code priority --------------------------------------------------------
    * args   : int sys                    I   satellite system
    *          int code                   I   obs code (CODE_???)
    *          const char *opt            I   options
    * return : int - priority (15 highest)
    *---------------------------------------------------------------------------------------*/
static int getcodepri(int sys, int code, const char *opt)
{
    // RTKLIB uses codepris array: priority = 14 - position_in_string
    // codepris[system][freq_index] = priority_string
    // GPS: L1="CPYWMNSL", L2="PYWCMNDLSX", L5="IQX"
    // GLO: L1="CPABX", L2="PCABX", L3="IQX"
    // GAL: L1="CABXZ", L2="IQX", L3="IQX", L4="ABCXZ", L5="IQX"
    // QZS: L1="CLSXZ", L2="LSX", L3="IQXDPZ", L4="LSXEZ"
    // SBS: L1="C", L2="IQX"
    // BDS: L1="IQXDPAN", L2="IQXDPZ", L3="DPX", L4="IQXA", L5="DPX"
    // IRN: L1="ABCX", L2="ABCX"
    
    if (code <= 0) return 0;
    
    // Get frequency index
    int idx = code2idx(sys, code);
    if (idx < 0 || idx >= MAXFREQ) return 0;
    
    // Get code character (third character of obs code, e.g., 'W' from "L2W")
    const char* obs_str = code2obs(code);
    if (!obs_str || strlen(obs_str) < 3) return 0;
    char code_char = obs_str[2];  // Third character
    
    // Parse code options (e.g., "-GL2W" means L2W has priority 15)
    const char *p, *optstr = "";
    char str[8] = "";
    switch (sys) {
        case SYS_GPS: optstr = "-GL%2s"; break;
        case SYS_GLO: optstr = "-RL%2s"; break;
        case SYS_GAL: optstr = "-EL%2s"; break;
        case SYS_QZS: optstr = "-JL%2s"; break;
        case SYS_SBS: optstr = "-SL%2s"; break;
        case SYS_BDS: optstr = "-CL%2s"; break;
        case SYS_IRN: optstr = "-IL%2s"; break;
        default: return 0;
    }
    
    if (opt) {
        for (p = opt; p && (p = strchr(p, '-')); p++) {
            if (sscanf(p, optstr, str) < 1) continue;
            // Options are usually like "1C", "2W", etc.
            // Match freq char (obs_str[1]) and attribute char (obs_str[2])
            if (str[0] == obs_str[1] && str[1] == obs_str[2]) return 15;
        }
    }
    
    // Code priority tables (matching RTKLIB's codepris array)
    static const char* codepris[7][MAXFREQ] = {
        // GPS: L1, L2, L5, L6, L7, L8, L9
        {"CPYWMNSL", "PYWCMNDLSX", "IQX", "", "", "", ""},
        // GLONASS: L1, L2, L3
        {"CPABX", "PCABX", "IQX", "", "", "", ""},
        // Galileo: L1, L2, L3, L4, L5
        {"CABXZ", "IQX", "IQX", "ABCXZ", "IQX", "", ""},
        // QZSS: L1, L2, L3, L4
        {"CLSXZ", "LSX", "IQXDPZ", "LSXEZ", "", "", ""},
        // SBAS: L1, L2
        {"C", "IQX", "", "", "", "", ""},
        // BeiDou: L1, L2, L3, L4, L5
        {"IQXDPAN", "IQXDPZ", "DPX", "IQXA", "DPX", "", ""},
        // IRNSS: L1, L2
        {"ABCX", "ABCX", "", "", "", "", ""}
    };
    
    int sys_idx = -1;
    switch (sys) {
        case SYS_GPS: sys_idx = 0; break;
        case SYS_GLO: sys_idx = 1; break;
        case SYS_GAL: sys_idx = 2; break;
        case SYS_QZS: sys_idx = 3; break;
        case SYS_SBS: sys_idx = 4; break;
        case SYS_BDS: sys_idx = 5; break;
        case SYS_IRN: sys_idx = 6; break;
        default: return 0;
    }
    
    if (sys_idx < 0 || idx >= MAXFREQ) return 0;
    const char* pri_str = codepris[sys_idx][idx];
    if (!pri_str || pri_str[0] == '\0') return 0;
    
    // Find code character in priority string
    const char* pos = strchr(pri_str, code_char);
    if (!pos) return 0;
    
    // Priority = 14 - position_in_string (RTKLIB formula)
    return 14 - (int)(pos - pri_str);
}
    
    /* Decode RINEX observation file header -------------------------------------------------*/
    static void decode_obsh(FILE* fp, char* buff, double ver, int* tsys,
                            StaInfo* sta, char tobs[][MAXOBSTYPE][4] = NULL, NavData* nav = NULL)
    {
        char* label = buff + 60;
        
        // Debug: check if tobs is NULL
        if (tobs == NULL) {
            LOG(WARNING) << "decode_obsh: tobs parameter is NULL!";
        } else {
            LOG(INFO) << "decode_obsh: tobs parameter is NOT NULL";
        }
        
        const char syscodes[] = "GREJSCI";  // GPS, GLONASS, Galileo, QZSS, SBAS, BDS, IRNSS
        const char frqcodes[] = "1256789";
        const char *defcodes[] = {
            "CWX    ",  /* GPS: L125____ */
            "CCXX X ",  /* GLO: L1234_6_ */
            "C XXXX ",  /* GAL: L1_5678_ */
            "CXXX   ",  /* QZS: L1256___ */
            "C X    ",  /* SBS: L1_5____ */
            "XIXIIX ",  /* BDS: L125678_ */
            "  A   A"   /* IRN: L__5___9 */
        };
        // If sta is NULL, we can't store station info, but we can still parse observation types
        // If tobs is NULL, we can't store observation types, but we can still parse station info
        
        if (strstr(label, "MARKER NAME")) {
            if (sta) {
                char name[61];
                strncpy(name, buff, 60);
                name[60] = '\0';
                std::string name_str = name;
                size_t start = name_str.find_first_not_of(" ");
                size_t end = name_str.find_last_not_of(" ");
                if (start != std::string::npos && end != std::string::npos) {
                    sta->name = name_str.substr(start, end - start + 1);
                }
            }
        }
        else if (strstr(label, "MARKER NUMBER")) {
            // 标记号，可以忽略
        }
        else if (strstr(label, "MARKER TYPE")) {
            // 标记类型，可以忽略
        }
        else if (strstr(label, "OBSERVER / AGENCY")) {
            // 观测者/机构，可以忽略
        }
        else if (strstr(label, "REC # / TYPE / VERS")) {
            if (sta) {
                char rectype[21], recver[21], recsno[21];
                strncpy(rectype, buff, 20);
                rectype[20] = '\0';
                strncpy(recver, buff + 20, 20);
                recver[20] = '\0';
                strncpy(recsno, buff + 40, 20);
                recsno[20] = '\0';
                
                std::string rectype_str = rectype;
                std::string recver_str = recver;
                std::string recsno_str = recsno;
                
                size_t start = rectype_str.find_first_not_of(" ");
                size_t end = rectype_str.find_last_not_of(" ");
                if (start != std::string::npos && end != std::string::npos) {
                    sta->rectype = rectype_str.substr(start, end - start + 1);
                }
                
                start = recver_str.find_first_not_of(" ");
                end = recver_str.find_last_not_of(" ");
                if (start != std::string::npos && end != std::string::npos) {
                    sta->recver = recver_str.substr(start, end - start + 1);
                }
                
                start = recsno_str.find_first_not_of(" ");
                end = recsno_str.find_last_not_of(" ");
                if (start != std::string::npos && end != std::string::npos) {
                    sta->recsno = recsno_str.substr(start, end - start + 1);
                }
            }
        }
        else if (strstr(label, "ANT # / TYPE")) {
            if (sta) {
                char antdes[21], antsno[21];
                strncpy(antdes, buff, 20);
                antdes[20] = '\0';
                strncpy(antsno, buff + 20, 20);
                antsno[20] = '\0';
                
                std::string antdes_str = antdes;
                std::string antsno_str = antsno;
                
                size_t start = antdes_str.find_first_not_of(" ");
                size_t end = antdes_str.find_last_not_of(" ");
                if (start != std::string::npos && end != std::string::npos) {
                    sta->antdes = antdes_str.substr(start, end - start + 1);
                }
                
                start = antsno_str.find_first_not_of(" ");
                end = antsno_str.find_last_not_of(" ");
                if (start != std::string::npos && end != std::string::npos) {
                    sta->antsno = antsno_str.substr(start, end - start + 1);
                }
            }
        }
        else if (strstr(label, "APPROX POSITION XYZ")) {
            if (sta) {
                double x, y, z;
                if (sscanf(buff, "%lf %lf %lf", &x, &y, &z) == 3) {
                    sta->pos[0] = x;
                    sta->pos[1] = y;
                    sta->pos[2] = z;
                }
            }
        }
        else if (strstr(label, "ANTENNA: DELTA H/E/N")) {
            if (sta) {
                double dh, de, dn;
                if (sscanf(buff, "%lf %lf %lf", &dh, &de, &dn) == 3) {
                    sta->del[0] = dh;
                    sta->del[1] = de;
                    sta->del[2] = dn;
                }
            }
        }
        else if (strstr(label,"ANTENNA: DELTA X/Y/Z")) {} /* opt ver.3 */
        else if (strstr(label,"ANTENNA: PHASECENTER")) {} /* opt ver.3 */
        else if (strstr(label,"ANTENNA: B.SIGHT XYZ")) {} /* opt ver.3 */
        else if (strstr(label,"ANTENNA: ZERODIR AZI")) {} /* opt ver.3 */
        else if (strstr(label,"ANTENNA: ZERODIR XYZ")) {} /* opt ver.3 */
        else if (strstr(label,"CENTER OF MASS: XYZ" )) {} /* opt ver.3 */
        else if (strstr(label, "SYS / # / OBS TYPES")) { /* ver.3 */
            // RINEX 3.x 观测类型解析
            
            const char *p = strchr(syscodes, buff[0]);
            if (!p) {
                LOG(WARNING) << "invalid system code: sys=" << buff[0];
                return;
            }
            
            int i = (int)(p - syscodes);
            int n = 0;
            // 解析观测类型数量
            if (sscanf(buff + 3, "%3d", &n) != 1 || n <= 0) {
                return;
            }
            
            LOG(INFO) << "Parsing obs types for sys=" << buff[0] << " (i=" << i << ") n=" << n;
            
            // 这里应该将观测类型存储到tobs数组中
            // 如果tobs不为空，则存储
            if (tobs != NULL) {
                int nt = 0;
                // 保存当前系统字符，因为续行时buff[0]可能改变
                char sys_char = buff[0];
                char current_line[1024];
                strcpy(current_line, buff);
                int k = 6;  // 第一行观测类型从第7列开始（索引6）
                int j = 0;  // 已解析的观测类型计数
                
                while (j < n && nt < MAXOBSTYPE-1) {
                    // 如果当前行已经解析完所有列，读取下一行
                    if (k >= 58) {
                        if (!fgets(current_line, sizeof(current_line), fp)) break;
                        // 续行: 标签在61-80列，观测类型从第7列开始（索引6）
                        // 检查续行是否有正确的标签
                        if (strlen(current_line) > 60) {
                            char continuation_label[21];
                            strncpy(continuation_label, current_line + 60, 20);
                            continuation_label[20] = '\0';
                            if (!strstr(continuation_label, "SYS / # / OBS TYPES")) {
                                // 不是续行，可能遇到其他头信息
                                LOG(WARNING) << "Expected continuation line for obs types, got: " << continuation_label;
                                break;
                            }
                        }
                        // 续行可能从第2列开始，而不是第7列，跳过前导空格
                        k = 0;
                        while (k < 60 && current_line[k] == ' ') k++;
                        // 如果跳过空格后仍然在标签区域，则重置到第6列
                        if (k >= 60) k = 6;
                    }
                    
                    // 检查索引是否有效
                    if (i < 0 || i >= 7) {
                        LOG(ERROR) << "Invalid system index i=" << i;
                        break;
                    }
                    if (nt >= MAXOBSTYPE) {
                        LOG(WARNING) << "Too many obs types for sys=" << sys_char << ", max=" << MAXOBSTYPE;
                        break;
                    }
                    
                    // 检查当前4列是否全为空格（空字段）
                    bool empty = true;
                    for (int m = 0; m < 4; m++) {
                        if (current_line[k+m] != ' ' && current_line[k+m] != '\0') {
                            empty = false;
                            break;
                        }
                    }
                    if (empty) {
                        k += 4;
                        continue; // 跳过空字段，但不增加j
                    }
                    
                    // 提取4个字符的字段
                    char field[5];
                    strncpy(field, current_line + k, 4);
                    field[4] = '\0';
                    
                    // 去除首尾空格
                    char *start = field;
                    char *end = field + strlen(field) - 1;
                    while (*start == ' ') start++;
                    while (end > start && *end == ' ') end--;
                    *(end + 1) = '\0';
                    
                    // 如果去除空格后为空，跳过
                    if (strlen(start) == 0) {
                        k += 4;
                        continue;
                    }
                    
                    // 确保目标字符串有空间，最多复制3个字符
                    setstr(tobs[i][nt], start, 3);
                    LOG(INFO) << "  Parsed obs type [" << nt << "] for sys=" << sys_char << ": '" << tobs[i][nt] << "'";
                    nt++;
                    j++; // 成功解析一个观测类型
                    k += 4;  // 每个观测类型占4列
                }
                
                tobs[i][nt][0] = '\0';
                LOG(INFO) << "Total " << nt << " obs types stored for sys=" << sys_char;
                
                // 验证解析数量
                if (nt != n) {
                    LOG(WARNING) << "Parsed " << nt << " obs types but expected " << n << " for sys=" << sys_char;
                }
                
                /* change BDS B1 code: 3.02 */
                if (i == 5 && fabs(ver - 3.02) < 1e-3) {
                    for (int j = 0; j < nt; j++) {
                        if (tobs[i][j][1] == '1') tobs[i][j][1] = '2';
                    }
                }
                /* if unknown code in ver.3, set default code */
                for (int j = 0; j < nt; j++) {
                    if (tobs[i][j][2]) continue;
                    const char *q = strchr(frqcodes, tobs[i][j][1]);
                    if (!q) continue;
                    tobs[i][j][2] = defcodes[i][(int)(q - frqcodes)];
                    LOG(INFO) << "set default for unknown code: sys=" << sys_char << " code=" << tobs[i][j];
                }
            } else {
                LOG(INFO) << "System " << buff[0] << " has " << n << " observation types (tobs array not provided)";
                // 跳过解析的行
                char current_line[1024];
                strcpy(current_line, buff);
                int k = 6;  // 第一行从第7列开始（索引6）
                int j = 0;
                while (j < n) {
                    if (k > 58) {
                        if (!fgets(current_line, sizeof(current_line), fp)) break;
                        k = 6;  // 续行从第7列开始（索引6），与第一行对齐
                    }
                    k += 4;
                    j++;
                }
            }
        }
        else if (strstr(label, "WAVELENGTH FACT L1/2")) {
            // 波长因子，可忽略 ver.2
        }
        else if (strstr(label, "# / TYPES OF OBSERV")) { /* ver.2 */
            // RINEX 2.x 观测类型解析
            int n = 0;
            if (sscanf(buff, "%6d", &n) != 1 || n <= 0) {
                return;
            }
            
            // 如果tobs不为空，则存储
            if (tobs != NULL) {
                int nt = 0;
                for (int i = 0, j = 10; i < n && nt < MAXOBSTYPE-1; i++, j += 6) {
                    if (j >= 58) {
                        if (!fgets(buff, 1024, fp)) break;
                        j = 10;
                    }
                    if (ver <= 2.99) {
                        char str[4];
                        setstr(str, buff + j, 2);
                        convcode(ver, SYS_GPS, str, tobs[0][nt]);
                        convcode(ver, SYS_GLO, str, tobs[1][nt]);
                        convcode(ver, SYS_GAL, str, tobs[2][nt]);
                        convcode(ver, SYS_QZS, str, tobs[3][nt]);
                        convcode(ver, SYS_SBS, str, tobs[4][nt]);
                        convcode(ver, SYS_BDS, str, tobs[5][nt]);
                    }
                    nt++;
                }
                tobs[0][nt][0] = '\0';
            } else {
                LOG(INFO) << "RINEX 2.x has " << n << " observation types (tobs array not provided)";
                // 跳过解析的行
                for (int i = 0, j = 10; i < n; i++, j += 6) {
                    if (j > 58) {
                        if (!fgets(buff, 1024, fp)) break;
                        j = 10;
                    }
                }
            }
        }
        else if (strstr(label, "SIGNAL STRENGTH UNIT")) {
            // 信号强度单位，可忽略 ver.3
        }
        else if (strstr(label, "INTERVAL")) {
            // 观测间隔，可忽略
        }
        else if (strstr(label, "TIME OF FIRST OBS")) {
            // 首次观测时间，可以忽略
            // 解析时间系统
            if (tsys) {
                // 使用rtklib.h中定义的宏
                // 检查buff长度是否足够
                if (strlen(buff) >= 51) {
                    if (!strncmp(buff + 48, "GPS", 3)) {
                        *tsys = T_SYS_GPS;
                    } else if (!strncmp(buff + 48, "GLO", 3)) {
                        *tsys = T_SYS_UTC;  // GLONASS时间系统在RTKLIB中标记为UTC
                    } else if (!strncmp(buff + 48, "GAL", 3)) {
                        *tsys = T_SYS_GAL;
                    } else if (!strncmp(buff + 48, "QZS", 3)) {
                        *tsys = T_SYS_QZS;  /* ver.3.02 */
                    } else if (!strncmp(buff + 48, "BDT", 3)) {
                        *tsys = T_SYS_BDS;  /* ver.3.02 */
                    } else if (!strncmp(buff + 48, "IRN", 3)) {
                        *tsys = T_SYS_GPS;  /* ver.3.03 IRN fallback to GPS */
                    } else {
                        // 默认使用GPS时间系统
                        *tsys = T_SYS_GPS;
                        LOG(WARNING) << "Unknown time system in TIME OF FIRST OBS, default to GPS";
                    }
                    LOG(INFO) << "Time system detected: " << std::string(buff + 48, 3) 
                              << " -> tsys=" << *tsys;
                } else {
                    LOG(WARNING) << "TIME OF FIRST OBS line too short, cannot parse time system";
                }
            }
        }
        else if (strstr(label, "TIME OF LAST OBS")) {
            // 最后观测时间，可以忽略
        }
        else if (strstr(label,"RCV CLOCK OFFS APPL" )) {} /* opt */
        else if (strstr(label,"SYS / DCBS APPLIED"  )) {} /* opt ver.3 */
        else if (strstr(label,"SYS / PCVS APPLIED"  )) {} /* opt ver.3 */
        else if (strstr(label,"SYS / SCALE FACTOR"  )) {} /* opt ver.3 */
        else if (strstr(label,"SYS / PHASE SHIFTS"  )) {} /* ver.3.01 */
        
        else if (strstr(label, "COMMENT")) {
            // 注释，可以忽略
        }
        else if (strstr(label, "GLONASS SLOT / FRQ #")) { /* ver.3.02 */
            for (int i=0;i<8;i++) {
                if (buff[4+i*7]!='R') continue;
                int prn=(int)str2num(buff,5+i*7,2);
                int fcn=(int)str2num(buff,8+i*7,2);
                if (prn<1||prn>MAX_PRN_GLO||fcn<-7||fcn>6) continue;
                if (nav) nav->glo_fcn[prn-1]=fcn+8;
            }
        }
        else if (strstr(label, "GLONASS COD/PHS/BIS")) { /* ver.3.02 */
            if (sta) {
                sta->glo_cp_bias[0]=str2num(buff, 5,8);
                sta->glo_cp_bias[1]=str2num(buff,18,8);
                sta->glo_cp_bias[2]=str2num(buff,31,8);
                sta->glo_cp_bias[3]=str2num(buff,44,8);
            }
        }
        else if (strstr(label, "LEAP SECONDS")) {
            if (nav) {
                nav->utc_gps[4]=str2num(buff, 0,6);
                nav->utc_gps[7]=str2num(buff, 6,6);
                nav->utc_gps[5]=str2num(buff,12,6);
                nav->utc_gps[6]=str2num(buff,18,6);
            }
        }
        else if (strstr(label,"# OF SALTELLITES")) {}
        else if (strstr(label,"PRN / # OF OBS" )) {}
        // 其他标签可以继续添加
    }
    
    /* Decode RINEX navigation file header --------------------------------------------------*/
    static void decode_navh(char* buff, NavData* nav = NULL)
    {
        int i,j;
        char *label=buff+60; 
        if (strstr(label,"ION ALPHA"           )) { /* opt ver.2 */
            if (nav) {
                for (i=0,j=2;i<4;i++,j+=12) nav->ion_gps[i]=str2num(buff,j,12);
            }
        }
        else if (strstr(label,"ION BETA"            )) { /* opt ver.2 */
            if (nav) {
                for (i=0,j=2;i<4;i++,j+=12) nav->ion_gps[i+4]=str2num(buff,j,12);
            }
        }
        else if (strstr(label,"DELTA-UTC: A0,A1,T,W")) { /* opt ver.2 */
            if (nav) {
                for (i=0,j=3;i<2;i++,j+=19) nav->utc_gps[i]=str2num(buff,j,19);
                for (;i<4;i++,j+=9) nav->utc_gps[i]=str2num(buff,j,9);
            }
        }
        else if (strstr(label,"IONOSPHERIC CORR"    )) { /* opt ver.3 */
            if (nav) {
                if (!strncmp(buff,"GPSA",4)) {
                    for (i=0,j=5;i<4;i++,j+=12) nav->ion_gps[i]=str2num(buff,j,12);
                }
                else if (!strncmp(buff,"GPSB",4)) {
                    for (i=0,j=5;i<4;i++,j+=12) nav->ion_gps[i+4]=str2num(buff,j,12);
                }
                else if (!strncmp(buff,"GAL",3)) {
                    for (i=0,j=5;i<4;i++,j+=12) nav->ion_gal[i]=str2num(buff,j,12);
                }
                else if (!strncmp(buff,"QZSA",4)) { /* v.3.02 */
                    for (i=0,j=5;i<4;i++,j+=12) nav->ion_qzs[i]=str2num(buff,j,12);
                }
                else if (!strncmp(buff,"QZSB",4)) { /* v.3.02 */
                    for (i=0,j=5;i<4;i++,j+=12) nav->ion_qzs[i+4]=str2num(buff,j,12);
                }
                else if (!strncmp(buff,"BDSA",4)) { /* v.3.02 */
                    for (i=0,j=5;i<4;i++,j+=12) nav->ion_cmp[i]=str2num(buff,j,12);
                }
                else if (!strncmp(buff,"BDSB",4)) { /* v.3.02 */
                    for (i=0,j=5;i<4;i++,j+=12) nav->ion_cmp[i+4]=str2num(buff,j,12);
                }
                else if (!strncmp(buff,"IRNA",4)) { /* v.3.03 */
                    for (i=0,j=5;i<4;i++,j+=12) nav->ion_irn[i]=str2num(buff,j,12);
                }
                else if (!strncmp(buff,"IRNB",4)) { /* v.3.03 */
                    for (i=0,j=5;i<4;i++,j+=12) nav->ion_irn[i+4]=str2num(buff,j,12);
                }
            }
        }
        else if (strstr(label,"TIME SYSTEM CORR"    )) { /* opt ver.3 */
            if (nav) {
                if (!strncmp(buff,"GPUT",4)) {
                    nav->utc_gps[0]=str2num(buff, 5,17);
                    nav->utc_gps[1]=str2num(buff,22,16);
                    nav->utc_gps[2]=str2num(buff,38, 7);
                    nav->utc_gps[3]=str2num(buff,45, 5);
                }
                else if (!strncmp(buff,"GLUT",4)) {
                    nav->utc_glo[0]=-str2num(buff,5,17); /* tau_C */
                }
                else if (!strncmp(buff,"GLGP",4)) {
                    nav->utc_glo[1]=str2num(buff, 5,17); /* tau_GPS */
                }
                else if (!strncmp(buff,"GAUT",4)) { /* v.3.02 */
                    nav->utc_gal[0]=str2num(buff, 5,17);
                    nav->utc_gal[1]=str2num(buff,22,16);
                    nav->utc_gal[2]=str2num(buff,38, 7);
                    nav->utc_gal[3]=str2num(buff,45, 5);
                }
                else if (!strncmp(buff,"QZUT",4)) { /* v.3.02 */
                     nav->utc_qzs[0]=str2num(buff, 5,17);
                     nav->utc_qzs[1]=str2num(buff,22,16);
                     nav->utc_qzs[2]=str2num(buff,38, 7);
                     nav->utc_qzs[3]=str2num(buff,45, 5);
                }
                else if (!strncmp(buff,"BDUT",4)) { /* v.3.02 */
                     nav->utc_cmp[0]=str2num(buff, 5,17);
                     nav->utc_cmp[1]=str2num(buff,22,16);
                     nav->utc_cmp[2]=str2num(buff,38, 7);
                     nav->utc_cmp[3]=str2num(buff,45, 5);
                }
                else if (!strncmp(buff,"SBUT",4)) { /* v.3.02 */
                     nav->utc_sbs[0]=str2num(buff, 5,17);
                     nav->utc_sbs[1]=str2num(buff,22,16);
                     nav->utc_sbs[2]=str2num(buff,38, 7);
                     nav->utc_sbs[3]=str2num(buff,45, 5);
                }
                else if (!strncmp(buff,"IRUT",4)) { /* v.3.03 */
                    nav->utc_irn[0]=str2num(buff, 5,17);
                    nav->utc_irn[1]=str2num(buff,22,16);
                    nav->utc_irn[2]=str2num(buff,38, 7);
                    nav->utc_irn[3]=str2num(buff,45, 5);
                    nav->utc_irn[8]=0.0; /* A2 */
                }
            }
        }
        else if (strstr(label,"LEAP SECONDS"        )) { /* opt */
            if (nav) {
                nav->utc_gps[4]=str2num(buff, 0,6);
                nav->utc_gps[7]=str2num(buff, 6,6);
                nav->utc_gps[5]=str2num(buff,12,6);
                nav->utc_gps[6]=str2num(buff,18,6);
            }
        }
    }
    
    /* Decode GLONASS navigation file header -----------------------------------------------*/
    static void decode_gnavh(char* buff)
    {
        char* label = buff + 60;
        
        // GLONASS导航文件头解析
        if (strstr(label, "CORR TO SYSTEM TIME")) {
            // 系统时间修正，可以忽略
        }
        else if (strstr(label, "LEAP SECONDS")) {
            // 闰秒，可以忽略
        }
        else if (strstr(label, "COMMENT")) {
            // 注释，可以忽略
        }
    }
    
    // /* Decode SBAS navigation file header --------------------------------------------------*/
    // static void decode_hnavh(char* buff)
    // {
    //     char* label = buff + 60;
        
    //     // SBAS导航文件头解析
    //     if (strstr(label, "IONOSPHERIC CORR")) {
    //         // 电离层修正，可以忽略
    //     }
    //     else if (strstr(label, "TIME SYSTEM CORR")) {
    //         // 时间系统修正，可以忽略
    //     }
    //     else if (strstr(label, "COMMENT")) {
    //         // 注释，可以忽略
    //     }
    // }
/* decode GEO NAV header -----------------------------------------------------*/
static void decode_hnavh(char *buff)
{
    char *label=buff+60;

    if      (strstr(label,"CORR TO SYTEM TIME"  )) {} /* opt */
    else if (strstr(label,"D-UTC A0,A1,T,W,S,U" )) {} /* opt */
    else if (strstr(label,"LEAP SECONDS"        )) {} /* opt */
}

    /* Save cycle slip indicators ----------------------------------------------------------
    * args   : const ObsPtr& obs          I   observation data
    *          std::map<std::pair<uint32_t, int>, uint8_t>& slips IO cycle slip indicators
    * return : void
    *---------------------------------------------------------------------------------------*/
    static void saveslips(const ObsPtr& obs, 
                         std::map<std::pair<uint32_t, int>, uint8_t>& slips)
    {
        if (!obs) return;
        
        for (size_t i = 0; i < obs->LLI.size(); ++i) {
            if (obs->LLI[i] != 0) {
                slips[std::make_pair(obs->sat, i)] = obs->LLI[i];
            }
        }
    }

    /* Restore cycle slip indicators -------------------------------------------------------
    * args   : ObsPtr& obs                IO  observation data
    *          std::map<std::pair<uint32_t, int>, uint8_t>& slips I  cycle slip indicators
    * return : void
    *---------------------------------------------------------------------------------------*/
    static void restslips(ObsPtr& obs, 
                         const std::map<std::pair<uint32_t, int>, uint8_t>& slips)
    {
        if (!obs) return;
        
        for (size_t i = 0; i < obs->LLI.size(); ++i) {
            auto key = std::make_pair(obs->sat, i);
            auto it = slips.find(key);
            if (it != slips.end()) {
                obs->LLI[i] = it->second;
            }
        }
    }

    /* Screen observation time -------------------------------------------------------------
    * args   : gtime_t time               I   observation time
    *          gtime_t ts,te              I   time start, end (0: all)
    *          double ti                  I   time interval (0: all)
    *          gtime_t* last_time         IO  last accepted time
    * return : bool - true if time passes screening, false otherwise
    *---------------------------------------------------------------------------------------*/
    static bool screent(gtime_t time, gtime_t ts, gtime_t te, double ti, 
                       gtime_t* last_time)
    {
        // Check time window
        if (ts.time != 0 && time_diff(time, ts) < 0) {
            return false;
        }
        if (te.time != 0 && time_diff(time, te) > 0) {
            return false;
        }
        
        // Check time interval
        if (ti > 0 && last_time->time != 0) {
            double dt = time_diff(time, *last_time);
            if (dt < ti - 1e-9) {  // slightly less than ti to avoid floating point errors
                return false;
            }
        }
        
        return true;
    }

    /* Add observation data to observation list --------------------------------------------
    * args   : ObsPtr obs                 I   observation data
    *          std::vector<std::vector<ObsPtr>>& obs_list IO observation list
    *          gtime_t* last_time         IO  last accepted time
    *          double ti                  I   time interval (0: all)
    * return : void
    *---------------------------------------------------------------------------------------*/
    static void addobsdata(ObsPtr obs, 
                          std::vector<std::vector<ObsPtr>>& obs_list,
                          gtime_t* last_time, double ti)
    {
        if (!obs) return;
        
        // Calculate time difference from last epoch
        double dt = (last_time->time == 0) ? 0 : time_diff(obs->time, *last_time);
        
        // Check if we need to start a new epoch
        // 1. First epoch
        // 2. Time interval sampling (ti > 0) and enough time passed
        // 3. No sampling (ti == 0) and time changed (new epoch)
        if (obs_list.empty() || 
            (ti > 0 && last_time->time != 0 && dt >= ti - 1e-3) ||
            (ti == 0 && last_time->time != 0 && fabs(dt) > 1e-4)) {
            
            obs_list.push_back(std::vector<ObsPtr>());
        }
        else if (ti > 0 && last_time->time != 0 && dt < ti - 1e-3 && fabs(dt) > 1e-4) {
            // Skip this epoch if sampling is enabled and interval not met
            // But only if it is indeed a new epoch (time changed)
            // If time is same (dt ~ 0), it belongs to current epoch
            return; 
        }
        
        // Add observation to the current epoch
        if (!obs_list.empty()) {
            obs_list.back().push_back(obs);
        }
        
        // Update last time
        // Only update if time actually changed (to avoid issues with multiple obs in same epoch)
        if (fabs(time_diff(obs->time, *last_time)) > 1e-9) {
            *last_time = obs->time;
        }
    }

    /* Signal index structure -------------------------------------------------------------
    * similar to RTKLIB's sigind_t structure
    *---------------------------------------------------------------------------------------*/
    struct SigIndex
    {
        int n;                          /* number of index */
        int code[MAXOBSTYPE];           /* obs code (CODE_???) */
        int type[MAXOBSTYPE];           /* obs type (0:C/A,1:L,2:D,3:S) */
        int idx[MAXOBSTYPE];            /* frequency index (0:L1,1:L2,...) */
        int pri[MAXOBSTYPE];            /* priority (15 highest) */
        int pos[MAXOBSTYPE];            /* assigned position (0:L1,1:L2,-1:unassigned) */
        double shift[MAXOBSTYPE];       /* phase shift (cycle) */
    };
    
    /* Set observation data index ---------------------------------------------------------
    * args   : double ver                 I   RINEX version
    *          int sys                    I   satellite system
    *          const char *opt            I   options
    *          char tobs[MAXOBSTYPE][4]   I   observation types
    *          SigIndex *ind              O   signal index
    * return : void
    *---------------------------------------------------------------------------------------*/
    static void set_index(double ver, int sys, const char *opt,
                          char tobs[MAXOBSTYPE][4], SigIndex *ind)
    {
        const char *p;
        char str[8]; const char *optstr="";
        double shift;
        int i,j,k,n;
        
        for (i=n=0;*tobs[i];i++,n++) {
            ind->code[i]=obs2code(tobs[i]+1);
            ind->type[i]=(p=strchr(obscodes,tobs[i][0]))?(int)(p-obscodes):0;
            ind->idx[i]=code2idx(sys,ind->code[i]);
            ind->pri[i]=getcodepri(sys,ind->code[i],opt);
            ind->pos[i]=-1;
        }
        /* parse phase shift options */
        switch (sys) {
            case SYS_GPS: optstr="-GL%2s=%lf"; break;
            case SYS_GLO: optstr="-RL%2s=%lf"; break;
            case SYS_GAL: optstr="-EL%2s=%lf"; break;
            case SYS_QZS: optstr="-JL%2s=%lf"; break;
            case SYS_SBS: optstr="-SL%2s=%lf"; break;
            case SYS_BDS: optstr="-CL%2s=%lf"; break;
            case SYS_IRN: optstr="-IL%2s=%lf"; break;
        }
        for (p=opt;p&&(p=strchr(p,'-'));p++) {
            if (sscanf(p,optstr,str,&shift)<2) continue;
            for (i=0;i<n;i++) {
                if (strcmp(code2obs(ind->code[i]),str)) continue;
                ind->shift[i]=shift;
                LOG(INFO) << "phase shift: sys=" << sys << " tobs=" << tobs[i] << " shift=" << shift;
            }
        }
        /* assign index for highest priority code */
        for (i=0;i<NFREQ;i++) {
            for (j=0,k=-1;j<n;j++) {
                if (ind->idx[j]==i&&ind->pri[j]&&(k<0||ind->pri[j]>ind->pri[k])) {
                    k=j;
                }
            }
            if (k<0) continue;
            
            for (j=0;j<n;j++) {
                if (ind->code[j]==ind->code[k]) ind->pos[j]=i;
            }
        }
        /* assign index of extended observation data */
        for (i=0;i<NEXOBS;i++) {
            for (j=0;j<n;j++) {
                if (ind->code[j]&&ind->pri[j]&&ind->pos[j]<0) break;
            }
            if (j>=n) break;
            
            for (k=0;k<n;k++) {
                if (ind->code[k]==ind->code[j]) ind->pos[k]=NFREQ+i;
            }
        }
        for (i=0;i<n;i++) {
            if (!ind->code[i]||!ind->pri[i]||ind->pos[i]>=0) continue;
            LOG(INFO) << "reject obs type: sys=" << sys << ", obs=" << tobs[i];
        }
        ind->n=n;
        
    #if 0 /* for debug */
        for (i=0;i<n;i++) {
            LOG(INFO) << "set_index: sys=" << sys << ",tobs=" << tobs[i] << " code=" << ind->code[i] << " pri=" << ind->pri[i] << " idx=" << ind->idx[i] << " pos=" << ind->pos[i] << " shift=" << ind->shift[i];
        }
    #endif
    }
    
    /* Decode observation epoch -----------------------------------------------------------
    * args   : FILE* fp                   I   file pointer
    *          char* buff                 I   buffer containing the epoch line
    *          double ver                 I   RINEX version
    *          gtime_t* time              O   epoch time
    *          int* flag                  O   epoch flag
    *          int* sats                  O   satellite IDs (array)
    * return : int - number of satellites in epoch, 0 for error
    *---------------------------------------------------------------------------------------*/
    static int decode_obsepoch(FILE *fp, char *buff, double ver, gtime_t *time,
                               int *flag, int *sats)
    {
        int i,j,n;
        char satid[8]="";
        
        if (ver<=2.99) { /* ver.2 */
            if ((n=(int)str2num(buff,29,3))<=0) return 0;
            
            /* epoch flag: 3:new site,4:header info,5:external event */
            *flag=(int)str2num(buff,28,1);
            
            if (3<=*flag&&*flag<=5) return n;
            
            if (str2time(buff,0,26,time)) {
                LOG(ERROR) << "rinex obs invalid epoch: epoch=" << std::string(buff, 26);
                return 0;
            }
            for (i=0,j=32;i<n;i++,j+=3) {
                if (j>=68) {
                    if (!fgets(buff,MAXRNXLEN,fp)) break;
                    j=32;
                }
                if (i<MAXOBS) {
                    strncpy(satid,buff+j,3);
                    sats[i]=satid2no(satid);
                }
            }
        }
        else { /* ver.3 */
        // RINEX 3.x epoch header format: fixed-width format
        // Format: > YYYY MM DD HH MM SS.ssssssss EEE NNN
        // Position: 0: '>', 1-28: time, 31: epoch flag, 32-34: satellite count
        LOG(INFO) << "decode_obsepoch ver.3: buff='" << std::string(buff, 80) << "'";
    
        if (buff[0] != '>') {
            LOG(ERROR) << "rinex obs invalid epoch (missing '>'): epoch=" << std::string(buff, 29);
            return 0;
        }
    
        // Parse satellite count first (position 32-34)
        if ((n = (int)str2num(buff, 32, 3)) <= 0) {
            LOG(ERROR) << "Failed to parse satellite count from epoch line: buff='" << std::string(buff, 80) << "'";
            return 0;
        }
        
        // Parse epoch flag (position 31)
        *flag = (int)str2num(buff, 31, 1);
    
        // Check for special flags (new site, header info, external event)
        if (3 <= *flag && *flag <= 5) {
            LOG(INFO) << "Special epoch flag: " << *flag << ", n=" << n;
            return n;
        }
    
        // Parse time (position 1-28)
        if (str2time(buff, 1, 28, time)) {
            LOG(ERROR) << "rinex obs invalid epoch: epoch=" << std::string(buff, 29);
            return 0;
        }
    
        // Log parsed epoch information
        double ep[6];
        time2epoch(*time, ep);
        LOG(INFO) << "Parsed epoch: time=" << (int)ep[0] << "/" << (int)ep[1] << "/" << (int)ep[2]
                << " " << (int)ep[3] << ":" << (int)ep[4] << ":" << std::fixed 
                << std::setprecision(3) << ep[5]
                << " flag=" << *flag << " n=" << n;
        
        if (n <= 0) {
            LOG(ERROR) << "Invalid number of satellites: " << n;
            return 0;
        }
    
        // For RINEX 3.x, we do not read the satellite ID lines here.
        // They will be read by the caller (readrnxobsb) one per satellite.
        // We leave the file pointer at the beginning of the first satellite's line.
        
        // We set the sats array to 0, as the caller will read the satellite IDs from the lines.
        for (int i = 0; i < n && i < MAXOBS; i++) {
            sats[i] = 0;
        }
        
        return n;
    }
    
    return n;
}

    /* Decode observation data ------------------------------------------------------------
    * args   : FILE* fp                   I   file pointer
    *          char* buff                 I   buffer containing the observation line
    *          double ver                 I   RINEX version
    *          int mask                   I   satellite system mask
    *          SigIndex index[7]          I   signal index array for each system
    *          ObsPtr obs                 O   observation data
    * return : bool - true if decoded successfully, false otherwise
    *---------------------------------------------------------------------------------------*/
    static bool decode_obsdata(FILE *fp, char *buff, double ver, int mask,
                               SigIndex index[7], ObsPtr obs)
    {
        SigIndex *ind;
        double val[MAXOBSTYPE]={0};
        uint8_t lli[MAXOBSTYPE]={0};
        char satid[8]="";
        int i,j,n,m,stat=1,p[MAXOBSTYPE],k[16],l[16];
        int sys_idx = -1;
        
        // Remove newline if present
        size_t len = strlen(buff);
        if (len > 0 && buff[len-1] == '\n') {
            buff[len-1] = '\0';
        }
        
        if (ver>2.99) { /* ver.3 */
            // Satellite ID is in first 3 columns (0-indexed: 0,1,2)
            if (len >= 3) {
                satid[0] = buff[0];
                satid[1] = buff[1];
                satid[2] = buff[2];
                satid[3] = '\0';
            } else {
                satid[0] = '\0';
            }
            //LOG(INFO) << "decode_obsdata: buff='" << buff << "', satid='" << satid << "'";
            obs->sat=(uint8_t)satid2no(satid);
            
            // Determine system index based on satellite ID first character
            switch (satid[0]) {
                case 'G': sys_idx = 0; break;  // GPS
                case 'R': sys_idx = 1; break;  // GLONASS
                case 'E': sys_idx = 2; break;  // Galileo
                case 'J': sys_idx = 3; break;  // QZSS
                case 'S': sys_idx = 4; break;  // SBAS
                case 'C': sys_idx = 5; break;  // BeiDou
                case 'I': sys_idx = 6; break;  // IRNSS
                default:
                    LOG(ERROR) << "decode_obsdata: unknown system satid=" << satid;
                    stat = 0;
                    break;
            }
        }
        if (!obs->sat) {
            LOG(ERROR) << "decode_obsdata: unsupported sat sat=" << satid;
            stat=0;
        }
        // Skip system mask check for mixed files (mask may be 0)
        // else if (!(satsys(obs->sat,NULL)&mask)) {
        //     stat=0;
        // }
        
        // Use the appropriate index for the satellite system
        if (stat && sys_idx >= 0 && sys_idx < 7) {
            ind = &index[sys_idx];
        } else {
            // Fallback to first index (GPS) if unknown
            ind = &index[0];
        }
        // switch (satsys(obs->sat,NULL)) {
        //     case SYS_GLO: ind=index+1; break;
        //     case SYS_GAL: ind=index+2; break;
        //     case SYS_QZS: ind=index+3; break;
        //     case SYS_SBS: ind=index+4; break;
        //     case SYS_BDS: ind=index+5; break;
        //     case SYS_IRN: ind=index+6; break;
        //     default:      ind=index  ; break;
        // }
        for (i=0,j=ver<=2.99?0:3;i<ind->n;i++,j+=16) {
            
        if (ver<=2.99) { /* ver.2 */
            if (j>=80) {
                if (!fgets(buff,MAXRNXLEN,fp)) break;
                j=0;
            }
        } else { /* ver.3 */
            // For RINEX 3, check if we need to read continuation line
            // RINEX 3 allows data to continue beyond column 80 on the same line
            // Only check for continuation line if we've exceeded the current line length
            size_t current_len = strlen(buff);
            if (j + 14 > current_len) {
                // We don't have enough data in the current line
                // Check if we need to read a continuation line
                if (j >= 80) {
                    // Save file position before reading next line
                    long file_pos_before_read = ftell(fp);
                    if (!fgets(buff,MAXRNXLEN,fp)) break;
                    
                    // Remove newline if present
                    size_t len = strlen(buff);
                    if (len > 0 && buff[len-1] == '\n') {
                        buff[len-1] = '\0';
                        len--;
                    }
                    
                    // Check if this is a continuation line (first character is space)
                    if (buff[0] == ' ') {
                        j = 1;  // continuation line starts at column 2
                    } else {
                        // Not a continuation line - this is the next satellite's line
                        // Backtrack file pointer so readrnxobsb can process it
                        fseek(fp, file_pos_before_read, SEEK_SET);
                        break;  // Stop parsing for current satellite
                    }
                } else {
                    // j < 80 but not enough data - no more data in current line
                    break;
                }
            }
        }
                if (stat) {
                    // Check if we have enough data in the buffer
                    size_t current_len = strlen(buff);
                    if (j + 14 > current_len) {
                        // Not enough data in current buffer
                        if (ver > 2.99 && j >= 80) {
                            // For RINEX 3, if j>=80, we might need a continuation line
                            // This will be handled in the next iteration
                            break;
                        } else {
                            // For RINEX 2 or j<80, no more data
                            break;
                        }
                    }
                    val[i]=str2num(buff,j,14)+ind->shift[i];
                    lli[i]=(uint8_t)str2num(buff,j+14,1)&3;
                }
        }
        if (!stat) return false;
        
        // Clear existing data in Obs structure
        obs->freqs.clear();
        obs->CN0.clear();
        obs->LLI.clear();
        obs->code.clear();
        obs->psr.clear();
        obs->psr_std.clear();
        obs->cp.clear();
        obs->cp_std.clear();
        obs->dopp.clear();
        obs->dopp_std.clear();
        obs->status.clear();
        
        // Initialize with zero values for NFREQ+NEXOBS frequencies
        // We'll fill in the values based on index->pos mapping
        for (int freq_idx = 0; freq_idx < NFREQ+NEXOBS; freq_idx++) {
            obs->freqs.push_back(0.0);
            obs->CN0.push_back(0.0);
            obs->LLI.push_back(0);
            obs->code.push_back(0);
            obs->psr.push_back(0.0);
            obs->psr_std.push_back(0.0);
            obs->cp.push_back(0.0);
            obs->cp_std.push_back(0.0);
            obs->dopp.push_back(0.0);
            obs->dopp_std.push_back(0.0);
            obs->status.push_back(0);
        }
        
        /* assign position in observation data */
        for (i=n=m=0;i<ind->n;i++) {
            
            p[i]=(ver<=2.11)?ind->idx[i]:ind->pos[i];
            
            if (ind->type[i]==0&&p[i]==0) k[n++]=i; /* C1? index */
            if (ind->type[i]==0&&p[i]==1) l[m++]=i; /* C2? index */
        }
        if (ver<=2.11) {
            
            /* if multiple codes (C1/P1,C2/P2), select higher priority */
            if (n>=2) {
                if (val[k[0]]==0.0&&val[k[1]]==0.0) {
                    p[k[0]]=-1; p[k[1]]=-1;
                }
                else if (val[k[0]]!=0.0&&val[k[1]]==0.0) {
                    p[k[0]]=0; p[k[1]]=-1;
                }
                else if (val[k[0]]==0.0&&val[k[1]]!=0.0) {
                    p[k[0]]=-1; p[k[1]]=0;
                }
                else if (ind->pri[k[1]]>ind->pri[k[0]]) {
                    p[k[1]]=0; p[k[0]]=NEXOBS<1?-1:NFREQ;
                }
                else {
                    p[k[0]]=0; p[k[1]]=NEXOBS<1?-1:NFREQ;
                }
            }
            if (m>=2) {
                if (val[l[0]]==0.0&&val[l[1]]==0.0) {
                    p[l[0]]=-1; p[l[1]]=-1;
                }
                else if (val[l[0]]!=0.0&&val[l[1]]==0.0) {
                    p[l[0]]=1; p[l[1]]=-1;
                }
                else if (val[l[0]]==0.0&&val[l[1]]!=0.0) {
                    p[l[0]]=-1; p[l[1]]=1; 
                }
                else if (ind->pri[l[1]]>ind->pri[l[0]]) {
                    p[l[1]]=1; p[l[0]]=NEXOBS<2?-1:NFREQ+1;
                }
                else {
                    p[l[0]]=1; p[l[1]]=NEXOBS<2?-1:NFREQ+1;
                }
            }
        }
        /* save observation data */
        for (i=0;i<ind->n;i++) {
            if (p[i]<0||val[i]==0.0) continue;
            int freq_idx = p[i];
            if (freq_idx >= NFREQ+NEXOBS) continue; // safety check
            
            switch (ind->type[i]) {
                case 0: // pseudorange
                    obs->psr[freq_idx] = val[i];
                    obs->code[freq_idx] = ind->code[i];
                    // Set frequency based on satellite system and code
                    {
                        int sys = satsys(obs->sat, NULL);
                        if (sys == SYS_GLO) {
                            // GLONASS frequencies
                            if (ind->code[i] == CODE_L1C || ind->code[i] == CODE_L1P) {
                                obs->freqs[freq_idx] = FREQ1_GLO; // Use base frequency, actual freq depends on fcn
                            } else if (ind->code[i] == CODE_L2C || ind->code[i] == CODE_L2P) {
                                obs->freqs[freq_idx] = FREQ2_GLO; // Use base frequency, actual freq depends on fcn
                            } else if (ind->code[i] >= CODE_L3I && ind->code[i] <= CODE_L3X) {
                                obs->freqs[freq_idx] = FREQ3_GLO;
                            }
                        } else if (sys == SYS_BDS) {
                            // BeiDou frequencies
                            if (ind->code[i] == CODE_L1I || ind->code[i] == CODE_L1Q) {
                                obs->freqs[freq_idx] = FREQ1_BDS;
                            } else if (ind->code[i] == CODE_L7I || ind->code[i] == CODE_L7Q) {
                                obs->freqs[freq_idx] = FREQ2_BDS;
                            } else if (ind->code[i] == CODE_L6I || ind->code[i] == CODE_L6Q) {
                                obs->freqs[freq_idx] = FREQ3_BDS;
                            }
                        } else {
                            // GPS, Galileo, QZSS, etc.
                            if (ind->code[i] == CODE_L1C || ind->code[i] == CODE_L1P || 
                                ind->code[i] == CODE_L1W || ind->code[i] == CODE_L1X) {
                                obs->freqs[freq_idx] = FREQ1;
                            } else if (ind->code[i] == CODE_L2C || ind->code[i] == CODE_L2P ||
                                       ind->code[i] == CODE_L2W || ind->code[i] == CODE_L2X) {
                                obs->freqs[freq_idx] = FREQ2;
                            } else if (ind->code[i] == CODE_L5I || ind->code[i] == CODE_L5Q ||
                                       ind->code[i] == CODE_L5X) {
                                obs->freqs[freq_idx] = FREQ5;
                            }
                        }
                    }
                    break;
                case 1: // carrier phase
                    obs->cp[freq_idx] = val[i];
                    obs->LLI[freq_idx] = lli[i];
                    break;
                case 2: // doppler
                    obs->dopp[freq_idx] = (float)val[i];
                    break;
                case 3: // signal strength (CN0)
                    obs->CN0[freq_idx] = (float)val[i];
                    break;
            }
        }
        
        // Remove any frequency slots that have no data
        // We'll keep all slots for now, but could compact if needed
        
        // Debug: print obs content
        // LOG(INFO) << "Decoded obs for sat " << obs->sat << " (satid=" << satid << "):";
        for (int freq_idx = 0; freq_idx < NFREQ+NEXOBS; freq_idx++) {
            if (obs->psr[freq_idx] != 0.0 || obs->cp[freq_idx] != 0.0 || 
                obs->dopp[freq_idx] != 0.0 || obs->CN0[freq_idx] != 0.0) {
                //LOG(INFO) << "  freq_idx=" << freq_idx 
                //          << " psr=" << obs->psr[freq_idx]
                //          << " cp=" << obs->cp[freq_idx]
                //        << " dopp=" << obs->dopp[freq_idx]
                //         << " CN0=" << obs->CN0[freq_idx]
                //          << " LLI=" << (int)obs->LLI[freq_idx]
                //         << " code=" << obs->code[freq_idx];
            }
        }
        
        return true;
    }
    
    /* Read one epoch of RINEX observation data --------------------------------------------
    * similar to RTKLIB's readrnxobsb() function
    * args   : FILE* fp                   I   file pointer
    *          double ver                 I   RINEX version
    *          int sys                    I   satellite system mask (for mixed files)
    *          char tobs[][MAXOBSTYPE][4] I   observation types for all systems
    *          gtime_t& obs_time          O   observation time
    *          std::vector<ObsPtr>& epoch_obs O observations in this epoch
    * return : bool - true if epoch read successfully, false otherwise (EOF or error)
    *---------------------------------------------------------------------------------------*/
    static bool readrnxobsb(FILE* fp, double ver, int sys,
                           char tobs[][MAXOBSTYPE][4],
                           gtime_t& obs_time, std::vector<ObsPtr>& epoch_obs)
    {
        char buff[1024];
        int flag = 0;
        int sats[MAXOBS] = {0};
        
        // Clear output
        epoch_obs.clear();
        
        // Read the first line of the epoch
        if (!fgets(buff, sizeof(buff), fp)) {
            return false;  // EOF
        }
        
        // Decode epoch header line
        int n = decode_obsepoch(fp, buff, ver, &obs_time, &flag, sats);
        if (n <= 0) {
            return false;  // Error or end of data
        }
        
        // Check for special flags (new site, header info, external event)
        if (3 <= flag && flag <= 5) {
            // Handle special case: read header information
            // In RTKLIB, this calls decode_obsh()
            LOG(INFO) << "Special epoch flag: " << flag << ", reading header info";
            
            // Skip header lines until next epoch
            while (fgets(buff, sizeof(buff), fp)) {
                if (strlen(buff) >= 60) {
                    char label[64];
                    strncpy(label, buff + 60, 20);
                    label[20] = '\0';
                    if (strstr(label, "END OF HEADER") || 
                        (buff[0] == '>' && ver > 2.99) ||
                        (isdigit(buff[0]) && ver <= 2.99)) {
                        // Found next epoch or end
                        // We need to backtrack to process this line
                        fseek(fp, -strlen(buff), SEEK_CUR);
                        break;
                    }
                }
            }
            return true;  // Continue to next epoch
        }
        
        // Set up signal index for all systems (like RTKLIB)
        SigIndex index[7];  // GPS, GLO, GAL, QZS, SBS, BDS, IRN
        const int nsys = 7;
        if (nsys >= 1) set_index(ver, SYS_GPS, "", tobs[0], index + 0);
        if (nsys >= 2) set_index(ver, SYS_GLO, "", tobs[1], index + 1);
        if (nsys >= 3) set_index(ver, SYS_GAL, "", tobs[2], index + 2);
        if (nsys >= 4) set_index(ver, SYS_QZS, "", tobs[3], index + 3);
        if (nsys >= 5) set_index(ver, SYS_SBS, "", tobs[4], index + 4);
        if (nsys >= 6) set_index(ver, SYS_BDS, "", tobs[5], index + 5);
        if (nsys >= 7) set_index(ver, SYS_IRN, "", tobs[6], index + 6);
        
        // For RINEX version 3, satellite IDs are on separate lines
        // For version 2, satellite IDs are in the epoch header line
        
        if (ver <= 2.99) {
            // Version 2: sats array already filled by decode_obsepoch
            for (int i = 0; i < n && i < MAXOBS; i++) {
                if (!fgets(buff, sizeof(buff), fp)) {
                    break;  // EOF
                }
                
                ObsPtr obs = std::make_shared<Obs>();
                obs->time = obs_time;
                obs->sat = sats[i];
                
                // Use system mask for mixed files
                if (decode_obsdata(fp, buff, ver, sys, index, obs)) {
                    epoch_obs.push_back(obs);
                }
            }
        } else {
            // Version 3: each satellite has its own line with satellite ID
            for (int i = 0; i < n && i < MAXOBS; i++) {
                if (!fgets(buff, sizeof(buff), fp)) {
                    break;  // EOF
                }

                ObsPtr obs = std::make_shared<Obs>();
                obs->time = obs_time;

                // Let decode_obsdata parse the satellite ID from the buffer
                // Use system mask for mixed files
                if (decode_obsdata(fp, buff, ver, sys, index, obs)) {
                    epoch_obs.push_back(obs);
                }
            }
        }
        
        return !epoch_obs.empty();
    }
    
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
                  char tobs[][MAXOBSTYPE][4])
    {
        char buff[1024];
        char label[64];
        int tsys = 0;  // 时间系统，暂时不使用
        
        // Debug: check if tobs is NULL
        if (tobs == NULL) {
            LOG(WARNING) << "readrnxh: tobs parameter is NULL!";
        } else {
            LOG(INFO) << "readrnxh: tobs parameter is NOT NULL";
        }
        
        // Initialize outputs
        if (ver) *ver = 2.10;
        if (type) *type = RINEX_TYPE_UNKNOWN;
        if (sys) *sys = SYS_GPS;
        if (sta) {
            sta->name = "";
            sta->marker = "";
            sta->antdes = "";
            sta->antsno = "";
            sta->rectype = "";
            sta->recver = "";
            sta->recsno = "";
            sta->pos[0] = sta->pos[1] = sta->pos[2] = 0.0;
            sta->del[0] = sta->del[1] = sta->del[2] = 0.0;
        }
        if (opt) opt[0] = '\0';
        // 注意：tobs 数组由调用者负责初始化
        
        // Read first line to get version and file type
        if (!fgets(buff, sizeof(buff), fp)) {
            return false;
        }
        
        // Parse version (columns 1-9 in RINEX)
        if (ver) {
            char ver_str[10];
            strncpy(ver_str, buff, 9);
            ver_str[9] = '\0';
            *ver = atof(ver_str);
        }
        
        // Parse file type character (position 20)
        char file_type_char = ' ';
        if (strlen(buff) >= 21) {
            file_type_char = buff[20];
        }
        
        // Convert file type char to enum
        if (type) {
            switch (file_type_char) {
                case 'O': *type = RINEX_TYPE_OBS; break;
                case 'N': *type = RINEX_TYPE_NAV; break;
                case 'G': *type = RINEX_TYPE_NAV; break; // GLONASS NAV
                case 'H': *type = RINEX_TYPE_NAV; break; // SBAS NAV
                case 'J': *type = RINEX_TYPE_NAV; break; // QZSS NAV (extension)
                case 'L': *type = RINEX_TYPE_NAV; break; // Galileo NAV (extension)
                case 'C': *type = RINEX_TYPE_CLK; break; // Clock
                default: *type = RINEX_TYPE_UNKNOWN; break;
            }
        }
        
        // Parse satellite system (position 40)
        if (sys && strlen(buff) >= 41) {
            char sys_char = buff[40];
            switch (sys_char) {
                case ' ':
                case 'G': *sys = SYS_GPS; break;
                case 'R': *sys = SYS_GLO; break;
                case 'E': *sys = SYS_GAL; break;
                case 'S': *sys = SYS_SBS; break;
                case 'J': *sys = SYS_QZS; break;
                case 'C': *sys = SYS_BDS; break;
                case 'I': *sys = SYS_IRN; break;
                case 'M': *sys = SYS_NONE; break; // mixed
                default: *sys = SYS_GPS; break;
            }
        }
        
        // Read header lines until END OF HEADER
        while (fgets(buff, sizeof(buff), fp)) {
            // 检查行长度，标签在61-80列
            if (strlen(buff) <= 60) {
                continue;
            }
            
            strncpy(label, buff + 60, 20);
            label[20] = '\0';
            
            // 根据文件类型调用相应的解码函数，传递 tobs 数组
            switch (file_type_char) {
                case 'O': 
                    LOG(INFO) << "readrnxh: calling decode_obsh for observation file";
                    if (tobs == NULL) {
                        LOG(ERROR) << "readrnxh: tobs is NULL when calling decode_obsh!";
                    }
                    decode_obsh(fp, buff, *ver, &tsys, sta, tobs); 
                    break;
                case 'N': decode_navh(buff); break;
                case 'G': decode_gnavh(buff); break;
                case 'H': decode_hnavh(buff); break;
                case 'J': decode_navh(buff); break; // extension
                case 'L': decode_navh(buff); break; // extension
                default: break;
            }
            
            // Check for END OF HEADER
            if (strstr(label, "END OF HEADER")) {
                return true;
            }
        }
        
        return true;
    }
    
    /* Read RINEX observation file body ------------------------------------------------------
    * similar to RTKLIB's readrnxobs() function
    * args   : FILE* fp                     I   file pointer
    *          double ver                   I   RINEX version
    *          int sys                      I   satellite system
    *          int rcv                      I   receiver index
    *          gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const PositioningOptions& prcopt I   processing options
    *          std::vector<std::vector<ObsPtr>>& obs IO observation data
    *          StaInfo* sta                 IO  station information
    *          const char* opt              I   options
    * return : bool - true if success, false otherwise
    *---------------------------------------------------------------------------------------*/
    bool readrnxobs(FILE* fp, double ver, int sys, int rcv,
                    gtime_t ts, gtime_t te, double ti,
                    const PrcOpt& prcopt,
                    std::vector<std::vector<ObsPtr>>& obs,
                    StaInfo* sta, const char* opt,
                    char tobs[][MAXOBSTYPE][4])
    {
        LOG(INFO) << "readrnxobs: Starting RTKLIB-style observation reading flow";
        
        // Clear output container
        obs.clear();
        
        // Cycle slip storage
        std::map<std::pair<uint32_t, int>, uint8_t> slips;
        
        // Last accepted time for interval checking
        gtime_t last_time = {0, 0};
        
        // 如果tobs参数为NULL，则使用默认观测类型
        // 需要为所有系统创建默认观测类型数组
        char default_tobs[7][MAXOBSTYPE][4] = {{{0}}};
        char (*tobs_ptr)[MAXOBSTYPE][4];
        if (tobs == NULL) {
            // 为所有系统设置默认观测类型
            // GPS
            const char* gps_tobs[] = {"C1C", "L1C", "D1C", "S1C", "C2S", "L2S", "D2S", "S2S", "C2W", "L2W", "D2W", "S2W", "C5Q", "L5Q", "D5Q", "S5Q"};
            for (int i = 0; i < 16 && i < MAXOBSTYPE; i++) {
                strcpy(default_tobs[0][i], gps_tobs[i]);
            }
            // GLONASS
            const char* glo_tobs[] = {"C1C", "L1C", "D1C", "S1C", "C2P", "L2P", "D2P", "S2P", "C2C", "L2C", "D2C", "S2C"};
            for (int i = 0; i < 12 && i < MAXOBSTYPE; i++) {
                strcpy(default_tobs[1][i], glo_tobs[i]);
            }
            // Galileo
            const char* gal_tobs[] = {"C1C", "L1C", "D1C", "S1C", "C5Q", "L5Q", "D5Q", "S5Q", "C7Q", "L7Q", "D7Q", "S7Q", "C8Q", "L8Q", "D8Q", "S8Q"};
            for (int i = 0; i < 16 && i < MAXOBSTYPE; i++) {
                strcpy(default_tobs[2][i], gal_tobs[i]);
            }
            // QZSS (similar to GPS)
            for (int i = 0; i < 16 && i < MAXOBSTYPE; i++) {
                strcpy(default_tobs[3][i], gps_tobs[i]);
            }
            // SBAS
            const char* sbs_tobs[] = {"C1C", "L1C", "D1C", "S1C"};
            for (int i = 0; i < 4 && i < MAXOBSTYPE; i++) {
                strcpy(default_tobs[4][i], sbs_tobs[i]);
            }
            // BeiDou
            const char* bds_tobs[] = {"C2I", "L2I", "D2I", "S2I", "C7I", "L7I", "D7I", "S7I"};
            for (int i = 0; i < 8 && i < MAXOBSTYPE; i++) {
                strcpy(default_tobs[5][i], bds_tobs[i]);
            }
            // IRNSS
            const char* irn_tobs[] = {"C5A", "L5A", "D5A", "S5A"};
            for (int i = 0; i < 4 && i < MAXOBSTYPE; i++) {
                strcpy(default_tobs[6][i], irn_tobs[i]);
            }
            tobs_ptr = default_tobs;
        } else {
            tobs_ptr = tobs;
        }
        
        // Loop reading epochs using readrnxobsb()
        int epoch_count = 0;
        int obs_count = 0;
        
        while (true) {
            gtime_t obs_time = {0, 0};
            std::vector<ObsPtr> epoch_obs;
            
            // Read one epoch
            // For mixed systems, pass all tobs arrays; for single system, use appropriate one
            bool success = readrnxobsb(fp, ver, sys, tobs_ptr, obs_time, epoch_obs);
            if (!success) {
                break;  // EOF or error
            }
            
            // Check if epoch has observations
            if (epoch_obs.empty()) {
                continue;
            }
            
            epoch_count++;
            
            // Process each observation in the epoch
            for (ObsPtr& obs_data : epoch_obs) {
                // 1. Save cycle slip indicators
                saveslips(obs_data, slips);
                
                // 2. Convert time to GPST if needed (already GPST in RINEX)
                //    For GLONASS, would need to convert from GLONASS time to GPST
                //    This is handled by time system conversion functions in RTKLIB
                
                // 3. Screen observation time
                if (!screent(obs_time, ts, te, ti, &last_time)) {
                    LOG(INFO) << "Epoch " << epoch_count << " sat " << obs_data->sat 
                              << " failed time screening";
                    continue;
                }
                
                // 4. Restore cycle slip indicators
                restslips(obs_data, slips);
                
                // 5. Add observation data to list
                addobsdata(obs_data, obs, &last_time, ti);
                obs_count++;
                
                LOG(INFO) << "Added observation: epoch=" << epoch_count 
                          << " sat=" << obs_data->sat 
                          << " time=" << (obs_time.time + obs_time.sec);
            }
            
            // In real RTKLIB, after processing all observations in epoch,
            // would call update_phase_llis() to update phase loss-of-lock indicators
            // and apply other quality control checks
            
            // Progress reporting (optional)
            if (epoch_count % 100 == 0) {
                LOG(INFO) << "Processed " << epoch_count << " epochs";
            }
        }
        
        // In a complete implementation, we would also:
        // - Handle multiple frequencies and observation types
        // - Apply antenna phase center corrections
        // - Apply relativistic corrections
        // - Apply tide corrections
        // - Apply antenna phase windup corrections
        // - Apply ionospheric and tropospheric corrections (if requested)
        
        LOG(INFO) << "readrnxobs: Completed RTKLIB-style observation reading flow";
        LOG(INFO) << "  Total epochs processed: " << epoch_count;
        LOG(INFO) << "  Total observations stored: " << obs_count;
        LOG(INFO) << "  Epochs in output: " << obs.size();
        if (!obs.empty()) {
            LOG(INFO) << "  Observations in first epoch: " << obs[0].size();
        }
        
        // Show cycle slip storage summary
        LOG(INFO) << "  Cycle slips detected: " << slips.size();
        for (const auto& slip : slips) {
            LOG(INFO) << "    Sat " << slip.first.first 
                      << " freq index " << slip.first.second 
                      << " LLI: " << (int)slip.second;
        }
        
        return epoch_count > 0;
    }
    
    /* Decode GPS/Galileo/QZSS/BDS ephemeris -------------------------------------
    * similar to RTKLIB's decode_eph() function
    * args   : double ver                   I   RINEX version
    *          uint32_t sat                 I   satellite number
    *          gtime_t toc                  I   clock correction reference time
    *          const double* data          I   ephemeris data array (29 elements)
    *          EphemPtr& eph               O   ephemeris structure
    * return : bool - true if decoded successfully, false otherwise
    *---------------------------------------------------------------------------------------*/
    static bool decode_eph(double ver, uint32_t sat, gtime_t toc, const double *data,
                          EphemPtr& eph)
    {
        int sys = satsys(sat, NULL);
        
        if (!(sys & (SYS_GPS | SYS_GAL | SYS_QZS | SYS_BDS | SYS_IRN))) {
            LOG(ERROR) << "ephemeris error: invalid satellite sat=" << sat;
            return false;
        }
        
        // Initialize ephemeris
        eph = std::make_shared<Ephem>();
        eph->sat = sat;
        eph->toc = toc;
        
        // Clock parameters
        eph->af0 = data[0];
        eph->af1 = data[1];
        eph->af2 = data[2];
        
        // Orbit parameters
        eph->A = SQR(data[10]);
        eph->e = data[8];
        eph->i0 = data[15];
        eph->OMG0 = data[13];
        eph->omg = data[17];
        eph->M0 = data[6];
        eph->delta_n = data[5];
        eph->OMG_dot = data[18];
        eph->i_dot = data[19];
        eph->crc = data[16];
        eph->crs = data[4];
        eph->cuc = data[7];
        eph->cus = data[9];
        eph->cic = data[12];
        eph->cis = data[14];
        
        if (sys == SYS_GPS || sys == SYS_QZS) {
            eph->iode = (uint32_t)data[3];      // IODE
            eph->iodc = (uint32_t)data[26];     // IODC
            eph->toe_tow = data[11];            // Toe (s) in GPS week
            eph->week = (uint32_t)data[21];     // GPS week
            eph->toe = adjweek(gpst2time(eph->week, data[11]), toc);
            eph->ttr = adjweek(gpst2time(eph->week, data[27]), toc);
            
            eph->code = (uint32_t)data[20];     // GPS: codes on L2 ch
            eph->health = (uint32_t)data[24];   // SV health
            eph->ura = uraindex(data[23]);      // URA index (m->index)
            // eph->flag = (int)data[22];       // GPS: L2 P data flag (not in Ephem struct)
            
            eph->tgd[0] = data[25];             // TGD
            // eph->fit = data[28];             // fit interval (h) (not in Ephem struct)
        }
        else if (sys == SYS_GAL) {
            eph->iode = (uint32_t)data[3];      // IODnav
            eph->toe_tow = data[11];            // Toe (s) in Galileo week
            eph->week = (uint32_t)data[21];     // Galileo week = GPS week
            eph->toe = adjweek(gpst2time(eph->week, data[11]), toc);
            eph->ttr = adjweek(gpst2time(eph->week, data[27]), toc);
            
            eph->code = (uint32_t)data[20];     // data sources
            eph->health = (uint32_t)data[24];   // sv health
            eph->ura = uraindex(data[23]);      // SISA index (m->index)
            
            eph->tgd[0] = data[25];             // BGD E5a/E1
            eph->tgd[1] = data[26];             // BGD E5b/E1
        }
        else if (sys == SYS_BDS) {
            // BDS time conversion handled separately if needed
            eph->iode = (uint32_t)data[3];      // AODE
            eph->iodc = (uint32_t)data[28];     // AODC
            eph->toe_tow = data[11];            // Toe (s) in BDT week
            eph->week = (uint32_t)data[21];     // bdt week
            // Note: BDS time conversion would need bdt2gpst, simplified for now
            eph->toe = adjweek(gpst2time(eph->week, data[11]), toc);
            eph->ttr = adjweek(gpst2time(eph->week, data[27]), toc);
            
            eph->health = (uint32_t)data[24];   // satH1
            eph->ura = uraindex(data[23]);      // URA index (m->index)
            
            eph->tgd[0] = data[25];             // TGD1 B1/B3
            eph->tgd[1] = data[26];             // TGD2 B2/B3
        }
        else if (sys == SYS_IRN) {
            eph->iode = (uint32_t)data[3];      // IODEC
            eph->toe_tow = data[11];            // Toe (s) in IRNSS week
            eph->week = (uint32_t)data[21];     // IRNSS week
            eph->toe = adjweek(gpst2time(eph->week, data[11]), toc);
            eph->ttr = adjweek(gpst2time(eph->week, data[27]), toc);
            eph->health = (uint32_t)data[24];   // SV health
            eph->ura = uraindex(data[23]);      // URA index (m->index)
            eph->tgd[0] = data[25];             // TGD
        }
        
        if (eph->iode < 0 || eph->iode > 1023) {
            LOG(WARNING) << "rinex nav invalid: sat=" << sat << " iode=" << eph->iode;
        }
        if (eph->iodc < 0 || eph->iodc > 1023) {
            LOG(WARNING) << "rinex nav invalid: sat=" << sat << " iodc=" << eph->iodc;
        }
        
        return true;
    }
    
    /* Decode GLONASS ephemeris --------------------------------------------------
    * similar to RTKLIB's decode_geph() function
    * args   : double ver                   I   RINEX version
    *          uint32_t sat                 I   satellite number
    *          gtime_t toc                  I   clock correction reference time
    *          const double* data          I   ephemeris data array (15 elements)
    *          GloEphemPtr& geph           O   GLONASS ephemeris structure
    * return : bool - true if decoded successfully, false otherwise
    *---------------------------------------------------------------------------------------*/
    static bool decode_geph(double ver, uint32_t sat, gtime_t toc, const double *data,
                           GloEphemPtr& geph)
    {
        geph = std::make_shared<GloEphem>();
        geph->sat = sat;
        geph->health = (int)data[6];
        geph->freqo = (int)data[10];
        geph->age = (uint32_t)data[2];
        geph->tau_n = -data[0];
        geph->gamma = data[1];
        geph->pos[0] = data[3] * 1e3;
        geph->pos[1] = data[7] * 1e3;
        geph->pos[2] = data[11] * 1e3;
        geph->vel[0] = data[4] * 1e3;
        geph->vel[1] = data[8] * 1e3;
        geph->vel[2] = data[12] * 1e3;
        geph->acc[0] = data[5] * 1e3;
        geph->acc[1] = data[9] * 1e3;
        geph->acc[2] = data[13] * 1e3;
        geph->toe = toc;
        geph->toc = toc;
        return true;
    }
    
    /* Read RINEX navigation data body (single ephemeris) ------------------------
    * similar to RTKLIB's readrnxnavb() function
    * args   : FILE* fp                     I   file pointer
    *          const char* opt              I   options
    *          double ver                   I   RINEX version
    *          int sys                      I   satellite system
    *          int* type                    O   ephemeris type (0:GPS/GAL/QZS/BDS, 1:GLONASS, 2:SBAS)
    *          EphemPtr& eph                O   GPS/GAL/QZS/BDS ephemeris
    *          GloEphemPtr& geph            O   GLONASS ephemeris
    *          SEphemPtr& seph              O   SBAS ephemeris
    * return : int - 1 if success, 0 if error, -1 if EOF
    *---------------------------------------------------------------------------------------*/
    static int readrnxnavb(FILE *fp, const char *opt, double ver, int sys,
                           int *type, EphemPtr& eph, GloEphemPtr& geph, SEphemPtr& seph)
    {
        gtime_t toc;
        double data[64];
        int i = 0, j, prn, sat = 0, sp = 3;
        char buff[MAXRNXLEN], id[8] = "";
        const char *p;
        
        // Set system mask (simplified - accept all systems for now)
        // mask = set_sysmask(opt);
        
        while (fgets(buff, MAXRNXLEN, fp)) {
            if (i == 0) {
                // Decode satellite field
                if (ver >= 3.0 || sys == SYS_GAL || sys == SYS_QZS) {
                    // ver.3 or GAL/QZS
                    snprintf(id, sizeof(id), "%.3s", buff);
                    sat = satid2no(id);
                    sp = 4;
                    if (ver >= 3.0) {
                        int detected_sys = satsys(sat, NULL);
                        if (!detected_sys) {
                            // Infer system from ID
                            if (id[0] == 'S') sys = SYS_SBS;
                            else if (id[0] == 'R') sys = SYS_GLO;
                            else sys = SYS_GPS;
                        } else {
                            sys = detected_sys;
                        }
                    }
                } else {
                    // ver.2
                    prn = (int)str2num(buff, 0, 2);
                    if (sys == SYS_SBS) {
                        sat = sat_no(SYS_SBS, prn + 100);
                    } else if (sys == SYS_GLO) {
                        sat = sat_no(SYS_GLO, prn);
                    } else if (93 <= prn && prn <= 97) {
                        sat = sat_no(SYS_QZS, prn + 100);
                    } else {
                        sat = sat_no(SYS_GPS, prn);
                    }
                }
                
                // Decode Toc field
                if (str2time(buff + sp, 0, 19, &toc)) {
                    LOG(ERROR) << "rinex nav toc error: " << std::string(buff, 23);
                    return 0;
                }
                
                // Decode data fields (first 3)
                for (j = 0, p = buff + sp + 19; j < 3; j++, p += 19) {
                    data[i++] = str2num(p, 0, 19);
                }
            } else {
                // Decode data fields (4 per line)
                for (j = 0, p = buff + sp; j < 4; j++, p += 19) {
                    data[i++] = str2num(p, 0, 19);
                }
                
                // Decode ephemeris when we have enough data
                if (sys == SYS_GLO && i >= 15) {
                    // GLONASS ephemeris
                    *type = 1;
                    return decode_geph(ver, sat, toc, data, geph) ? 1 : 0;
                } else if (sys == SYS_SBS && i >= 15) {
                    // SBAS ephemeris (not implemented yet)
                    *type = 2;
                    LOG(WARNING) << "SBAS ephemeris decoding not yet implemented";
                    return 0;
                } else if (i >= 31) {
                    // GPS/Galileo/QZSS/BDS ephemeris
                    *type = 0;
                    return decode_eph(ver, sat, toc, data, eph) ? 1 : 0;
                }
            }
        }
        return -1;  // EOF
    }
    
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
                    const char* opt)
    {
        int stat, type;
        EphemPtr eph;
        GloEphemPtr geph;
        SEphemPtr seph;
        int count = 0;
        
        // Read RINEX navigation data body
        while ((stat = readrnxnavb(fp, opt, ver, sys, &type, eph, geph, seph)) >= 0) {
            if (stat) {
                // Add ephemeris to navigation data
                EphemBasePtr eph_ptr;
                uint32_t sat_id = 0;
                
                switch (type) {
                    case 0:  // GPS/Galileo/QZSS/BDS
                        if (eph) {
                            sat_id = eph->sat;
                            eph_ptr = eph;
                            nav[sat_id].push_back(eph_ptr);
                            count++;
                        }
                        break;
                    case 1:  // GLONASS
                        if (geph) {
                            sat_id = geph->sat;
                            eph_ptr = geph;
                            nav[sat_id].push_back(eph_ptr);
                            count++;
                        }
                        break;
                    case 2:  // SBAS
                        // SBAS ephemeris not fully implemented yet
                        LOG(WARNING) << "SBAS ephemeris storage not yet implemented";
                        break;
                }
            }
        }
        
        LOG(INFO) << "readrnxnav: Read " << count << " ephemerides";
        return count > 0;
    }
    
    /* Read RINEX file from file pointer -----------------------------------------------------
    * similar to RTKLIB's readrnxfp() function
    * args   : FILE* fp                     I   file pointer
    *          int rcv                      I   receiver index (1: rover, 2: base, etc.)
    *          gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const PositioningOptions& prcopt I   processing options
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
                           StaInfo* sta)
    {
        double ver = 0.0;
        int type = RINEX_TYPE_UNKNOWN;
        int sys = 0;
        char opt[256] = "";
        char tobs[7][MAXOBSTYPE][4] = {{{0}}};
        
        // Read header
        if (!readrnxh(fp, &ver, &type, &sys, sta, opt, tobs)) {
            LOG(ERROR) << "Failed to read RINEX header";
            return RINEX_TYPE_UNKNOWN;
        }
        
        // Read file body based on type
        switch (type) {
            case RINEX_TYPE_OBS:
                if (!readrnxobs(fp, ver, sys, rcv, ts, te, ti, prcopt, obs, sta, opt, tobs)) {
                    LOG(ERROR) << "Failed to read RINEX observation data";
                    return RINEX_TYPE_UNKNOWN;
                }
                break;
                
            case RINEX_TYPE_NAV:
                if (!readrnxnav(fp, ver, sys, ts, te, nav, opt)) {
                    LOG(ERROR) << "Failed to read RINEX navigation data";
                    return RINEX_TYPE_UNKNOWN;
                }
                break;
                
            case RINEX_TYPE_CLK:
                LOG(ERROR) << "RINEX clock file type not supported in this function";
                return RINEX_TYPE_UNKNOWN;
            case RINEX_TYPE_SP3:
                LOG(WARNING) << "RINEX type " << type << " not yet supported";
                break;
                
            default:
                LOG(ERROR) << "Unknown RINEX file type: " << type;
                return RINEX_TYPE_UNKNOWN;
        }
        
        return static_cast<RinexFileType>(type);
    }
    
    /* Read RINEX file with path expansion and decompression ---------------------------------
    * similar to RTKLIB's readrnxfile() function
    * args   : const std::string& file      I   file path
    *          int rcv                      I   receiver index
    *          gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const PositioningOptions& prcopt I   processing options
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
                     StaInfo* sta)
    {
        // Check if file is empty (read from stdin)
        if (file.empty()) {
            return readmxfp(rcv, ts, te, ti, prcopt, obs, nav, sta);
        }
        
        // Expand wildcards in path
        std::vector<std::string> files;
        int n = expath(file, files);
        if (n <= 0) {
            LOG(ERROR) << "No files found for path: " << file;
            return false;
        }
        
        bool success = false;
        for (const auto& f : files) {
            LOG(INFO) << "Reading file: " << f;
            
            // Open file
            FILE* fp = fopen(f.c_str(), "r");
            if (!fp) {
                LOG(ERROR) << "Cannot open file: " << f;
                continue;
            }
            
            // Read file
            RinexFileType type = readrnxfp(fp, rcv, ts, te, ti, prcopt, obs, nav, sta);
            fclose(fp);
            
            if (type != RINEX_TYPE_UNKNOWN) {
                success = true;
            }
            
            // TODO: Handle compressed files (.gz, .Z, .zip, etc.)
            // In RTKLIB, this is done by calling a decompression function
        }
        
        return success;
    }
    
    /* Read from stdin if file is empty ------------------------------------------------------
    * similar to RTKLIB's readmxfp() function
    * args   : int rcv                      I   receiver index
    *          gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const PositioningOptions& prcopt I   processing options
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
                  StaInfo* sta)
    {
        LOG(WARNING) << "Reading from stdin not yet implemented";
        return false;
    }
    
    /* Read RINEX observation file with time window filtering ---------------------------------
    * similar to RTKLIB's readrnxt() function
    * args   : const std::string& file      I   RINEX observation file path
    *          int rcv                      I   receiver index (1: rover, 2: base, etc.)
    *          gtime_t ts,te                I   time start, end (0: all)
    *          double ti                    I   time interval (0: all)
    *          const PositioningOptions& prcopt I   processing options
    *          std::vector<std::vector<ObsPtr>>& obs IO observation data
    *          StaInfo* sta                 O   station information (optional)
    * return : bool - true if success, false otherwise
    *---------------------------------------------------------------------------------------*/
    bool readrnxt(const std::string& file, int rcv,
                  gtime_t ts, gtime_t te, double ti,
                  const PrcOpt& prcopt,
                  std::vector<std::vector<ObsPtr>>& obs,
                  StaInfo* sta)
    {
        std::map<uint32_t, std::vector<EphemBasePtr>> nav;
        return readrnxfile(file, rcv, ts, te, ti, prcopt, obs, nav, sta);
    }
    
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
                    std::map<uint32_t, StaInfo>& sta)
    {
        LOG(INFO) << "readobsnav: ts=" << (ts.time == 0 ? "0" : std::to_string(ts.time + ts.sec)) 
                  << " n=" << infiles.size();
        
        // Clear output containers
        obs.clear();
        nav.clear();
        sta.clear();
        
        // Check input sizes
        if (infiles.size() != index.size()) {
            LOG(ERROR) << "Number of files (" << infiles.size() 
                      << ") doesn't match number of indices (" << index.size() << ")";
            return false;
        }
        
        // Process each file
        int ind = -1;
        int nobs = 0;
        int rcv = 2;  // receiver index: 1=rover, 2=base, etc.
        
        for (size_t i = 0; i < infiles.size(); ++i) {
            // Check for user interrupt (simplified)
            // if (checkbrk("")) return false;
            
            // Update receiver index when file index changes
            if (index[i] != ind) {
                if (obs.size() > static_cast<size_t>(nobs)) {
                    rcv++;
                }
                ind = index[i];
                nobs = obs.size();
            }
            
            const std::string& file = infiles[i];
            LOG(INFO) << "Processing file " << i << ": " << file 
                      << " (index=" << index[i] << ", rcv=" << rcv << ")";
            
            // Read file based on index
            // index: 0=rover obs, 1=base obs, 2=nav, etc.
            if (index[i] == 0 || index[i] == 1) {
                // Observation file (rover or base)
                StaInfo sta_info;
                if (!readrnxt(file, rcv, ts, te, ti, prcopt, obs, &sta_info)) {
                    LOG(ERROR) << "Failed to read observation file: " << file;
                    return false;
                }
                
                // Store station information
                if (rcv >= 1 && rcv <= 2) {  // rover or base station
                    sta[rcv] = sta_info;
                    
                    // If station name is empty, assign a default name
                    if (sta[rcv].name.empty()) {
                        sta[rcv].name = (rcv == 1) ? "ROVR" : "BASE";
                    }
                }
                
            } else if (index[i] == 2) {
                // Navigation file
                std::map<uint32_t, std::vector<EphemBasePtr>> file_nav;
                std::vector<std::vector<ObsPtr>> dummy_obs;
                
                if (!readrnxfile(file, 0, ts, te, ti, prcopt, dummy_obs, file_nav, nullptr)) {
                    LOG(ERROR) << "Failed to read navigation file: " << file;
                    return false;
                }
                
                if (file_nav.empty()) {
                    LOG(ERROR) << "No ephemerides in navigation file: " << file;
                    return false;
                }
                
                // Merge navigation data
                for (const auto& entry : file_nav) {
                    uint32_t sat = entry.first;
                    const auto& eph_list = entry.second;
                    
                    if (nav.find(sat) == nav.end()) {
                        nav[sat] = eph_list;
                    } else {
                        nav[sat].insert(nav[sat].end(), eph_list.begin(), eph_list.end());
                    }
                }
            }
        }
        
        // Check if we have data
        if (obs.empty()) {
            LOG(ERROR) << "No observation data";
            return false;
        }
        
        if (nav.empty()) {
            LOG(ERROR) << "No navigation data";
            return false;
        }
        
        // Sort observation data by time
        size_t nepoch = sortobs(obs);
        LOG(INFO) << "Sorted " << nepoch << " observation epochs";
        
        // Remove duplicate ephemeris
        uniqnav(nav);
        
        // Set time span for progress display
        if (ts.time == 0 || te.time == 0) {
            // Find first and last rover (rcv=1) observations
            gtime_t first_time = {0, 0};
            gtime_t last_time = {0, 0};
            
            for (const auto& epoch : obs) {
                if (epoch.empty()) continue;
                
                gtime_t epoch_time = epoch[0]->time;
                
                if (first_time.time == 0 || time_diff(epoch_time, first_time) < 0) {
                    first_time = epoch_time;
                }
                if (last_time.time == 0 || time_diff(epoch_time, last_time) > 0) {
                    last_time = epoch_time;
                }
            }
            
            if (ts.time == 0 && first_time.time != 0) {
                ts = first_time;
            }
            if (te.time == 0 && last_time.time != 0) {
                te = last_time;
            }
            
            settspan(ts, te);
        }
        
        LOG(INFO) << "readobsnav completed: " << obs.size() << " epochs, " 
                  << nav.size() << " satellites with ephemeris";
        
        return true;
    }

    /* Read RINEX clock file ----------------------------------------------------------------
    * similar to RTKLIB's readrnxclk() function
    * args   : FILE* fp                     I   file pointer
    *          const char* opt              I   options (optional)
    *          int index                    I   file index
    *          std::vector<PreciseClockEpochPtr>& clocks IO precise clock data
    * return : bool - true if success, false otherwise
    *---------------------------------------------------------------------------------------*/
    bool readrnxclk(FILE* fp, const char* opt, int index,
                    std::vector<PreciseClockEpochPtr>& clocks)
    {
        char buff[MAXRNXLEN];
        gtime_t time;
        double data[2];
        int sat, mask;
        
        // 设置系统掩码（简化版，默认接受所有系统）
        mask = SYS_ALL;
        
        // 如果需要，可以从opt中解析系统掩码，这里简化处理
        // mask = set_sysmask(opt);
        
        while (fgets(buff, sizeof(buff), fp)) {
            // 解析时间
            if (str2time(buff, 8, 26, &time)) {
                LOG(WARNING) << "rinex clk invalid epoch: " << std::string(buff, 34);
                continue;
            }
            
            // 只读取AS（卫星钟）记录
            if (strncmp(buff, "AS", 2) != 0) {
                continue;
            }
            
            // 解析卫星ID
            char satid[8] = "";
            strncpy(satid, buff + 3, 4);
            sat = satid2no(satid);
            if (sat == 0) {
                LOG(WARNING) << "rinex clk invalid satellite: " << satid;
                continue;
            }
            
            // 检查系统掩码
            if (!(satsys(sat, NULL) & mask)) {
                continue;
            }
            
            // 解析钟差和标准差
            for (int i = 0, j = 40; i < 2; i++, j += 20) {
                data[i] = str2num(buff, j, 19);
            }
            
            // 查找是否已存在相同时间的历元
            PreciseClockEpochPtr epoch_ptr = nullptr;
            for (auto& epoch : clocks) {
                if (fabs(time_diff(time, epoch->time)) < 1E-9) {
                    epoch_ptr = epoch;
                    break;
                }
            }
            
            // 如果不存在，创建新历元
            if (!epoch_ptr) {
                epoch_ptr = std::make_shared<PreciseClockEpoch>();
                epoch_ptr->time = time;
                epoch_ptr->index = index;
                clocks.push_back(epoch_ptr);
            }
            
            // 存储钟差数据
            epoch_ptr->clk[sat] = data[0];
            epoch_ptr->std[sat] = data[1];
        }
        
        return !clocks.empty();
    }
    
    /* Compare precise clock epochs for sorting --------------------------------------------*/
    static bool cmp_pclk(const PreciseClockEpochPtr& a, const PreciseClockEpochPtr& b)
    {
        double tt = time_diff(a->time, b->time);
        if (tt < -1E-9) return true;
        if (tt > 1E-9) return false;
        return a->index < b->index;
    }
    
    /* Combine precise clock data from multiple files --------------------------------------*/
    static void combpclk(std::vector<PreciseClockEpochPtr>& clocks)
    {
        if (clocks.empty()) return;
        
        // 按时间和文件索引排序
        std::sort(clocks.begin(), clocks.end(), cmp_pclk);
        
        // 合并相同时间的历元
        std::vector<PreciseClockEpochPtr> merged;
        for (size_t i = 0; i < clocks.size(); ++i) {
            if (merged.empty() || fabs(time_diff(clocks[i]->time, merged.back()->time)) > 1E-9) {
                merged.push_back(clocks[i]);
            } else {
                // 合并相同时间的钟差数据
                for (const auto& entry : clocks[i]->clk) {
                    merged.back()->clk[entry.first] = entry.second;
                    merged.back()->std[entry.first] = clocks[i]->std[entry.first];
                }
            }
        }
        
        clocks = merged;
    }
    
    /* Read RINEX clock file with path expansion ---------------------------------------------
    * similar to RTKLIB's readrnxc() function
    * args   : const std::string& file      I   file path (wild-card * expanded)
    *          std::vector<PreciseClockEpochPtr>& clocks IO precise clock data
    * return : int - number of precise clock epochs read
    *---------------------------------------------------------------------------------------*/
    int readrnxc(const std::string& file, std::vector<PreciseClockEpochPtr>& clocks)
    {
        int index = 0, n = 0;
        std::vector<std::string> files;
        
        // 扩展通配符路径
        n = expath(file, files);
        if (n <= 0) {
            LOG(ERROR) << "No files found for path: " << file;
            return 0;
        }
        
        // 读取每个时钟文件
        for (int i = 0; i < n; ++i) {
            LOG(INFO) << "Reading clock file: " << files[i];
            
            // 打开文件
            FILE* fp = fopen(files[i].c_str(), "r");
            if (!fp) {
                LOG(ERROR) << "Cannot open file: " << files[i];
                continue;
            }
            
            // 读取时钟文件
            if (!readrnxclk(fp, "", index, clocks)) {
                LOG(ERROR) << "Failed to read clock file: " << files[i];
            }
            
            fclose(fp);
            index++;
        }
        
        if (clocks.empty()) {
            return 0;
        }
        
        // 合并相同历元的钟差数据
        combpclk(clocks);
        
        LOG(INFO) << "readrnxc: read " << clocks.size() << " precise clock epochs";
        return clocks.size();
    }
    
}   // namespace gnss_comm
