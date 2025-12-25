/**
 * gnss_rtk_utils.cpp: RTK positioning utility functions
 * 
 * This file implements utility functions for RTK positioning, including
 * data format conversions and synchronization functions.
 */

#include "gnss_comm/rinex_reader.hpp"
#include "gnss_comm/gnss_constant.hpp"
#include "gnss_comm/gnss_utility.hpp"
#include "gnss_comm/rtk_common.hpp"
#include <algorithm>
#include <cstring>
#include <vector>
#include <map>

using namespace gnss_comm;

namespace gnss_comm
{
    /**
     * Convert Obs to obsd_t format
     * args   : const ObsPtr& obs    I   observation data (gnss_comm format)
     *          obsd_t& obsd         O   observation data (RTKLIB format)
     *          int rcv               I   receiver number (1=rover, 2=base)
     * return : void
     */
    void obs_to_obsd(const ObsPtr& obs, obsd_t& obsd, int rcv)
    {
        memset(&obsd, 0, sizeof(obsd_t));
        
        obsd.time = obs->time;
        obsd.sat = (uint8_t)obs->sat;
        obsd.rcv = (uint8_t)rcv;  // 1=rover, 2=base
        
        // Convert observation data
        int nfreq = std::min((int)std::max({obs->psr.size(), obs->cp.size(), obs->code.size()}), NFREQ+NEXOBS);
        for (int i = 0; i < nfreq; i++) {
            if (i < (int)obs->psr.size() && obs->psr[i] != 0.0) {
                obsd.P[i] = obs->psr[i];
            }
            if (i < (int)obs->cp.size() && obs->cp[i] != 0.0) {
                obsd.L[i] = obs->cp[i];
            }
            if (i < (int)obs->dopp.size() && obs->dopp[i] != 0.0) {
                obsd.D[i] = (float)obs->dopp[i];
            }
            if (i < (int)obs->code.size()) {
                obsd.code[i] = obs->code[i];
            }
            if (i < (int)obs->LLI.size()) {
                obsd.LLI[i] = obs->LLI[i];
            }
            if (i < (int)obs->CN0.size() && obs->CN0[i] > 0.0) {
                // Convert CN0 (dB-Hz) to SNR (0.001 dBHz)
                obsd.SNR[i] = (uint16_t)(obs->CN0[i] * 1000.0 + 0.5);
            }
        }
    }
    
    /**
     * Convert vector of Obs to array of obsd_t
     * args   : const std::vector<ObsPtr>& obs_vec  I   observation data vector
     *          obsd_t* obsd_array                   O   observation data array
     *          int max_obs                          I   maximum number of observations
     *          int rcv                              I   receiver number (1=rover, 2=base)
     * return : int - number of converted observations
     */
    int convert_obs_to_obsd(const std::vector<ObsPtr>& obs_vec, 
                            obsd_t* obsd_array, int max_obs, int rcv)
    {
        if (!obsd_array || max_obs <= 0) return 0;
        
        int n = std::min((int)obs_vec.size(), max_obs);
        for (int i = 0; i < n; i++) {
            if (obs_vec[i]) {
                obs_to_obsd(obs_vec[i], obsd_array[i], rcv);
            }
        }
        return n;
    }
    
    /**
     * Convert EphemBasePtr to nav_t format (simplified)
     * Note: This is a simplified conversion. Full implementation would require
     * converting all ephemeris types (GPS, GLONASS, Galileo, BeiDou, etc.)
     * args   : const std::map<uint32_t, std::vector<EphemBasePtr>>& nav_map  I   navigation data map
     *          NavData& nav                                                    O   navigation data (RTKLIB format)
     * return : void
     */
    void convert_nav_to_rtklib(const std::map<uint32_t, std::vector<EphemBasePtr>>& nav_map,
                                NavData& nav)
    {
        // Clear existing data
        nav.ephems.clear();
        nav.glo_ephems.clear();
        nav.n = 0;
        nav.ng = 0;
        
        // Set maximum sizes
        if (nav.nmax == 0) nav.nmax = 1024;
        if (nav.ngmax == 0) nav.ngmax = 64;
        
        // Convert ephemeris data
        for (const auto& entry : nav_map) {
            uint32_t sat = entry.first;
            const auto& eph_list = entry.second;
            
            for (const auto& eph_base : eph_list) {
                if (!eph_base) continue;
                
                // Determine satellite system
                int sys = satsys(sat, NULL);
                
                if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_BDS || sys == SYS_IRN) {
                    // GPS/Galileo/QZSS/BeiDou/IRNSS ephemeris
                    EphemPtr eph = std::dynamic_pointer_cast<Ephem>(eph_base);
                    if (eph && nav.ephems.size() < (size_t)nav.nmax) {
                        nav.ephems.push_back(eph);
                        nav.n = nav.ephems.size();
                    }
                } else if (sys == SYS_GLO) {
                    // GLONASS ephemeris
                    GloEphemPtr geph = std::dynamic_pointer_cast<GloEphem>(eph_base);
                    if (geph && nav.glo_ephems.size() < (size_t)nav.ngmax) {
                        nav.glo_ephems.push_back(geph);
                        nav.ng = nav.glo_ephems.size();
                    }
                } else if (sys == SYS_SBS) {
                    // SBAS ephemeris
                    SEphemPtr seph = std::dynamic_pointer_cast<SEphem>(eph_base);
                    if (seph && nav.seph.size() < 64) {
                        nav.seph.push_back(seph);
                    }
                }
            }
        }
    }
    
    /**
     * Synchronize observation epochs between rover and base
     * args   : const std::vector<std::vector<ObsPtr>>& rover_obs  I   rover observations
     *          const std::vector<std::vector<ObsPtr>>& base_obs    I   base observations
     *          std::vector<std::vector<ObsPtr>>& synced_obs        O   synchronized observations
     *          double time_tol                                     I   time tolerance (seconds)
     * return : int - number of synchronized epochs
     */
    int sync_obs_epochs(const std::vector<std::vector<ObsPtr>>& rover_obs,
                       const std::vector<std::vector<ObsPtr>>& base_obs,
                       std::vector<std::vector<ObsPtr>>& synced_obs,
                       double time_tol)
    {
        synced_obs.clear();
        
        if (rover_obs.empty() || base_obs.empty()) {
            return 0;
        }
        
        size_t i = 0, j = 0;
        
        while (i < rover_obs.size() && j < base_obs.size()) {
            if (rover_obs[i].empty() || base_obs[j].empty()) {
                if (rover_obs[i].empty()) i++;
                if (base_obs[j].empty()) j++;
                continue;
            }
            
            // Calculate time difference
            double dt = time_diff(rover_obs[i][0]->time, base_obs[j][0]->time);
            
            if (fabs(dt) < time_tol) {
                // Synchronized epoch - combine observations
                std::vector<ObsPtr> epoch_obs;
                epoch_obs.insert(epoch_obs.end(), rover_obs[i].begin(), rover_obs[i].end());
                epoch_obs.insert(epoch_obs.end(), base_obs[j].begin(), base_obs[j].end());
                synced_obs.push_back(epoch_obs);
                i++;
                j++;
            } else if (dt < 0) {
                // Rover time is earlier
                i++;
            } else {
                // Base time is earlier
                j++;
            }
        }
        
        return synced_obs.size();
    }
    
    /**
     * Separate observations by receiver
     * args   : const std::vector<ObsPtr>& epoch_obs  I   observations for one epoch
     *          std::vector<ObsPtr>& rover_obs         O   rover observations
     *          std::vector<ObsPtr>& base_obs          O   base observations
     * return : void
     * note   : Since Obs doesn't have rcv field, we assume observations are
     *          already separated by file order (rover first, then base)
     */
    void separate_obs_by_rcv(const std::vector<ObsPtr>& epoch_obs,
                             std::vector<ObsPtr>& rover_obs,
                             std::vector<ObsPtr>& base_obs)
    {
        rover_obs.clear();
        base_obs.clear();
        
        // For now, assume first half is rover, second half is base
        // In a real implementation, we would check rcv field or use file index
        size_t n = epoch_obs.size();
        size_t mid = n / 2;
        
        for (size_t i = 0; i < n; i++) {
            if (i < mid) {
                rover_obs.push_back(epoch_obs[i]);
            } else {
                base_obs.push_back(epoch_obs[i]);
            }
        }
    }
}
