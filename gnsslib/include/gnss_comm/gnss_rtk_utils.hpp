/**
 * gnss_rtk_utils.hpp: RTK positioning utility functions header
 */

#ifndef GNSS_RTK_UTILS_HPP_
#define GNSS_RTK_UTILS_HPP_

#include "gnss_constant.hpp"
#include <vector>
#include <map>

namespace gnss_comm
{
    // Forward declarations
    class Obs;
    typedef std::shared_ptr<Obs> ObsPtr;
    class EphemBase;
    typedef std::shared_ptr<EphemBase> EphemBasePtr;
    
    /**
     * Convert Obs to obsd_t format
     */
    void obs_to_obsd(const ObsPtr& obs, obsd_t& obsd, int rcv);
    
    /**
     * Convert vector of Obs to array of obsd_t
     */
    int convert_obs_to_obsd(const std::vector<ObsPtr>& obs_vec, 
                            obsd_t* obsd_array, int max_obs, int rcv);
    
    /**
     * Convert navigation data map to NavData
     */
    void convert_nav_to_rtklib(const std::map<uint32_t, std::vector<EphemBasePtr>>& nav_map,
                                NavData& nav);
    
    /**
     * Synchronize observation epochs between rover and base
     */
    int sync_obs_epochs(const std::vector<std::vector<ObsPtr>>& rover_obs,
                       const std::vector<std::vector<ObsPtr>>& base_obs,
                       std::vector<std::vector<ObsPtr>>& synced_obs,
                       double time_tol = 0.1);
    
    /**
     * Separate observations by receiver
     */
    void separate_obs_by_rcv(const std::vector<ObsPtr>& epoch_obs,
                             std::vector<ObsPtr>& rover_obs,
                             std::vector<ObsPtr>& base_obs);
}

#endif

