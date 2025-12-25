/**
 * gnss_rtk.cpp: RTK positioning implementation
 * 
 * This file implements RTK positioning similar to RTKLIB's postpos.c and rtkpos.c
 * 
 * Copyright (C) 2024
 * Adapted from RTKLIB by T.TAKASU
 */

#include "gnss_comm/rinex_reader.hpp"
#include "gnss_comm/gnss_constant.hpp"
#include "gnss_comm/gnss_utility.hpp"
#include "gnss_comm/rtk_common.hpp"
#include "gnss_comm/gnss_rtk_utils.hpp"
#include "gnss_comm/gnss_ros.hpp"
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <ros/ros.h>
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <gnss_comm/GnssMeasMsg.h>
#include <glog/logging.h>

using namespace gnss_comm;

// static std::string time2str(const gtime_t &t);
static void print_solution(const sol_t &sol, int epoch, const double *rb = NULL);
static int interpolate_obs(const std::vector<ObsPtr>& obs1, 
                          const std::vector<ObsPtr>& obs2,
                          gtime_t target_time, obsd_t *obs, int max_obs);

static void ecef2pos(const double *r, double *pos)
{
    const double RE_WGS84 = 6378137.0;
    const double FE_WGS84 = 1.0/298.257223563;
    double e2=FE_WGS84*(2.0-FE_WGS84),r2=r[0]*r[0]+r[1]*r[1],z,zk,v=RE_WGS84,sinp;
    for (z=r[2],zk=0.0;fabs(z-zk)>=1E-4;) {
        zk=z;
        sinp=z/sqrt(r2+z*z);
        v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);
        z=r[2]+v*e2*sinp;
    }
    pos[0]=r2>1E-12?atan(z/sqrt(r2)):(r[2]>0.0?M_PI/2.0:-M_PI/2.0);
    pos[1]=r2>1E-12?atan2(r[1],r[0]):0.0;
    pos[2]=sqrt(r2+z*z)-v;
}

// Comparator for obsd_t
bool compare_obsd(const obsd_t& a, const obsd_t& b) {
    return a.sat < b.sat;
}

#include <fstream>
// ...

// Global variables for RTK state
static std::vector<std::vector<ObsPtr>> base_obs;
static rtk_t rtk;
static NavData nav_data;
static ros::Publisher pub_pvt;
static int epoch_counter = 0;
static size_t base_idx = 0;
static std::ofstream sol_file;
// Statistics
static int stat_total_epochs = 0;
static int stat_fixed_epochs = 0;

void meas_callback(const gnss_comm::GnssMeasMsgConstPtr& msg) {
    // std::cout << "DEBUG: Callback called" << std::endl;
    // Convert ROS message to ObsPtr
    std::vector<ObsPtr> rov_obs = gnss_comm::msg2meas(msg);
    if (rov_obs.empty()) {
         std::cout << "DEBUG: rov_obs empty" << std::endl;
         return;
    }
    
    // Use ROS time (sim time) to handle data jumps
    // Requires: /use_sim_time=true and rosbag play --clock
    ros::Time t_ros = ros::Time::now();
    
    // Safety check: if sim time is not active or valid (e.g. 0), fall back to msg time?
    // But since we want to fix jumps, we prefer t_ros if it is non-zero.
    if (!t_ros.isZero()) {
         gtime_t t_now_gpst;
         t_now_gpst.time = t_ros.sec;
         t_now_gpst.sec = t_ros.nsec * 1e-9;
         
         // ros::Time is UTC-like, so convert UTC->GPST
         t_now_gpst = utc2gpst(t_now_gpst);
         
         // Overwrite observation time
         for(size_t i=0; i<rov_obs.size(); i++) {
             rov_obs[i]->time = t_now_gpst;
         }
    } else {
         // std::cout << "DEBUG: ros::Time::now() is zero! Waiting for clock..." << std::endl;
    }
    
    // Convert to double time
    double t_rov = rov_obs[0]->time.time + rov_obs[0]->time.sec;
    
    // Check for time jump
    if (base_idx > 0 && base_idx < base_obs.size()) {
        double t_base = base_obs[base_idx][0]->time.time + base_obs[base_idx][0]->time.sec;
        if (t_rov < t_base - 10.0) { // Rover is behind Base by > 10s
             // std::cout << "Time reset detected: Rov " << std::fixed << t_rov << " Base " << t_base << std::endl;
             base_idx = 0;
        }
    }

    // Find base observations
    while (base_idx + 1 < base_obs.size() && 
           time_diff(rov_obs[0]->time, base_obs[base_idx+1][0]->time) >= 0.0) {
        base_idx++;
    }
    
    // Prepare observation buffer
    obsd_t obsd_buf[MAXOBS * 2];
    int n = 0;
    
    int nu = convert_obs_to_obsd(rov_obs, obsd_buf, MAXOBS, 1);
    // Sort Rover Observations
    std::sort(obsd_buf, obsd_buf + nu, compare_obsd);
    n += nu;
    
    // Authenticate base obs and interpolate
    int nr = 0;
    if (base_idx + 1 < base_obs.size()) {
         double t_b1 = base_obs[base_idx][0]->time.time + base_obs[base_idx][0]->time.sec;
         double t_b2 = base_obs[base_idx+1][0]->time.time + base_obs[base_idx+1][0]->time.sec;
         
         if (t_rov >= t_b1 && t_rov <= t_b2) {
             if (t_b2 - t_b1 < 10.0) { // Relaxed gap to 10s
                 nr = interpolate_obs(base_obs[base_idx], base_obs[base_idx+1], 
                                      rov_obs[0]->time, obsd_buf + n, MAXOBS);
             }
         } else {
             if (fabs(t_rov - t_b1) < 1.5) {
                 nr = convert_obs_to_obsd(base_obs[base_idx], obsd_buf + n, MAXOBS, 2);
             } else if (fabs(t_rov - t_b2) < 1.5) {
                 nr = convert_obs_to_obsd(base_obs[base_idx+1], obsd_buf + n, MAXOBS, 2);
             }
         }
    } else if (!base_obs.empty()) {
         if (fabs(time_diff(rov_obs[0]->time, base_obs[base_idx][0]->time)) < 2.0) {
             nr = convert_obs_to_obsd(base_obs[base_idx], obsd_buf + n, MAXOBS, 2);
         }
    }
    
    if (nr > 0) {
        // Sort Base Observations
        std::sort(obsd_buf + nu, obsd_buf + nu + nr, compare_obsd);
    }
    n += nr;
    
    if (nr == 0) {
        // ROS_WARN("No base obs found for rover time: %f", t_rov);
        return;
    }
    
    // Call rtkpos
    int stat = rtkpos(&rtk, obsd_buf, n, &nav_data);
    
    // Output
    print_solution(rtk.sol, epoch_counter++, rtk.rb);
    
    // Determine if solution is valid for publishing
    if (rtk.sol.rr[0] != 0.0) {
        gnss_comm::GnssPVTSolnMsg sol_msg;
        double pos[3];
        ecef2pos(rtk.sol.rr, pos);
        
        uint32_t week;
        double tow = time2gpst(rtk.sol.time, &week);
        
        sol_msg.time.week = week;
        sol_msg.time.tow = tow;
        sol_msg.latitude = pos[0] * R2D;
        sol_msg.longitude = pos[1] * R2D;
        sol_msg.altitude = pos[2];
        
        // Baseline
        double baseline[3];
        for (int k=0; k<3; k++) baseline[k] = rtk.sol.rr[k] - rtk.rb[k];
        sol_msg.baseline_length = sqrt(baseline[0]*baseline[0] + baseline[1]*baseline[1] + baseline[2]*baseline[2]);

        if (rtk.sol.stat == SOLQ_FIX) sol_msg.carr_soln = 2;
        else if (rtk.sol.stat == SOLQ_FLOAT) sol_msg.carr_soln = 1;
        else sol_msg.carr_soln = 0;
        
        sol_msg.num_sv = rtk.sol.ns;
        
        if (rtk.sol.stat == SOLQ_FIX) sol_msg.fix_type = 3; // Fixed
        else if (rtk.sol.stat == SOLQ_FLOAT) sol_msg.fix_type = 2; // Float
        else if (rtk.sol.ns >= 4) sol_msg.fix_type = 1; // Single/3D
        else sol_msg.fix_type = 0;
        
        pub_pvt.publish(sol_msg);
    }
}
/**
 * RTK positioning main function
 * Similar to RTKLIB's postpos() function
 */
int main(int argc, char* argv[])
{
    // Initialize Google Logging
    google::InitGoogleLogging(argv[0]);
    
    std::cout << "=== RTK Positioning (VER 2) ===" << std::endl;
    
    // File paths
    // std::string rover_obs_file = "/home/rzb/gnsslib_ws/RINEX.txt";
    std::string rover_obs_file = ""; // Use ROS topic
    std::string base_obs_file = "/home/rzb/gnsslib_ws/hkks334x.20o";
    std::string nav_file = "/home/rzb/gnsslib_ws/hkks334x.20n";
    
    std::cout << "流动站观测文件: " << rover_obs_file << std::endl;
    std::cout << "基站观测文件: " << base_obs_file << std::endl;
    std::cout << "导航电文文件: " << nav_file << std::endl;
    
    // Time window (0 means read all)
    gtime_t ts = {0, 0};
    gtime_t te = {0, 0};
    double ti = 0.0;  // Time interval (0 means all)
    
    // Processing options
    PrcOpt prcopt;
    prcopt.mode = PMODE_KINEMA;  // RTK kinematic mode
    
    // Initialize ROS
    ros::init(argc, argv, "gnss_rtk_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    // Get parameter from launch file
    nh_priv.param<std::string>("rover_obs_file", rover_obs_file, "");
    
    // Print file paths again to confirm
    std::cout << "流动站观测文件 (Param): " << rover_obs_file << std::endl;
    
    pub_pvt = nh.advertise<gnss_comm::GnssPVTSolnMsg>("rtk_solution", 100);
    prcopt.nf = 2;  // Use L1 and L2
    prcopt.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_BDS;  // Multi-GNSS
    prcopt.elmin = 10.0 * M_PI / 180.0;  // Elevation mask: 10 degrees
    prcopt.sateph = EPHOPT_BRDC;  // Use broadcast ephemeris
    
    // Open output file
    sol_file.open("/home/rzb/gnsslib_ws/gnss_comm-main/output/rtk_result.txt");
    if (sol_file.is_open()) {
        sol_file << "历元 | 时间 | 状态 | 卫星数 | X(m) | Y(m) | Z(m) | 基线长度(m)" << std::endl;
        sol_file << "----------------------------------------------------------------" << std::endl;
    } else {
        std::cerr << "Warning: Could not open output file" << std::endl;
    }
    
    // Default standard deviations and process noise
    prcopt.std[0] = 30.0;          // Initial bias sigma (m)
    prcopt.std[1] = 0.03;          // Initial iono sigma (m)
    prcopt.std[2] = 0.3;           // Initial trop sigma (m)
    prcopt.prn[0] = 1.0E-4;        // Bias process noise (m/sqrt(s))
    prcopt.prn[1] = 1.0E-3;        // Iono process noise (m/sqrt(s))
    prcopt.prn[2] = 1.0E-4;        // Trop process noise (m/sqrt(s))
    prcopt.prn[3] = 1.0;           // Accel horizontal
    prcopt.prn[4] = 1.0;           // Accel vertical
    
    prcopt.modear = ARMODE_CONT;  // Continuous AR mode
    prcopt.glomodear = 2;  // Auto GLONASS AR
    prcopt.tropopt = TROPOPT_SAAS;  // Saastamoinen troposphere model
    prcopt.ionoopt = IONOOPT_BRDC;  // Broadcast ionosphere model
    prcopt.dynamics = 0;  // Static positioning
    prcopt.soltype = 0;  // Forward solution
    prcopt.niter = 5;  // Number of iterations
    prcopt.maxout = 30;  // Max number of outliers
    prcopt.minfix = 10;  // Min number of fixes for validation
    prcopt.minlock = 0;  // Min lock count
    prcopt.maxtdiff = 30.0;  // Max time difference
    prcopt.maxinno = 30.0;  // Max innovation
    prcopt.maxgdop = 30.0;  // Max GDOP
    prcopt.thresar[0] = 2.0;  // AR threshold
    prcopt.thresar[1] = 0.9999;  // AR threshold
    prcopt.err[1] = 0.003;  // Phase error (m)
    prcopt.err[2] = 0.003;  // Phase error (m)
    prcopt.err[3] = 0.0;  // Phase error (m)
    prcopt.err[4] = 0.0;  // Doppler frequency (Hz)
    prcopt.eratio[0] = 100.0;  // Error ratio
    prcopt.eratio[1] = 100.0;  // Error ratio
    
    // Read observation and navigation data
    // Read observation and navigation data
    std::vector<std::string> infiles;
    std::vector<int> index;
    
    // Rover observation (index 0) - DEPRECATED for ROS topic
    // infiles.push_back(rover_obs_file);
    // index.push_back(0);
    
    // Base observation (index 1)
    infiles.push_back(base_obs_file);
    index.push_back(1);
    
    // Navigation file (index 2)
    infiles.push_back(nav_file);
    index.push_back(2);
    
    // Use global base_obs
    // std::vector<std::vector<ObsPtr>> obs; 
    std::map<uint32_t, std::vector<EphemBasePtr>> nav;
    std::map<uint32_t, StaInfo> sta;
    
    std::cout << "\n1. 读取基站观测和导航数据..." << std::endl;
    if (!readobsnav(ts, te, ti, infiles, index, prcopt, base_obs, nav, sta)) {
        std::cerr << "错误: 无法读取观测或导航数据" << std::endl;
        return 1;
    }
    
    std::cout << "成功读取 " << base_obs.size() << " 个历元的基站观测数据" << std::endl;
    std::cout << "成功读取 " << nav.size() << " 颗卫星的导航数据" << std::endl;
    
    if (base_obs.empty()) {
        std::cerr << "错误: 没有基站观测数据" << std::endl;
        return 1;
    }

    // Convert navigation data to global nav_data
    nav_data.nmax = 1024;
    nav_data.ngmax = 256;
    convert_nav_to_rtklib(nav, nav_data);
    
    // Convert PrcOpt to prcopt_t for RTK functions
    // Note: prcopt_t has limited fields, only set available ones
    prcopt_t prcopt_rtk = {0};
    prcopt_rtk.mode = prcopt.mode;
    prcopt_rtk.nf = prcopt.nf;
    prcopt_rtk.elmin = prcopt.elmin;
    prcopt_rtk.ionoopt = prcopt.ionoopt;
    prcopt_rtk.tropopt = prcopt.tropopt;
    prcopt_rtk.navsys = prcopt.navsys;
    prcopt_rtk.maxgdop = prcopt.maxgdop;
    prcopt_rtk.maxtdiff = prcopt.maxtdiff;
    prcopt_rtk.maxinno = prcopt.maxinno;
    prcopt_rtk.sateph = prcopt.sateph;
    
    // Copy std and prn
    for (int i=0; i<3; i++) prcopt_rtk.std[i] = prcopt.std[i];
    for (int i=0; i<6; i++) prcopt_rtk.prn[i] = prcopt.prn[i];
    for (int i=0; i<5; i++) prcopt_rtk.err[i] = prcopt.err[i];
    for (int i=0; i<NFREQ; i++) prcopt_rtk.eratio[i] = prcopt.eratio[i];
    
    // Increase measurement noise for robustness
    prcopt_rtk.err[1] = 0.01; // 1cm phase error
    prcopt_rtk.err[2] = 0.01; // 1cm elevation dependent error
    
    prcopt_rtk.thresar[0] = 2.0; // AR threshold ratio
    prcopt_rtk.niter = 5; // Filter iterations
    prcopt_rtk.modear = prcopt.modear == 0 ? 1 : prcopt.modear; // Ensure AR is on (1=continuous)
    prcopt_rtk.maxout = 5;
    prcopt_rtk.minlock = 5;
    prcopt_rtk.minfix = 10;
    
    // Initialize RTK structure
    // rtk_t rtk; // Use global
    rtkinit(&rtk, &prcopt_rtk);
    
    // Set base station position (if available from station info)
    // In RTKLIB, this comes from RINEX header or user input
    // Index 1 corresponds to base station file in our configuration
    if (sta.find(1) != sta.end() && sta[1].pos[0] != 0.0) {
        // Base station position from RINEX header
        rtk.rb[0] = sta[1].pos[0];
        rtk.rb[1] = sta[1].pos[1];
        rtk.rb[2] = sta[1].pos[2];
        // Also update options for reference
        prcopt_rtk.rb[0] = sta[1].pos[0];
        prcopt_rtk.rb[1] = sta[1].pos[1];
        prcopt_rtk.rb[2] = sta[1].pos[2];
        
        std::cout << "基站位置 (从RINEX头读取): "
                  << std::fixed << std::setprecision(3)
                  << rtk.rb[0] << " " << rtk.rb[1] << " " << rtk.rb[2] << std::endl;
    } else {
        // Use predefined position for HKKS
        std::cout << "警告: 基站位置未设置 (索引1未找到或为0)，将使用预设位置 (HKKS)" << std::endl;
        
        rtk.rb[0] = -2429525.8966;
        rtk.rb[1] =  5377816.6360;
        rtk.rb[2] =  2412152.7354;
        
        prcopt_rtk.rb[0] = rtk.rb[0];
        prcopt_rtk.rb[1] = rtk.rb[1];
        prcopt_rtk.rb[2] = rtk.rb[2];

        std::cout << "使用预设基站位置: " 
                  << std::fixed << std::setprecision(3)
                  << rtk.rb[0] << " " << rtk.rb[1] << " " << rtk.rb[2] << std::endl;
    }
    
    // Initialize rover position for SPP (to avoid 0,0,0 start)
    rtk.sol.rr[0] = rtk.rb[0];
    rtk.sol.rr[1] = rtk.rb[1];
    rtk.sol.rr[2] = rtk.rb[2];
    
    std::cout << "\n4. 开始RTK解算..." << std::endl;
    std::cout << "历元 | 时间 | 状态 | 卫星数 | X(m) | Y(m) | Z(m) | 基线长度(m)" << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;
    
    // Process each epoch
    
    // Base observation and navigation data
    // std::vector<std::vector<ObsPtr>> base_obs; // Moved to global
    
    // nav_data moved to global
    nav_data.nmax = 1024;
    nav_data.ngmax = 256;
    convert_nav_to_rtklib(nav, nav_data);
    
    std::cout << "导航数据: GPS/GAL/QZS/BDS=" << nav_data.n 
              << ", GLONASS=" << nav_data.ng << std::endl;
    
    if (rover_obs_file == "") {
        // ROS Mode
        std::cout << "进入 ROS 回调模式 (话题: /ublox_driver/range_meas)..." << std::endl;
        ros::Subscriber sub_meas = nh.subscribe("/ublox_driver/range_meas", 100, meas_callback);
        ros::spin();
    } else {
        // File Mode
        std::cout << "进入文件处理模式 (流动站: " << rover_obs_file << ")..." << std::endl;
        
        std::vector<std::string> rov_files;
        std::vector<int> rov_idx;
        rov_files.push_back(rover_obs_file);
        rov_idx.push_back(0);
        
        std::vector<std::vector<ObsPtr>> rover_obs;
        std::map<uint32_t, std::vector<EphemBasePtr>> rov_nav;
        std::map<uint32_t, StaInfo> rov_sta;
        
        if (!readobsnav(ts, te, ti, rov_files, rov_idx, prcopt, rover_obs, rov_nav, rov_sta)) {
             // If rover_obs is not empty, ignore the error (likely just weak nav data check)
             if (rover_obs.empty()) {
                  std::cerr << "错误: 无法读取流动站文件或文件为空" << std::endl;
                  return 1;
             } else {
                  std::cout << "警告: readobsnav 返回失败，但已读取到 " << rover_obs.size() << " 个历元的观测数据，继续处理。" << std::endl;
             }
        }
        
        // Process file epochs
        obsd_t obsd_buf[MAXOBS * 2];
        for (size_t i=0; i<rover_obs.size(); i++) {
             if (rover_obs[i].empty()) continue;
             
             // Time-based matching with base
             while (base_idx + 1 < base_obs.size() && 
                    time_diff(rover_obs[i][0]->time, base_obs[base_idx+1][0]->time) >= 0.0) {
                 base_idx++;
             }
             
             int n = 0;
             int nu = convert_obs_to_obsd(rover_obs[i], obsd_buf, MAXOBS, 1);
             n += nu;
             
             int nr = 0;
             // Interpolation logic
             if (base_idx + 1 < base_obs.size()) {
                 double t_rov = rover_obs[i][0]->time.time + rover_obs[i][0]->time.sec;
                 double t_b1 = base_obs[base_idx][0]->time.time + base_obs[base_idx][0]->time.sec;
                 double t_b2 = base_obs[base_idx+1][0]->time.time + base_obs[base_idx+1][0]->time.sec;
                 
                 if (t_rov >= t_b1 && t_rov <= t_b2) {
                     if (t_b2 - t_b1 < 6.0) {
                         nr = interpolate_obs(base_obs[base_idx], base_obs[base_idx+1], 
                                              rover_obs[i][0]->time, obsd_buf + n, MAXOBS);
                     }
                 } else {
                     if (fabs(t_rov - t_b1) < 1.0) nr = convert_obs_to_obsd(base_obs[base_idx], obsd_buf + n, MAXOBS, 2);
                     else if (fabs(t_rov - t_b2) < 1.0) nr = convert_obs_to_obsd(base_obs[base_idx+1], obsd_buf + n, MAXOBS, 2);
                 }
             } else if (!base_obs.empty()) {
                 if (fabs(time_diff(rover_obs[i][0]->time, base_obs[base_idx][0]->time)) < 2.0) {
                     nr = convert_obs_to_obsd(base_obs[base_idx], obsd_buf + n, MAXOBS, 2);
                 }
             }
             n += nr;
             
             if (nr == 0) continue;
             
             int stat = rtkpos(&rtk, obsd_buf, n, &nav_data);
             (void)stat;
             print_solution(rtk.sol, i, rtk.rb);
             
             // Publish to ROS even in file mode
             if (ros::ok() && rtk.sol.rr[0] != 0.0) {
                 gnss_comm::GnssPVTSolnMsg sol_msg;
                 double pos[3];
                 ecef2pos(rtk.sol.rr, pos);
                 uint32_t week;
                 double tow = time2gpst(rtk.sol.time, &week);
                 sol_msg.time.week = week; sol_msg.time.tow = tow;
                 sol_msg.latitude = pos[0] * R2D;
                 sol_msg.longitude = pos[1] * R2D;
                 sol_msg.altitude = pos[2];
                 double baseline[3];
                 for (int k=0; k<3; k++) baseline[k] = rtk.sol.rr[k] - rtk.rb[k];
                 sol_msg.baseline_length = sqrt(baseline[0]*baseline[0] + baseline[1]*baseline[1] + baseline[2]*baseline[2]);
                 if (rtk.sol.stat == SOLQ_FIX) sol_msg.carr_soln = 2;
                 else if (rtk.sol.stat == SOLQ_FLOAT) sol_msg.carr_soln = 1;
                 else sol_msg.carr_soln = 0;
                 sol_msg.num_sv = rtk.sol.ns;
                 if (rtk.sol.stat == SOLQ_FIX) sol_msg.fix_type = 3;
                 else if (rtk.sol.stat == SOLQ_FLOAT) sol_msg.fix_type = 2;
                 else if (rtk.sol.ns >= 4) sol_msg.fix_type = 1;
                 else sol_msg.fix_type = 0;
                 pub_pvt.publish(sol_msg);
             }
        }
    }
    
    // Print Statistics
    double fix_rate = stat_total_epochs > 0 ? (double)stat_fixed_epochs / stat_total_epochs * 100.0 : 0.0;
    std::cout << "\n=== 解算统计 ===" << std::endl;
    std::cout << "总历元: " << stat_total_epochs << std::endl;
    std::cout << "固定历元: " << stat_fixed_epochs << std::endl;
    std::cout << "固定率: " << std::fixed << std::setprecision(1) << fix_rate << "%" << std::endl;
    
    // Cleanup
    rtkfree(&rtk);
    
    return 0;
}

/**
 * Convert gtime_t to string
 */
/**
 * Convert gtime_t to string (YY/MM/DD HH:MM:SS.SSS)
 */
static std::string time2str(const gtime_t &t)
{
    double ep[6];
    time2epoch(t, ep);
    std::stringstream ss;
    ss << std::setfill('0')
       << std::setw(2) << (int)ep[0]%100 << "/"
       << std::setw(2) << (int)ep[1] << "/"
       << std::setw(2) << (int)ep[2] << " "
       << std::setw(2) << (int)ep[3] << ":"
       << std::setw(2) << (int)ep[4] << ":"
       << std::setw(6) << std::fixed << std::setprecision(3) << ep[5];
    return ss.str();
}



/**
 * Print solution
 */
static void print_solution(const sol_t &sol, int epoch, const double *rb)
{
    // Update stats
    if (sol.stat != SOLQ_NONE) {
        stat_total_epochs++;
        if (sol.stat == SOLQ_FIX) stat_fixed_epochs++;
    }

    // Periodic stats (every 100 epochs)
    if (stat_total_epochs % 100 == 0 && stat_total_epochs > 0) {
        double current_rate = (double)stat_fixed_epochs / stat_total_epochs * 100.0;
        std::cout << "--- 统计 (N=" << stat_total_epochs << "): 固定率 " << std::fixed << std::setprecision(1) << current_rate << "% ---" << std::endl;
    }

    std::stringstream ss;
    ss << std::setw(5) << epoch << " | "
       << time2str(sol.time) << " | ";
    
    std::cout << ss.str();
    if (sol_file.is_open()) sol_file << ss.str();
    
    // Solution status
    const char* stat_str = "NONE";
    switch (sol.stat) {
        case SOLQ_NONE: stat_str = "NONE"; break;
        case SOLQ_SINGLE: stat_str = "SINGLE"; break;
        case SOLQ_DGPS: stat_str = "DGPS"; break;
        case SOLQ_FLOAT: stat_str = "FLOAT"; break;
        case SOLQ_FIX: stat_str = "FIX"; break;
        default: stat_str = "UNK"; break;
    }
    // Use stringstream for remaining output
    std::stringstream ss2;
    ss2 << std::setw(6) << stat_str << " | "
        << std::setw(3) << sol.ns << " | "
        << "R:" << std::fixed << std::setprecision(1) << std::setw(4) << sol.ratio << " | ";
     
     // Output position
     if (sol.rr[0] != 0.0 || sol.rr[1] != 0.0 || sol.rr[2] != 0.0) {
         ss2 << std::fixed << std::setprecision(3)
             << std::setw(12) << sol.rr[0] << " "
             << std::setw(12) << sol.rr[1] << " "
             << std::setw(12) << sol.rr[2] << " | ";
         
         // Baseline length if base station is available
         if (rb && (rb[0] != 0.0 || rb[1] != 0.0 || rb[2] != 0.0)) {
             double dx = sol.rr[0] - rb[0];
             double dy = sol.rr[1] - rb[1];
             double dz = sol.rr[2] - rb[2];
             double b_len = sqrt(dx*dx + dy*dy + dz*dz);
             ss2 << std::fixed << std::setprecision(3) << std::setw(10) << b_len;
         } else {
              ss2 << "N/A";
         }
         ss2 << std::endl;
     } else {
         ss2 << "N/A | N/A | N/A | N/A" << std::endl;
     }
     
     std::cout << ss2.str() << std::flush;
     if (sol_file.is_open()) sol_file << ss2.str() << std::flush;
}

/**
 * Interpolate base observations
 */
static int interpolate_obs(const std::vector<ObsPtr>& obs1, 
                          const std::vector<ObsPtr>& obs2,
                          gtime_t target_time, obsd_t *obs, int max_obs)
{
    obsd_t buf1[MAXOBS], buf2[MAXOBS];
    
    int n1 = convert_obs_to_obsd(obs1, buf1, MAXOBS, 2);
    int n2 = convert_obs_to_obsd(obs2, buf2, MAXOBS, 2);
    
    if (n1 == 0 || n2 == 0) {
         std::cout << "DEBUG: interpolate_obs empty input n1=" << n1 << " n2=" << n2 << " max_obs=" << max_obs << std::endl;
        return 0;
    }

    double t1 = buf1[0].time.time + buf1[0].time.sec;
    double t2 = buf2[0].time.time + buf2[0].time.sec;
    double t = target_time.time + target_time.sec;
    
    // Validate time
    if (t < t1 || t > t2 || t2 <= t1) {
         std::cout << "DEBUG: interpolate_obs time mismatch t=" << std::fixed << t << " t1=" << t1 << " t2=" << t2 << std::endl;
        return 0;
    }
    
    double alpha = (t - t1) / (t2 - t1);
    int n = 0;
    
    for (int i=0; i<n1; i++) {
        for (int j=0; j<n2; j++) {
            if (buf1[i].sat != buf2[j].sat) continue;
            
            if (n >= max_obs) break;
            
            obs[n] = buf1[i];
            obs[n].time = target_time;
            
            // Interpolate P and L
            for (int f=0; f<NFREQ; f++) {
                if (buf1[i].L[f] == 0.0 || buf2[j].L[f] == 0.0 ||
                    buf1[i].P[f] == 0.0 || buf2[j].P[f] == 0.0) {
                    obs[n].L[f] = obs[n].P[f] = 0.0;
                    continue;
                }
                
                // Check cycle slip or large gap
                if (fabs(buf1[i].L[f] - buf2[j].L[f]) > 200000.0) { // arbitrary threshold for cycle slip
                     obs[n].L[f] = obs[n].P[f] = 0.0;
                     continue;
                }
                
                obs[n].P[f] = (1.0 - alpha) * buf1[i].P[f] + alpha * buf2[j].P[f];
                obs[n].L[f] = (1.0 - alpha) * buf1[i].L[f] + alpha * buf2[j].L[f];
            }
            n++;
            break;
        }
    }
    return n;
}


