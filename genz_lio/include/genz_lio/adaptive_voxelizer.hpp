#pragma once
// GenZ-LIO: Scale-Aware Adaptive Voxelization
// Implements Algorithm 1 from the paper:
// "GenZ-LIO: Generalizable LiDAR-Inertial Odometry Beyond Indoor-Outdoor Boundaries"

#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace genz_lio {

struct AdaptiveVoxelizerParams {
    // Sliding window for scale indicator
    int   Nw          = 5;

    // Voxel size bounds [dmin, dmax] in meters
    double d_min      = 0.02;
    double d_max      = 1.0;

    // Voxelized point count bounds [Nmin, Nmax]
    double N_min      = 1000.0;
    double N_max      = 4000.0;

    // Scene scale threshold (m): Ndesired saturates to Nmax above this
    double tau_m      = 30.0;

    // Setpoint interpolation exponent p > 1
    double p_exp      = 2.0;

    // Normalisation scaling factors (Eq. 4)
    double lambda_p   = 0.1;
    double lambda_d   = 0.2;

    // PD gain bounds
    double Kp_min     = 1e-6;
    double Kp_max     = 1e-4;
    double Kd_min     = 1e-9;
    double Kd_max     = 1e-7;
};

class AdaptiveVoxelizer {
public:
    explicit AdaptiveVoxelizer(const AdaptiveVoxelizerParams &params = AdaptiveVoxelizerParams())
        : p_(params), d_t_(params.d_max * 0.25), e_prev_(0.0), initialized_(false)
    {}

    // Set initial voxel size
    void setInitialVoxelSize(double d0) { d_t_ = std::clamp(d0, p_.d_min, p_.d_max); }

    // Current voxel size
    double voxelSize() const { return d_t_; }

    // -------------------------------------------------------------------------
    // Main entry: run Algorithm 1 for one scan.
    //   scan_in     – deskewed LiDAR points (body frame, any PointT with x,y,z)
    //   dt_scan     – time between consecutive scans [s]
    //   scan_out    – voxelized scan for state update (voxel dt)
    //   scan_merge  – voxelized scan for map integration (voxel dt/2)
    // -------------------------------------------------------------------------
    template <typename PointT>
    void process(const pcl::PointCloud<PointT> &scan_in,
                 double dt_scan,
                 pcl::PointCloud<PointT> &scan_out,
                 pcl::PointCloud<PointT> &scan_merge)
    {
        // ── Step 1: Temporary voxelization with previous voxel size ──────────
        pcl::PointCloud<PointT> V_temp;
        voxelizeCloud(scan_in, d_t_, V_temp);
        int N_temp = static_cast<int>(V_temp.size());

        // ── Step 2: Compute median range (scale indicator mt) ─────────────────
        std::vector<double> ranges;
        ranges.reserve(N_temp);
        for (const auto &pt : V_temp.points)
            ranges.push_back(std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z));

        double m_t = 0.0;
        if (!ranges.empty()) {
            std::nth_element(ranges.begin(), ranges.begin() + ranges.size() / 2, ranges.end());
            m_t = ranges[ranges.size() / 2];
        }

        // Sliding-window smoothing → m_bar_t
        window_.push_back(m_t);
        if ((int)window_.size() > p_.Nw) window_.pop_front();
        double m_bar = 0.0;
        for (double v : window_) m_bar += v;
        m_bar /= static_cast<double>(window_.size());

        // ── Step 3: Scale-informed setpoint Ndesired_t (Eq. 1–2) ─────────────
        double N_desired;
        if (m_bar >= p_.tau_m) {
            N_desired = p_.N_max;
        } else {
            double rho = 1.0 - std::pow(1.0 - m_bar / p_.tau_m, p_.p_exp);
            N_desired  = p_.N_min + (p_.N_max - p_.N_min) * rho;
        }

        // ── Step 4: Tracking error (Eq. 3) ───────────────────────────────────
        double e_t  = N_desired - static_cast<double>(N_temp);
        double de_t = initialized_ ? (e_t - e_prev_) / dt_scan : 0.0;
        initialized_ = true;

        // ── Step 5: Sensitivity-informed gain scheduling (Eq. 4–6) ───────────
        double phi_t    = std::min(m_bar, p_.tau_m) / p_.tau_m;
        double psi_p    = std::min(std::abs(e_t),  p_.lambda_p * N_desired)
                          / (p_.lambda_p * N_desired + 1e-9);
        double psi_d    = std::min(std::abs(de_t), p_.lambda_d * N_desired / dt_scan)
                          / (p_.lambda_d * N_desired / dt_scan + 1e-9);

        double Gamma_p  = std::sqrt(phi_t * psi_p);
        double Gamma_d  = std::sqrt(phi_t * psi_d);

        double Kp = p_.Kp_min + (p_.Kp_max - p_.Kp_min) * Gamma_p;
        double Kd = p_.Kd_min + (p_.Kd_max - p_.Kd_min) * Gamma_d;

        // ── Step 6: PD voxel-size update (Eq. 7–8) ───────────────────────────
        double delta_d = -Kp * e_t - Kd * de_t;
        d_t_ = std::clamp(d_t_ + delta_d, p_.d_min, p_.d_max);
        e_prev_ = e_t;

        // ── Step 7: Bi-resolution voxelization ───────────────────────────────
        voxelizeCloud(scan_in, d_t_ / 2.0, scan_merge);   // fine – for map integration
        voxelizeCloud(scan_merge,  d_t_,   scan_out);      // coarse – for state update

        // ── Diagnostics ──────────────────────────────────────────────────────
        last_m_bar_    = m_bar;
        last_N_desired_= N_desired;
        last_N_temp_   = N_temp;
        last_e_t_      = e_t;
        last_Kp_       = Kp;
        last_Kd_       = Kd;
    }

    // Diagnostics accessors
    double lastScaleIndicator() const { return last_m_bar_; }
    double lastNDesired()       const { return last_N_desired_; }
    int    lastNTemp()          const { return last_N_temp_; }
    double lastError()          const { return last_e_t_; }
    double lastKp()             const { return last_Kp_; }
    double lastKd()             const { return last_Kd_; }

private:
    template <typename PointT>
    static void voxelizeCloud(const pcl::PointCloud<PointT> &in,
                              double voxel_size,
                              pcl::PointCloud<PointT> &out)
    {
        if (in.empty()) { out.clear(); return; }
        pcl::VoxelGrid<PointT> vg;
        typename pcl::PointCloud<PointT>::Ptr ptr(new pcl::PointCloud<PointT>(in));
        vg.setInputCloud(ptr);
        float vs = static_cast<float>(std::max(voxel_size, 0.01));
        vg.setLeafSize(vs, vs, vs);
        vg.filter(out);
    }

    AdaptiveVoxelizerParams p_;
    double d_t_;        // current voxel size
    double e_prev_;     // previous tracking error
    bool   initialized_;
    std::deque<double> window_;

    // Diagnostics
    double last_m_bar_    = 0.0;
    double last_N_desired_= 0.0;
    int    last_N_temp_   = 0;
    double last_e_t_      = 0.0;
    double last_Kp_       = 0.0;
    double last_Kd_       = 0.0;
};

} // namespace genz_lio
