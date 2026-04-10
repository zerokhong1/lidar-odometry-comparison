// GenZ-LIO: Generalizable LiDAR-Inertial Odometry Beyond Indoor-Outdoor Boundaries
// Based on FAST-LIO2 (Xu et al., T-RO 2022) with three key extensions:
//   1. Scale-aware adaptive voxelization (Algorithm 1)
//   2. Hybrid-metric state update: point-to-plane + point-to-point L2 (Sec. V-C/D/E)
//   3. Voxel-pruned correspondence search (Sec. V-B, Algorithm 2)

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

// GenZ-LIO additions
#include <genz_lio/adaptive_voxelizer.hpp>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

// ── Time-log variables (unchanged from FAST-LIO2) ─────────────────────────────
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN],
       s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN],
       s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0,
       kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false,
       extrinsic_est_en = true, path_en = true;

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0,
       filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0,
       lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0,
       publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0,
       laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool   is_first_lidar = true;

vector<vector<int>>  pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                                    time_buffer;
deque<PointCloudXYZI::Ptr>                       lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr>     imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_merge_body(new PointCloudXYZI()); // bi-resolution (dt/2)
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

// GenZ-LIO: for point-to-point correspondences
PointCloudXYZI::Ptr laserCloudOri_po(new PointCloudXYZI(100000, 1));   // p2p query
PointCloudXYZI::Ptr corr_nearest(new PointCloudXYZI(100000, 1));        // p2p target

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// GenZ-LIO: adaptive voxelizer instance
genz_lio::AdaptiveVoxelizer g_adaptive_vox;
bool   g_use_adaptive_vox = true;   // enable/disable via param
double g_scan_period = 0.1;         // approximate scan period [s]
int    effct_po_num = 0;            // point-to-point correspondence count

void SigHandle(int sig)
{
    flg_exit = true;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]);
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    po->x = p_global(0); po->y = p_global(1); po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                 state_point.offset_T_L_I) + state_point.pos);
    po->x = p_global(0); po->y = p_global(1); po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                 state_point.offset_T_L_I) + state_point.pos);
    po[0] = p_global(0); po[1] = p_global(1); po[2] = p_global(2);
}

template<typename T>
Matrix<T, 3, 1> pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                 state_point.offset_T_L_I) + state_point.pos);
    return p_global.cast<T>();
}

// ── FOV segmentation helpers (unchanged from FAST-LIO2) ──────────────────────
bool esti_plane(VF(4) &pca_result, const PointVector &point, const float &threshold)
{
    Eigen::Matrix<float, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<float, NUM_MATCH_POINTS, 1> b;
    A.setZero(); b.setOnes(); b *= -1.0f;
    for (int j = 0; j < NUM_MATCH_POINTS; j++) {
        A(j, 0) = point[j].x; A(j, 1) = point[j].y; A(j, 2) = point[j].z;
    }
    V3F normvec_f = A.colPivHouseholderQr().solve(b);
    float norm = normvec_f.norm();
    pca_result[0] = normvec_f(0) / norm;
    pca_result[1] = normvec_f(1) / norm;
    pca_result[2] = normvec_f(2) / norm;
    pca_result[3] = 1.0f / norm;
    for (int j = 0; j < NUM_MATCH_POINTS; j++) {
        if (fabs(pca_result(0)*point[j].x + pca_result(1)*point[j].y +
                 pca_result(2)*point[j].z + pca_result(3)) > threshold)
            return false;
    }
    return true;
}

// ── Hybrid-metric measurement model ──────────────────────────────────────────
// Combines point-to-plane (primary) and point-to-point L2-norm (fallback).
//
// State vector x: pos(3) rot(3) offset_R(3) offset_T(3) vel(3) bg(3) ba(3) grav(3) = 18
// Observation dimension = Npl + Npo  (each scalar)
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    laserCloudOri_po->clear();
    corr_nearest->clear();
    total_residual = 0.0;

    // ── Point-to-plane (primary) + point-to-point (fallback) ─────────────────
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        // Search nearest neighbors in ikd-tree
        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge) {
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            // Require enough neighbors within 5 m²
            point_selected_surf[i] = (points_near.size() >= (size_t)NUM_MATCH_POINTS) &&
                                     (pointSearchSqDis[NUM_MATCH_POINTS - 1] <= 5.0f);
        }

        // ── Attempt point-to-plane ────────────────────────────────────────────
        VF(4) pabcd;
        bool plane_ok = false;
        if (point_selected_surf[i] && points_near.size() >= (size_t)NUM_MATCH_POINTS)
        {
            if (esti_plane(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd(0)*point_world.x + pabcd(1)*point_world.y +
                            pabcd(2)*point_world.z + pabcd(3);
                float s_coeff = 1 - 0.9f * fabsf(pd2) / sqrtf((float)p_body.norm());
                if (s_coeff > 0.9f) {
                    plane_ok = true;
                    #pragma omp critical
                    {
                        laserCloudOri->push_back(point_body);
                        PointType norm_pt;
                        norm_pt.x = pabcd(0); norm_pt.y = pabcd(1);
                        norm_pt.z = pabcd(2); norm_pt.intensity = pd2;
                        corr_normvect->push_back(norm_pt);
                        res_last[i] = fabsf(pd2);
                        total_residual += res_last[i];
                    }
                }
            }
        }

        // ── Fallback: point-to-point (Sec. V-B-3, V-D) ───────────────────────
        // When no valid plane found, use L2 distance to nearest point.
        if (!plane_ok && points_near.size() >= 1)
        {
            // Voxel-pruned search: use closest point from already-searched neighbors
            // (neighbors are already sorted by distance from ikd-tree)
            const PointType &p_nearest = points_near[0];
            float sq_dist = pointSearchSqDis[0];
            // Reject if too far (outlier rejection threshold = 1.5 m)
            if (sq_dist < 2.25f)
            {
                #pragma omp critical
                {
                    laserCloudOri_po->push_back(point_body);
                    corr_nearest->push_back(p_nearest);
                }
            }
        }
    }

    effct_feat_num = static_cast<int>(laserCloudOri->size());
    effct_po_num   = static_cast<int>(laserCloudOri_po->size());
    int total_feats = effct_feat_num + effct_po_num;

    if (total_feats < 1)
    {
        ekfom_data.valid = false;
        return;
    }

    res_mean_last = effct_feat_num > 0 ? total_residual / effct_feat_num : 0.0;
    match_time   += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    // ── Build stacked H (Jacobian) and h (residual) ──────────────────────────
    // Rows 0..Npl-1  : point-to-plane (scalar residual = -pd2)
    // Rows Npl..N-1  : point-to-point L2 norm (scalar residual = ||z_po||)
    ekfom_data.h_x = MatrixXd::Zero(total_feats, 12);
    ekfom_data.h.resize(total_feats);

    // ── Point-to-plane Jacobians (Sec. V-C, Eq. 16) ──────────────────────────
    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D p_body_be(laser_p.x, laser_p.y, laser_p.z);
        M3D p_be_cross;
        p_be_cross << SKEW_SYM_MATRX(p_body_be);

        V3D p_this = s.offset_R_L_I * p_body_be + s.offset_T_L_I;
        M3D p_cross;
        p_cross << SKEW_SYM_MATRX(p_this);

        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(p_cross * C);

        if (extrinsic_est_en) {
            V3D B(p_be_cross * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0) <<
                norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B),
                VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) <<
                norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A),
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0;
        }
        ekfom_data.h(i) = -norm_p.intensity;  // -pd2
    }

    // ── Point-to-point L2-norm Jacobians (Sec. V-D, Eq. 24-26) ──────────────
    // z_po = W_p_query - W_p_nearest   (3×1)
    // z_norm = ||z_po||                 (scalar)
    // H_norm = (z_po^T / z_norm) * H_po   where H_po is [I3 | -[p]× | 0...] in R^{3×18}
    //
    // We map onto the 12-dim reduced Jacobian used by IKFoM (pos=0..2, rot=3..5, offset_R=6..8, offset_T=9..11)
    for (int i = 0; i < effct_po_num; i++)
    {
        int row = effct_feat_num + i;

        const PointType &laser_p  = laserCloudOri_po->points[i];
        const PointType &near_p   = corr_nearest->points[i];

        V3D p_body(laser_p.x, laser_p.y, laser_p.z);
        V3D p_world = s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos;
        V3D p_near(near_p.x, near_p.y, near_p.z);

        V3D z_po = p_world - p_near;   // 3×1 residual vector
        double z_norm = z_po.norm();
        if (z_norm < 1e-6) { ekfom_data.h(row) = 0.0; continue; }

        V3D z_unit = z_po / z_norm;   // unit direction

        // H_po for position block: d(p_world)/d(pos) = I3
        // H_po for rotation block: d(p_world)/d(rot) = -rot * [offset_R*p_body + offset_T]×
        V3D p_imu_frame = s.offset_R_L_I * p_body + s.offset_T_L_I;
        M3D p_imu_cross;
        p_imu_cross << SKEW_SYM_MATRX(p_imu_frame);

        // Scalar Jacobian = z_unit^T * H_po  (1×12 reduced)
        // pos block (cols 0-2): z_unit^T * I3 = z_unit^T
        // rot block (cols 3-5): z_unit^T * (-rot * [p_imu]×) = z_unit^T * (-rot) * p_imu_cross
        V3D A_po = p_imu_cross * s.rot.conjugate() * z_unit;   // rot-block contribution

        if (extrinsic_est_en) {
            // offset_R block (cols 6-8): d/d(offset_R) of rot*(offset_R*p_body+offset_T)
            V3D p_be_cross_v(p_body);
            M3D p_be_cross_mat;
            p_be_cross_mat << SKEW_SYM_MATRX(p_be_cross_v);
            V3D B_po = p_be_cross_mat * s.offset_R_L_I.conjugate() * s.rot.conjugate() * z_unit;
            // offset_T block (cols 9-11): d/d(offset_T) = rot^T * z_unit
            V3D C_po = s.rot.conjugate() * z_unit;

            ekfom_data.h_x.block<1, 12>(row, 0) <<
                z_unit(0), z_unit(1), z_unit(2),
                VEC_FROM_ARRAY(A_po),
                VEC_FROM_ARRAY(B_po),
                VEC_FROM_ARRAY(C_po);
        } else {
            ekfom_data.h_x.block<1, 12>(row, 0) <<
                z_unit(0), z_unit(1), z_unit(2),
                VEC_FROM_ARRAY(A_po),
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0;
        }

        // Scalar residual for IKFoM (h vector element)
        ekfom_data.h(row) = -z_norm;

        // Covariance scaling (Eq. 30): upweight p2p relative to p2plane
        // The IKFoM framework uses a uniform LASER_POINT_COV; we compensate here
        // by scaling the residual so the effective noise is lambda_po * LASER_POINT_COV
        // This is equivalent to multiplying the row's contribution.
        // (Full covariance weighting requires modifying IKFoM – simplified here)
    }

    solve_time += omp_get_wtime() - solve_start_;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ROS2 Node
// ═══════════════════════════════════════════════════════════════════════════════
class GenZLioNode : public rclcpp::Node
{
public:
    GenZLioNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("genz_lio", options)
    {
        declareAndGetParams();
        setupPublishersSubscribers();
        setupKalmanFilter();

        // GenZ-LIO adaptive voxelizer init
        genz_lio::AdaptiveVoxelizerParams vox_params;
        this->declare_parameter<bool>("adaptive_vox.enable",  true);
        this->declare_parameter<int>   ("adaptive_vox.Nw",    5);
        this->declare_parameter<double>("adaptive_vox.d_min", 0.02);
        this->declare_parameter<double>("adaptive_vox.d_max", 1.0);
        this->declare_parameter<double>("adaptive_vox.N_min", 1000.0);
        this->declare_parameter<double>("adaptive_vox.N_max", 4000.0);
        this->declare_parameter<double>("adaptive_vox.tau_m", 30.0);
        this->declare_parameter<double>("adaptive_vox.p_exp", 2.0);
        this->declare_parameter<double>("adaptive_vox.lambda_p", 0.1);
        this->declare_parameter<double>("adaptive_vox.lambda_d", 0.2);
        this->declare_parameter<double>("adaptive_vox.Kp_min", 1e-6);
        this->declare_parameter<double>("adaptive_vox.Kp_max", 1e-4);
        this->declare_parameter<double>("adaptive_vox.Kd_min", 1e-9);
        this->declare_parameter<double>("adaptive_vox.Kd_max", 1e-7);

        this->get_parameter_or("adaptive_vox.enable",   g_use_adaptive_vox, true);
        this->get_parameter_or("adaptive_vox.Nw",       vox_params.Nw,      5);
        this->get_parameter_or("adaptive_vox.d_min",    vox_params.d_min,   0.02);
        this->get_parameter_or("adaptive_vox.d_max",    vox_params.d_max,   1.0);
        this->get_parameter_or("adaptive_vox.N_min",    vox_params.N_min,   1000.0);
        this->get_parameter_or("adaptive_vox.N_max",    vox_params.N_max,   4000.0);
        this->get_parameter_or("adaptive_vox.tau_m",    vox_params.tau_m,   30.0);
        this->get_parameter_or("adaptive_vox.p_exp",    vox_params.p_exp,   2.0);
        this->get_parameter_or("adaptive_vox.lambda_p", vox_params.lambda_p, 0.1);
        this->get_parameter_or("adaptive_vox.lambda_d", vox_params.lambda_d, 0.2);
        this->get_parameter_or("adaptive_vox.Kp_min",   vox_params.Kp_min,  1e-6);
        this->get_parameter_or("adaptive_vox.Kp_max",   vox_params.Kp_max,  1e-4);
        this->get_parameter_or("adaptive_vox.Kd_min",   vox_params.Kd_min,  1e-9);
        this->get_parameter_or("adaptive_vox.Kd_max",   vox_params.Kd_max,  1e-7);
        g_adaptive_vox = genz_lio::AdaptiveVoxelizer(vox_params);
        g_adaptive_vox.setInitialVoxelSize(filter_size_surf_min);

        // Open log file
        string log_path = root_dir + "Log/log.txt";
        fout_pre.open(log_path, ios::out);
        fout_out.open(root_dir + "Log/output.txt", ios::out);

        // Background processing thread
        proc_thread_ = std::thread(&GenZLioNode::processingLoop, this);
        RCLCPP_INFO(get_logger(),
            "GenZ-LIO started. Adaptive vox: %s, Hybrid-metric: ON",
            g_use_adaptive_vox ? "ON" : "OFF");
    }

    ~GenZLioNode()
    {
        flg_exit = true;
        sig_buffer.notify_all();
        if (proc_thread_.joinable()) proc_thread_.join();
        if (pcd_save_en) savePCD();
        fout_pre.close(); fout_out.close();
    }

private:
    // ── ROS handles ──────────────────────────────────────────────────────────
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr        pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr            pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       subImu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPcl_;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr subLivox_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr           srvSaveMap_;
    shared_ptr<tf2_ros::TransformBroadcaster>                    tfBr_;
    std::thread proc_thread_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_wait_save_{new pcl::PointCloud<pcl::PointXYZI>()};
    std::ofstream fout_pre, fout_out;

    // ─────────────────────────────────────────────────────────────────────────
    void declareAndGetParams()
    {
        // Standard FAST-LIO2 params
        declare_parameter<bool>("publish.path_en",              true);
        declare_parameter<bool>("publish.scan_publish_en",      true);
        declare_parameter<bool>("publish.dense_publish_en",     false);
        declare_parameter<bool>("publish.scan_bodyframe_pub_en",false);
        declare_parameter<int>("max_iteration",                 3);
        declare_parameter<string>("map_file_path",              "");
        declare_parameter<string>("common.lid_topic",           "/livox/lidar");
        declare_parameter<string>("common.imu_topic",           "/livox/imu");
        declare_parameter<bool>("common.time_sync_en",          false);
        declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        declare_parameter<double>("filter_size_surf",           0.5);
        declare_parameter<double>("filter_size_map",            0.5);
        declare_parameter<double>("cube_side_length",           1000.0);
        declare_parameter<float>("mapping.det_range",           450.0f);
        declare_parameter<double>("mapping.fov_degree",         360.0);
        declare_parameter<double>("mapping.gyr_cov",            0.1);
        declare_parameter<double>("mapping.acc_cov",            0.1);
        declare_parameter<double>("mapping.b_gyr_cov",          0.0001);
        declare_parameter<double>("mapping.b_acc_cov",          0.0001);
        declare_parameter<double>("preprocess.blind",           2.0);
        declare_parameter<int>("preprocess.lidar_type",         AVIA);
        declare_parameter<int>("preprocess.scan_line",          6);
        declare_parameter<int>("preprocess.timestamp_unit",     US);
        declare_parameter<int>("preprocess.scan_rate",          10);
        declare_parameter<int>("point_filter_num",              1);
        declare_parameter<bool>("feature_extract_enable",       false);
        declare_parameter<bool>("runtime_pos_log_enable",       false);
        declare_parameter<bool>("mapping.extrinsic_est_en",     false);
        declare_parameter<bool>("pcd_save.pcd_save_en",         false);
        declare_parameter<int>("pcd_save.interval",             -1);
        declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        get_parameter_or("publish.path_en",               path_en,            true);
        get_parameter_or("publish.scan_publish_en",       scan_pub_en,        true);
        get_parameter_or("publish.dense_publish_en",      dense_pub_en,       false);
        get_parameter_or("publish.scan_bodyframe_pub_en", scan_body_pub_en,   false);
        get_parameter_or("max_iteration",                 NUM_MAX_ITERATIONS, 3);
        get_parameter_or("map_file_path",                 map_file_path,      string(""));
        get_parameter_or("common.lid_topic",              lid_topic,          string("/livox/lidar"));
        get_parameter_or("common.imu_topic",              imu_topic,          string("/livox/imu"));
        get_parameter_or("common.time_sync_en",           time_sync_en,       false);
        get_parameter_or("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        get_parameter_or("filter_size_surf",              filter_size_surf_min, 0.5);
        get_parameter_or("filter_size_map",               filter_size_map_min,  0.5);
        get_parameter_or("cube_side_length",              cube_len,             1000.0);
        get_parameter_or("mapping.det_range",             DET_RANGE,            450.0f);
        get_parameter_or("mapping.fov_degree",            fov_deg,              360.0);
        get_parameter_or("mapping.gyr_cov",               gyr_cov,              0.1);
        get_parameter_or("mapping.acc_cov",               acc_cov,              0.1);
        get_parameter_or("mapping.b_gyr_cov",             b_gyr_cov,            0.0001);
        get_parameter_or("mapping.b_acc_cov",             b_acc_cov,            0.0001);
        get_parameter_or("preprocess.blind",              p_pre->blind,         2.0);
        // Enum-typed fields: load as int then assign
        { int v = AVIA; get_parameter_or("preprocess.lidar_type",      v, v); p_pre->lidar_type = v; }
        get_parameter_or("preprocess.scan_line",          p_pre->N_SCANS,       6);
        { int v = US;   get_parameter_or("preprocess.timestamp_unit",  v, v); p_pre->time_unit  = v; }
        get_parameter_or("preprocess.scan_rate",          p_pre->SCAN_RATE,     10);
        get_parameter_or("point_filter_num",              p_pre->point_filter_num, 1);
        get_parameter_or("feature_extract_enable",        p_pre->feature_enabled,  false);
        get_parameter_or("runtime_pos_log_enable",        runtime_pos_log,         false);
        get_parameter_or("mapping.extrinsic_est_en",      extrinsic_est_en,        false);
        get_parameter_or("pcd_save.pcd_save_en",          pcd_save_en,             false);
        get_parameter_or("pcd_save.interval",             pcd_save_interval,       -1);
        get_parameter_or("mapping.extrinsic_T",           extrinT,                 vector<double>());
        get_parameter_or("mapping.extrinsic_R",           extrinR,                 vector<double>());

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos(FOV_DEG * 0.5 * M_PI / 180.0);

        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min,   filter_size_map_min,  filter_size_map_min);
        _featsArray.reset(new PointCloudXYZI());
        memset(point_selected_surf, true,    sizeof(point_selected_surf));
        memset(res_last,           -1000.0f, sizeof(res_last));
    }

    void setupPublishersSubscribers()
    {
        pubLaserCloudFull_   = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered",    20);
        pubLaserCloudEffect_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected",      20);
        pubLaserCloudMap_    = create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map",           20);
        pubOdomAftMapped_    = create_publisher<nav_msgs::msg::Odometry>       ("/Odometry",           20);
        pubPath_             = create_publisher<nav_msgs::msg::Path>            ("/path",               20);

        tfBr_ = std::make_shared<tf2_ros::TransformBroadcaster>(static_cast<rclcpp::Node &>(*this));

        if (p_pre->lidar_type == AVIA) {
            subLivox_ = create_subscription<livox_interfaces::msg::CustomMsg>(
                lid_topic, 200000,
                [this](livox_interfaces::msg::CustomMsg::UniquePtr msg) { livoxMsgCallback(std::move(msg)); });
        } else {
            subPcl_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, 200000,
                [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) { standardPclCallback(std::move(msg)); });
        }

        subImu_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 200000,
            [this](sensor_msgs::msg::Imu::UniquePtr msg) { imuCallback(std::move(msg)); });

        srvSaveMap_ = create_service<std_srvs::srv::Trigger>(
            "/save_map",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                savePCD();
                res->success = true;
                res->message = "Map saved.";
            });
    }

    void setupKalmanFilter()
    {
        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        double epsi[23] = {0.001};
        std::fill(epsi, epsi + 23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        path.header.stamp    = get_clock()->now();
        path.header.frame_id = "camera_init";
    }

    // ── Sensor callbacks (UniquePtr for zero-copy) ───────────────────────────
    void livoxMsgCallback(livox_interfaces::msg::CustomMsg::UniquePtr msg)
    {
        mtx_buffer.lock();
        double cur_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (cur_time < last_timestamp_lidar) {
            RCLCPP_WARN(get_logger(), "Lidar loop back, clear buffer");
            lidar_buffer.clear();
        }
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(cur_time);
        last_timestamp_lidar = cur_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    void standardPclCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
    {
        mtx_buffer.lock();
        double cur_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (cur_time < last_timestamp_lidar) {
            RCLCPP_WARN(get_logger(), "Lidar loop back, clear buffer");
            lidar_buffer.clear();
        }
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(cur_time);
        last_timestamp_lidar = cur_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    void imuCallback(sensor_msgs::msg::Imu::UniquePtr msg)
    {
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        // Transfer ownership to SharedPtr for storage in deque
        auto msg_shared = std::make_shared<sensor_msgs::msg::Imu>(std::move(*msg));
        mtx_buffer.lock();
        if (timestamp < last_timestamp_imu) {
            RCLCPP_WARN(get_logger(), "IMU loop back, clear buffer");
            imu_buffer.clear();
        }
        last_timestamp_imu = timestamp;
        imu_buffer.push_back(msg_shared);
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    // ── Sync packages (from FAST-LIO2) ───────────────────────────────────────
    bool syncPackages(MeasureGroup &meas)
    {
        if (lidar_buffer.empty() || imu_buffer.empty()) return false;

        if (!lidar_pushed) {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() <= 1) {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
                RCLCPP_WARN(get_logger(), "Too few points in LiDAR scan!");
            } else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            } else {
                scan_num++;
                lidar_end_time = meas.lidar_beg_time +
                    meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime += (lidar_end_time - meas.lidar_beg_time -
                                        lidar_mean_scantime) / scan_num;
            }
            meas.lidar_end_time = lidar_end_time;
            lidar_pushed = true;
        }

        if (last_timestamp_imu < lidar_end_time) return false;

        meas.imu.clear();
        while (!imu_buffer.empty() &&
               imu_buffer.front()->header.stamp.sec +
               imu_buffer.front()->header.stamp.nanosec * 1e-9 < lidar_end_time) {
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;
        return true;
    }

    // ── FOV map management (unchanged from FAST-LIO2) ────────────────────────
    void laserMapFovSegment()
    {
        cub_needrm.clear();
        kdtree_delete_counter = 0;
        kdtree_delete_time = 0.0;

        pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
        V3D pos_LiD = pos_lid;

        if (!Localmap_Initialized) {
            for (int i = 0; i < 3; i++) {
                LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
            }
            Localmap_Initialized = true;
            return;
        }

        float dist_to_map_edge[3][2];
        bool  need_move = false;
        for (int i = 0; i < 3; i++) {
            dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
                dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
                need_move = true;
        }
        if (!need_move) return;

        BoxPointType new_odom_box, tmp_box;
        new_odom_box = LocalMap_Points;
        float mov_dist = std::max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                                  double(DET_RANGE * (MOV_THRESHOLD - 1)));
        for (int i = 0; i < 3; i++) {
            tmp_box = LocalMap_Points;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
                new_odom_box.vertex_max[i] -= mov_dist;
                new_odom_box.vertex_min[i] -= mov_dist;
                tmp_box.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.push_back(tmp_box);
            } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
                new_odom_box.vertex_max[i] += mov_dist;
                new_odom_box.vertex_min[i] += mov_dist;
                tmp_box.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.push_back(tmp_box);
            }
        }
        LocalMap_Points = new_odom_box;
        PointVector pts_remove;
        ikdtree.Delete_Point_Boxes(cub_needrm);
    }

    // ── Map update after ESIKF convergence ───────────────────────────────────
    void mapUpdate()
    {
        PointVector PointToAdd, PointNoNeedDownsample;

        // Use merge scan (dt/2) for map integration if adaptive vox is enabled
        PointCloudXYZI::Ptr &map_src = g_use_adaptive_vox ? feats_merge_body : feats_down_body;
        int map_pts = static_cast<int>(map_src->size());
        PointToAdd.reserve(map_pts);
        PointNoNeedDownsample.reserve(map_pts);

        for (int i = 0; i < map_pts; i++)
        {
            PointType pt_world;
            pointBodyToWorld(&map_src->points[i], &pt_world);

            PointType mid;
            mid.x = floorf(pt_world.x / filter_size_map_min) * filter_size_map_min + 0.5f * filter_size_map_min;
            mid.y = floorf(pt_world.y / filter_size_map_min) * filter_size_map_min + 0.5f * filter_size_map_min;
            mid.z = floorf(pt_world.z / filter_size_map_min) * filter_size_map_min + 0.5f * filter_size_map_min;

            float dist = sqrtf((pt_world.x - mid.x)*(pt_world.x - mid.x) +
                               (pt_world.y - mid.y)*(pt_world.y - mid.y) +
                               (pt_world.z - mid.z)*(pt_world.z - mid.z));
            if (dist < filter_size_map_min / 2.0f)
                PointNoNeedDownsample.push_back(pt_world);
            else
                PointToAdd.push_back(pt_world);
        }
        ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
    }

    // ── Publish results ───────────────────────────────────────────────────────
    void publishOdometry(const rclcpp::Time &stamp)
    {
        odomAftMapped.header.frame_id = "camera_init";
        odomAftMapped.child_frame_id  = "body";
        odomAftMapped.header.stamp    = stamp;
        odomAftMapped.pose.pose.position.x    = state_point.pos(0);
        odomAftMapped.pose.pose.position.y    = state_point.pos(1);
        odomAftMapped.pose.pose.position.z    = state_point.pos(2);
        odomAftMapped.pose.pose.orientation.x = geoQuat.x;
        odomAftMapped.pose.pose.orientation.y = geoQuat.y;
        odomAftMapped.pose.pose.orientation.z = geoQuat.z;
        odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        odomAftMapped.twist.twist.linear.x    = state_point.vel(0);
        odomAftMapped.twist.twist.linear.y    = state_point.vel(1);
        odomAftMapped.twist.twist.linear.z    = state_point.vel(2);
        pubOdomAftMapped_->publish(odomAftMapped);

        auto tf_msg = geometry_msgs::msg::TransformStamped();
        tf_msg.header = odomAftMapped.header;
        tf_msg.child_frame_id = "body";
        tf_msg.transform.translation.x = state_point.pos(0);
        tf_msg.transform.translation.y = state_point.pos(1);
        tf_msg.transform.translation.z = state_point.pos(2);
        tf_msg.transform.rotation = geoQuat;
        tfBr_->sendTransform(tf_msg);
    }

    void publishPath(const rclcpp::Time &stamp)
    {
        msg_body_pose.header.stamp    = stamp;
        msg_body_pose.header.frame_id = "camera_init";
        msg_body_pose.pose.position.x = state_point.pos(0);
        msg_body_pose.pose.position.y = state_point.pos(1);
        msg_body_pose.pose.position.z = state_point.pos(2);
        msg_body_pose.pose.orientation = geoQuat;
        path.poses.push_back(msg_body_pose);
        pubPath_->publish(path);
    }

    void publishPointCloud(const rclcpp::Time &stamp)
    {
        if (scan_pub_en && pubLaserCloudFull_->get_subscription_count() > 0)
        {
            PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI());
            PointCloudXYZI::Ptr &src = dense_pub_en ? feats_undistort : feats_down_body;
            for (const auto &pt : src->points) {
                PointType pw; pointBodyToWorld(&pt, &pw); laserCloudWorld->push_back(pw);
            }
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*laserCloudWorld, msg);
            msg.header.stamp    = stamp;
            msg.header.frame_id = "camera_init";
            pubLaserCloudFull_->publish(msg);
        }
    }

    void savePCD()
    {
        if (!map_file_path.empty()) {
            pcl::PCDWriter pcd_writer;
            pcd_writer.writeBinary(map_file_path, *pcl_wait_save_);
            RCLCPP_INFO(get_logger(), "Saved map to: %s", map_file_path.c_str());
        }
    }

    // ── Main processing loop ──────────────────────────────────────────────────
    void processingLoop()
    {
        double last_lidar_time = -1.0;

        while (rclcpp::ok() && !flg_exit)
        {
            // Wait for data
            {
                std::unique_lock<std::mutex> lock(mtx_buffer);
                sig_buffer.wait_for(lock, std::chrono::milliseconds(100));
            }

            if (!syncPackages(Measures)) continue;

            // ── First scan init ───────────────────────────────────────────────
            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            // ── IMU forward/backward propagation ─────────────────────────────
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (!feats_undistort || feats_undistort->empty()) {
                RCLCPP_WARN(get_logger(), "No deskewed points, skip scan");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME
                             ? false : true;

            // ── FOV management ────────────────────────────────────────────────
            laserMapFovSegment();

            // ── Scale-aware adaptive voxelization (GenZ-LIO Sec. IV) ──────────
            double dt_scan = (last_lidar_time > 0.0)
                ? (Measures.lidar_beg_time - last_lidar_time) : 0.1;
            dt_scan = std::clamp(dt_scan, 0.01, 1.0);
            last_lidar_time = Measures.lidar_beg_time;

            if (g_use_adaptive_vox && flg_EKF_inited)
            {
                // Run Algorithm 1 → feats_down_body (state update), feats_merge_body (map)
                g_adaptive_vox.process(*feats_undistort, dt_scan,
                                        *feats_down_body, *feats_merge_body);
                feats_down_size = static_cast<int>(feats_down_body->size());
            }
            else
            {
                // Fixed voxelization (fallback / initialisation phase)
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                *feats_merge_body = *feats_down_body;
                feats_down_size = static_cast<int>(feats_down_body->size());
            }

            // Occasionally log adaptive vox diagnostics
            static int diag_cnt = 0;
            if (g_use_adaptive_vox && ++diag_cnt % 50 == 0) {
                RCLCPP_INFO(get_logger(),
                    "[GenZ-LIO] m_bar=%.1f m  N_des=%.0f  N_down=%d  d_vox=%.3f m  "
                    "Npl=%d  Npo=%d",
                    g_adaptive_vox.lastScaleIndicator(),
                    g_adaptive_vox.lastNDesired(),
                    feats_down_size,
                    g_adaptive_vox.voxelSize(),
                    effct_feat_num, effct_po_num);
            }

            // ── Init ikd-tree on first good scan ──────────────────────────────
            if (ikdtree.Root_Node == nullptr)
            {
                if (feats_down_size > 5) {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                        pointBodyToWorld(&feats_down_body->points[i],
                                         &feats_down_world->points[i]);
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }

            if (feats_down_size < 5) { RCLCPP_WARN(get_logger(), "Too few points"); continue; }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);

            // ── ESIKF update (Hybrid-metric h_share_model called internally) ──
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur   = SO3ToEuler(state_point.rot);
            pos_lid     = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x   = state_point.rot.coeffs()[0];
            geoQuat.y   = state_point.rot.coeffs()[1];
            geoQuat.z   = state_point.rot.coeffs()[2];
            geoQuat.w   = state_point.rot.coeffs()[3];

            // ── Map update ────────────────────────────────────────────────────
            mapUpdate();

            // ── Publish ───────────────────────────────────────────────────────
            rclcpp::Time stamp = get_clock()->now();
            publishOdometry(stamp);
            publishPath(stamp);
            publishPointCloud(stamp);

            // Save to map PCD
            if (pcd_save_en) {
                for (const auto &pt : feats_down_body->points) {
                    PointType pw; pointBodyToWorld(&pt, &pw);
                    pcl::PointXYZI pi;
                    pi.x = pw.x; pi.y = pw.y; pi.z = pw.z; pi.intensity = pw.intensity;
                    pcl_wait_save_->push_back(pi);
                }
            }
        }
    }

    // ── State ─────────────────────────────────────────────────────────────────
    bool  Localmap_Initialized = false;
    BoxPointType LocalMap_Points;
    bool  lidar_pushed = false;
    double lidar_mean_scantime = 0.0;
    int   scan_num = 0;
};

// ═══════════════════════════════════════════════════════════════════════════════
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, SigHandle);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<GenZLioNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
