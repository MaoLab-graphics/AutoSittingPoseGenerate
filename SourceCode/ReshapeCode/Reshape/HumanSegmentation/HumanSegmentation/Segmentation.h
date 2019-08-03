// Segmentation

#pragma once

#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "Mesh.h"
#include <vector>
#include <map>

namespace ManSeg {

	// -------------------- Constants --------------------

	// Landmark setting
	static const string LANDMARK_GROUND_TRUTH_PATH = "ground_truth/";
	static const string LANDMARK_ERROR_PATH = "landmark_error/";
	static const double INF_ERR = -std::numeric_limits<double>::max();
	static const bool COMPUTE_ERROR = false;
	static const bool FIND_FEATURE_POINTS = false;
	static const bool FIND_LANDMARKS = false;

	// Paint Setting, Color : 0-head, 1-arm, 2-leg, 3-waist, 4-chest, 5-hip, 6-elbow, 7-wrist, 8-knee, 9-ankle
	static const bool PAINT_SEG_ON_MESH = false;
	static const bool PAINT_SEG_OVER_MESH = false;
	static const bool PAINT_MESH_USING_LABEL = true;
	static const bool PAINT_BALL = false;
	static const float BALL_RADIUS = 20;
	static const int BALL_CUTS = 4;

	// Segmentation setting
	static const bool REFINE_SEG = true;
	static const bool SMOOTH_SEG_BY_CUT_TRIANGLES = false;
	static const bool EXTEND_BODY_SKE = true;
	static const bool REFINE_SHOULDER = true;
	static const bool PRINT_TO_FILE = true;

	// Segmentation adjust setting
	static const bool MANUAL_BODY_SEG = true;
	static const bool STRAIGHT_BODY_CUT = false;
	static const int MANUAL_BODY_CUT_UP = 8;
	static const int MANUAL_BODY_CUT_MID = 23;
	static const int MANUAL_BODY_CUT_DOWN = 32;
	static const int HEAD_ADJUST = -2;
	static const int NECK_ADJUST = 0;
	static const int L_KNEE_ADJUST = -2;
	static const int R_KNEE_ADJUST = -2;
	static const int L_ANKLE_ADJUST = -3;
	static const int R_ANKLE_ADJUST = 0;
	static const int L_ELBOW_ADJUST = -2;
	static const int R_ELBOW_ADJUST = -1;
	static const int L_WRIST_ADJUST = 0;
	static const int R_WRIST_ADJUST = 0;

	// Human body slicing setting
	static const int SLICE_INTERVAL = 2;
	static const int COS_INTERVAL = 1;
	static const int COS_INTERVAL_SP = 8;

	// Skeleton smooth setting
	static const int SMOOTH_SKELETON_TIMES = 30;
	static const int SMOOTH_SKELETON_INTERVAL = 1;
	
	// -------------------- Class Segmentation --------------------

	class Segmentation {
	public:
		Segmentation() {}
		Segmentation(Mesh* in_mesh);
		Segmentation(Mesh* in_mesh, VoxelHandler* vxh);
	private:
		struct PlyPart {
			float x; float y; float z; float nx; float ny; float nz;
			unsigned char r; unsigned char g; unsigned char b; unsigned char a;
		};
	protected:
		typedef std::vector<bool> BoolVec;
		typedef std::vector<char> CharVec;
		typedef std::vector<int> IntVec;
		typedef std::vector<IntVec> Int2DVec;
		typedef std::vector<Int2DVec> Int3DVec;
		typedef std::vector<float> FltVec;
		typedef std::vector<Vertex> VtxVec;
		typedef std::vector<std::vector<Vertex>> Vtx2DVec;
	protected:
		enum PartType { p_head = 0, p_arm0 = 1, p_arm1 = 2, p_leg0 = 3, p_leg1 = 4, p_body = 5, p_part_amount };
	protected:
		Mesh* _human;
		VoxelHandler* _voxelh;
		Color _preset_color[13];
	public:
		void write_ply(string ply_path);
		void write_ply(VtxVec& points, string ply_path);
		void write_off(string off_path);
	protected:
		void search_and_label_skeleton_points(IntVec& ske, IntVec& end);
		void find_skeleton_leaves(Int2DVec& leaves);
	};

	// -------------------- Class SkeletalSegmentation --------------------

	class SkeletalSegmentation : public Segmentation {
	public:
		SkeletalSegmentation() {}
		SkeletalSegmentation(Mesh* in_mesh) : Segmentation(in_mesh) {}
		SkeletalSegmentation(Mesh* in_mesh, VoxelHandler* vxh) : Segmentation(in_mesh, vxh) {}
	public:
		void do_human_segmentation();
		void do_human_segmentation(const string& data_path);
		void read_HKS_and_curvature(const string& path);
		void write_smooth_skeleton();
		void search_loop(const Plane& plane, Vertex currske, IntVec& ret);
		void compute_total_landmark_error();
	protected:
		enum DetailPartType {
			head = 0, neck = 1, upbody = 50, midbody = 51, downbody = 52,
			arm0 = 10, forearm0 = 11, hand0 = 12, arm1 = 20, forearm1 = 21, hand1 = 22,
			thigh0 = 30, shank0 = 31, foot0 = 32, thigh1 = 40, shank1 = 41, foot1 = 42,
		};
	protected:
		// head:0-head,1-neck; arm:0-arm,1-elbow,2-wrist; leg:0-leg,1-knee,2-ankle
		enum HeadCut { c_head = 0, c_neck = 1, c_head_cut_amount };
		enum ArmCut { c_arm = 0, c_elbow = 1, c_wrist = 2, c_arm_cut_amount };
		enum LegCut { c_leg = 0, c_knee = 1, c_ankle = 2, c_leg_cut_amount };
		enum BodyCut { c_nip = 0, c_up = 1, c_waist = 2, c_down = 3, c_hip = 4, c_body_cut_amount };
	protected:
		// finding landmarks using curvatures and HKS
		enum CurvatureType {
			c_min, c_max, c_mean, c_gauss, c_mean_gauss, c_cvt_type_amount
		};
		struct LandMarks {
			int shoulder[2], armpit[2], waist[4], hip[4], nipple[2], chest[4], crotch, neck[4], head[4], navel[2], waist_low_back[4], waist_high_back[4],
				leg[2], knee0[4], knee1[4], ankle0[4], ankle1[4], heel[2], toe[2], gluteal[2],
				elbow0[4], elbow1[4], wrist0[4], wrist1[4], knee_max[2], knee_min[2];
		};
		enum LandMarksID {
			lm_neck_front, lm_neck_back, lm_chest_front, lm_nipple0, lm_nipple1, lm_gluteal0, lm_gluteal1,
			lm_shoulder0, lm_shoulder1, lm_toe0, lm_toe1, lm_heel0, lm_heel1,
			lm_elbow0_min, lm_elbow0_max, lm_elbow1_min, lm_elbow1_max,
			lm_knee0_min, lm_knee0_max, lm_knee1_min, lm_knee1_max, lm_landmark_amount
		};
	private:
		// parameters for slices
		struct Parms {
			float avg_sdf, min_sdf, max_sdf;
			float avg_r, min_r, max_r, avg_r_ske, min_r_ske, max_r_ske;
			float ecc, girth, angle, ecc_ske, df_ske;
			double cvt_min[CurvatureType::c_cvt_type_amount], cvt_min_avg[CurvatureType::c_cvt_type_amount];
			Vertex center;
			Parms() {
				avg_sdf = min_sdf = max_sdf = avg_r = min_r = max_r = ecc = -1; center = Vertex(0, 0, 0);
				for (int i = 0; i < CurvatureType::c_cvt_type_amount; ++i) {
					cvt_min_avg[i] = 0, cvt_min[i] = -std::numeric_limits<double>::max();
				}
			}
		};
		typedef std::vector<Parms> ParmVec;
	protected:
		// 6 skeletal curves, head, L arm, R arm, L leg, R leg, body; each include many skeleton points
		VtxVec _skeleton[PartType::p_part_amount];
		// each skeleton point generates a cutting plane to cut the mesh into slices along skeletal curve
		std::vector<Plane> _planes[PartType::p_part_amount];
		Plane _shoulder_cutter[2];
		// body range (store in index of body skeletal curve)
		int _body_border[2];
		// each skeleton point generates one slice, this stores the intersecting faces with the cutting plane
		Int2DVec _slices_face[PartType::p_part_amount];
		IntVec _shoulder_face[2];
		// stores the intersecting vertices with the cutting plane (accurate)
		Vtx2DVec _slices_vtx[PartType::p_part_amount];
		// stores the intersecting vertices with the cutting plane (not accurate, on mesh)
		Int2DVec _slices_vidx[PartType::p_part_amount];
		// use through HeadCut, ArmCut, LegCut, BodyCut; stores the slice index of joints
		IntVec _cuts[PartType::p_part_amount];
		// stores the parameters of each slice
		ParmVec _parms[PartType::p_part_amount];
		// each face on mesh has an label indicating which part it belongs to, see DetailPartType
		CharVec _part_labels;
		// HKS
		std::vector<double> _hks;
		IntVec _hks_points;
		// curvatures
		std::vector<double> _curvature[CurvatureType::c_cvt_type_amount];
		// landmarks
		LandMarks _landmarks;
		int _ground_truths[LandMarksID::lm_landmark_amount];
		double _landmark_errors[LandMarksID::lm_landmark_amount][CurvatureType::c_cvt_type_amount + 1];
	private:
		// refine segmentation
		void cut_ring_by_plane(const IntVec& ring, const Plane& plane, Color upcolor, Color dncolor);
		void color_faces_BFS(int seed, const IntVec& border0, const IntVec& border1, const Color& color);
		void refine_split_by_slice();
		void label_faces_BFS(const IntVec& border, const Plane& plane, char label, CharVec& labels, char flag);
		void refine_arm_split_by_slice(const Int2DVec& border, Plane plane[], Int3DVec skerings);
		void split_body_by_contour();
		void refine_cutting(CharVec& labels);
	private:
		// landmark
		void find_landmarks();
		void HKS_paint_features();
		void find_crotch();
		void find_armpit();
		void find_shoulder();
		void find_chest();
		void find_navel();
		void find_foot();
		void find_hip();
		void find_neck();
		void find_limbs_landmarks();
		void get_candidates_geodesic(int vtx_seed, float max_dist, IntVec& out_candidates);
		void extract_landmark_from_candidates(const IntVec& candidates, int landmark_id, int& out_idx);
		void refine_split_by_feature_point();
		void read_landmark_ground_truth();
		void write_landmark_error();
		int intersect_triangle(const Vertex& center, const Vertex& ray);
		bool intersect_triangle(const Vertex& orig, const Vertex& dir, const Vertex& v0,
			const Vertex& v1, const Vertex& v2, float& t, float& u, float& v, Vertex& intersect);
	private:
		// split mesh by slice
		void split_mesh_by_slice();
		void compute_parameter_by_slice(int pi, ParmVec& parms);
		void smooth_skeleton_points(int start, int end, int loop_times);
		void extend_body_skeleton(Int2DVec& leaves, const IntVec border[]);
		void split_head_by_slice();
		void split_arm_by_slice(int pi);
		void split_leg_by_slice(int pi);
		void split_body_by_slice();
	private:
		// slicing
		bool trace_is_end(int current, int target, int v0, int v1);
		void search_loop(const Plane& plane, int start, IntVec& ret);
		void search_loop(const Plane& plane, Vertex currske, IntVec& ret, VtxVec& retv);
	private:
		// geodesic
		void geodesic_dist_from_single_vertex(int seed, float range);
		float geodesic_dist(int seed, int dest);
		float geodesic_dist_slow(int seed, int dest);
		void geodesic_dist();
	protected:
		// geometric
		float point_distance(int p0, int p1);
		float point_plane_distance(const Plane& plane, const Vertex& p);
		int face_cross_plane(int face, const Plane& plane);
		Vertex plane_edge_intersection(const Plane& plane, int v0, int v1);
		Plane create_plane(const Vertex& center, const Vertex& ph, const Vertex& pt);
		Plane create_plane_cross(const Vertex& p, const Vertex& na, const Vertex& nb);
		Vertex triangle_center(int face);
		float triangle_distance_s(int f0, int f1);
		Vertex project_to_plane(const Vertex& v, const Plane& plane);
		void find_connected_vertices(int f0, int f1, int& v0, int& v1);
		void connect_two_vertex(Plane& plane, int start, int end);
		// paint
		void paint_ring_vertices(Vector n, const VtxVec& vs, const Color& color);
		void paint_ball(const Vertex& center, float radius, int cuts, const Color& color);
		void paint_ball(const int& index, const VtxVec& ring, float radius, int cuts, const Color& color);
	};
}

#endif