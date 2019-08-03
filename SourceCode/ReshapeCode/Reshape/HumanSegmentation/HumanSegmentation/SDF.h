// SDF

#pragma once

#ifndef SDF_H
#define SDF_H

#include "Global.h"
#include <vector>

namespace ManSeg {

	// -------------------- Types --------------------

	typedef std::vector<Vertex> VtxVec;
	typedef std::vector<VtxVec> Vtx2DVec;
	struct WeightedSample { float w, x, y; };
	typedef std::vector<WeightedSample> WspVec;

	// -------------------- Constants --------------------

	static const float ANGLE_ST_DEV_DIVIDER = (float)3.0;
	static const float GOLDEN_RATIO = (float)3.0 - std::sqrt((float)5.0);
	static const float OUTLIER_RATIO = (float)1.5;
	static const float NORMALIZATION_ALPHA = (float)5.0;
	
	// -------------------- Global Function --------------------

	void calculate_sdf_for_mesh(float angle, string in_file, string out_file);

	// -------------------- Cone Ray --------------------

	class Cone_Ray {
	private:
		typedef std::vector<float> FltVec;
	public:
		Cone_Ray(bool uniform, int size) { sampling(uniform, size); }
		Vector gen_ray(const Vector& origin, Vector base[2], int index) const;
		float compute_sdf_value_from_samples(FltVec& ray_lens) const;
	private:
		void sampling(bool uniform, int size);
	private:
		WspVec weighted_samples;
	};

	// -------------------- Shape Diameter Function --------------------

	class Shape_Diameter_Function {
	public:
		Shape_Diameter_Function(const Mesh* in_mesh, VoxelHandler* vxh);
		void compute_sdf(int ray_num, bool uniform, float angle, std::string out_file);
		void log_normalize_sdf_values();
	private:
		typedef std::vector<float> FltVec;
	private:
		void make_plane_base(int face, Vector output[2]) const;
		void make_normal(int face, Vector& normal) const;
		bool compute_ray_length(int face, const Vector& ray, float& ret) const;
		bool intersect_triangle(const Vertex& orig, const Vector& dir, int face, float& t) const;
		Vertex triangle_center(int face) const;
		bool in_same_direction(int face1, int face2) const;
	private:
		const Mesh* mesh;
		VoxelHandler* voxelh;
		VtxVec face_centers;
		VtxVec face_normals;
		FltVec face_normal_lens;
		FltVec sdf_values;
		bool check_boundary(const Voxel& vx, int face, const Vertex& ray, float& ret) const;
	};
}

#endif