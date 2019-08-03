// Skeletonization.h

#pragma once

#ifndef SKELETONIZATION_H
#define SKELETONIZATION_H

#include "Global.h"
#include <vector>

namespace ManSeg {

	// -------------------- Constant --------------------

	static const int DIVIDE_LEVEL = 5;

	// -------------------- Class Skeleton Generator --------------------

	class SkeletonGenerator {
	public:
		SkeletonGenerator(Mesh* in_mesh, VoxelHandler* vxh);
		SkeletonGenerator(Mesh* in_mesh, VoxelHandler* vxh, bool cut);
	private:
		Mesh* mesh;
		VoxelHandler* voxelh;
	private:
		typedef std::vector<int> IntVec;
		enum DirectionType { DIR_U, DIR_D, DIR_W, DIR_N, DIR_E, DIR_S };
		// thining
		void thining();
		void construct_neighbor_array(const Voxel& vx, bool ns[][3][3]);
		void transform_neighbor_array(bool ns[][3][3], DirectionType dir);
		bool match_mask(const bool ns[][3][3]);
		// construct skeleton
		void search_and_label_skeleton_points(IntVec& ske, IntVec& end);
		void connect_skeleton();
		void connect_skeleton_point_greedy(int start, int end, IntVec& vec);
		void cut_skeleton();
		void smooth_skeleton();
		// force field
		void generate_force_field();
		void generate_skeleton_from_force_field();
		bool intersect_triangle(const Vertex& orig, const Vertex& dir, const Vertex& v0, const Vertex& v1,
			const Vertex& v2, float& t, float& u, float& v, Vertex& intersect);
		bool check_boundary(const Voxel& vx, const Vertex& center, const Vertex& ray, Vertex& ret);
		Vertex compute_force_slow(const Vertex& center, const Vertex& ray);
		Vertex compute_force_fast(const Vertex& center, const Vertex& ray);
	};
}

#endif