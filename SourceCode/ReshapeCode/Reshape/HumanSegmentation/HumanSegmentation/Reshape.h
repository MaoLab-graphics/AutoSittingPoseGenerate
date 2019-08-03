// Reshape

#pragma once

#ifndef RESHAPE_H
#define RESHAPE_H

#include "Segmentation.h"

namespace ManSeg {

	// -------------------- Constants --------------------

	static const bool CHAIR_SEAT_DEFORM = true;
	static const bool CHAIR_BACK_DEFORM = true;

	// -------------------- Class Reshape --------------------

	class Reshape : public SkeletalSegmentation {
	public:
		typedef std::vector<std::string> StrVec;
		Reshape(Mesh* in_mesh, VoxelHandler* vxh) : SkeletalSegmentation(in_mesh, vxh) {}
	public:
		// main
		void deform_to_seat(int humanNum,int chairNum);
	public:
		// main
		void deform_to_sitting_pose();
		void init_chair(const string& chair_path, StrVec seat_name, StrVec back_name,
			StrVec arm_names, StrVec leg_names, StrVec foot_names, Vector scale_factor,int flag);
		// skinning
		void subdivide_mesh(float epsilon, VtxVec& vertices_out, std::vector<Triangle>& triangles_out,
			IntVec& triangle_neighbors_out, std::vector<double>& bone_weights_out);
		std::map<int, Vertex> compute_optimized_center_of_rotation(float epsilon = 0.1f, float sigma = 0.1f);
		// functions
		Vertex find_centroid(const VtxVec& vs);
		Vertex find_closest(const VtxVec& vs, Vertex target);
		IntVec find_plane_intersecting_faces(const Mesh& m, const Plane& plane);
		Vertex circle_line_intersect(Vertex cen, float r, Vertex v0, Vertex v1);
	private:
		struct Chair {
			bool has_seat, has_back, is_wheel_chair, is_lounger;
			int arm_num, leg_num, foot_num;
			std::vector<Mesh> arms;
			std::vector<Mesh> legs;
			std::vector<Mesh> feet;
			Mesh seat, back;
			std::vector<Mesh*> p;
			void write_to_ply(const string& path);
		};
		Chair _chair;
	};
}

#endif