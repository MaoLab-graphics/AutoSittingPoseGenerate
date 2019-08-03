// Voxel.h

#pragma once

#ifndef VOXEL_H
#define VOXEL_H

#include "Global.h"
#include <vector>

namespace ManSeg {

	// -------------------- Struct Voxel --------------------

	struct Voxel {
		enum VoxelType {
			undefined, inside, boundary, outside, skeleton,
			deleted, maximum, BS_maximum, split
		};
		float force_mag;
		int x, y, z;
		int BS_code, SS_code, cluster_num;
		int label; // part ID / (for MassSpringSystem) point index in MassSpringSystem._centers
		int face_start, face_num;
		VoxelType type;
		bool visited;
		Voxel(int px = -1, int py = -1, int pz = -1, VoxelType pt = undefined)
			: x(px), y(py), z(pz), type(pt) {
			face_start = face_num = 0;
		}
	};

	// -------------------- Class Voxel Handler --------------------

	class VoxelHandler {
	private:
		typedef std::vector<int> IntVec;
	public:
		// (right hand coordinate) neighbor number: up=16 down=10 front(z-)=4 back(z+)=22 left(x+)=14 right(x-)=12
		std::vector<Voxel> voxels;  // index = z*len_y*len_x + y*len_x + x;
		int vxlen[3];  // how many voxels each dimension
		float vxorg[3];  // origin of voxels
		float vxsize;  // leaf size of voxel
		IntVec boundary_faces;
	protected:
		Mesh* mesh;
	public:
		VoxelHandler(Mesh* in_mesh, float voxel_size);
		VoxelHandler(Mesh* in_mesh, float voxel_size, float multiplier);
		void write_ply(string ply_path);
	public:
		int voxel_index(int x, int y, int z);
		int vertex_in_which_voxel(const Vertex& vertex);
		float voxel_distance_s(int index0, int index1);
		Vertex voxel_center(int index);
		bool voxel_neighbor(int index, int num, int& neighbor);
		bool voxel_six_neighbor(int index, int num, int& neighbor);
	public:
		bool is_object(int index);
		bool is_inside(int index);
		bool is_boundary(int index);
		bool is_skeleton(int index);
		bool is_object_border(int index);
	public:
		void voxels_reset_label();
		void voxels_reset_SS_code();
		void voxels_set_unvisited();
	private:
		void voxels_init(float voxel_size, float multiplier = -1);
		void find_boundary_voxel();
		void update_boundary(int mini[], int maxi[], int face);
		void update_boundary_face(int mini[], int maxi[], int face);
		void find_outside_voxel();
		void find_inside_voxel();
	};
}

#endif