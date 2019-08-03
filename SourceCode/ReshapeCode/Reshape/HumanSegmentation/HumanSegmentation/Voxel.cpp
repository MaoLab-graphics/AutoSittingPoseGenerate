// Voxel

#include "Voxel.h"
#include "Mesh.h"

namespace ManSeg {
	// constructor
	VoxelHandler::VoxelHandler(Mesh* in_mesh, float voxel_size) : mesh(in_mesh) {
		voxels_init(voxel_size);
		find_boundary_voxel();
		find_outside_voxel();
		find_inside_voxel();
	}
	VoxelHandler::VoxelHandler(Mesh* in_mesh, float voxel_size, float multiplier) : mesh(in_mesh) {
		voxels_init(voxel_size, multiplier);
		find_boundary_voxel();
		find_outside_voxel();
		find_inside_voxel();
	}
	void VoxelHandler::write_ply(string ply_path) {
		struct PlyPart {
			float x; float y; float z; float nx; float ny; float nz;
			unsigned char r; unsigned char g; unsigned char b; unsigned char a;
		};
		int size = 0;
		for (int i = 0; i < voxels.size(); i++) {
			if (voxels[i].type == Voxel::skeleton || voxels[i].type == Voxel::inside || voxels[i].type == Voxel::boundary)
				size++;
		}
		char t[256];
		sprintf(t, "%d", size);
		string size_str = t;
		FILE* fp = fopen(ply_path.c_str(), "wb");
		string header = "ply\nformat binary_little_endian 1.0\ncomment VCGLIB generated\nelement vertex ";
		header += size_str;
		header += "\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\nelement face 0\nproperty list uchar int vertex_indices\nend_header\n";
		int len = (int)strlen(header.c_str());
		fwrite(header.c_str(), len, 1, fp);
		PlyPart* ply = new PlyPart[size];
		int count = 0;
		for (int i = 0; i < voxels.size(); i++) {
			if (voxels[i].type == Voxel::skeleton || voxels[i].type == Voxel::inside || voxels[i].type == Voxel::boundary) {
				Vertex center = voxel_center(i);
				ply[count].x = (float)center.x;
				ply[count].y = (float)center.y;
				ply[count].z = (float)center.z;
				ply[count].nx = 0;
				ply[count].ny = 0;
				ply[count].nz = 0;
				ply[count].r = 255;
				ply[count].g = 0;
				ply[count].b = 0;
				ply[count].a = 255;
				count++;
			}
		}
		fwrite(ply, sizeof(PlyPart)*size, 1, fp);
		delete[] ply;
		fclose(fp);
	}
	// judge
	bool VoxelHandler::is_object(int index)
	{
		Voxel::VoxelType t = voxels[index].type;
		if (t == Voxel::inside || t == Voxel::skeleton /*|| t == Voxel::boundary*/)
			return true;
		return false;
	}
	bool VoxelHandler::is_inside(int index)
	{
		Voxel::VoxelType t = voxels[index].type;
		if (t == Voxel::inside)
			return true;
		return false;
	}
	bool VoxelHandler::is_boundary(int index)
	{
		Voxel::VoxelType t = voxels[index].type;
		if (t == Voxel::boundary)
			return true;
		return false;
	}
	bool VoxelHandler::is_object_border(int index)
	{
		Voxel::VoxelType t = voxels[index].type;
		if (t == Voxel::inside || t == Voxel::skeleton || t == Voxel::boundary)
			return true;
		return false;
	}
	bool VoxelHandler::is_skeleton(int index)
	{
		Voxel::VoxelType t = voxels[index].type;
		if (t == Voxel::skeleton)
			return true;
		return false;
	}
	// init
	void VoxelHandler::voxels_init(float voxel_size, float multiplier)
	{
		// find bounding box
		vxsize = voxel_size;
		int mini[3] = { 0 }, maxi[3] = { 0 };
		float min[3] = { DECIMAL_MAX, DECIMAL_MAX, DECIMAL_MAX };
		float max[3] = { DECIMAL_MIN, DECIMAL_MIN, DECIMAL_MIN };
		for (int i = 0; i < mesh->v.size(); i++)
		{
			const Vertex& vtx = mesh->v[i];
			if (vtx.x > max[0]) { max[0] = vtx.x; maxi[0] = i; }
			if (vtx.x < min[0]) { min[0] = vtx.x; mini[0] = i; }
			if (vtx.y > max[1]) { max[1] = vtx.y; maxi[1] = i; }
			if (vtx.y < min[1]) { min[1] = vtx.y; mini[1] = i; }
			if (vtx.z > max[2]) { max[2] = vtx.z; maxi[2] = i; }
			if (vtx.z < min[2]) { min[2] = vtx.z; mini[2] = i; }
		}
		// resize
		if (multiplier > 0) {
			for (int i = 0; i < 3; i++) {
				float len = max[i] - min[i];
				float d = (len * multiplier - len) / 2;
				min[i] -= d;
				max[i] += d;
			}
		}
		// origin of bounding box
		for (int i = 0; i < 3; i++)
			vxorg[i] = min[i];
		// voxel amount of each dimension
		float tempf[3];
		tempf[0] = (float)((max[0] - min[0]) / voxel_size);
		tempf[1] = (float)((max[1] - min[1]) / voxel_size);
		tempf[2] = (float)((max[2] - min[2]) / voxel_size);
		for (int i = 0; i < 3; i++)
			vxlen[i] = (int)ceil(tempf[i]);
		// generate voxels
		voxels.resize(vxlen[0] * vxlen[1] * vxlen[2]);
		for (int i = 0; i < vxlen[2]; i++)
		{
			for (int j = 0; j < vxlen[1]; j++)
			{
				for (int k = 0; k < vxlen[0]; k++)
				{
					int index = i*vxlen[1] * vxlen[0] + j*vxlen[0] + k;
					voxels[index].x = k;
					voxels[index].y = j;
					voxels[index].z = i;
				}
			}
		}
	}
	void VoxelHandler::find_boundary_voxel()
	{
		for (int i = 0; i < mesh->t.size(); i++)
		{
			// find bounding box of each triangle
			Voxel vx[3];
			vx[0] = voxels[vertex_in_which_voxel(mesh->v[mesh->t[i].v[0]])];
			vx[1] = voxels[vertex_in_which_voxel(mesh->v[mesh->t[i].v[1]])];
			vx[2] = voxels[vertex_in_which_voxel(mesh->v[mesh->t[i].v[2]])];
			int mini[3], maxi[3];
			mini[0] = std::min(std::min(vx[0].x, vx[1].x), vx[2].x);
			mini[1] = std::min(std::min(vx[0].y, vx[1].y), vx[2].y);
			mini[2] = std::min(std::min(vx[0].z, vx[1].z), vx[2].z);
			maxi[0] = std::max(std::max(vx[0].x, vx[1].x), vx[2].x);
			maxi[1] = std::max(std::max(vx[0].y, vx[1].y), vx[2].y);
			maxi[2] = std::max(std::max(vx[0].z, vx[1].z), vx[2].z);
			// initiate boundary voxels
			update_boundary(mini, maxi, i);
		}
		int count = 0;
		for (int i = 0; i < voxels.size(); i++)
		{
			if (voxels[i].type == Voxel::boundary) {
				voxels[i].face_start = count;
				count += voxels[i].face_num;
			}
		}
		for (int i = 0; i < voxels.size(); i++)
			voxels[i].label = 0;
		boundary_faces.resize(count);
		for (int i = 0; i < mesh->t.size(); i++)
		{
			// find bounding box of each triangle
			Voxel vx[3];
			vx[0] = voxels[vertex_in_which_voxel(mesh->v[mesh->t[i].v[0]])];
			vx[1] = voxels[vertex_in_which_voxel(mesh->v[mesh->t[i].v[1]])];
			vx[2] = voxels[vertex_in_which_voxel(mesh->v[mesh->t[i].v[2]])];
			int mini[3], maxi[3];
			mini[0] = std::min(std::min(vx[0].x, vx[1].x), vx[2].x);
			mini[1] = std::min(std::min(vx[0].y, vx[1].y), vx[2].y);
			mini[2] = std::min(std::min(vx[0].z, vx[1].z), vx[2].z);
			maxi[0] = std::max(std::max(vx[0].x, vx[1].x), vx[2].x);
			maxi[1] = std::max(std::max(vx[0].y, vx[1].y), vx[2].y);
			maxi[2] = std::max(std::max(vx[0].z, vx[1].z), vx[2].z);
			// initiate boundary voxels
			update_boundary_face(mini, maxi, i);
		}
		for (int i = 0; i < voxels.size(); i++)
			voxels[i].label = -1;
	}
	void VoxelHandler::update_boundary_face(int mini[], int maxi[], int face)
	{
		for (int iz = mini[2]; iz <= maxi[2]; iz++)
		{
			for (int iy = mini[1]; iy <= maxi[1]; iy++)
			{
				for (int ix = mini[0]; ix <= maxi[0]; ix++)
				{
					int index = voxel_index(ix, iy, iz);
					boundary_faces[voxels[index].face_start + voxels[index].label] = face;
					voxels[index].label++;
				}
			}
		}
	}
	void VoxelHandler::update_boundary(int mini[], int maxi[], int face)
	{
		for (int iz = mini[2]; iz <= maxi[2]; iz++)
		{
			for (int iy = mini[1]; iy <= maxi[1]; iy++)
			{
				for (int ix = mini[0]; ix <= maxi[0]; ix++)
				{
					int index = voxel_index(ix, iy, iz);
					voxels[index].type = Voxel::boundary;
					voxels[index].face_num++;
				}
			}
		}
	}
	void VoxelHandler::find_outside_voxel()
	{
		// using BFS (DFS is also OK)
		ArrayQueue<int> Q((int)voxels.size());
		voxels[0].type = Voxel::outside;
		Q.push(0);
		while (!Q.empty())
		{
			int current = Q.front();
			Q.pop();
			for (int i = 0; i < 27; i++)
			{
				int neighbor = -1;
				if (voxel_neighbor(current, i, neighbor))
				{
					if (voxels[neighbor].type == Voxel::undefined)
					{
						voxels[neighbor].type = Voxel::outside;
						Q.push(neighbor);
					}
				}
			}
		}
	}
	void VoxelHandler::find_inside_voxel()
	{
		// not outside or boundary, then voxel is inside
		for (int i = 0; i < voxels.size(); i++)
		{
			if (voxels[i].type == Voxel::undefined)
				voxels[i].type = Voxel::inside;
		}
	}
	// reset
	void VoxelHandler::voxels_reset_label()
	{
		for (int i = 0; i < voxels.size(); i++)
			voxels[i].label = -1;
	}
	void VoxelHandler::voxels_reset_SS_code()
	{
		for (int i = 0; i < (int)voxels.size(); i++)
			voxels[i].SS_code = -1;
	}
	void VoxelHandler::voxels_set_unvisited()
	{
		for (int i = 0; i < voxels.size(); i++)
			voxels[i].visited = false;
	}
	// functions
	int VoxelHandler::voxel_index(int x, int y, int z) {
		if (x < 0 || x >= vxlen[0] ||
			y < 0 || y >= vxlen[1] ||
			z < 0 || z >= vxlen[2])
			return -1;  // out of range
		return z*vxlen[1] * vxlen[0] + y*vxlen[0] + x;
	}
	int VoxelHandler::vertex_in_which_voxel(const Vertex& vertex) {
		int ix = (int)((vertex.x - vxorg[0]) / vxsize);
		if (ix < 0 || ix >= vxlen[0]) return -1;
		int iy = (int)((vertex.y - vxorg[1]) / vxsize);
		if (iy < 0 || iy >= vxlen[1]) return -1;
		int iz = (int)((vertex.z - vxorg[2]) / vxsize);
		if (iz < 0 || iz >= vxlen[2]) return -1;
		int ret = voxel_index(ix, iy, iz);
		return ret;
	}
	bool VoxelHandler::voxel_neighbor(int index, int num, int& neighbor)
	{
		if (num == 13)  // itself
			return false;
		const Voxel& current = voxels[index];
		int temp[3];
		temp[0] = current.x + (num % 3) - 1;
		temp[1] = current.y + ((int)((float)num / (float)3) % 3) - 1;
		temp[2] = current.z + (int)((float)num / (float)9) - 1;
		if (temp[0] < 0 || temp[0] >= vxlen[0] ||
			temp[1] < 0 || temp[1] >= vxlen[1] ||
			temp[2] < 0 || temp[2] >= vxlen[2])
			return false;  // out of range
		neighbor = voxel_index(temp[0], temp[1], temp[2]);
		return true;
	}
	bool VoxelHandler::voxel_six_neighbor(int index, int num, int& neighbor)
	{
		const int nummap[6] = { 12, 14, 10, 16, 4, 22 };
		num = nummap[num];
		const Voxel& current = voxels[index];
		int temp[3];
		temp[0] = current.x + (num % 3) - 1;
		temp[1] = current.y + ((int)((float)num / (float)3) % 3) - 1;
		temp[2] = current.z + (int)((float)num / (float)9) - 1;
		if (temp[0] < 0 || temp[0] >= vxlen[0] ||
			temp[1] < 0 || temp[1] >= vxlen[1] ||
			temp[2] < 0 || temp[2] >= vxlen[2])
			return false;  // out of range
		neighbor = voxel_index(temp[0], temp[1], temp[2]);
		return true;
	}
	Vertex VoxelHandler::voxel_center(int index) {
		Vertex ret;
		ret.x = (float)voxels[index].x*vxsize + vxorg[0];
		ret.y = (float)voxels[index].y*vxsize + vxorg[1];
		ret.z = (float)voxels[index].z*vxsize + vxorg[2];
		float half = 0.5;
		ret.x += half*vxsize;
		ret.y += half*vxsize;
		ret.z += half*vxsize;
		return ret;
	}
	float VoxelHandler::voxel_distance_s(int index0, int index1)
	{
		Vertex v0;
		v0.x = (float)voxels[index0].x*vxsize;
		v0.y = (float)voxels[index0].y*vxsize;
		v0.z = (float)voxels[index0].z*vxsize;
		Vertex v1;
		v1.x = (float)voxels[index1].x*vxsize;
		v1.y = (float)voxels[index1].y*vxsize;
		v1.z = (float)voxels[index1].z*vxsize;
		float temp = v0.x - v1.x;
		float ret = temp*temp;
		temp = v0.y - v1.y;
		ret += temp*temp;
		temp = v0.z - v1.z;
		ret += temp*temp;
		return ret;
	}
}