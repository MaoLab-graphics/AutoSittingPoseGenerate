// Functions

#include "Global.h"
#include "Functions.h"
#include "Reshape.h"
#include "Voxel.h"
#include "Skeletonization.h"
#include <sstream>
using namespace std;

namespace ManSeg {

	// -------------------- Function --------------------

	string get_current_system_time() {
		struct tm* p_tm; time_t timep;
		time(&timep); p_tm = localtime(&timep);
		char date[60] = { 0 };
		sprintf_s(date, "%d-%02d-%02d %02d:%02d:%02d",
			(int)p_tm->tm_year + 1900, (int)p_tm->tm_mon + 1, (int)p_tm->tm_mday,
			(int)p_tm->tm_hour, (int)p_tm->tm_min, (int)p_tm->tm_sec);
		return string(date);
	}
	void print_time_message(const string& mes) {
		std::cout << mes << "\t" << get_current_system_time() << std::endl;
	}
	void split_mesh(float voxel_size, const string& filename) {

		Mesh mesh;
		string path = MESH_PATH;
		mesh.read_from_file(path + filename + ".ply");
		mesh.mesh_file_name = filename;
		VoxelHandler vxh(&mesh, voxel_size);
		SkeletonGenerator se(&mesh, &vxh);
		SkeletalSegmentation seg(&mesh, &vxh);
		seg.do_human_segmentation();
		//mesh.update_all_vertex();
		mesh.write_to_file(path + "out_" + filename + "_.ply");
		mesh.write_to_ply(path + "out_" + filename + ".ply");
	}
	void transfer_coordinate(const string& in_file, const string& out_file) {
		Mesh mesh;
		mesh.read_from_file(in_file);
		Mesh mesho;
		mesho.read_from_file(out_file);
		mesho.transfer_vertex_coord(mesh);
		mesho.update_all_vertex();
		mesho.write_to_file(out_file);
	}
	void extract_skeleton(float voxel_size, const string& in_file) {
		Mesh mesh;
		mesh.read_from_file(in_file);
		VoxelHandler vxh(&mesh, voxel_size);
		vxh.write_ply("mesh/insidevoxel.ply");
		SkeletonGenerator se(&mesh, &vxh, false);
		SkeletalSegmentation seg(&mesh, &vxh);
		seg.write_ply("mesh/ske_uncut.ply");
		VoxelHandler vxhs(&mesh, voxel_size);
		SkeletonGenerator ses(&mesh, &vxhs, true);
		SkeletalSegmentation segs(&mesh, &vxhs);
		segs.write_smooth_skeleton();
	}
	void hks_feature_points(const string& in_file, const string& out_file) {
		Mesh mesh;
		string path = MESH_PATH;
		mesh.read_from_file(path + in_file + ".ply");
		string hks_path = "hks_" + in_file + ".txt";
		FILE* fp = fopen(hks_path.c_str(), "rb");
		std::vector<double> hks(mesh.v.size());
		fread(&hks[0], mesh.v.size() * sizeof(double), 1, fp);
		fclose(fp);
		for (int i = 0; i < (int)mesh.v.size(); ++i) {
			bool flag = true;
			for (int j = 0; j < mesh.vvn[i]; ++j) {
				int n = mesh.vv(i, j);
				if (n != i && hks[i] <= hks[n]) flag = false;
				/*for (int k = 0; k < mesh.vvn[n]; ++k) {
					int nn = mesh.vv(n, k);
					if (nn != i && hks[i] <= hks[nn]) flag = false;
				}*/
			}
			if (flag) mesh.paint_ball_openmesh(mesh.v[i],BALL_RADIUS,BALL_CUTS, mesh.YELLOW);
			//else mesh.color_vertex(i, mesh.WHITE);
		}
		mesh.write_to_file(path + out_file + ".ply");
		return;
		double maxhks = -std::numeric_limits<double>::max(), minhks = std::numeric_limits<double>::max();
		for (auto h : hks) {
			if (h > maxhks) maxhks = h;
			if (h < minhks) minhks = h;
		}
		for (int i = 0; i < (int)mesh.v.size(); ++i) {

			double h = (hks[i] - minhks) / (maxhks - minhks);
			h = -6.0*std::pow(h - 0.5, 2.0) + 1.0;
			if (h < 0.0) h = 0.0;
			if (h > 1.0) h = 1.0;
			// HSV
			int H = 240.0 - (240.0*(h));
			float S = 1, V = 1;
			// HSV to RGB
			mesh.color_vertex(i, RGBA::HSV2RGB(H, S, V));
		}
		mesh.write_to_file(path + out_file + ".ply");
	}
	void compute_landmark_total_error() {

		SkeletalSegmentation seg;
		seg.compute_total_landmark_error();
	}
	void deform_to_sitting_pose(int personNum,int chairnum) {
		stringstream ss;
		ss << personNum;
		string personString = ss.str();
		string filename = "p" + personString;
		Mesh mesh;
		string path = MESH_PATH;
		mesh.read_from_file(path + filename + ".ply");
		mesh.mesh_file_name = filename;
		mesh.read_skeleton_data("skedata_" + filename + ".bin");
		float voxel_size = 10;
		if (personNum % 1000 == 2)
			voxel_size = 8;
		VoxelHandler vxh(&mesh, voxel_size);
		SkeletonGenerator se(&mesh, &vxh);
		Reshape reshape(&mesh, &vxh);
		reshape.deform_to_seat(personNum,chairnum);

	}
}