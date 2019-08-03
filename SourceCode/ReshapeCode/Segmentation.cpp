// Segmentation

#include "SDF.h"
#include "Segmentation.h"
#include "Mesh.h"
#include "Voxel.h"
#include "MassSpringSystem.h"
#include "Skeletonization.h"
#include "Kmeans.h"
#include "GMM.h"
#include "GraphCut.h"
#include <string>
#include <time.h>
#include <fstream>
#include <set>

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
		Voxel_Handler vxh(&mesh, voxel_size);
		Skeleton_Generator se(&mesh, &vxh);
		SkeletalSegmentation seg(&mesh, &vxh);
		seg.run();
		//mesh.update_all_vertex();
		mesh.write_to_file(path + "out_" + filename + "_.ply");
		mesh.write_to_ply(path + "out_" + filename + ".ply");
		seg.write_off(path + "out_" + filename + ".off");
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
		Voxel_Handler vxh(&mesh, voxel_size);
		vxh.write_ply("mesh/insidevoxel.ply");
		Skeleton_Generator se(&mesh, &vxh, false);
		SkeletalSegmentation seg(&mesh, &vxh);
		seg.write_ply("mesh/ske_uncut.ply");
		Voxel_Handler vxhs(&mesh, voxel_size);
		Skeleton_Generator ses(&mesh, &vxhs, true);
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
			if (flag) mesh.color_vertex(i, mesh.RED);
			else mesh.color_vertex(i, mesh.WHITE);
		}
		std::vector<int> hks_c(mesh.v.size());
		double maxd = -std::numeric_limits<float>::max(), mind = std::numeric_limits<float>::max();
		for (auto h : hks) {
			if (h > maxd) maxd = h;
			if (h < mind) mind = h;
		}
		for (int i = 0; i < (int)hks.size();++i) {
			double h = (hks[i] - mind) / (maxd - mind);
			hks_c[i] = (int)(767.0*std::sqrt(h));
		}
		for (int i = 0; i < (int)mesh.v.size(); ++i) {
			Color color; color[0] = color[1] = color[2] = 0;
			if (hks_c[i] > 511) {
				color[2] = hks_c[i] - 255; color[0] = color[1] = 255;
			}
			else if (hks_c[i] > 255) {
				color[1] = hks_c[i] - 255; color[0] = 255;
			}
			else color[0] = hks_c[i];
			mesh.color_vertex(i, color);
		}
		mesh.write_to_file(path + out_file + ".ply");
	}
	void compute_landmark_total_error() {

		SkeletalSegmentation seg;
		seg.compute_total_landmark_error();
	}
	void deform_to_sitting_pose(float voxel_size, const string& filename) {

		Mesh mesh;
		string path = MESH_PATH;
		mesh.read_from_file(path + filename + ".ply");
		mesh.mesh_file_name = filename;
		mesh.read_skeleton_data("skedata_" + filename + ".bin");
		Voxel_Handler vxh(&mesh, voxel_size);
		Skeleton_Generator se(&mesh, &vxh);
		Reshape reshape(&mesh, &vxh);
		reshape.deform_to_seat();
		mesh.write_to_file(path + "_out_" + filename + ".ply");
	}

	// -------------------- Class Segmentation --------------------

	Segmentation::Segmentation(Mesh* in_mesh) : _mesh(in_mesh) {
		_preset_color[0] = _mesh->RED; _preset_color[1] = _mesh->YELLOW; _preset_color[2] = _mesh->GREEN;
		_preset_color[3] = _mesh->CYAN; _preset_color[4] = _mesh->BLUE; _preset_color[5] = _mesh->PURPLE;
		_preset_color[6] = _mesh->WHITE;
		_preset_color[7] = _mesh->RED; _preset_color[8] = _mesh->YELLOW; _preset_color[9] = _mesh->GREEN;
		_preset_color[10] = _mesh->CYAN; _preset_color[11] = _mesh->BLUE; _preset_color[12] = _mesh->PURPLE;
	}
	Segmentation::Segmentation(Mesh* in_mesh, Voxel_Handler* vxh) : _mesh(in_mesh), _voxelh(vxh) {
		_preset_color[0] = _mesh->RED; _preset_color[1] = _mesh->YELLOW; _preset_color[2] = _mesh->GREEN;
		_preset_color[3] = _mesh->CYAN; _preset_color[4] = _mesh->BLUE; _preset_color[5] = _mesh->PURPLE;
		_preset_color[6] = _mesh->WHITE;
		_preset_color[7] = _mesh->RED; _preset_color[8] = _mesh->YELLOW; _preset_color[9] = _mesh->GREEN;
		_preset_color[10] = _mesh->CYAN; _preset_color[11] = _mesh->BLUE; _preset_color[12] = _mesh->PURPLE;
	}
	void Segmentation::write_off(string off_path) {
		std::fstream fs(off_path, std::ios::out);
		fs << "OFF\n" << (int)_mesh->v.size() << ' ' << (int)_mesh->t.size() << ' ' << 0 << '\n';
		for (int i = 0; i < (int)_mesh->v.size(); ++i) {
			fs << _mesh->v[i].x << ' ' << _mesh->v[i].y << ' ' << _mesh->v[i].z << '\n';
		}
		for (int i = 0; i < (int)_mesh->t.size(); ++i) {
			fs << 3 << ' ' << _mesh->t[i].v[0] << ' ' << _mesh->t[i].v[1] << ' ' << _mesh->t[i].v[2] << ' '
				<< (int)_mesh->tcolor[i].r << ' ' << (int)_mesh->tcolor[i].g << ' ' << (int)_mesh->tcolor[i].b << '\n';
		}
		fs.close();
	}
	void Segmentation::write_ply(string ply_path) {
		int size = 0;
		for (int i = 0; i < _voxelh->voxels.size(); i++) {
			if (_voxelh->voxels[i].type == Voxel::skeleton /*|| _voxelh->voxels[i].type == Voxel::inside*/)
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
		for (int i = 0; i < _voxelh->voxels.size(); i++) {
			if (_voxelh->voxels[i].type == Voxel::skeleton /*|| _voxelh->voxels[i].type == Voxel::inside*/) {
				Vertex center = _voxelh->voxel_center(i);
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
	void Segmentation::write_ply(VtxVec& points, string ply_path) {
		int size = (int)points.size();
		char t[256]; sprintf(t, "%d", size); string size_str = t;
		FILE* fp = fopen(ply_path.c_str(), "wb");
		string header = "ply\nformat binary_little_endian 1.0\ncomment VCGLIB generated\nelement vertex ";
		header += size_str;
		header += "\nproperty float x\nproperty float y\nproperty float z\n";
		header += "property float nx\nproperty float ny\nproperty float nz\n";
		header += "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";
		header += "element face 0\nproperty list uchar int vertex_indices\nend_header\n";
		int len = (int)strlen(header.c_str());
		fwrite(header.c_str(), len, 1, fp);
		PlyPart* ply = new PlyPart[size];
		for (int i = 0; i < size; ++i) {
			Vertex& pt = points[i];
			ply[i].x = (float)pt.x;
			ply[i].y = (float)pt.y;
			ply[i].z = (float)pt.z;
			ply[i].nx = 0;
			ply[i].ny = 0;
			ply[i].nz = 0;
			ply[i].r = 0;
			ply[i].g = 0;
			ply[i].b = 0;
			ply[i].a = 255;
		}
		fwrite(ply, sizeof(PlyPart)*size, 1, fp);
		delete[] ply;
		fclose(fp);
	}
	void Segmentation::search_and_label_skeleton_points(IntVec& ske, IntVec& end)
	{
		// collect skeleton points and label their type
		_voxelh->voxels_reset_label();
		end.clear();
		ske.clear();
		for (int idx = 0; idx < (int)_voxelh->voxels.size(); idx++)
		{
			if (_voxelh->is_skeleton(idx))
			{
				int count = 0;
				for (int i = 0; i < 27; i++)
				{
					int ni = -1;
					if (_voxelh->voxel_neighbor(idx, i, ni))
						if (_voxelh->is_skeleton(ni))
							count++;
				}
				ske.push_back(idx);
				_voxelh->voxels[idx].label = count;
			}
		}
		// deal with triangle problem
		_voxelh->voxels_set_unvisited();
		int count_triangle = 0;
		for (int idx = 0; idx < (int)ske.size(); idx++)
		{
			if (_voxelh->voxels[ske[idx]].label > 2)
			{
				IntVec ns;
				IntVec nsv;
				ns.push_back(ske[idx]);
				nsv.push_back(0);
				for (int i = 0; i < 27; i++)
				{
					int ni = -1;
					if (_voxelh->voxel_neighbor(ske[idx], i, ni))
					{
						if (_voxelh->is_skeleton(ni))
						{
							ns.push_back(ni);
							nsv.push_back(0);
						}
					}
				}
				for (int i = 0; i < (int)ns.size(); i++)
				{
					for (int j = 0; j < 27; j++)
					{
						int ni = -1;
						if (_voxelh->voxel_neighbor(ns[i], j, ni))
							if (_voxelh->is_skeleton(ni))
								for (int k = 0; k < (int)ns.size(); k++)
									if (ni == ns[k])
										nsv[k]++;
					}
				}
				IntVec tri;
				for (int i = 0; i < (int)ns.size(); i++)
				{
					if (nsv[i] >= 2)
						tri.push_back(ns[i]);
				}
				if ((int)tri.size() >= 3)
				{
					count_triangle++;
					Vertex vtx = { 0, 0, 0 };
					for (int i = 0; i < (int)tri.size(); i++)
					{
						vtx += _voxelh->voxel_center(tri[i]);
					}
					float trisize = (float)tri.size();
					vtx.x /= trisize; vtx.y /= trisize; vtx.z /= trisize;
					float mindist = std::numeric_limits<float>::max(); int index = -1;
					for (int i = 0; i < (int)tri.size(); i++)
					{
						float dist = (_voxelh->voxel_center(tri[i]) - vtx).square_length();
						if (dist < mindist)
						{
							index = tri[i];
							mindist = dist;
						}
					}
					for (int i = 0; i < (int)tri.size(); i++)
					{
						if (tri[i] != index)
						{
							if (!_voxelh->voxels[tri[i]].visited)
							{
								_voxelh->voxels[tri[i]].label -= 1;
								_voxelh->voxels[tri[i]].visited = true;
							}
						}
					}

				}
			}
		}
		// collect end points
		for (int idx = 0; idx < (int)ske.size(); idx++)
		{
			if (_voxelh->voxels[ske[idx]].label <= 1)
			{
				end.push_back(ske[idx]);
			}
		}
	}
	void Segmentation::find_skeleton_leaves(Int2DVec& leaves) {
		// collect skeleton points and end points, also label their type
		IntVec end_points;
		IntVec skeleton_points;
		search_and_label_skeleton_points(skeleton_points, end_points);
		if (end_points.size() != 5) std::cout << "end point amount is not 5 !" << std::endl;
		// find leaves
		std::vector<IntVec> temp_leaves(5);
		for (int idx = 0; idx < (int)end_points.size(); idx++) {
			BoolVec visited(_voxelh->voxels.size(), false);
			int current = end_points[idx];
			while (1) {
				visited[current] = true;
				bool stop = false;
				const Voxel& vx = _voxelh->voxels[current];
				if (_voxelh->voxels[current].label > 2)
					break;
				temp_leaves[idx].push_back(current);
				int next = -1;
				for (int i = 0; i < 27; i++) {
					int ni = -1;
					if (_voxelh->voxel_neighbor(current, i, ni)) {
						if (_voxelh->is_skeleton(ni)) {
							if (_voxelh->voxels[ni].label > 2)
								stop = true;
							if (!visited[ni])
								next = ni;
						}
					}
				}
				if (stop)
					break;
				if (next == -1) {
					std::cout << "error when finding leaf\n"; break;
				}
				current = next;
			}
		}
		// sort
		int corr[5] = { 0, 1, 2, 3, 4 };
		for (int i = 0; i < 4; i++) {
			for (int j = i + 1; j < 5; j++) {
				if (temp_leaves[corr[i]].size() > temp_leaves[corr[j]].size()) {
					int temp = corr[i]; corr[i] = corr[j]; corr[j] = temp;
				}
			}
		}
		leaves.resize(PartType::p_part_amount);
		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < (int)temp_leaves[corr[i]].size(); j++) {
				leaves[i].push_back(temp_leaves[corr[i]][j]);
			}
		}
		// find body
		BoolVec visited(_voxelh->voxels.size(), false);
		for (int i = 0; i < (int)leaves.size(); i++) {
			for (int j = 0; j < (int)leaves[i].size(); j++) {
				visited[leaves[i][j]] = true;
			}
		}
		int bodyidx = PartType::p_body;
		int current = leaves[0][leaves[0].size() - 1];
		while (1) {
			visited[current] = true;
			bool stop = true;
			const Voxel& vx = _voxelh->voxels[current];
			leaves[bodyidx].push_back(current);
			int next = -1;
			for (int i = 0; i < 27; i++) {
				int ni = -1;
				if (_voxelh->voxel_neighbor(current, i, ni)) {
					if (_voxelh->is_skeleton(ni)) {
						if (!visited[ni]) {
							stop = false;
							next = ni;
						}
					}
				}
			}
			if (stop)
				break;
			if (next == -1) {
				std::cout << "error when finding leaf\n"; break;
			}
			current = next;
		}
	}

	// -------------------- Class SkeletalSegmentation --------------------

	void SkeletalSegmentation::run() {
		//split_mesh_by_border_voxel();
		//split_mesh_by_SS_code();
		if (FIND_LANDMARKS) read_landmark_ground_truth();
		split_mesh_by_slice();
		if (REFINE_SEG) refine_split_by_slice();
		if (FIND_LANDMARKS || FIND_FEATURE_POINTS) find_landmarks();
		if (FIND_LANDMARKS) write_landmark_error();
		print_time_message("split end");
	}
	void SkeletalSegmentation::write_smooth_skeleton() {
		// init
		for (int idx = 0; idx < (int)_mesh->v.size(); idx++) {
			_mesh->color_vertex(idx, _preset_color[6]);
		}
		// find leaves
		Int2DVec leaves;
		find_skeleton_leaves(leaves);
		for (int i = 0; i < PartType::p_part_amount; i++) {
			_slices_face[i].resize((int)leaves[i].size());
			for (int j = 0; j < (int)leaves[i].size(); j++) {
				_skeleton[i].push_back(_voxelh->voxel_center(leaves[i][j]));
			}
		}
		smooth_skeleton_points(0, 5, SMOOTH_SKELETON_TIMES);
		VtxVec write_skepts, body_skepts;
		for (int i = 0; i < PartType::p_part_amount; i++) {
			for (int j = 0; j < (int)leaves[i].size(); j++)
				write_skepts.push_back(_skeleton[i][j]);
			if (i == 5) {
				for (int j = 0; j < (int)leaves[i].size(); j++)
					body_skepts.push_back(_skeleton[i][j]);
			}
		}
		write_ply(write_skepts, "mesh/ske_refined.ply");
		//write_ply(body_skepts, "mesh/body_ske.ply");
	}
	// refine segmentation
	void SkeletalSegmentation::smooth_split(CharVec& labels) {
		int body_id = 3;
		CharVec labels_old(_mesh->t.size());
		std::memcpy(&labels_old[0], &labels[0], sizeof(labels_old[0]) * labels_old.size());
		for (int idx = 0; idx < (int)_mesh->t.size(); idx++) {
			int curr_label = labels_old[idx], neighbor_label = -1;
			if (curr_label != -1) {
				int count_num[20] = { 0 };
				bool flag = false;
				for (int i = 0; i < 3; i++) {
					int n = _mesh->tt(idx, i);
					if (n >= 0 && labels_old[n] != curr_label) {
						neighbor_label = labels_old[n];
						flag = true;
						break;
					}
				}
				if (flag) {
					for (int i = 0; i < 3; i++) {
						int n = _mesh->tt(idx, i);
						if (n >= 0 && labels_old[n] == curr_label)
							if (_mesh->triangle_dihedral_angle(idx, n, false) < SMOOTH_ANGLE)
								labels[idx] = neighbor_label;
					}
				}
			}
		}
	}
	void SkeletalSegmentation::classify_vertices_by_label(const CharVec& labels, CharVec& ret) {
		for (int idx = 0; idx < (int)_mesh->v.size(); ++idx) {
			int max_label = -1, max_num = 0, total_num = _mesh->vtn[idx];
			int count_num[20] = { 0 };
			for (int i = 0; i < total_num; i++) {
				int n = _mesh->vt(idx, i);
				if (n >= 0) {
					int label = labels[n];
					if (++count_num[label] > max_num) {
						max_label = label;
						max_num = count_num[label];
					}
				}
			}
			ret[idx] = max_label;
		}
	}
	void read_skeleton(string ply_path, std::vector<Vector>& ske) {
		struct PlyPart {
			float x; float y; float z;
			float nx; float ny; float nz;
			unsigned char r; unsigned char g; unsigned char b; unsigned char a;
		};
		int size = 0;
		char t[1024];
		FILE* fp = fopen(ply_path.c_str(), "rb");
		char* cmp = "element vertex"; int len = strlen(cmp);
		char* end = "end_header"; int endlen = strlen(end);
		while (1) {
			fgets(t, 1024, fp);
			if (memcmp(t, cmp, len) == 0) {
				for (char* p = (char*)(t + (len + 1)); *p >= '0' && *p <= '9'; ++p) {
					size = 10 * size + (*p - '0');
				}
			}
			if (memcmp(t, end, endlen) == 0)
				break;
		}
		std::vector<PlyPart> vec(size);
		fread(&vec[0], size*sizeof(PlyPart), 1, fp);
		ske.resize(size);
		for (int i = 0; i < size; ++i) {
			ske[i].x = vec[i].x;
			ske[i].y = vec[i].y;
			ske[i].z = vec[i].z;
		}
		fclose(fp);
	}
	void smooth_skeleton_points_single(std::vector<Vector>& ske, int loop_times) {
		const int interval = 1;
		for (int i = 0; i < loop_times; ++i) {
			auto temp = ske;
			for (int j = interval; j < (int)ske.size() - interval; ++j) {
				ske[j] = (temp[j - interval] + temp[j + interval]) / (float)2;
			}
		}
	}
	void SkeletalSegmentation::label_faces_BFS(const IntVec& border, const Plane& plane, char label, CharVec& labels, char flag) {
		int face_num = (int)_mesh->t.size();
		BoolVec visited(face_num, false);
		ArrayQueue<int> Q(face_num);
		for (int i = 0; i < (int)border.size(); i++) {
			visited[border[i]] = true;
			Q.push(border[i]);
			if (flag < -1 || labels[border[i]] == flag) labels[border[i]] = label;
		}
		int cnt = 0;
		while (!Q.empty()) {
			int curr = Q.front(); Q.pop();
			for (int i = 0; i < 3; i++) {
				int n = _mesh->tt(curr, i);
				if (n >= 0) {
					int temp = face_cross_plane(n, plane);
					if (!visited[n] && temp == 3 && (flag < -1 || labels[n] == flag)) {
						visited[n] = true; Q.push(n);
						labels[n] = label;
						++cnt;
					}
				}
			}
		}
		//std::cout << (int)label << " " << cnt << std::endl;
	}
	void SkeletalSegmentation::filter(CharVec& labels) {
		const int part_ampunt = 20;
		float limit = (float)0.0000001;
		float sigma_c = FILTER_RADIUS / 2, radius_s = 4 * sigma_c * sigma_c;
		float sigma_s = 0.5, sigma_a = 200;
		auto weight = [](float x, float sigma){return std::exp((-x*x) / (2 * sigma*sigma)); };
		int face_num = (int)_mesh->t.size();
		ArrayQueue<int> Q(face_num);
		BoolVec visited(face_num, false);
		IntVec indices(face_num); int count = 0;
		int part_exist[part_ampunt] = { 0 };
		int part_map[part_ampunt] = { 0 };
		for (int it = 0; it < FILTER_ITERATION; ++it) {
			CharVec labels_old = labels;
			bool flag = false;
			for (int idx = 0; idx < (int)_mesh->t.size(); idx++) {
				visited[idx] = true; Q.push(idx);
				while (!Q.empty()) {
					int curr = Q.front(); Q.pop();
					for (int i = 0; i < 3; i++) {
						int n = _mesh->tt(curr, i);
						if (n >= 0 && !visited[n] && triangle_distance_s(idx, n) < radius_s) {
							if (_mesh->triangle_dihedral_angle(idx, n, false) > SMOOTH_ANGLE) {
								visited[n] = true; Q.push(n); indices[count++] = n;
							}
						}
					}
				}
				if (count > 0) {
					// part
					int part_num = 0;
					std::memset(part_exist, -1, part_ampunt * sizeof(int));
					std::memset(part_map, -1, part_ampunt * sizeof(int));
					visited[idx] = false;
					for (int j = 0; j < count; ++j) {
						int t = indices[j];
						visited[t] = false;
						if (part_exist[labels_old[t]] < 0) {
							part_exist[labels_old[t]] = part_num;
							part_map[part_num] = labels_old[t];
							++part_num;
						}
					}
					// average
					float sum = 0;
					for (int j = 0; j < count; ++j) {
						int t = indices[j];
						sum += part_exist[labels_old[t]];
					}
					float avg = sum / count;
					// new label
					sum = 0; float sum2 = 0;
					for (int j = 0; j < count; ++j) {
						int t = indices[j];
						float temp = (float)1;
						temp *= weight(std::sqrt(triangle_distance_s(idx, t)), sigma_c);
						//temp *= weight(abs((float)part_exist[labels_old[idx]] - (float)part_exist[labels_old[t]]), sigma_s);
						//temp *= weight(_mesh->triangle_area(t), sigma_a);
						//temp *= 1 / std::sqrt(triangle_distance_s(idx, t));
						temp *= _mesh->triangle_area(t);
						sum += temp*part_exist[labels_old[t]];
						sum2 += temp;
					}
					float newlabel = sum / sum2;
					float comp0 = abs(std::floor(newlabel) - newlabel);
					float comp1 = abs(std::ceil(newlabel) - newlabel);
					int newlabel0 = std::floor(newlabel);
					int newlabel1 = std::ceil(newlabel);
					int label_new = comp0 < comp1 ? newlabel0 : newlabel1;
					if (part_map[label_new] >= 0 && labels[idx] != part_map[label_new]) {
						flag = true;
						labels[idx] = part_map[label_new];
					}
				}
				count = 0;
				Q.clear();
			}
			if (!flag)
				break;
		}
	}
	void SkeletalSegmentation::refine_cutting(CharVec& labels) {
		int max_it = 100;
		for (int it = 0; it < max_it; ++it) {
			int vertex_num = (int)_mesh->v.size();
			const Mesh& cmesh = *_mesh;
			BoolVec visited(vertex_num, false);
			auto is_border = [&labels, &cmesh](int v) {
				int first = labels[cmesh.vt(v, 0)];
				for (int i = 1; i < cmesh.vtn[v]; ++i)
					if (labels[cmesh.vt(v, i)] != first) return true;
				return false;
			};
			auto next_border = [&is_border, &visited, &cmesh](int v) {
				int ret[10] = { -1, -1 }, count = 0;
				for (int i = 0; i < cmesh.vvn[v]; ++i) {
					int n = cmesh.vv(v, i);
					if (!visited[n] && is_border(n)) {
						ret[count++] = n;
					}
				}
				if (count == 1) return ret[0];
				else if (count == 2) {
					for (int i = 0; i < 2; ++i) {
						bool flag = false;
						for (int j = 0; j < cmesh.vvn[ret[i]]; ++j) {
							int n = cmesh.vv(ret[i], j);
							if (n != v && n != ret[1 - i]) {
								return ret[1 - i];
							}
						}
					}
					return ret[0];
				}
				else if (count > 2) {
					std::cout << "found more than 2 neighbor in refine_cutting\n";
					return ret[0];
				}
				else return -1;
			};
			auto do_refine = [&labels, &cmesh](int b, int c, int f0, int f1) {
				//Vector& cb = cmesh.v[b] - cmesh.v[c], cf0 = cmesh.v[f0] - cmesh.v[c], cf1 = cmesh.v[f1] - cmesh.v[c];
				//float cbs = cb.length(), cf0s = cf0.length(), cf1s = cf1.length();
				//float cos0 = cb.dot(cf0) / (cbs*cf0s), cos1 = cb.dot(cf1) / (cbs*cf1s);
				//if (cos1 < cos0)
				{
					int tri = -1, label0 = labels[cmesh.vt(c, 0)], label1 = -1;
					for (int i = 0; i < cmesh.vtn[c]; ++i) {
						int ti = cmesh.vt(c, i);
						if (labels[ti] != label0) label1 = labels[ti];
						const Triangle& t = cmesh.t[ti];
						bool flag = true;
						for (int j = 0; j < 3; ++j)
							if (t.v[j] != c && t.v[j] != f0 && t.v[j] != f1) flag = false;
						if (flag) tri = ti;
					}
					if (tri >= 0 && label1 >= 0) {
						labels[tri] = (labels[tri] == label0 ? label1 : label0);
						return true;
					}
				}
				return false;
			};
			IntVec ring;
			bool stop = true;
			for (int idx = 0; idx < vertex_num; ++idx) {
				if (!visited[idx] && is_border(idx)) {
					visited[idx] = true; ring.push_back(idx); int curr = idx;
					while ((curr = next_border(curr)) >= 0) {
						visited[curr] = true; ring.push_back(curr);
					}
					int ring_size = (int)ring.size();
					if (ring_size > 2) {
						for (int i = 0; i < ring_size; ++i) {
							int back = i - 1 >= 0 ? i - 1 : i - 1 + ring_size;
							int front = i + 1 < ring_size ? i + 1 : i + 1 - ring_size;
							int front2 = i + 2 < ring_size ? i + 2 : i + 2 - ring_size;
							if (do_refine(ring[back], ring[i], ring[front], ring[front2])) {
								ring[front] = i++; stop = false;
							}
						}
					}
					ring.clear();
				}
			}
			if (stop)
				break;
		}
	}
	void SkeletalSegmentation::classify_vertices_by_filter(const CharVec& labels, CharVec& ret) {
		const int part_ampunt = 20;
		float limit = (float)0.0000001;
		float sigma_c = VERTEX_FILTER_RADIUS / 2, radius_s = 4 * sigma_c * sigma_c;
		float sigma_s = 0.5;
		auto weight = [](float x, float sigma){return std::exp((-x*x) / (2 * sigma*sigma)); };
		int vertex_num = (int)_mesh->v.size();
		int face_num = (int)_mesh->t.size();
		ArrayQueue<int> Q(face_num);
		BoolVec visited(face_num, false);
		IntVec indices(face_num); int count = 0;
		int part_exist[part_ampunt] = { 0 };
		int part_map[part_ampunt] = { 0 };
		for (int idx = 0; idx < vertex_num; idx++) {
			Q.push(_mesh->vt(idx, 0));
			const Vector& vtx = _mesh->v[idx];
			while (!Q.empty()) {
				int curr = Q.front(); Q.pop();
				for (int i = 0; i < 3; i++) {
					int n = _mesh->tt(curr, i);
					if (n >= 0 && !visited[n] && (vtx - triangle_center(n)).square_length() < radius_s) {
						visited[n] = true; Q.push(n); indices[count++] = n;
					}
				}
			}
			if (count > 0) {
				// part
				int part_num = 0;
				std::memset(part_exist, -1, part_ampunt * sizeof(int));
				std::memset(part_map, -1, part_ampunt * sizeof(int));
				for (int j = 0; j < count; ++j) {
					int t = indices[j];
					visited[t] = false;
					if (part_exist[labels[t]] < 0) {
						part_exist[labels[t]] = part_num;
						part_map[part_num] = labels[t];
						++part_num;
					}
				}
				// average
				float sum = 0;
				for (int j = 0; j < count; ++j) {
					int t = indices[j];
					sum += part_exist[labels[t]];
				}
				float avg = sum / count;
				// new label
				sum = 0; float sum2 = 0;
				for (int j = 0; j < count; ++j) {
					int t = indices[j];
					//float temp = (float)1;
					float temp = (vtx - triangle_center(t)).length();
					temp = weight(temp, sigma_c);
					//temp *= weight(abs((float)part_exist[labels_old[idx]] - (float)part_exist[labels_old[t]]), sigma_s);
					sum += temp*part_exist[labels[t]];
					sum2 += temp;
				}
				float newlabel = sum / sum2;
				float comp0 = abs(std::floor(newlabel) - newlabel);
				float comp1 = abs(std::ceil(newlabel) - newlabel);
				int newlabel0 = std::floor(newlabel);
				int newlabel1 = std::ceil(newlabel);
				int label_new = comp0 < comp1 ? newlabel0 : newlabel1;
				if (part_map[label_new] >= 0) {
					ret[idx] = part_map[label_new];
				}
			}
			count = 0;
			Q.clear();
		}
	}
	void SkeletalSegmentation::color_split_vertex(const CharVec& labels) {
		int max_it = COLOR_SMOOTH_ITER_TIMES;
		int vnum = (int)_mesh->v.size();
		int tnum = (int)_mesh->t.size();
		const int type_num = 10;
		Color colors[] = { _mesh->WHITE, _mesh->MYCOLOR[5], _mesh->MYCOLOR[1], _mesh->MYCOLOR[7], _mesh->MYCOLOR[3], _mesh->MYCOLOR[0],
			_mesh->BLUE, _mesh->MYCOLOR[10], _mesh->MYCOLOR[2], _mesh->MYCOLOR[4], _mesh->WHITE };
		std::vector<Color> vcolor(vnum, _mesh->BLACK);
		for (int i = 0; i < vnum; ++i) {
			int fn = _mesh->vtn[i]; unsigned int sum[3] = { 0 };
			for (int j = 0; j < fn; ++j) {
				int t = _mesh->vt(i, j);
				const Color& c = colors[labels[t]];
				sum[0] += c[0]; sum[1] += c[1]; sum[2] += c[2];
			}
			vcolor[i][0] = (unsigned char)(sum[0] / fn);
			vcolor[i][1] = (unsigned char)(sum[1] / fn);
			vcolor[i][2] = (unsigned char)(sum[2] / fn);
		}
		std::vector<Color> vcolor_switch = vcolor;
		std::vector<Color>* latest = &vcolor;
		std::vector<Color>* old = &vcolor_switch;
		for (int it = 0; it < max_it; ++it) {
			if (latest == &vcolor) { latest = &vcolor_switch; old = &vcolor; }
			else { latest = &vcolor; old = &vcolor_switch; }
			for (int i = 0; i < vnum; ++i) {
				int vn = _mesh->vvn[i];
				unsigned int sum[3] = { 0 };
				for (int j = 0; j < vn; ++j) {
					int n = _mesh->vv(i, j);
					const Color& c = (*old)[n];
					sum[0] += c[0]; sum[1] += c[1]; sum[2] += c[2];
				}
				(*latest)[i][0] = (unsigned char)(sum[0] / vn);
				(*latest)[i][1] = (unsigned char)(sum[1] / vn);
				(*latest)[i][2] = (unsigned char)(sum[2] / vn);
			}
		}
		for (int i = 0; i < vnum; ++i) {
			_mesh->color_vertex(i, (*latest)[i]);
		}
	}
	void SkeletalSegmentation::refine_arm_split_by_slice(const Int2DVec& border, Plane plane[], Int3DVec skerings) {
		// center
		std::vector<Vertex> cens;
		for (int i = 0; i < 5; i++) {
			Vertex cen = plane[i].p;
			cens.push_back(cen);
		}
		// find lower arm point
		int armpt[2] = { 0 }; Vertex legpt[2];
		for (int i = 1; i < 3; i++) {
			int si = (cens[i] - cens[3]).square_length() < (cens[i] - cens[4]).square_length() ? 3 : 4;
			IntVec& arm_ring = skerings[i][border[i][0]]; IntVec& leg_ring = skerings[si][border[si][0]];
			int curr = leg_ring[0]; float mindist = std::numeric_limits<float>::max();
			for (int j = 0; j < (int)leg_ring.size(); j++) {
				int idx = leg_ring[j];
				for (int k = 0; k < (int)arm_ring.size(); k++) {
					float dist = triangle_distance_s(arm_ring[k], idx);
					if (dist < mindist) {
						mindist = dist; curr = idx;
					}
				}
			}
			legpt[i - 1] = triangle_center(curr); int dest = 0; Vertex vtx;
			while (1) {
				float mindist = std::numeric_limits<float>::max(); int minidx = 0;
				for (int j = 0; j < 3; j++) {
					int n = _mesh->tt(curr, j);
					if (n >= 0) {
						for (int k = 0; k < (int)arm_ring.size(); k++) {
							float dist = triangle_distance_s(arm_ring[k], n);
							if (dist < mindist) {
								mindist = dist; minidx = n;
							}
						}
					}
				}
				if (mindist < (float)0.1) {
					dest = minidx; vtx = triangle_center(curr); break;
				}
				curr = minidx;
			}
			mindist = std::numeric_limits<float>::max();
			for (int j = 0; j < 3; j++) {
				int vidx = _mesh->t[curr].v[j];
				float dist = (_mesh->v[vidx] - vtx).square_length();
				if (dist < mindist) {
					mindist = dist; armpt[i - 1] = vidx;
				}
			}
			_mesh->color_vertex(armpt[i - 1], _preset_color[0]);
		}
		// rotate plane
		for (int i = 1; i < 3; i++) {
			Vertex armv = _mesh->v[armpt[i - 1]];
			Vertex armv2 = _mesh->v[armpt[2 - i]];
			//float arm2head[3] = { plane[0].p[0] - armv.x, plane[0].p[1] - armv.y, plane[0].p[2] - armv.z };
			Vector arm2head = armv2 - armv; arm2head.normalize();
			Vertex p0 = armv + plane[i].n;
			Vertex p1 = armv + arm2head;
			Vector ray = p1 - p0; ray.normalize();
			float t = (armv - p0).dot(plane[0].n);
			t /= (ray.dot(plane[0].n));
			Plane cut; cut.p = armv;
			cut.n = ray*t + p0 - armv; cut.n.normalize();
			IntVec cut_ret;
			search_loop(cut, armpt[i - 1], cut_ret);
			int maxidx = 0; float maxdist = std::numeric_limits<float>::min();
			for (int j = 0; j < (int)cut_ret.size(); j++) {
				for (int k = 0; k < 3; k++) {
					int fv = _mesh->t[cut_ret[j]].v[k];
					float dist = (_mesh->v[fv] - legpt[i - 1]).square_length();
					if (dist > maxdist) {
						if (point_plane_distance(cut, _mesh->v[fv]) > 0) {
							maxidx = fv; maxdist = dist;
						}
					}
				}
			}
			//_mesh->color_vertex(maxidx, _preset_color[0]);
			//connect_two_vertex(cut, maxidx, armpt[i - 1]);
		}
	}
	void SkeletalSegmentation::split_body_by_contour() {

		int size = (int)_slices_vtx[PartType::p_body].size();
		int interval = COS_INTERVAL * 3;
		_body_border[0] = 0;
		_body_border[1] = size - 1;

		// find body cut
		const Vertex ap[2] = { _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]] };
		Plane projecter; projecter.p = (ap[0] + ap[1]) / 2; projecter.n = (ap[0] - ap[1]).normalized();
		int half = (int)(_skeleton[PartType::p_body].size() / 2);
		Plane divider = _planes[PartType::p_body][half];
		divider.n = (_planes[PartType::p_body][half].n.cross(projecter.n)).normalized();

		Vertex cen[2] = { Vertex(0, 0, 0), Vertex(0, 0, 0) };
		auto slice = _slices_vtx[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_arm]];
		for (auto it = slice.begin(); it != slice.end(); ++it) cen[0] += *it;
		slice = _slices_vtx[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_arm]];
		for (auto it = slice.begin(); it != slice.end(); ++it) cen[1] += *it;
		cen[0].x /= (float)size; cen[0].y /= (float)size; cen[0].z /= (float)size;
		cen[1].x /= (float)size; cen[1].y /= (float)size; cen[1].z /= (float)size;
		Vertex ske = _skeleton[PartType::p_body][_skeleton[PartType::p_body].size() - 1];
		divider = create_plane_cross(ske, ap[0] - ske, ap[1] - ske);

		VtxVec min_vec(size), max_vec(size);
		for (int i = 0; i < size; ++i) {
			const auto& slice = _slices_vtx[PartType::p_body][i];
			if (slice.size() == 0) continue;
			float min_dist = DECIMAL_MAX, max_dist = DECIMAL_MIN;
			Vertex min_vtx, max_vtx;
			for (auto it = slice.begin(); it != slice.end(); ++it) {
				Vertex projected = project_to_plane(*it, projecter);
				float dist = point_plane_distance(divider, projected);
				if (dist > max_dist) {
					max_dist = dist; max_vtx = projected;
				}
				if (dist < min_dist) {
					min_dist = dist; min_vtx = projected;
				}
			}
			min_vec[i] = min_vtx;
			max_vec[i] = max_vtx;
		}
		float up_dist_sum_max = 0, up_dist_sum_min = 0;
		for (int i = SLICE_INTERVAL; i < half; ++i) {
			up_dist_sum_max += abs(point_plane_distance(divider, max_vec[SLICE_INTERVAL]));
			up_dist_sum_min += abs(point_plane_distance(divider, min_vec[SLICE_INTERVAL]));
		}
		float down_dist_sum_max = 0, down_dist_sum_min = 0;
		for (int i = half; i < size - SLICE_INTERVAL; ++i) {
			down_dist_sum_max += abs(point_plane_distance(divider, max_vec[SLICE_INTERVAL]));
			down_dist_sum_min += abs(point_plane_distance(divider, min_vec[SLICE_INTERVAL]));
		}
		Plane mid = _planes[PartType::p_body][half];
		VtxVec* front_vec, *back_vec;
		if (up_dist_sum_max > up_dist_sum_min) {
			front_vec = &max_vec; back_vec = &min_vec;
		}
		else {
			front_vec = &min_vec; back_vec = &max_vec;
		}
		if (max_vec[half].z < min_vec[half].z) {
			front_vec = &max_vec; back_vec = &min_vec;
		}
		else {
			front_vec = &min_vec; back_vec = &max_vec;
		}
		if (point_plane_distance(mid, (*front_vec)[SLICE_INTERVAL]) > 0)
			mid.n *= -1;
		// sort front
		//// sort
		//for (int i = 0; i < size; ++i) {
		//	dist_vec[i] = point_plane_distance(mid, (*front_vec)[i]);
		//}
		//for (int i = 1; i < size; ++i) {
		//	for (int j = i; j - 1 >= 0 && dist_vec[j] < dist_vec[j - 1]; --j) {
		//		float t = dist_vec[j]; dist_vec[j] = dist_vec[j - 1]; dist_vec[j - 1] = t;
		//		Vertex v = (*front_vec)[j]; (*front_vec)[j] = (*front_vec)[j - 1]; (*front_vec)[j - 1] = v;
		//	}
		//}
		FltVec dist_vec_front(size);
		float diff_front_avg = 0;
		for (int i = 0; i < size; ++i) {
			dist_vec_front[i] = abs(point_plane_distance(divider, (*front_vec)[i]));
			if (i >= interval + SLICE_INTERVAL && i < half) {
				diff_front_avg += abs(dist_vec_front[i] - dist_vec_front[i - interval]);

			}
		}
		diff_front_avg /= (half - SLICE_INTERVAL - interval);
		// sort back
		FltVec dist_vec_back(size);
		float diff_back_avg = 0;
		for (int i = 0; i < size; ++i) {
			dist_vec_back[i] = abs(point_plane_distance(divider, (*back_vec)[i]));
			if (i >= half && i < size - SLICE_INTERVAL - interval) {
				diff_back_avg += abs(dist_vec_back[i] - dist_vec_back[i - interval]);
			}
		}
		diff_back_avg /= (size - SLICE_INTERVAL - interval - half);
		//
		FltVec dist_vec(size);
		float diff_avg = 0;
		for (int i = 0; i < size; ++i) {
			dist_vec[i] = dist_vec_back[i] + dist_vec_front[i];
			if (i >= interval + SLICE_INTERVAL && i < size - SLICE_INTERVAL - interval) {
				diff_avg += abs(dist_vec[i] - dist_vec[i - interval]);
			}
		}
		diff_avg /= (size - 2 * SLICE_INTERVAL - 2 * interval);

		// find chest cut
		FltVec cos_front(size, -1);
		int chest = -1;
		{
			float maxcosine = -1;
			std::vector<float> cosines(size, -1);
			const float adder = (float)(interval*interval*diff_front_avg*diff_front_avg);
			for (int i = std::max(_body_border[0] + 2, interval + SLICE_INTERVAL); i < half; ++i) {
				float y0_y1 = (float)(dist_vec_front[i - interval] - dist_vec_front[i]);
				float y2_y1 = (float)(dist_vec_front[i + interval] - dist_vec_front[i]);
				if (y2_y1 > 0) y2_y1 = -y2_y1;
				float len0 = adder + y0_y1*y0_y1; len0 = sqrt(len0);
				float len1 = adder + y2_y1*y2_y1; len1 = sqrt(len1);
				float cosine = (y0_y1*y2_y1 - adder) / (len0*len1);
				if (y0_y1 < 0 || y0_y1 < -y2_y1 || cosine > 0) cosine = -1;
				cosines[i] = cosine;
				if (cosine > maxcosine) {
					maxcosine = cosine; chest = i;
				}
				cos_front[i] = cosine;
			}
		}
		// hip max
		int hip_max = -1;
		{
			float maxdist = DECIMAL_MIN;
			for (int i = half; i < std::min(_body_border[1] - 2, size - SLICE_INTERVAL - interval); ++i) {
				if (dist_vec_back[i] > maxdist) {
					maxdist = dist_vec_back[i]; hip_max = i;
				}
			}
		}
		// find hip cut
		FltVec cos_back(size, -1);
		int hip = -1;
		{
			float maxcosine = -1;
			std::vector<float> cosines(size, -1);
			const float adder = (float)(interval*interval*diff_back_avg*diff_back_avg);
			for (int i = half; i < std::min(_body_border[1] - 2, hip_max); ++i) {
				float y0_y1 = (float)(dist_vec_back[i - interval] - dist_vec_back[i]);
				if (y0_y1 > 0) y0_y1 = -y0_y1;
				float y2_y1 = (float)(dist_vec_back[i + interval] - dist_vec_back[i]);
				float len0 = adder + y0_y1*y0_y1; len0 = sqrt(len0);
				float len1 = adder + y2_y1*y2_y1; len1 = sqrt(len1);
				float cosine = (y0_y1*y2_y1 - adder) / (len0*len1);
				if (y2_y1 < 0 || y2_y1 < -y0_y1 || cosine > 0) cosine = -1;
				cosines[i] = cosine;
				if (cosine > maxcosine) {
					maxcosine = cosine; hip = i;
				}
				cos_back[i] = cosine;
			}
		}


		Plane cutter = divider; cutter.n = (divider.n.cross(projecter.n)).normalized();
		/*for (int i = 1; i < front_vec->size(); ++i) {
			for (int j = i; j > 0; --j) {
				float d = point_plane_distance(cutter, (*front_vec)[j - 1]);
				float d2 = point_plane_distance(cutter, (*front_vec)[j]);
				if (d > d2) { auto t = (*front_vec)[j - 1]; (*front_vec)[j - 1] = (*front_vec)[j]; (*front_vec)[j] = t; }
			}
		}
		for (int i = 1; i < back_vec->size(); ++i) {
			for (int j = i; j > 0; --j) {
				float d = point_plane_distance(cutter, (*back_vec)[j - 1]);
				float d2 = point_plane_distance(cutter, (*back_vec)[j]);
				if (d > d2) { auto t = (*back_vec)[j - 1]; (*back_vec)[j - 1] = (*back_vec)[j]; (*back_vec)[j] = t; }
			}
		}*/
		FltVec h_front;
		FltVec dev_front;
		for (int i = 0; i < front_vec->size() - 1; ++i) {
			float x0 = point_plane_distance(cutter, (*front_vec)[i]);
			float x1 = point_plane_distance(cutter, (*front_vec)[i + 1]);
			float y0 = point_plane_distance(divider, (*front_vec)[i]);
			float y1 = point_plane_distance(divider, (*front_vec)[i + 1]);
			dev_front.push_back((y1 - y0) / (x1 - x0));
			h_front.push_back(y0);
		}
		FltVec h_back;
		FltVec dev_back;
		for (int i = 0; i < front_vec->size() - 1; ++i) {
			float x0 = point_plane_distance(cutter, (*back_vec)[i]);
			float x1 = point_plane_distance(cutter, (*back_vec)[i + 1]);
			float y0 = point_plane_distance(divider, (*back_vec)[i]);
			float y1 = point_plane_distance(divider, (*back_vec)[i + 1]);
			dev_back.push_back((y1 - y0) / (x1 - x0));
			h_back.push_back(y0);
		}
		FltVec cvt_front;
		for (int i = 0; i < dev_front.size() - 1; ++i) {
			float x0 = point_plane_distance(cutter, (*front_vec)[i]);
			float x1 = point_plane_distance(cutter, (*front_vec)[i + 1]);
			float y0 = dev_front[i];
			float y1 = dev_front[i + 1];
			float dev2 = abs((y1 - y0) / (x1 - x0));
			float t = dev2 / std::pow(1 + y0*y0, 1.5f);
			cvt_front.push_back(t);
		}
		FltVec cvt_back;
		for (int i = 0; i < dev_back.size() - 1; ++i) {
			float x0 = point_plane_distance(cutter, (*back_vec)[i]);
			float x1 = point_plane_distance(cutter, (*back_vec)[i + 1]);
			float y0 = dev_back[i];
			float y1 = dev_back[i + 1];
			float dev2 = abs((y1 - y0) / (x1 - x0));
			float t = dev2 / std::pow(1 + y0*y0, 1.5f);
			cvt_back.push_back(t);
		}

		// set
		_cuts[PartType::p_body].resize(BodyCut::c_body_cut_amount);
		_cuts[PartType::p_body][BodyCut::c_up] = chest;
		_cuts[PartType::p_body][BodyCut::c_down] = hip + (hip_max - hip) / 4;
		_cuts[PartType::p_body][BodyCut::c_waist] = (_cuts[PartType::p_body][BodyCut::c_up] + _cuts[PartType::p_body][BodyCut::c_down]) / 2;
		
		if (MANUAL_BODY_SEG) {
			_cuts[PartType::p_body][BodyCut::c_up] = MANUAL_BODY_CUT_UP;
			_cuts[PartType::p_body][BodyCut::c_down] = MANUAL_BODY_CUT_DOWN;
			_cuts[PartType::p_body][BodyCut::c_waist] = (MANUAL_BODY_CUT_UP + MANUAL_BODY_CUT_DOWN) / 2;
		}

		int body_cut_up = _cuts[PartType::p_body][BodyCut::c_up];
		int body_cut_mid = _cuts[PartType::p_body][BodyCut::c_waist];
		int body_cut_down = _cuts[PartType::p_body][BodyCut::c_down];

		if (PAINT_SEG_OVER_MESH) {
			paint_ring_vertices(_planes[PartType::p_body][body_cut_up].n, _slices_vtx[PartType::p_body][body_cut_up], _mesh->MYCOLOR[21]);
			//paint_ring_vertices(_planes[PartType::p_body][body_cut_mid].n, _slices_vtx[PartType::p_body][body_cut_mid], _mesh->MYCOLOR[22]);
			paint_ring_vertices(_planes[PartType::p_body][body_cut_down].n, _slices_vtx[PartType::p_body][body_cut_down], _mesh->MYCOLOR[28]);
		}
		
		// write scalars to txt
		if (PRINT_TO_FILE) {
			std::string fname = ".\\parm_cut"; fname += ".txt";
			std::fstream fs(fname.c_str(), std::ios::out);
			const char* blank = "\t\t";
			//for (int i = 0; i < size; i++)
			//	fs << i << blank << dist_vec_front[i] << blank << dist_vec_back[i] << blank << dist_vec[i] << std::endl;
			for (int i = 0; i < cvt_front.size(); i++)
				fs << i << blank << dist_vec_front[i] << blank << dist_vec_back[i] << blank << cos_front[i] << blank << cos_back[i] << std::endl;
			fs.close();
		}

		// info
		std::cout << "body info, chest : " << _cuts[PartType::p_body][BodyCut::c_up] << ", waist : " << _cuts[PartType::p_body][BodyCut::c_waist] << ", hip : " << hip << " " << hip_max << std::endl;
	}
	void SkeletalSegmentation::refine_split_by_slice() {

		int face_num = (int)_mesh->t.size();
		_part_labels.resize(face_num);
		for (auto it = _part_labels.begin(); it != _part_labels.end(); ++it) *it = -1;
		CharVec& labels = _part_labels;

		//// label limbs
		//int face_num = (int)_mesh->t.size();
		//CharVec labels(face_num, -1);
		//int seeds[5] = { 0 };
		//CharVec labels_detail(face_num, -1);
		//int idxs[6] = { 0, 1, 1, 2, 2, 3 };
		//for (int idx = 0; idx < 5; ++idx) {
		//	int cutpos = (idx == 0 ? 1 : 0);
		//	int label_value = (idx == 0 ? 1 : idx * 10);
		//	// paint main cuts
		//	for (int i = 0; i < (int)_slices_face[idx][_cuts[idx][cutpos]].size(); ++i) {
		//		labels[_slices_face[idx][_cuts[idx][cutpos]][i]] = idxs[idx];
		//		labels_detail[_slices_face[idx][_cuts[idx][cutpos]][i]] = label_value;
		//	}
		//	// find end triangle
		//	ArrayQueue<int> Q(face_num);
		//	for (int i = 0; i < (int)_slices_face[idx].size(); ++i) {
		//		if (_slices_face[idx][i].size() > 0) {
		//			seeds[idx] = _slices_face[idx][i][0]; break;
		//		}
		//	}
		//	// paint part
		//	labels[seeds[idx]] = idxs[idx]; Q.push(seeds[idx]);
		//	labels_detail[seeds[idx]] = label_value;
		//	while (!Q.empty()) {
		//		int curr = Q.front(); Q.pop();
		//		for (int i = 0; i < 3; i++) {
		//			int n = _mesh->tt(curr, i);
		//			if (n >= 0 && labels[n] != idxs[idx]) {
		//				labels[n] = idxs[idx]; Q.push(n);
		//				labels_detail[n] = label_value;
		//			}
		//		}
		//	}
		//}
		//for (int idx = 0; idx < face_num; ++idx) {
		//	if (labels[idx] < 0) labels[idx] = 3;
		//	if (labels_detail[idx] < 0) labels_detail[idx] = 5 * 10;
		//}
		//// label others
		//for (int i = 0; i < 5; i++) {
		//	int start = (i == 0 ? 0 : 1);
		//	int end = (i == 0 ? 0 : 2);
		//	for (int j = start; j <= end; j++) {
		//		// arm to elbow to wrist / leg to knee to ankle
		//		int num = _cuts[i][j], mid = 0;
		//		for (int k = 0; k < num; k++) {
		//			if (_slices_face[i][k].size() > 0) {
		//				mid = _slices_face[i][k][0]; break;
		//			}
		//		}
		//		if (i <= 2 && i > 0) label_faces_BFS(mid, _slices_face[i][num], 5 + j, labels);
		//		else if (i >= 3 && i <= 4) label_faces_BFS(mid, _slices_face[i][num], 7 + j, labels);
		//		else label_faces_BFS(mid, _slices_face[i][_cuts[i][0]], 10, labels);
		//		label_faces_BFS(mid, _slices_face[i][num], 10 * i + j, labels_detail);
		//	}
		//}
		//for (auto it = labels.begin(); it != labels.end(); ++it) {
		//	if (*it == 0) *it = 10;
		//	else if (*it == 10) *it = 0;
		//}

		// label limbs
		int seeds[PartType::p_part_amount - 1] = { 0 };
		for (int i = 0; i < PartType::p_part_amount - 1; ++i) {
			for (int j = 0; j < _slices_face[i].size(); ++j) {
				if (_slices_face[i][j].size() > 0) {
					seeds[i] = _slices_face[i][j][0]; break;
				}
			}
		}
		for (int i = _cuts[PartType::p_head].size() - 1; i >= 0; --i) {
			int cut = _cuts[PartType::p_head][i];
			label_faces_BFS(_slices_face[PartType::p_head][cut], _planes[PartType::p_head][cut], PartType::p_head * 10 + i, labels, -2);
		}
		for (int i = PartType::p_head + 1; i < PartType::p_part_amount - 1; ++i) {
			for (int j = 0; j < _cuts[i].size(); ++j) {
				int cut = _cuts[i][j];
				label_faces_BFS(_slices_face[i][cut], _planes[i][cut], i * 10 + j, labels, -2);
			}
		}

		// body border
		int body_len = _slices_face[PartType::p_body].size();
		for (int i = 0; i < body_len / 2; ++i) {
			auto& slice = _slices_face[PartType::p_body][i];
			for (auto it = slice.begin(); it != slice.end(); ++it) {
				if (labels[*it] >= DetailPartType::arm0 && labels[*it] < DetailPartType::thigh0) {
					_body_border[0] = i; break;
				}
			}
		}
		for (int i = body_len - 1; i >= body_len / 2; --i) {
			auto& slice = _slices_face[PartType::p_body][i];
			for (auto it = slice.begin(); it != slice.end(); ++it) {
				if (labels[*it] >= DetailPartType::thigh0 && labels[*it] <= DetailPartType::foot1) {
					_body_border[1] = i; break;
				}
			}
		}

		// landmark
		find_crotch();
		find_armpit();

		// body segmentation
		split_body_by_contour();
		//split_body_by_slice();

		// body label
		int bodyidx = PartType::p_body;
		int cut = _cuts[bodyidx][BodyCut::c_down];
		label_faces_BFS(_slices_face[bodyidx][cut], _planes[bodyidx][cut], DetailPartType::midbody, labels, -1);
		cut = _cuts[bodyidx][BodyCut::c_up];
		label_faces_BFS(_slices_face[bodyidx][cut], _planes[bodyidx][cut], DetailPartType::upbody, labels, DetailPartType::midbody);
		for (int i = 0; i < labels.size(); ++i) {
			if (labels[i] < 0) labels[i] = DetailPartType::downbody;
		}
		
		//filter(labels);
		//refine_cutting(labels);
		//color_split_vertex(labels);

		//// Color face
		//if (PAINT_MESH_USING_LABEL)
		//{
		//	Color tcolors[256];
		//	tcolors[0] = _mesh->WHITE; tcolors[1] = _mesh->RED;
		//	tcolors[10] = _mesh->WHITE; tcolors[11] = _mesh->BLUE; tcolors[12] = _mesh->WHITE;
		//	tcolors[20] = _mesh->WHITE; tcolors[21] = _mesh->BLUE; tcolors[22] = _mesh->WHITE;
		//	tcolors[30] = _mesh->WHITE; tcolors[31] = _mesh->GREEN; tcolors[32] = _mesh->WHITE;
		//	tcolors[40] = _mesh->WHITE; tcolors[41] = _mesh->GREEN; tcolors[42] = _mesh->WHITE;
		//	tcolors[50] = _mesh->CYAN; tcolors[51] = _mesh->WHITE; tcolors[52] = _mesh->YELLOW;
		//	for (int idx = 0; idx < (int)_mesh->t.size(); ++idx) {
		//		_mesh->color_face(idx, tcolors[labels[idx]]);
		//	}
		//}
		if (PAINT_MESH_USING_LABEL)
		{
			Color tcolors[256];
			tcolors[0] = _mesh->MYCOLOR[11]; tcolors[1] = _mesh->MYCOLOR[1];
			tcolors[10] = _mesh->MYCOLOR[3]; tcolors[11] = _mesh->MYCOLOR[0]; tcolors[12] = _mesh->MYCOLOR[2];
			tcolors[20] = _mesh->MYCOLOR[3]; tcolors[21] = _mesh->MYCOLOR[0]; tcolors[22] = _mesh->MYCOLOR[2];
			tcolors[30] = _mesh->MYCOLOR[7]; tcolors[31] = _mesh->MYCOLOR[2]; tcolors[32] = _mesh->MYCOLOR[9];
			tcolors[40] = _mesh->MYCOLOR[7]; tcolors[41] = _mesh->MYCOLOR[2]; tcolors[42] = _mesh->MYCOLOR[9];
			tcolors[50] = _mesh->MYCOLOR[2]; tcolors[51] = _mesh->MYCOLOR[7]; tcolors[52] = _mesh->MYCOLOR[5];

			tcolors[0] = _mesh->MYCOLOR[11]; tcolors[1] = _mesh->RED;
			tcolors[10] = _mesh->YELLOW; tcolors[11] = _mesh->CYAN; tcolors[12] = _mesh->GREEN;
			tcolors[20] = _mesh->YELLOW; tcolors[21] = _mesh->CYAN; tcolors[22] = _mesh->GREEN;
			tcolors[30] = _mesh->CYAN; tcolors[31] = _mesh->GREEN; tcolors[32] = _mesh->BLUE;
			tcolors[40] = _mesh->CYAN; tcolors[41] = _mesh->GREEN; tcolors[42] = _mesh->BLUE;
			tcolors[50] = _mesh->MYCOLOR[11]; tcolors[51] = _mesh->MYCOLOR[11]; tcolors[52] = _mesh->MYCOLOR[11];

			tcolors[0] = _mesh->MYCOLOR[11]; tcolors[1] = _mesh->RED;
			tcolors[10] = _mesh->MYCOLOR[13]; tcolors[11] = _mesh->BLUE; tcolors[12] = _mesh->MYCOLOR[12];
			tcolors[20] = _mesh->MYCOLOR[13]; tcolors[21] = _mesh->BLUE; tcolors[22] = _mesh->MYCOLOR[12];
			tcolors[30] = _mesh->GREEN; tcolors[31] = _mesh->MYCOLOR[16]; tcolors[32] = _mesh->MYCOLOR[17];
			tcolors[40] = _mesh->GREEN; tcolors[41] = _mesh->MYCOLOR[16]; tcolors[42] = _mesh->MYCOLOR[17];
			tcolors[50] = _mesh->CYAN; tcolors[51] = _mesh->MYCOLOR[15]; tcolors[52] = _mesh->YELLOW;

			for (int idx = 0; idx < (int)_mesh->t.size(); ++idx) {
				_mesh->color_face(idx, tcolors[labels[idx]]);
			}

			if (SMOOTH_SEG_BY_CUT_TRIANGLES) {

				int colori[16][2] = { { 0, 1 }, { 1, 50 }, { 10, 50 }, { 11, 10 }, { 12, 11 }, { 20, 50 }, { 21, 20 }, { 22, 21 },
				{ 30, 52 }, { 31, 30 }, { 32, 31 }, { 40, 52 }, { 41, 40 }, { 42, 41 }, { 50, 51 }, { 51, 52 } };

				int cnt = 0;
				for (int idx = 0; idx < PartType::p_part_amount; ++idx) {
					if (idx == PartType::p_body) {
						for (int i = 1; i <= 3; i += 2) {
							int cut = _cuts[idx][i];
							cut_ring_by_plane(_slices_face[idx][cut], _planes[idx][cut], tcolors[colori[cnt][0]], tcolors[colori[cnt][1]]);
							++cnt;
						}
					}
					else {
						for (int i = 0; i < (int)_cuts[idx].size(); ++i) {
							int cut = _cuts[idx][i];
							cut_ring_by_plane(_slices_face[idx][cut], _planes[idx][cut], tcolors[colori[cnt][0]], tcolors[colori[cnt][1]]);
							++cnt;
						}
					}
				}
			}

			//for (int idx = 0; idx < (int)_mesh->v.size(); ++idx) {
			//	Vertex& vtx = _mesh->v[idx];
			//	int cnt = 0; int rgb[3] = { 0 };
			//	auto lbl = labels[_mesh->vt(idx, 0)];
			//	vtx.r = tcolors[lbl][0]; vtx.g = tcolors[lbl][1]; vtx.b = tcolors[lbl][2];
			//	for (int i = 0; i < _mesh->vtn[idx]; ++i) {
			//		int t = _mesh->vt(idx, i);
			//		if (t >= 0) {
			//			if (labels[t] != lbl) {
			//				vtx.r = vtx.g = vtx.b = 255; break;
			//			}
			//			//auto& tri = _mesh->t[t];
			//			//++cnt; rgb[0] += tri.r; rgb[1] += tri.g; rgb[2] += tri.b;
			//		}
			//	}
			//	//if (cnt > 0) {
			//	//	rgb[0] /= cnt; rgb[1] /= cnt; rgb[2] /= cnt;
			//	//}
			//	//vtx.r = rgb[0]; vtx.g = rgb[1]; vtx.b = rgb[2];
			//}

			//// smooth
			//for (int it = 0; it < 3; ++it) {
			//	for (int idx = 0; idx < (int)_mesh->v.size(); ++idx) {
			//		Vertex& vtx = _mesh->v[idx];
			//		int cnt = 0; int rgb[3] = { 0 };
			//		for (int i = 0; i < _mesh->vvn[idx]; ++i) {
			//			int v = _mesh->vv(idx, i);
			//			if (v >= 0) {
			//				Vertex& nv = _mesh->v[v];
			//				++cnt; rgb[0] += nv.r; rgb[1] += nv.g; rgb[2] += nv.b;
			//			}
			//		}
			//		if (cnt > 0) {
			//			rgb[0] /= cnt; rgb[1] /= cnt; rgb[2] /= cnt;
			//		}
			//		vtx.r = rgb[0]; vtx.g = rgb[1]; vtx.b = rgb[2];
			//	}
			//}

			//for (int idx = 0; idx < (int)_mesh->v.size(); ++idx) {
			//	Vertex& vtx = _mesh->v[idx];
			//	Color vcolor; vcolor[0] = vtx.r; vcolor[1] = vtx.g; vcolor[2] = vtx.b;
			//	_mesh->color_vertex(idx, vcolor);
			//}
		}
		
		//// Color face
		//Color colors[] = { _mesh->WHITE, _mesh->MYCOLOR[5], _mesh->MYCOLOR[1], _mesh->MYCOLOR[7], _mesh->MYCOLOR[3], _mesh->MYCOLOR[0],
		//	_mesh->MYCOLOR[6], _mesh->MYCOLOR[5], _mesh->MYCOLOR[2], _mesh->MYCOLOR[4] };
		//for (int part_idx = 5; part_idx >= 0; --part_idx) {
		//	for (int idx = 0; idx < (int)_mesh->t.size(); ++idx) {
		//		if (labels[idx] == part_idx)
		//			_mesh->color_face(idx, colors[part_idx]);
		//	}
		//}
		//for (int idx = 0; idx < (int)_mesh->t.size(); ++idx) {
		//	if (labels[idx] == 6)
		//		_mesh->color_face(idx, colors[6]);
		//}
	}
	void SkeletalSegmentation::cut_ring_by_plane(const IntVec& ring, const Plane& plane, Color upcolor, Color dncolor) {

		for (int i = 0; i < (int)ring.size(); ++i) {
			int t = ring[i];
			auto& tri = _mesh->t[t];
			for (int j = 0; j < 3; ++j) {
				int v0 = tri.v[j]; Vertex& vtx = _mesh->v[v0]; RGBA& vc = _mesh->vcolor[v0];
				if (point_plane_distance(plane, vtx) > 0) {
					vc.r = upcolor[0]; vc.g = upcolor[1]; vc.b = upcolor[2];
				}
				else {
					vc.r = dncolor[0]; vc.g = dncolor[1]; vc.b = dncolor[2];
				}
			}
			// cut triangles
			int cnt = 0, vs[4] = { 0 };
			Vertex intersect[2], mid;
			for (int j = 0; j < 3; ++j) {
				int v0 = tri.v[j];
				int v1 = tri.v[(j + 1) % 3];
				if (point_plane_distance(plane, _mesh->v[v0]) * point_plane_distance(plane, _mesh->v[v1]) < 0) {
					vs[2 * cnt] = v0; vs[2 * cnt + 1] = v1;
					intersect[cnt++] = plane_edge_intersection(plane, v0, v1);
				}
				else {
					mid = (_mesh->v[v0] + _mesh->v[v1]) / 2;
				}
			}
			int vn = _mesh->v.size();
			if (vs[1] == vs[2]) {
				//auto vh0 = _mesh->add_vertex(intersect[0]);
				//auto vh1 = _mesh->add_vertex(intersect[1]);
				//auto vh2 = _mesh->add_vertex(mid);
				//_mesh->add_face(vh0, _mesh->vh[vs[1]], vh1);
				//_mesh->add_face(_mesh->vh[vs[0]], vh0, vh2);
				//_mesh->add_face(vh2, vh1, _mesh->vh[vs[3]]);

				_mesh->v.push_back(intersect[0]);
				_mesh->v.push_back(intersect[1]);
				_mesh->v.push_back(mid);
				_mesh->t.push_back(Triangle(vn, vs[1], vn + 1));
				_mesh->t.push_back(Triangle(vs[0], vn, vn + 2));
				_mesh->t.push_back(Triangle(vn + 2, vn + 1, vs[3]));
			}
			else if (vs[0] == vs[3]) {
				//auto vh0 = _mesh->add_vertex(intersect[1]);
				//auto vh1 = _mesh->add_vertex(intersect[0]);
				//auto vh2 = _mesh->add_vertex(mid);
				//_mesh->add_face(vh0, _mesh->vh[vs[0]], vh1);
				//_mesh->add_face(_mesh->vh[vs[2]], vh0, vh2);
				//_mesh->add_face(vh2, vh1, _mesh->vh[vs[1]]);

				_mesh->v.push_back(intersect[1]);
				_mesh->v.push_back(intersect[0]);
				_mesh->v.push_back(mid);
				_mesh->t.push_back(Triangle(vn, vs[0], vn + 1));
				_mesh->t.push_back(Triangle(vs[2], vn, vn + 2));
				_mesh->t.push_back(Triangle(vn + 2, vn + 1, vs[1]));
			}
			else {
				std::cout << "error in cut_ring_by_plane : unexpected situation" << std::endl;
			}
			tri.v[0] = vn; tri.v[1] = vn + 1; tri.v[2] = vn + 2;
			_mesh->vcolor.push_back({ upcolor });
			_mesh->vcolor.push_back({ upcolor });
			if (point_plane_distance(plane, mid) > 0) {
				_mesh->vcolor.push_back({ upcolor });
			}
			else {
				_mesh->vcolor.push_back({ dncolor });
			}
		}
	}
	// landmark
	void SkeletalSegmentation::find_landmarks() {

		CharVec& labels_detail = _part_labels;
		// hks
		std::vector<double>& hks = _hks;

		// find feature point candidate
		IntVec& feature_points = _hks_points;
		for (int i = 0; i < (int)_mesh->v.size(); ++i) {
			bool flag = true;
			for (int j = 0; j < _mesh->vvn[i]; ++j) {
				int n = _mesh->vv(i, j);
				if (n != i && hks[i] <= hks[n]) flag = false;
				//for (int k = 0; k < _mesh->vvn[n]; ++k) {
				//	int nn = _mesh->vv(n, k);
				//	if (nn != i && hks[i] <= hks[nn]) flag = false;
				//}
			}
			if (flag) {
				feature_points.push_back(i);
				//paint_ball(_mesh->v[i], BALL_RADIUS, BALL_CUTS, _mesh->WHITE);
				//_mesh->color_vertex(i, _mesh->PURPLE);
			}
		}

		// find toe and heel
		find_foot();

		// find landmarks of arm and leg
		find_limbs_landmarks();

		// find hip
		find_hip();

		auto closest_vertex_in_triangle = [this](int part, int cut, int face) {
			Plane& plane = this->_planes[part][cut];
			int vidx = this->_mesh->t[face].v[0]; int ret = vidx;
			float mindist = this->point_plane_distance(plane, this->_mesh->v[vidx]);
			for (int i = 1; i < 3; ++i) {
				vidx = this->_mesh->t[face].v[i];
				float dist = this->point_plane_distance(plane, this->_mesh->v[vidx]);
				if (mindist > dist) { mindist = dist; ret = vidx; }
			}
			return ret;
		};

		auto find_feature = [closest_vertex_in_triangle, this](int part, int cut, int size, int* dest, int* out) {
			IntVec& slice = _slices_face[part][cut];
			for (int i = 0; i < size; ++i) {
				float min_dist = DECIMAL_MAX; int vtx = -1, face = -1;
				for (auto it = slice.begin(); it != slice.end(); ++it) {
					//float dist = (_mesh->triangle_center(*it) - _mesh->v[dest[i]]).square_length();
					//if (dist < min_dist) { min_dist = dist; face = *it; }
					for (int j = 0; j < 3; ++j) {
						int v = _mesh->t[*it].v[j];
						float dist = this->geodesic_dist(v, dest[i]);
						//float dist = (_mesh->v[v] - _mesh->v[dest[i]]).square_length();
						if (dist < min_dist) { min_dist = dist; vtx = v; }
					}
				}
				//out[i] = closest_vertex_in_triangle(part, cut, face);
				out[i] = vtx;
			}
		};
		auto find_feature_near_far = [closest_vertex_in_triangle, this](int part, int cut, int dest, int out[]) {
			IntVec& slice = _slices_face[part][cut];
			float min_dist = DECIMAL_MAX; int face = -1;
			for (auto it = slice.begin(); it != slice.end(); ++it) {
				float dist = (_mesh->triangle_center(*it) - _mesh->v[dest]).square_length();
				if (dist < min_dist) { min_dist = dist; face = *it; }
			}
			out[0] = closest_vertex_in_triangle(part, cut, face);
			float max_dist = DECIMAL_MIN; face = -1;
			for (auto it = slice.begin(); it != slice.end(); ++it) {
				float dist = (_mesh->triangle_center(*it) - _mesh->v[dest]).square_length();
				if (dist > max_dist) { max_dist = dist; face = *it; }
			}
			out[1] = closest_vertex_in_triangle(part, cut, face);
		};
		auto find_feature_plane = [closest_vertex_in_triangle, this](int part, int cut, int* out) {
			const VtxVec& slice_vtx = _slices_vtx[part][cut];
			Vertex slice_cen(0, 0, 0);
			for (auto it = slice_vtx.begin(); it != slice_vtx.end(); ++it) slice_cen += *it;
			slice_cen.x /= (float)slice_vtx.size(); slice_cen.y /= (float)slice_vtx.size(); slice_cen.z /= (float)slice_vtx.size();
			const IntVec& slice = _slices_face[part][cut];
			Vertex cen[3] = { _skeleton[part][cut + SLICE_INTERVAL], _skeleton[part][cut], _skeleton[part][cut - SLICE_INTERVAL] };
			Vector va = cen[0] - cen[1], vb = cen[2] - cen[1], vc = va.cross(vb), vd = vc.cross(_planes[part][cut].n);
			vc.normalize(); vd.normalize();
			Plane xcut(slice_cen, vc), zcut(slice_cen, vd);
			for (int i = 0; i < 4; ++i) {
				Plane& plane0 = (i < 2 ? zcut : xcut), plane1 = (i < 2 ? xcut : zcut);
				for (auto it = slice.begin(); it != slice.end(); ++it) {
					Vertex tri_cen = this->_mesh->triangle_center(*it);
					int temp = this->face_cross_plane(*it, plane0);
					if (temp > 0 && temp < 3) {
						bool flag = (this->point_plane_distance(plane1, tri_cen) > 0);
						if (i % 2 == 0 && flag || i % 2 == 1 && !flag) {
							out[i] = _mesh->t[*it].v[0];
						}
					}
				}
				//this->_mesh->color_vertex(out[i], _mesh->BLUE);
			}
		};

		// LR FB
		auto find_feature_plane2 = [closest_vertex_in_triangle, this](int part, int cut, const Vertex& cen0, const Vertex& cen1, int* out) {
			const VtxVec& slice_vtx = _slices_vtx[part][cut];
			Vertex slice_cen(0, 0, 0);
			for (auto it = slice_vtx.begin(); it != slice_vtx.end(); ++it) slice_cen += *it;
			slice_cen.x /= (float)slice_vtx.size(); slice_cen.y /= (float)slice_vtx.size(); slice_cen.z /= (float)slice_vtx.size();
			const IntVec& slice = _slices_face[part][cut];
			Vertex cen = _skeleton[part][cut];
			Vector vc = cen0 - cen1, vd = vc.cross(_planes[part][cut].n);
			vc.normalize(); vd.normalize();
			Plane xcut(slice_cen, vc), zcut(slice_cen, vd);
			for (int i = 0; i < 4; ++i) {
				Plane& plane0 = (i < 2 ? zcut : xcut), plane1 = (i < 2 ? xcut : zcut);
				for (auto it = slice.begin(); it != slice.end(); ++it) {
					Vertex tri_cen = this->_mesh->triangle_center(*it);
					int temp = this->face_cross_plane(*it, plane0);
					if (temp > 0 && temp < 3) {
						bool flag = (this->point_plane_distance(plane1, tri_cen) > 0);
						if (i % 2 == 0 && flag || i % 2 == 1 && !flag) {
							out[i] = _mesh->t[*it].v[0];
						}
					}
				}
				//this->_mesh->color_vertex(out[i], _mesh->BLUE);
			}
		};

		const VtxVec leg_ring[2] = { _slices_vtx[PartType::p_leg0][_cuts[PartType::p_leg0][LegCut::c_leg]], _slices_vtx[PartType::p_leg1][_cuts[PartType::p_leg1][LegCut::c_leg]] };
		const VtxVec knee_ring[2] = { _slices_vtx[PartType::p_leg0][_cuts[PartType::p_leg0][LegCut::c_knee]], _slices_vtx[PartType::p_leg1][_cuts[PartType::p_leg1][LegCut::c_knee]] };
		const VtxVec ankle_ring[2] = { _slices_vtx[PartType::p_leg0][_cuts[PartType::p_leg0][LegCut::c_ankle]], _slices_vtx[PartType::p_leg1][_cuts[PartType::p_leg1][LegCut::c_ankle]] };
		const VtxVec elbow_ring[2] = { _slices_vtx[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_elbow]], _slices_vtx[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_elbow]] };
		const VtxVec wrist_ring[2] = { _slices_vtx[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_wrist]], _slices_vtx[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_wrist]] };
		const VtxVec& head_ring = _slices_vtx[PartType::p_head][_cuts[PartType::p_head][HeadCut::c_head]];
		const VtxVec& neck_ring = _slices_vtx[PartType::p_head][_cuts[PartType::p_head][HeadCut::c_neck]];
		const VtxVec upbody_ring = _slices_vtx[PartType::p_body][_cuts[PartType::p_body][BodyCut::c_up]];
		const VtxVec midbody_ring = _slices_vtx[PartType::p_body][_cuts[PartType::p_body][BodyCut::c_waist]];
		const VtxVec dnbody_ring = _slices_vtx[PartType::p_body][_cuts[PartType::p_body][BodyCut::c_down]];
		const VtxVec arm_ring[2] = { _slices_vtx[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_arm]],
			_slices_vtx[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_arm]] };

		// neck
		//find_feature(PartType::p_head, _cuts[PartType::p_head][1], 2, _landmarks.shoulder, _landmarks.neck);
		find_feature_plane2(PartType::p_head, _cuts[PartType::p_head][HeadCut::c_neck], _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]], _landmarks.neck);
		//paint_ball(_landmarks.neck[0], neck_ring, BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[0]);
		//paint_ball(_landmarks.neck[1], neck_ring, BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[0]);
		//paint_ball(_landmarks.neck[2], neck_ring, BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[0]);
		//paint_ball(_landmarks.neck[3], neck_ring, BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[0]);
		find_neck();

		// find shoulder
		find_shoulder();

		// chest
		int chest = _cuts[PartType::p_body][BodyCut::c_up];
		find_feature_plane2(PartType::p_body, chest, _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]], _landmarks.chest);
		//paint_ball(_landmarks.chest[2], upbody_ring, BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[1]);
		//paint_ball(_landmarks.chest[3], upbody_ring, BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[1]);
		find_chest();

		return;

		// shoulder
		if (REFINE_SHOULDER) {
			if (_landmarks.shoulder[0] >= 0) paint_ball(_landmarks.shoulder[0], arm_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[9]);
			if (_landmarks.shoulder[1] >= 0) paint_ball(_landmarks.shoulder[1], arm_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[9]);
		}

		// armpit
		paint_ball(_landmarks.armpit[0], arm_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[9]);
		paint_ball(_landmarks.armpit[1], arm_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[9]);
		// crotch
		paint_ball(_landmarks.crotch, leg_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[5]);
		// waist
		int waist = _cuts[PartType::p_body][BodyCut::c_waist];
		//find_feature(PartType::p_body, waist, 2, _landmarks.armpit, _landmarks.waist);
		find_feature_plane2(PartType::p_body, waist, _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]], _landmarks.waist);
		paint_ball(_landmarks.waist[0], midbody_ring, BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[2]);
		paint_ball(_landmarks.waist[1], midbody_ring, BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[2]);
		// hip
		int hip = _cuts[PartType::p_body][BodyCut::c_down];
		//find_feature(PartType::p_body, hip, 2, _landmarks.armpit, _landmarks.hip);
		find_feature_plane2(PartType::p_body, hip, _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]], _landmarks.hip);
		paint_ball(_landmarks.hip[0], dnbody_ring, BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[8]);
		paint_ball(_landmarks.hip[1], dnbody_ring, BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[8]);
		// back high point
		//find_feature_plane2(PartType::p_body, (waist + navel_cut) / 2, _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]], _landmarks.waist_high_back);
		//paint_ball(_mesh->v[_landmarks.waist_high_back[2]], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[2]);
		// back low point
		//find_feature_plane2(PartType::p_body, (hip + navel_cut) / 2, _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]], _landmarks.waist_low_back);
		//paint_ball(_mesh->v[_landmarks.waist_low_back[2]], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[8]);
		//paint_ball(_mesh->v[_landmarks.waist_low_back[3]], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[8]);
		// leg
		//find_feature_plane(PartType::p_leg0, _cuts[PartType::p_leg0][0], _landmarks.leg0);
		//find_feature_plane(PartType::p_leg1, _cuts[PartType::p_leg1][0], _landmarks.leg1);
		int leg0[2] = { 0 }, leg1[2] = { 0 };
		find_feature(PartType::p_leg0, _cuts[PartType::p_leg0][0], 2, _landmarks.hip, leg0);
		find_feature(PartType::p_leg1, _cuts[PartType::p_leg1][0], 2, _landmarks.hip, leg1);
		_landmarks.leg[0] = (point_distance(leg0[0], _landmarks.crotch) > point_distance(leg0[1], _landmarks.crotch) ? leg0[0] : leg0[1]);
		_landmarks.leg[1] = (point_distance(leg1[0], _landmarks.crotch) > point_distance(leg1[1], _landmarks.crotch) ? leg1[0] : leg1[1]);
		paint_ball(_landmarks.leg[0], leg_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[5]);
		paint_ball(_landmarks.leg[1], leg_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[5]);
		// knee
		//int knee_dest0[2] = { _landmarks.leg[0], _landmarks.crotch };
		//int knee_dest1[2] = { _landmarks.leg[1], _landmarks.crotch };
		//find_feature(PartType::p_leg0, _cuts[PartType::p_leg0][1], 2, knee_dest0, _landmarks.knee0);
		//find_feature(PartType::p_leg1, _cuts[PartType::p_leg1][1], 2, knee_dest1, _landmarks.knee1);
		//find_feature(PartType::p_leg0, _cuts[PartType::p_leg0][1], 1, &_landmarks.heel[0], &_landmarks.knee0[2]);
		//find_feature(PartType::p_leg1, _cuts[PartType::p_leg1][1], 1, &_landmarks.heel[1], &_landmarks.knee1[2]);
		find_feature_plane(PartType::p_leg0, _cuts[PartType::p_leg0][1], _landmarks.knee0);
		find_feature_plane(PartType::p_leg1, _cuts[PartType::p_leg1][1], _landmarks.knee1);
		paint_ball(_landmarks.knee0[0], knee_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[6]);
		paint_ball(_landmarks.knee0[1], knee_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[6]);
		paint_ball(_landmarks.knee0[2], knee_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[6]);
		paint_ball(_landmarks.knee0[3], knee_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[6]);
		paint_ball(_landmarks.knee1[0], knee_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[6]);
		paint_ball(_landmarks.knee1[1], knee_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[6]);
		paint_ball(_landmarks.knee1[2], knee_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[6]);
		paint_ball(_landmarks.knee1[3], knee_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[6]);
		// ankle
		//find_feature(PartType::p_leg0, _cuts[PartType::p_leg0][2], 2, _landmarks.knee0, _landmarks.ankle0);
		//find_feature(PartType::p_leg1, _cuts[PartType::p_leg1][2], 2, _landmarks.knee1, _landmarks.ankle1);
		find_feature_plane(PartType::p_leg0, _cuts[PartType::p_leg0][2], _landmarks.ankle0);
		find_feature_plane(PartType::p_leg1, _cuts[PartType::p_leg1][2], _landmarks.ankle1);
		paint_ball(_landmarks.ankle0[0], ankle_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[7]);
		paint_ball(_landmarks.ankle0[1], ankle_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[7]);
		paint_ball(_landmarks.ankle0[2], ankle_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[7]);
		paint_ball(_landmarks.ankle0[3], ankle_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[7]);
		paint_ball(_landmarks.ankle1[0], ankle_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[7]);
		paint_ball(_landmarks.ankle1[1], ankle_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[7]);
		paint_ball(_landmarks.ankle1[2], ankle_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[7]);
		paint_ball(_landmarks.ankle1[3], ankle_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[7]);
		// head
		find_feature(PartType::p_head, _cuts[PartType::p_head][0], 2, _landmarks.neck, _landmarks.head);
		find_feature_plane2(PartType::p_head, _cuts[PartType::p_head][0], _mesh->v[_landmarks.head[0]], _mesh->v[_landmarks.head[1]], _landmarks.head);
		paint_ball(_landmarks.head[0], head_ring, BALL_RADIUS, BALL_CUTS, _mesh->RED);
		paint_ball(_landmarks.head[1], head_ring, BALL_RADIUS, BALL_CUTS, _mesh->RED);
		// elbow
		int elbow_dest0[2] = { _landmarks.shoulder[0], _landmarks.armpit[0] };
		int elbow_dest1[2] = { _landmarks.shoulder[1], _landmarks.armpit[1] };
		//find_feature(PartType::p_arm0, _cuts[PartType::p_arm0][1], elbow_dest0, _landmarks.elbow0);
		//find_feature(PartType::p_arm1, _cuts[PartType::p_arm1][1], elbow_dest1, _landmarks.elbow1);
		//find_feature_near_far(PartType::p_arm0, _cuts[PartType::p_arm0][1], _landmarks.armpit[0], _landmarks.elbow0);
		//find_feature_near_far(PartType::p_arm1, _cuts[PartType::p_arm1][1], _landmarks.armpit[1], _landmarks.elbow1);
		find_feature_plane(PartType::p_arm0, _cuts[PartType::p_arm0][1], _landmarks.elbow0);
		find_feature_plane(PartType::p_arm1, _cuts[PartType::p_arm1][1], _landmarks.elbow1);
		paint_ball(_landmarks.elbow0[0], elbow_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[3]);
		paint_ball(_landmarks.elbow0[1], elbow_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[3]);
		paint_ball(_landmarks.elbow0[2], elbow_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[3]);
		paint_ball(_landmarks.elbow0[3], elbow_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[3]);
		paint_ball(_landmarks.elbow1[0], elbow_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[3]);
		paint_ball(_landmarks.elbow1[1], elbow_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[3]);
		paint_ball(_landmarks.elbow1[2], elbow_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[3]);
		paint_ball(_landmarks.elbow1[3], elbow_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[3]);
		// wrist
		find_feature(PartType::p_arm0, _cuts[PartType::p_arm0][2], 2, _landmarks.elbow0, _landmarks.wrist0);
		find_feature(PartType::p_arm1, _cuts[PartType::p_arm1][2], 2, _landmarks.elbow1, _landmarks.wrist1);
		find_feature_plane(PartType::p_arm0, _cuts[PartType::p_arm0][2], _landmarks.wrist0);
		find_feature_plane(PartType::p_arm1, _cuts[PartType::p_arm1][2], _landmarks.wrist1);
		paint_ball(_landmarks.wrist0[2], wrist_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[4]);
		paint_ball(_landmarks.wrist0[3], wrist_ring[0], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[4]);
		paint_ball(_landmarks.wrist1[2], wrist_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[4]);
		paint_ball(_landmarks.wrist1[3], wrist_ring[1], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[4]);

		//int foot0 = -1, foot1 = -1;// vertex index
		//IntVec hips, heads;
		//for (auto it = feature_points.begin(); it != feature_points.end(); ++it) {
		//	int f = _mesh->vt(*it, 0);
		//	if (labels[f] == 0) { heads.push_back(*it); }
		//	if (labels[f] == 5) { hips.push_back(*it); }
		//	if (labels_detail[f] == 32) foot0 = *it;
		//	if (labels_detail[f] == 42) foot1 = *it;
		//}
		//int shoulder = -1;// skeleton index
		//for (int i = 0; i < 5; ++i) {
		//	for (int j = 0; j < (int)_slices_face[i].size(); ++j) {
		//		for (int k = 0; k < (int)_slices_face[i][j].size(); ++k) {
		//			bool flag = false; int face = _slices_face[i][j][k];
		//			for (int m = 0; m < 3; ++m) {
		//				int v = _mesh->t[face].v[m];
		//				for (auto it = feature_points.begin(); it != feature_points.end(); ++it) {
		//					if (*it == v) {
		//						if (i == 0) shoulder = j;
		//						flag = true; break;
		//					}
		//				}
		//			}
		//			if (flag) break;
		//		}
		//	}
		//}
		//for (int i = 0; i < (int)_slices_face[0][shoulder].size(); ++i) {
		//	int face = _slices_face[0][shoulder][i];
		//	//_mesh->color_face(face, _mesh->RED);
		//}
		//// len
		//std::cout << "head len : " << point_plane_distance(_planes[0][_cuts[0][0]], _mesh->v[heads[0]]) << std::endl;
		//std::cout << "3 hip len : " << point_plane_distance(_planes[3][_cuts[3][0]], _mesh->v[hips[1]]) << std::endl;
		//std::cout << "4 hip len : " << point_plane_distance(_planes[4][_cuts[4][0]], _mesh->v[hips[1]]) << std::endl;
		//std::cout << 1 << " arm len : " << (_skeleton[1][_cuts[1][0]] - _skeleton[1][_cuts[1][1]]).length()
		//	<< "  forearm len : " << (_skeleton[1][_cuts[1][1]] - _skeleton[1][_cuts[1][2]]).length() << std::endl;
		//std::cout << 2 << " arm len : " << (_skeleton[2][_cuts[2][0]] - _skeleton[2][_cuts[2][1]]).length()
		//	<< "  forearm len : " << (_skeleton[2][_cuts[2][1]] - _skeleton[2][_cuts[2][2]]).length() << std::endl;
		//std::cout << "neck to hip : " << (_skeleton[5][MANUAL_BODY_CUT_DOWN] - _skeleton[0][_cuts[0][0]]).length() << std::endl;
		//std::cout << "shoulder to hip : " << (_skeleton[5][MANUAL_BODY_CUT_DOWN] - _skeleton[0][shoulder]).length() << std::endl;
		//std::cout << "3 foot len : " << point_plane_distance(_planes[3][_cuts[3][2]], _mesh->v[foot0]) << std::endl;
		//std::cout << "4 foot len : " << point_plane_distance(_planes[4][_cuts[4][2]], _mesh->v[foot1]) << std::endl;
	}
	void SkeletalSegmentation::HKS_paint_features() {
		FILE* fp = fopen("hks.txt", "rb");
		std::vector<double> hks(_mesh->v.size());
		fread(&hks[0], _mesh->v.size() * sizeof(double), 1, fp);
		fclose(fp);
		IntVec features;
		for (int i = 0; i < (int)_mesh->v.size(); ++i) {
			bool flag = true;
			for (int j = 0; j < _mesh->vvn[i]; ++j) {
				int n = _mesh->vv(i, j);
				if (n != i && hks[i] <= hks[n]) flag = false;
				/*for (int k = 0; k < mesh.vvn[n]; ++k) {
				int nn = mesh.vv(n, k);
				if (nn != i && hks[i] <= hks[nn]) flag = false;
				}*/
			}
			if (flag) features.push_back(i);
			//if (flag) _mesh->color_vertex(i, _mesh->RED);
			//else _mesh->color_vertex(i, _mesh->WHITE);
		}
	}
	void SkeletalSegmentation::find_crotch() {
		// find crotch
		float min_dist = DECIMAL_MAX;
		int leg_cut = 0;
		std::set<int> pts;
		const float crotch_min_dist = 100;
		for (int i = 0; i < 2; ++i) {
			int leg_id = PartType::p_leg0 + i;
			int leg_id_op = PartType::p_leg0 + 1 - i;
			for (auto f : _slices_face[leg_id][_cuts[leg_id][leg_cut]]) {
				for (int j = 0; j < 3; ++j) {
					int v = _mesh->t[f].v[j];
					for (auto f_op : _slices_face[leg_id_op][_cuts[leg_id_op][leg_cut]]) {
						for (int k = 0; k < 3; ++k) {
							int v_op = _mesh->t[f_op].v[k];
							float dist = (_mesh->v[v] - _mesh->v[v_op]).square_length();
							if (dist < min_dist) {
								if (dist < crotch_min_dist) {
									pts.insert(v); pts.insert(v_op);
								}
								min_dist = dist; _landmarks.crotch = v;
							}
						}
					}
				}
			}
		}
		paint_ball(_mesh->v[_landmarks.crotch], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[25]);
		////
		//Vertex cen(0, 0, 0);
		//for (auto v : pts) {
		//	cen += _mesh->v[v];
		//}
		//cen /= pts.size(); min_dist = DECIMAL_MAX;
		//for (int i = 0; i < 2; ++i) {
		//	int leg_id = PartType::p_leg0 + i;
		//	for (auto f : _slices_face[leg_id][_cuts[leg_id][leg_cut]]) {
		//		for (int j = 0; j < 3; ++j) {
		//			int v = _mesh->t[f].v[j];
		//			float dist = (cen - _mesh->v[v]).square_length();
		//			if (dist < min_dist) {
		//				min_dist = dist; _landmarks.crotch = v;
		//			}
		//		}
		//	}
		//}
	}
	void SkeletalSegmentation::find_armpit() {
		//// find armpit
		//min_dist = DECIMAL_MAX;
		//int dest = -1, leg_cut = _cuts[PartType::p_leg0][0];
		//auto leg_slice = _slices_face[PartType::p_leg0][leg_cut];
		//for (auto it = leg_slice.begin(); it != leg_slice.end(); ++it) {
		//	Vertex tri_cen = _mesh->triangle_center(*it);
		//	float dist; closest_skept(PartType::p_body, tri_cen, dist);
		//	if (dist < min_dist) {
		//		min_dist = dist; dest = closest_vertex_in_triangle(PartType::p_leg0, leg_cut, *it);
		//	}
		//}
  		for (int i = 0; i < 2; ++i) {
			float min_dist = DECIMAL_MAX;
			int low_vtx = -1, low_face = -1;
			int arm_num = (i == 0 ? PartType::p_arm0 : PartType::p_arm1);
			int arm_cut = _cuts[arm_num][ArmCut::c_arm];
			auto arm_slice = _slices_face[arm_num][arm_cut];
			auto arm_slice_vtx = _slices_vtx[arm_num][arm_cut];
			for (auto it = arm_slice.begin(); it != arm_slice.end(); ++it) {
				float min_dist2 = DECIMAL_MAX;
				int vtx = -1;
				for (int j = 0; j < 3; ++j) {
					int v = _mesh->t[*it].v[j];
					for (auto k : arm_slice_vtx) {
						float dist2 = (k - _mesh->v[v]).length();
						if (dist2 < min_dist2) {
							min_dist2 = dist2; vtx = v;
						}
					}
				}
				float dist = geodesic_dist_slow(vtx, _landmarks.crotch);
				if (dist < min_dist) {
					min_dist = dist; low_face = *it; low_vtx = vtx;
				}
			}
			//_landmarks.armpit[i] = closest_vertex_in_triangle(arm_num, arm_cut, low_face);
			_landmarks.armpit[i] = low_vtx;
			paint_ball(_mesh->v[_landmarks.armpit[i]], BALL_RADIUS, BALL_CUTS, _mesh->MYCOLOR[29]);
		}
	}
	bool SkeletalSegmentation::intersect_triangle(const Vertex& orig, const Vertex& dir, const Vertex& v0,
		const Vertex& v1, const Vertex& v2, float& t, float& u, float& v, Vertex& intersect)
	{
		// E1
		Vertex E1 = v1 - v0;
		// E2
		Vertex E2 = v2 - v0;
		// P
		Vertex P = dir.cross(E2);
		// determinant
		float det = E1.dot(P);
		// keep det > 0, modify T accordingly
		Vertex T;
		if (det > 0)
			T = orig - v0;
		else
		{
			T = v0 - orig;
			det = -det;
		}
		// If determinant is near zero, ray lies in plane of triangle
		if (det < 0.0001f)
			return false;
		// Calculate u and make sure u <= 1
		u = T.dot(P);
		if (u < 0.0f || u > det)
			return false;
		// Q
		Vertex Q = T.cross(E1);
		// Calculate v and make sure u + v <= 1
		v = dir.dot(Q);
		if (v < 0.0f || u + v > det)
			return false;
		// Calculate t, scale parameters, ray intersects triangle
		t = E2.dot(Q);
		float fInvDet = 1.0f / det;
		t *= fInvDet;
		u *= fInvDet;
		v *= fInvDet;
		if (t < 0.0001f)
			return false;
		intersect.x = t*dir.x;
		intersect.y = t*dir.y;
		intersect.z = t*dir.z;
		intersect += orig;
		return true;
	}
	int SkeletalSegmentation::intersect_triangle(const Vertex& center, const Vertex& ray)
	{
		float min_t = -1;
		int ret = -1;
		for (int i = 0; i < _mesh->t.size(); ++i)
		{
			if (_mesh->t[i].v[0] < 0 || _mesh->t[i].v[1] < 0 || _mesh->t[i].v[2] < 0) continue;
			const Vertex& v0 = _mesh->v[_mesh->t[i].v[0]];
			const Vertex& v1 = _mesh->v[_mesh->t[i].v[1]];
			const Vertex& v2 = _mesh->v[_mesh->t[i].v[2]];
			Vertex intersect; float t, u, v;
			if (intersect_triangle(center, ray, v0, v1, v2, t, u, v, intersect))
			{
				if (min_t < 0 || t < min_t)
				{
					min_t = t;
					ret = i;
				}
			}
		}
		if (ret < 0)
		{
			std::cout << "error : no intersection found\n";
		}
		return ret;
	}
	void SkeletalSegmentation::find_shoulder() {

		//// find shoulder
		//int min_shoulder[2] = { INTEGER_MAX, INTEGER_MAX };
		//float min_dist[2] = { DECIMAL_MAX, DECIMAL_MAX };
		//for (auto it = feature_points.begin(), pit = partid.begin(); it != feature_points.end(); ++it, ++pit) {
		//	int f = _mesh->vt(*it, 0);
		//	*pit = labels_detail[f];
		//	if (*pit == DetailPartType::upbody) {
		//		float dist = (_skeleton[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_arm]] - _mesh->v[*it]).square_length();
		//		float dist2 = (_skeleton[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_arm]] - _mesh->v[*it]).square_length();
		//		if (dist < min_dist[0]) {
		//			min_dist[0] = dist;
		//			_landmarks.shoulder[0] = *it;
		//		}
		//		if (dist2 < min_dist[1]) {
		//			min_dist[1] = dist2;
		//			_landmarks.shoulder[1] = *it;
		//		}
		//	}
		//}
		//// find shoulder
		//int min_shoulder[2] = { INTEGER_MAX, INTEGER_MAX };
		//for (auto it = feature_points.begin(), pit = partid.begin(); it != feature_points.end(); ++it, ++pit) {
		//	int f = _mesh->vt(*it, 0);
		//	*pit = labels_detail[f];
		//	if (*pit == DetailPartType::upbody || *pit == DetailPartType::arm0 || *pit == DetailPartType::arm1) {
		//		float dist, dist2;
		//		int pos = closest_skept(PartType::p_head, _mesh->v[*it], dist);
		//		int pos2 = closest_skept(PartType::p_body, _mesh->v[*it], dist2);
		//		if (dist > dist2) pos = pos2 + (int)_skeleton[PartType::p_head].size();
		//		dist = (_skeleton[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_arm]] - _mesh->v[*it]).square_length();
		//		dist2 = (_skeleton[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_arm]] - _mesh->v[*it]).square_length();
		//		int t = dist < dist2 ? 0 : 1;
		//		if (pos < min_shoulder[t]) {
		//			min_shoulder[t] = pos;
		//			_landmarks.shoulder[t] = *it;
		//		}
		//	}
		//}

		//// find shoulder
		//Vertex refined_armpit[2];
		//for (int i = 0; i < 2; ++i) {
		//	float mindist = DECIMAL_MAX;
		//	for (auto it = arm_ring[i].begin(); it != arm_ring[i].end(); ++it) {
		//		float dist = (*it - _mesh->v[_landmarks.armpit[i]]).square_length();
		//		if (dist < mindist) {
		//			mindist = dist; refined_armpit[i] = *it;
		//		}
		//	}
		//}
		//Vertex shoulder_seed[2];
		//for (int i = 0; i < 2; ++i) {
		//	float maxdist = DECIMAL_MIN;
		//	for (auto it = arm_ring[i].begin(); it != arm_ring[i].end(); ++it) {
		//		float dist = (*it - refined_armpit[i]).square_length();
		//		if (dist > maxdist) {
		//			maxdist = dist; shoulder_seed[i] = *it;
		//		}
		//	}
		//}
		//for (int i = 0; i < 2; ++i) {
		//	float mindist = DECIMAL_MAX;
		//	for (auto it = feature_points.begin(), pit = partid.begin(); it != feature_points.end(); ++it, ++pit) {
		//		int f = _mesh->vt(*it, 0);
		//		*pit = labels_detail[f];
		//		if (*pit == DetailPartType::upbody || *pit == DetailPartType::arm0 || *pit == DetailPartType::arm1) {
		//			float dist = (_mesh->v[*it] - shoulder_seed[i]).square_length();
		//			if (dist < mindist) {
		//				mindist = dist; _landmarks.shoulder[i] = *it;
		//			}
		//		}
		//	}
		//}

		_landmarks.shoulder[0] = _landmarks.shoulder[1] = -1;
		IntVec& feature_points = _hks_points;
		// find shoulder
		const float SHOULDER_BORDER_ROTATE_ANGLE = 30;
		const Vertex ap[2] = { _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]] };
		Plane arm_mid[2]; arm_mid[0].p = ap[0]; arm_mid[0].n = (ap[1] - ap[0]).normalized();
		arm_mid[1].p = ap[1]; arm_mid[1].n = (ap[0] - ap[1]).normalized();
		Vector n[2]; n[0] = _planes[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_arm]].n;
		n[1] = _planes[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_arm]].n;
		Plane border[2][4];
		Vertex ske = _skeleton[PartType::p_body][_skeleton[PartType::p_body].size() - 1];
		Vector axis = ((ap[0] - ske).cross(ap[1] - ske)).normalized();
		for (int i = 0; i < 2; ++i) {
			//auto axis = arm_mid[i].n.cross(n[i]);
			border[i][3] = border[i][2] = border[i][1] = border[i][0] = arm_mid[i];
			//border[i][0].p -= arm_mid[i].n * _parms[PartType::p_arm0 + i][_cuts[PartType::p_arm0 + i][ArmCut::c_wrist]].avg_r;
			//border[i][1].p -= arm_mid[i].n * _parms[PartType::p_arm0 + i][_cuts[PartType::p_arm0 + i][ArmCut::c_wrist]].avg_r;
			border[i][0].n = Quaternion::rotate(border[i][0].n, axis, SHOULDER_BORDER_ROTATE_ANGLE);
			border[i][1].n = Quaternion::rotate(border[i][1].n, axis, -SHOULDER_BORDER_ROTATE_ANGLE);
			border[i][2].n = arm_mid[i].n.cross(axis);
			//border[i][2].n = _planes[PartType::p_body][SLICE_INTERVAL].n;
			float cosine = _planes[PartType::p_body][SLICE_INTERVAL].n.dot(border[i][2].n);
			cosine /= (_planes[PartType::p_body][SLICE_INTERVAL].n.length() * border[i][2].n.length());
			if (cosine < 0) border[i][2].n *= -1;
		}
		for (int idx = 0; idx < 2; ++idx) {
			char arm_label = idx == 0 ? DetailPartType::arm0 : DetailPartType::arm1;
			BoolVec visited(_mesh->v.size(), false);
			ArrayQueue<int> Q(_mesh->v.size());
			IntVec points;
			for (int i = 0; i < _mesh->v.size(); ++i) {
				int n = i;
				float temp = point_plane_distance(border[idx][0], _mesh->v[n]);
				float temp2 = point_plane_distance(border[idx][1], _mesh->v[n]);
				float temp3 = point_plane_distance(border[idx][2], _mesh->v[n]);
				char label = _part_labels[_mesh->vt(n, 0)];
				if (n >= 0 && !visited[n] && temp3 > 0) {
					if ((temp > 0 && temp2 < 0) || (temp < 0 && temp2 > 0)) {
						if (label == DetailPartType::upbody || label == arm_label) {
							points.push_back(n);
						}
					}
				}
			}
			//// BFS
			//while (!Q.empty()) {
			//	int curr = Q.front(); Q.pop();
			//	for (int i = 0; i < _mesh->vvn[curr]; i++) {
			//		int n = _mesh->vv(curr, i);
			//		float temp = point_plane_distance(border[idx][0], _mesh->v[n]);
			//		float temp2 = point_plane_distance(border[idx][1], _mesh->v[n]);
			//		float temp3 = point_plane_distance(border[idx][2], _mesh->v[n]);
			//		char label = _part_labels[_mesh->vt(n, 0)];
			//		if (n >= 0 && !visited[n] && temp3 > 0) {
			//			if ((temp > 0 && temp2 < 0) || (temp < 0 && temp2 > 0)) {
			//				if (label == DetailPartType::upbody || label == arm_label) {
			//					visited[n] = true; Q.push(n); points.push_back(n); ++cnt;
			//				}
			//			}
			//		}
			//	}
			//}
			IntVec candidates;
			if (1) {
				Vertex necks[] = { _mesh->v[_landmarks.neck[0]], _mesh->v[_landmarks.neck[1]] };
				Vertex neck = point_plane_distance(arm_mid[idx], necks[0]) < point_plane_distance(arm_mid[idx], necks[1]) ? necks[0] : necks[1];
				float min_dist = DECIMAL_MAX;
				Vertex side(0, 0, 0);
				for (auto v : points) {
					float dist = point_plane_distance(arm_mid[idx], _mesh->v[v]);
					if (dist < min_dist) { min_dist = dist; side = _mesh->v[v]; }
				}
				border[idx][3].p = side;
				border[idx][3].n = (axis.cross((neck - side).normalized())).normalized();
				float cosine = _planes[PartType::p_body][SLICE_INTERVAL].n.dot(border[idx][3].n);
				cosine /= (_planes[PartType::p_body][SLICE_INTERVAL].n.length() * border[idx][3].n.length());
				if (cosine < 0) border[idx][3].n *= -1;
				for (auto v : points) {
					float temp = point_plane_distance(border[idx][3], _mesh->v[v]);
					if (temp > 0) candidates.push_back(v);
				}
			}
			//// distance
			//if (0) {
			//	float max_dist = DECIMAL_MIN, min_dist = DECIMAL_MAX;
			//	for (auto v : points) {
			//		float dist = point_distance(v, _landmarks.armpit[idx]);
			//		if (dist > max_dist) max_dist = dist;
			//		if (dist < min_dist) min_dist = dist;
			//	}
			//	float mean_dist = min_dist + (max_dist - min_dist) * (float)0.0;
			//	for (auto v : points) {
			//		float dist = point_distance(v, _landmarks.armpit[idx]);
			//		if (dist > mean_dist) {
			//			candidates.push_back(v);
			//		}
			//	}
			//}
			//// shoot ray
			//if (0) {
			//	int tri = intersect_triangle(_mesh->v[_landmarks.armpit[idx]], border[idx][2].n);
			//	if (tri >= 0) get_candidates_geodesic(_mesh->t[tri].v[0], _parms[PartType::p_arm0 + idx][_cuts[PartType::p_arm0 + idx][ArmCut::c_arm]].avg_r * 2, candidates);
			//}
			for (auto v : candidates) {
				_mesh->color_vertex(v, _mesh->RED);
			}
			if(candidates.size() > 0) extract_landmark_from_candidates(candidates, LandMarksID::lm_shoulder0 + idx, _landmarks.shoulder[idx]);
		}
		//// find feature point
		//float maxdist[2] = { DECIMAL_MIN, DECIMAL_MIN };
		//for (auto v : feature_points) {
		//	if (labels[v]) {
		//		float dist = point_distance(v, _landmarks.armpit[0]);
		//		float dist2 = point_distance(v, _landmarks.armpit[1]);
		//		if (dist < dist2) {
		//			shoulder_region[0].push_back(v);
		//			if (dist > maxdist[0]) {
		//				maxdist[0] = dist; _landmarks.shoulder[0] = v;
		//			}
		//		}
		//		else {
		//			shoulder_region[1].push_back(v);
		//			if (dist2 > maxdist[1]) {
		//				maxdist[1] = dist2; _landmarks.shoulder[1] = v;
		//			}
		//		}
		//	}
		//}
		//for (int i = 0; i < 2; ++i) {
		//	if (_landmarks.shoulder[i] >= 0) continue;
		//	double max_val = -std::numeric_limits<double>::max();
		//	for (auto v : shoulder_region[i]) {
		//		if (_curvature_gauss[v] > max_val) {
		//			max_val = _curvature_gauss[v]; _landmarks.shoulder[i] = v;
		//		}
		//	}
		//}

		// find shoulder
		VtxVec arm_ring[2] = { _slices_vtx[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_arm]],
			_slices_vtx[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_arm]] };

		if (REFINE_SHOULDER) {

			// refine arm split
			for (int i = 0; i < 2; ++i) {
				if (_landmarks.shoulder[i] < 0) {
					if (PAINT_SEG_OVER_MESH) paint_ring_vertices(_planes[i + 1][_cuts[i + 1][ArmCut::c_arm]].n, arm_ring[i], _mesh->PURPLE);
					float maxdist = DECIMAL_MIN;
					Vertex shoulder_vtx(0, 0, 0);
					for (auto it = arm_ring[i].begin(); it != arm_ring[i].end(); ++it) {
						float dist = (*it - _mesh->v[_landmarks.armpit[i]]).square_length();
						if (dist > maxdist) {
							maxdist = dist; shoulder_vtx = *it;
						}
					}
					//if (PAINT_BALL) paint_ball(shoulder_vtx, BALL_RADIUS, BALL_CUTS, _mesh->PURPLE);
				}
				else {
					//if (PAINT_BALL) paint_ball(_mesh->v[_landmarks.shoulder[i]], BALL_RADIUS, BALL_CUTS, _mesh->BLUE);
					int ske_idx = _cuts[i + 1][ArmCut::c_arm];
					Vertex ske_head = _skeleton[i + 1][ske_idx - SLICE_INTERVAL];
					Vertex ske_mid = _skeleton[i + 1][ske_idx];
					Vertex ske_tail = _skeleton[i + 1][ske_idx + SLICE_INTERVAL];
					Vector axis = (ske_head - ske_mid).cross(ske_tail - ske_mid);
					Vertex cen = _mesh->v[_landmarks.armpit[i]] + (_skeleton[i + 1][ske_idx - 1] - _skeleton[i + 1][ske_idx]);
					Plane plane = create_plane_cross(cen, axis, _mesh->v[_landmarks.shoulder[i]] - cen);
					IntVec ring_face;
					arm_ring[i].clear();
					search_loop(plane, _mesh->v[_landmarks.shoulder[i]], ring_face, arm_ring[i]);
					if (PAINT_SEG_OVER_MESH) paint_ring_vertices(plane.n, arm_ring[i], _mesh->PURPLE);
				}
			}
		}
		else if (PAINT_SEG_OVER_MESH) {
			paint_ring_vertices(_planes[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_arm]].n, arm_ring[0], _mesh->PURPLE);
			paint_ring_vertices(_planes[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_arm]].n, arm_ring[1], _mesh->PURPLE);
		}
	}
	void SkeletalSegmentation::find_chest() {

		_landmarks.nipple[0] = _landmarks.nipple[1] = -1;
		IntVec& feature_points = _hks_points;
		CharVec& labels = _part_labels;

		const Vertex ap[2] = { _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]] };
		Plane plane_down = _planes[PartType::p_body][_cuts[PartType::p_body][BodyCut::c_up]];
		Plane plane_up = _planes[PartType::p_body][SLICE_INTERVAL]; plane_up.n *= -1;
		Plane plane_LR; plane_LR.p = (ap[0] + ap[1]) / 2; plane_LR.n = (ap[1] - ap[0]).normalized();
		Vertex ske = _skeleton[PartType::p_body][_skeleton[PartType::p_body].size() - 1];
		Plane plane_FB = create_plane_cross(ske, ap[0] - ske, ap[1] - ske);
		if (plane_FB.n.z > 0) plane_FB.n *= -1;
		Plane plane_side[2]; plane_side[0].n = plane_LR.n; plane_side[1].n = plane_LR.n * -1;
		plane_side[0].p = ap[0] + plane_side[0].n * (abs(point_plane_distance(plane_LR, ap[0])) * (float)0.1);
		plane_side[1].p = ap[1] + plane_side[1].n * (abs(point_plane_distance(plane_LR, ap[1])) * (float)0.1);

		IntVec candidates[2];
		for (int i = 0; i < _mesh->v.size(); ++i) {
			int f = _mesh->vt(i, 0);
			if (labels[f] == DetailPartType::upbody) {
				Vertex v = _mesh->v[i];
				if (point_plane_distance(plane_down, v) > 0 && point_plane_distance(plane_up, v) > 0
					&& point_plane_distance(plane_FB, v) > 0 && point_plane_distance(plane_side[0], v) > 0
					&& point_plane_distance(plane_side[1], v) > 0) {
					if (point_plane_distance(plane_LR, v) > 0) {
						candidates[0].push_back(i);
					}
					else {
						candidates[1].push_back(i);
					}
				}
			}
		}
		extract_landmark_from_candidates(candidates[0], LandMarksID::lm_nipple0, _landmarks.nipple[0]);
		extract_landmark_from_candidates(candidates[1], LandMarksID::lm_nipple1, _landmarks.nipple[1]);

		//// HKS
		//for (auto i : feature_points) {
		//	int part_id = DetailPartType::upbody;
		//	int f = _mesh->vt(i, 0);
		//	Vertex v = _mesh->v[i];
		//	if (labels[f] == part_id && i != _landmarks.shoulder[0] && i != _landmarks.shoulder[1]
		//		&& point_plane_distance(plane_down, v) > 0 && point_plane_distance(plane_up, v) > 0 && point_plane_distance(plane_FB, v) > 0) {
		//		paint_ball(v, BALL_RADIUS, BALL_CUTS, _mesh->YELLOW);
		//	}
		//}
		//// curvature
		//float max_val[2] = { DECIMAL_MIN, DECIMAL_MIN };
		//int max_idx[2] = { -1, -1 };
		//for (int i = 0; i < _mesh->v.size(); ++i) {
		//	int f = _mesh->vt(i, 0);
		//	if (labels[f] == DetailPartType::upbody) {
		//		Vertex v = _mesh->v[i];
		//		if (point_plane_distance(plane_down, v) > 0 && point_plane_distance(plane_up, v) > 0 && point_plane_distance(plane_FB, v) > 0) {
		//			auto val = _curvature_gauss[i] + _curvature_mean[i];
		//			if (point_plane_distance(plane_LR, v) > 0) {
		//				if (val > max_val[0]) {
		//					max_val[0] = val; max_idx[0] = i;
		//				}
		//			}
		//			else {
		//				if (val > max_val[1]) {
		//					max_val[1] = val; max_idx[1] = i;
		//				}
		//			}
		//		}
		//	}
		//}

		int chest_front = -1;
		if (point_plane_distance(plane_FB, _mesh->v[_landmarks.chest[2]]) > 0) {
			chest_front = _landmarks.chest[2];
		}
		else {
			chest_front = _landmarks.chest[3];
		}
		float search_range = _parms[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_wrist]].avg_r;
		IntVec chest_front_candidates;
		if (chest_front >= 0) {
			get_candidates_geodesic(chest_front, search_range, chest_front_candidates);
			extract_landmark_from_candidates(chest_front_candidates, LandMarksID::lm_chest_front, _landmarks.chest[2]);
		}

		// find chest region
		Vertex neck_pt = _skeleton[PartType::p_head][_cuts[PartType::p_head][HeadCut::c_neck]];
		Vertex chest_pt = _skeleton[PartType::p_body][_cuts[PartType::p_body][BodyCut::c_up]];
		Plane plane_chest; plane_chest.p = (neck_pt + chest_pt) / 2; plane_chest.n = (chest_pt - neck_pt).normalized();
		const Vertex sd[2] = { _mesh->v[_landmarks.shoulder[0]], _mesh->v[_landmarks.shoulder[1]] };
		Plane plane_arm[2]; plane_arm[0].p = ap[0]; plane_arm[1].p = ap[1];
		plane_arm[0].n = ((sd[0] - ap[0]).cross((sd[0] - ap[0]).cross(ap[1] - ap[0]))).normalized();
		plane_arm[1].n = ((sd[1] - ap[1]).cross((sd[1] - ap[1]).cross(ap[0] - ap[1]))).normalized();
		if (plane_arm[0].n.dot((ap[1] - ap[0]).normalized()) < 0) plane_arm[0].n *= -1;
		if (plane_arm[1].n.dot((ap[0] - ap[1]).normalized()) < 0) plane_arm[1].n *= -1;
		for (int i = 0; i < _mesh->v.size(); ++i) {
			int f = _mesh->vt(i, 0);
			if (labels[f] == DetailPartType::upbody) {
				Vertex v = _mesh->v[i];
				if (point_plane_distance(plane_down, v) > 0 && point_plane_distance(plane_chest, v) > 0
					&& point_plane_distance(plane_arm[0], v) > 0 && point_plane_distance(plane_arm[1], v) > 0) {
					if (point_plane_distance(plane_LR, v) > 0) {
						_mesh->color_vertex(i, _mesh->BLUE);
					}
					else {
						_mesh->color_vertex(i, _mesh->GREEN);
					}
				}
			}
		}
	}
	void SkeletalSegmentation::find_foot() {

		_landmarks.toe[0] = _landmarks.toe[1] = -1;
		_landmarks.heel[0] = _landmarks.heel[1] = -1;
		IntVec& feature_points = _hks_points;
		CharVec& labels = _part_labels;

		int leg[2] = { PartType::p_leg0, PartType::p_leg1 };
		int ankle[2] = { _cuts[leg[0]][LegCut::c_ankle], _cuts[leg[1]][LegCut::c_ankle] };
		Vertex center[2] = { Vertex(0, 0, 0), Vertex(0, 0, 0) };
		Plane plane_toe[2], plane_heel[2];
		int toe_candidate[2] = { -1, -1 };
		for (int i = 0; i < 2; ++i) {
			float max_dist = DECIMAL_MIN;
			int part_id = (i == 0 ? DetailPartType::foot0 : DetailPartType::foot1);
			Vertex toe(0, 0, 0);
			for (auto v : _slices_vtx[leg[i]][ankle[i]]) center[i] += v;
			center[i] /= _slices_vtx[leg[i]][ankle[i]].size();
			for (int v = 0; v < _mesh->v.size(); ++v) {
				if (labels[_mesh->vt(v, 0)] == part_id) {
					float dist = (center[i] - _mesh->v[v]).square_length();
					if (dist > max_dist) {
						max_dist = dist; toe = _mesh->v[v];
						toe_candidate[i] = v;
					}
				}
			}
			plane_toe[i] = create_plane((toe + center[i]) / 2, toe, center[i]);
			plane_heel[i] = create_plane(center[i] + (toe - center[i]) / 3, toe, center[i]);
		}
		Plane plane_ankle[2] = { _planes[leg[0]][ankle[0]], _planes[leg[1]][ankle[1]] };

		//// HKS
		//int hks_toe[2] = { -1, -1 };
		//int hks_heel[2] = { -1, -1 };
		//for (int i = 0; i < 2; ++i) {
		//	float min_dist = DECIMAL_MAX, max_dist = DECIMAL_MIN;
		//	int part_id = (i == 0 ? DetailPartType::foot0 : DetailPartType::foot1);
		//	for (auto v : feature_points) {
		//		int f = _mesh->vt(v, 0);
		//		if (labels[f] == part_id) {
		//			float dist = (center[i] - _mesh->v[v]).square_length();
		//			if (point_plane_distance(plane_toe[i], _mesh->v[v]) < 0 && point_plane_distance(plane_ankle[i], _mesh->v[v]) > 0 && dist < min_dist) {
		//				hks_heel[i] = v; min_dist = dist;
		//			}
		//			if (point_plane_distance(plane_toe[i], _mesh->v[v]) > 0 && dist > max_dist) {
		//				hks_toe[i] = v; max_dist = dist;
		//			}
		//		}
		//	}
		//}
		//// curvature
		//int cvt_toe[2] = { -1, -1 };
		//int cvt_heel[2] = { -1, -1 };
		//for (int i = 0; i < 2; ++i) {
		//	float max_val_toe = DECIMAL_MIN, max_val_heel = DECIMAL_MIN;
		//	int part_id = (i == 0 ? DetailPartType::foot0 : DetailPartType::foot1);
		//	for (int v = 0; v < _mesh->v.size(); ++v) {
		//		int f = _mesh->vt(v, 0);
		//		if (labels[f] == part_id) {
		//			float val = _curvature_max[v];
		//			if (point_plane_distance(plane_toe[i], _mesh->v[v]) > 0) {
		//				if (val > max_val_toe) {
		//					max_val_toe = val; cvt_toe[i] = v;
		//				}
		//			}
		//			if (point_plane_distance(plane_heel[i], _mesh->v[v]) < 0 && point_plane_distance(plane_ankle[i], _mesh->v[v]) > 0) {
		//				if (val > max_val_heel) {
		//					max_val_heel = val; cvt_heel[i] = v;
		//				}
		//			}
		//		}
		//	}
		//}

		for (int i = 0; i < 2; ++i) {
			IntVec candidates[2];
			int part_id = (i == 0 ? DetailPartType::foot0 : DetailPartType::foot1);
			for (int v = 0; v < _mesh->v.size(); ++v) {
				int f = _mesh->vt(v, 0);
				if (labels[f] == part_id) {
					if (point_plane_distance(plane_toe[i], _mesh->v[v]) > 0) {
						//candidates[0].push_back(v);
					}
					if (point_plane_distance(plane_heel[i], _mesh->v[v]) < 0 && point_plane_distance(plane_ankle[i], _mesh->v[v]) > 0) {
						candidates[1].push_back(v);
					}
				}
			}
			get_candidates_geodesic(toe_candidate[i], _parms[PartType::p_leg0 + i][ankle[i]].avg_r * 2, candidates[0]);
			extract_landmark_from_candidates(candidates[0], LandMarksID::lm_toe0 + i, _landmarks.toe[i]);
			extract_landmark_from_candidates(candidates[1], LandMarksID::lm_heel0 + i, _landmarks.heel[i]);
		}

		//// paint
		//for (int i = 0; i < 2; ++i) {
		//	_landmarks.toe[i] = cvt_toe[i];
		//	_landmarks.heel[i] = cvt_heel[i];
		//	if (_landmarks.toe[i] >= 0)
		//		paint_ball(_mesh->v[_landmarks.toe[i]], BALL_RADIUS, BALL_CUTS, _mesh->YELLOW);
		//	if (_landmarks.heel[i] >= 0)
		//		paint_ball(_mesh->v[_landmarks.heel[i]], BALL_RADIUS, BALL_CUTS, _mesh->YELLOW);
		//}
	}
	void SkeletalSegmentation::find_hip() {

		_landmarks.gluteal[0] = _landmarks.gluteal[1] = -1;
		IntVec& feature_points = _hks_points;
		CharVec& labels = _part_labels;

		Plane plane_up = _planes[PartType::p_body][_cuts[PartType::p_body][BodyCut::c_down]]; plane_up.n *= -1;
		Plane plane_dn = plane_up; plane_dn.p += Vector(0, -1500, 0); plane_dn.n *= -1;
		const Vertex ap[2] = { _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]] };
		Vertex ske = _skeleton[PartType::p_body][_skeleton[PartType::p_body].size() - 1];
		Plane plane_LR; plane_LR.p = ske; plane_LR.n = (ap[1] - ap[0]).normalized();
		Plane plane_FB = create_plane_cross(ske, ap[0] - ske, ap[1] - ske);
		if (plane_FB.n.z > 0) plane_FB.n *= -1;
		Vertex side_vtx_max(0, 0, 0), side_vtx_min(0, 0, 0);
		float side_dist_max = DECIMAL_MIN, side_dist_min = DECIMAL_MAX;
		for (int i = 0; i < _mesh->v.size(); ++i) {
			int f = _mesh->vt(i, 0);
			if (labels[f] == DetailPartType::downbody) {
				float dist = point_plane_distance(plane_LR, _mesh->v[i]);
				if (dist > side_dist_max) {
					side_vtx_max = _mesh->v[i]; side_dist_max = dist;
				}
				if (dist < side_dist_min) {
					side_vtx_min = _mesh->v[i]; side_dist_min = dist;
				}
			}
		}
		Plane plane_side[2]; plane_side[0].n = plane_LR.n*-1; plane_side[1].n = plane_LR.n;
		plane_side[0].p = side_vtx_max + plane_side[0].n * (side_dist_max * (float)0.3);
		plane_side[1].p = side_vtx_min + plane_side[1].n * (side_dist_min * (float)0.3);

		IntVec candidates[2];
		for (int i = 0; i < _mesh->v.size(); ++i) {
			int f = _mesh->vt(i, 0);
			if (labels[f] == DetailPartType::downbody) {
				Vertex v = _mesh->v[i];
				if (point_plane_distance(plane_up, v) > 0 && point_plane_distance(plane_dn, v) > 0 && point_plane_distance(plane_FB, v) < 0
					&& point_plane_distance(plane_side[0], v) > 0 && point_plane_distance(plane_side[1], v) > 0) {
					if (point_plane_distance(plane_LR, v) > 0) {
						candidates[0].push_back(i);
					}
					else {
						candidates[1].push_back(i);
					}
				}
			}
		}
		extract_landmark_from_candidates(candidates[0], LandMarksID::lm_gluteal0, _landmarks.gluteal[0]);
		extract_landmark_from_candidates(candidates[1], LandMarksID::lm_gluteal1, _landmarks.gluteal[1]);
	}
	void SkeletalSegmentation::find_navel() {

		//// find navel
		//for (int i = 0; i < 2; ++i) {
		//	for (auto it = feature_points.begin(); it != feature_points.end(); ++it) {
		//		int f = _mesh->vt(*it, 0);
		//		if (labels_detail[f] == DetailPartType::midbody) {
		//			_landmarks.navel[i] = *it;
		//		}
		//	}
		//}
		//int navel_cut = -1;
		//for (int i = 0; i < (int)_slices_face[PartType::p_body].size(); ++i) {
		//	for (auto it = _slices_face[PartType::p_body][i].begin(); it != _slices_face[PartType::p_body][i].end(); ++it) {
		//		for (int j = 0; j < 3; ++j) {
		//			int v = _mesh->t[*it].v[j];
		//			if (v == _landmarks.navel[0]) {
		//				navel_cut = i; break;
		//			}
		//		}
		//	}
		//}
	}
	void SkeletalSegmentation::find_neck() {

		IntVec& feature_points = _hks_points;
		CharVec& labels = _part_labels;
		Plane plane_up = _planes[PartType::p_head][_cuts[PartType::p_head][HeadCut::c_neck]]; plane_up.n *= -1;
		const Vertex ap[2] = { _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]] };
		Vertex neck_cen = _parms[PartType::p_head][_cuts[PartType::p_head][HeadCut::c_neck]].center;
		Plane plane_FB = create_plane_cross(neck_cen, ap[0] - neck_cen, ap[1] - neck_cen);
		if (plane_FB.n.z > 0) plane_FB.n *= -1;

		int neck_front = -1, neck_back = -1;
		if (point_plane_distance(plane_FB, _mesh->v[_landmarks.neck[2]]) > 0) {
			neck_front = _landmarks.neck[2]; neck_back = _landmarks.neck[3];
		}
		else {
			neck_front = _landmarks.neck[3]; neck_back = _landmarks.neck[2];
		}

		float search_range = _parms[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_wrist]].avg_r * 2;
		IntVec neck_front_candidates, neck_back_candidates;
		if (neck_front >= 0) {
			get_candidates_geodesic(neck_front, search_range, neck_front_candidates);
			IntVec refined_neck_front_candidates;
			for (auto v : neck_front_candidates) {
				if (point_plane_distance(plane_up, _mesh->v[v]) > 0)
					refined_neck_front_candidates.push_back(v);
			}
			extract_landmark_from_candidates(refined_neck_front_candidates, LandMarksID::lm_neck_front, _landmarks.neck[2]);
		}
		if (neck_back >= 0) {
			get_candidates_geodesic(neck_back, search_range, neck_back_candidates);
			extract_landmark_from_candidates(neck_back_candidates, LandMarksID::lm_neck_back, _landmarks.neck[3]);
		}
	}
	void SkeletalSegmentation::find_limbs_landmarks() {

		auto find_feature_plane = [this](int part, int cut, const Plane& xcut, const Plane& zcut, int* out) {
			const VtxVec& slice_vtx = _slices_vtx[part][cut];
			Vertex slice_cen(0, 0, 0);
			for (auto it = slice_vtx.begin(); it != slice_vtx.end(); ++it) slice_cen += *it;
			slice_cen.x /= (float)slice_vtx.size(); slice_cen.y /= (float)slice_vtx.size(); slice_cen.z /= (float)slice_vtx.size();
			const IntVec& slice = _slices_face[part][cut];
			Vertex cen = _skeleton[part][cut];
			for (int i = 0; i < 4; ++i) {
				const Plane& plane0 = (i < 2 ? zcut : xcut), plane1 = (i < 2 ? xcut : zcut);
				for (auto it = slice.begin(); it != slice.end(); ++it) {
					Vertex tri_cen = this->_mesh->triangle_center(*it);
					int temp = this->face_cross_plane(*it, plane0);
					if (temp > 0 && temp < 3) {
						bool flag = (this->point_plane_distance(plane1, tri_cen) > 0);
						if (i % 2 == 0 && flag || i % 2 == 1 && !flag) {
							int vtx = -1; float min_dist = DECIMAL_MAX;
							for (int v = 0; v < 3; ++v) {
								for (Vertex sv : slice_vtx) {
									float dist = (_mesh->v[_mesh->t[*it].v[v]] - sv).square_length();
									if (dist < min_dist) {
										min_dist = dist; vtx = _mesh->t[*it].v[v];
									}
								}
							}
							out[i] = vtx;
						}
					}
				}
			}
		};

		float search_range = _parms[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_wrist]].avg_r;

		/*for (int i = PartType::p_leg0; i <= PartType::p_leg1; ++i) {
			LandMarksID id[2][2] = { { lm_knee0_min, lm_knee0_max }, { lm_knee1_min, lm_knee1_max } };
			for (int j = LegCut::c_knee; j <= LegCut::c_knee; ++j) {
				int cut = _cuts[i][j];
				const IntVec& slice = _slices_face[i][cut];
				IntVec candidates;
				int idx = i - PartType::p_leg0;
				int heel_idx = _landmarks.heel[idx];
				std::cout << "heel:" << heel_idx << std::endl;
				Vertex heel(0, 0, 0);
				float min_dist = DECIMAL_MAX;
				for (int v : _slices_vidx[i][_cuts[i][LegCut::c_ankle]]) {
					float dist = point_distance(v, heel_idx);
					if (dist < min_dist) {
						min_dist = dist; heel = _mesh->v[v];
					}
				}
				Vertex knee = _parms[i][cut].center;
				Vertex ankle = _parms[i][_cuts[i][LegCut::c_ankle]].center;
				Plane xcut(knee, (heel - ankle).cross(knee - ankle).normalized());
				Plane zcut(knee, xcut.n.cross(_planes[i][cut].n).normalized());
				if (zcut.n.dot(((heel - ankle).normalized() + (knee - ankle).normalized()).normalized()) <= 0) zcut.n *= -1;
				int out[4];
				find_feature_plane(i, cut, xcut, zcut, out);
				int knee_min = out[2];
				get_candidates_geodesic(knee_min, search_range, candidates);
				extract_landmark_from_candidates(candidates, id[idx][0], _landmarks.knee_min[idx]);
			}
		}
		return;

		for (int i = PartType::p_leg0; i <= PartType::p_leg1; ++i) {
			LandMarksID id[2][2] = { { lm_knee0_min, lm_knee0_max }, { lm_knee1_min, lm_knee1_max } };
			for (int j = LegCut::c_knee; j <= LegCut::c_knee; ++j) {
				const IntVec& slice = _slices_face[i][_cuts[i][j]];
				IntVec candidates;
				float min_dist = DECIMAL_MAX;
				int min_idx = -1;
				int idx = i - PartType::p_leg0;
				for (auto t : slice) {
					for (int k = 0; k < 3; ++k) {
						int v = _mesh->t[t].v[k];
						float dist = geodesic_dist(v, _landmarks.heel[idx]);
						if (dist < min_dist) {
							min_dist = dist; min_idx = v;
						}
					}
				}
				get_candidates_geodesic(min_idx, search_range, candidates);
				extract_landmark_from_candidates(candidates, id[idx][0], _landmarks.knee_min[idx]);
			}
		}*/

		for (int i = PartType::p_arm0; i <= PartType::p_leg1; ++i) {
			LandMarksID id[8] = {
				lm_elbow0_min, lm_elbow0_max, lm_elbow1_min, lm_elbow1_max,
				lm_knee0_min, lm_knee0_max, lm_knee1_min, lm_knee1_max };
			for (int j = ArmCut::c_elbow; j < ArmCut::c_wrist; ++j) {
				const IntVec& slice = _slices_face[i][_cuts[i][j]];
				IntVec candidates;
				for (auto t : slice) {
					for (int k = 0; k < 3; ++k) {
						int v = _mesh->t[t].v[k];
						if (v >= 0) candidates.push_back(v);
					}
				}
				int index = (i - PartType::p_arm0) * 2 + (j - ArmCut::c_elbow);
				int vtx = -1;
				extract_landmark_from_candidates(candidates, id[index], vtx);
				extract_landmark_from_candidates(candidates, id[index + 1], vtx);
			}
		}
	}
	void SkeletalSegmentation::get_candidates_geodesic(int vtx_seed, float max_dist, IntVec& out_candidates) {

		FltVec gd_dist((int)_mesh->v.size(), DECIMAL_MAX);
		Vertex& seedv = _mesh->v[vtx_seed];
		std::vector<bool> visited((int)_mesh->v.size(), false);
		ArrayQueue<int> Q((int)_mesh->v.size());
		gd_dist[vtx_seed] = 0;
		visited[vtx_seed] = true;
		Q.push(vtx_seed);
		while (!Q.empty()) {
			int curr = Q.front(); Q.pop();
			if (gd_dist[curr] > max_dist * (float)1.5) continue;
			for (int i = 0; i < _mesh->vvn[curr]; i++) {
				int n = _mesh->vv(curr, i);
				if (n >= 0) {
					if (!visited[n]) {
						visited[n] = true; Q.push(n);
					}
					float dist = point_distance(curr, n);
					dist += gd_dist[curr];
					if (dist < gd_dist[n])
						gd_dist[n] = dist;
				}
			}
		}
		for (int i = 0; i < _mesh->v.size(); ++i) {
			if (gd_dist[i] <= max_dist) out_candidates.push_back(i);
		}
	}
	void SkeletalSegmentation::extract_landmark_from_candidates(const IntVec& candidates, int landmark_id, int& out_idx) {

		enum LandMarksID {
			lm_neck_front, lm_neck_back, lm_chest_front, lm_nipple0, lm_nipple1, lm_gluteal0, lm_gluteal1,
			lm_shoulder0, lm_shoulder1, lm_toe0, lm_toe1, lm_heel0, lm_heel1,
			lm_elbow0_min, lm_elbow0_max, lm_elbow1_min, lm_elbow1_max,
			lm_knee0_min, lm_knee0_max, lm_knee1_min, lm_knee1_max, lm_landmark_amount
		};
		const Color landmark_colors[LandMarksID::lm_landmark_amount] = {
			_mesh->MYCOLOR[20], _mesh->RED, _mesh->MYCOLOR[21], _mesh->YELLOW, _mesh->YELLOW, _mesh->CYAN, _mesh->CYAN,
			_mesh->PURPLE, _mesh->PURPLE, _mesh->MYCOLOR[30], _mesh->MYCOLOR[30], _mesh->BLUE, _mesh->BLUE,
			_mesh->GREEN, _mesh->GREEN, _mesh->GREEN, _mesh->GREEN,
			_mesh->MYCOLOR[26], _mesh->MYCOLOR[26], _mesh->MYCOLOR[26], _mesh->MYCOLOR[26]
		};

		for (auto v : candidates) {
			//_mesh->color_vertex(v, _mesh->RED);
		}
		// plane
		const Vertex ap[2] = { _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]] };
		Plane plane_LR; plane_LR.p = (ap[0] + ap[1]) / 2; plane_LR.n = (ap[1] - ap[0]).normalized();
		// curvature
		const int cvt_amount = CurvatureType::c_cvt_type_amount;
		std::vector<double> max_val(cvt_amount, -std::numeric_limits<double>::max());
		std::vector<double> min_val(cvt_amount, std::numeric_limits<double>::max());
		IntVec index_max(cvt_amount, -1), index_min(cvt_amount, -1);
		for (auto v : candidates) {
			double val[cvt_amount] = { 0 };
			for (int i = 0; i < cvt_amount; ++i) {
				val[i] = _curvature[i][v];
				if (val[i] > max_val[i]) {
					max_val[i] = val[i]; index_max[i] = v;
				}
				if (val[i] < min_val[i]) {
					min_val[i] = val[i]; index_min[i] = v;
				}
			}
		}
		IntVec ret(cvt_amount + 1, -1);
		if ((landmark_id < LandMarksID::lm_elbow0_min && landmark_id != LandMarksID::lm_chest_front && landmark_id != LandMarksID::lm_neck_front)
			|| (landmark_id >= LandMarksID::lm_elbow0_min && landmark_id % 2 == 0)) {
			for (int i = 0; i < cvt_amount; ++i) {
				if (index_max[i] >= 0) {
					ret[i] = index_max[i];
					//paint_ball(_mesh->v[out_idx], BALL_RADIUS, BALL_CUTS, _mesh->RED);
				}
			}
		}
		else {
			for (int i = 0; i < cvt_amount; ++i) {
				if (index_min[i] >= 0) {
					ret[i] = index_min[i];
					//paint_ball(_mesh->v[out_idx], BALL_RADIUS, BALL_CUTS, _mesh->BLUE);
				}
			}
		}
		// hks
		for (auto c : candidates) {
			for (auto v : _hks_points) {
				if (c == v) ret[cvt_amount] = v;
			}
		}

		Color colors[] = { _mesh->BLUE, _mesh->RED, _mesh->GREEN, _mesh->CYAN, _mesh->PURPLE, _mesh->YELLOW };

		if (FIND_LANDMARKS) {
			// ground truth
			int gt = _ground_truths[landmark_id];
			if (landmark_id <= LandMarksID::lm_chest_front)
				gt = _ground_truths[landmark_id];
			else if (landmark_id < LandMarksID::lm_shoulder0) {
				Vertex ske = _skeleton[PartType::p_body][_skeleton[PartType::p_body].size() - 1];
				if (landmark_id >= LandMarksID::lm_gluteal0 && landmark_id <= LandMarksID::lm_gluteal1)
					plane_LR.p = ske;
				int offset = landmark_id % 2 == 0 ? landmark_id - 1 : landmark_id;
				int temp[2] = { -1, -1 };
				if (point_plane_distance(plane_LR, _mesh->v[_ground_truths[offset]]) > 0) {
					temp[0] = _ground_truths[offset]; temp[1] = _ground_truths[offset + 1];
				}
				else {
					temp[1] = _ground_truths[offset]; temp[0] = _ground_truths[offset + 1];
				}
				gt = temp[landmark_id - offset];
			}
			else if (landmark_id < LandMarksID::lm_toe0) {
				int offset = landmark_id % 2 == 0 ? landmark_id - 1 : landmark_id;
				int min_vtx = -1;
				float min_dist = DECIMAL_MAX;
				for (int v = 0; v < _mesh->v.size(); ++v) {
					float dist = point_distance(v, _ground_truths[offset]);
					if (dist < min_dist && _part_labels[_mesh->vt(v, 0)] <= DetailPartType::hand1
						&& _part_labels[_mesh->vt(v, 0)] >= DetailPartType::arm0) {
						min_dist = dist; min_vtx = v;
					}
				}
				int temp[2] = { -1, -1 };
				if (_part_labels[_mesh->vt(min_vtx, 0)] < DetailPartType::arm1) {
					temp[0] = _ground_truths[offset]; temp[1] = _ground_truths[offset + 1];
				}
				else {
					temp[1] = _ground_truths[offset]; temp[0] = _ground_truths[offset + 1];
				}
				gt = temp[landmark_id - offset];
			}
			else {
				int step = landmark_id >= LandMarksID::lm_elbow0_min ? 2 : 1;
				int offset = landmark_id % 2 == 0 ? landmark_id - 1 : landmark_id;
				if (landmark_id >= LandMarksID::lm_elbow0_min) {
					offset = landmark_id;
					if (landmark_id == LandMarksID::lm_elbow1_min || landmark_id == LandMarksID::lm_elbow1_max
						|| landmark_id == LandMarksID::lm_knee1_min || landmark_id == LandMarksID::lm_knee1_max)
						offset = landmark_id - step;
				}
				int gt1 = _ground_truths[offset], gt2 = _ground_truths[offset + step];
				int t1 = _mesh->vt(gt1, 0), t2 = _mesh->vt(gt2, 0);
				char label[2] = { _part_labels[t1], _part_labels[t2] };
				int temp[2] = { -1, -1 };
				if (label[0] < label[1]) {
					temp[0] = _ground_truths[offset]; temp[1] = _ground_truths[offset + step];
				}
				else {
					temp[1] = _ground_truths[offset]; temp[0] = _ground_truths[offset + step];
				}
				gt = landmark_id == offset ? temp[0] : temp[1];
			}
			if (gt < 0) {
				std::cout << "ground truth < 0" << std::endl;
				return;
			}
			else paint_ball(_mesh->v[gt], BALL_RADIUS, BALL_CUTS, _mesh->BLACK);
			
			for (int i = 0; i < ret.size(); ++i) {
				if (ret[i] >= 0 && gt >= 0) {
					Vertex v1 = _mesh->v[ret[i]], v2 = _mesh->v[gt];
					double dist1 = (double)v1.x - (double)v2.x;
					double dist2 = (double)v1.y - (double)v2.y;
					double dist3 = (double)v1.z - (double)v2.z;
					double error = sqrt(dist1*dist1 + dist2*dist2 + dist3*dist3);
					_landmark_errors[landmark_id][i] = error;
					paint_ball(_mesh->v[ret[i]], BALL_RADIUS, BALL_CUTS, colors[i]);
				}
				else _landmark_errors[landmark_id][i] = INF_ERR;
			}
		}

		if (FIND_FEATURE_POINTS) {
			for (int i = 0; i < ret.size(); ++i) {
				if (ret[i] >= 0) {
					paint_ball(_mesh->v[ret[i]], BALL_RADIUS, BALL_CUTS, landmark_colors[landmark_id]);
				}
			}
		}

		if (landmark_id == LandMarksID::lm_heel0 || landmark_id == LandMarksID::lm_heel1
			|| landmark_id == LandMarksID::lm_shoulder0 || landmark_id == LandMarksID::lm_shoulder1) {
			out_idx = ret[CurvatureType::c_cvt_type_amount] >= 0 ? ret[CurvatureType::c_cvt_type_amount] : ret[CurvatureType::c_min];
		}
	}
	void SkeletalSegmentation::read_landmark_ground_truth() {

		string path = LANDMARK_GROUND_TRUTH_PATH + "gt_" + _mesh->mesh_file_name + ".txt";
		std::fstream fs;
		fs.open(path, std::ios::in);
		string str;
		for (int i = 0; i < LandMarksID::lm_landmark_amount; ++i) {
			fs >> str;
			int id = -1;
			if (str == "neck_front") id = LandMarksID::lm_neck_front;
			else if (str == "neck_back") id = LandMarksID::lm_neck_back;
			else if (str == "chest_front") id = LandMarksID::lm_chest_front;
			else if (str == "nipple0") id = LandMarksID::lm_nipple0;
			else if (str == "nipple1") id = LandMarksID::lm_nipple1;
			else if (str == "gluteal0") id = LandMarksID::lm_gluteal0;
			else if (str == "gluteal1") id = LandMarksID::lm_gluteal1;
			else if (str == "shoulder0") id = LandMarksID::lm_shoulder0;
			else if (str == "shoulder1") id = LandMarksID::lm_shoulder1;
			else if (str == "toe0") id = LandMarksID::lm_toe0;
			else if (str == "toe1") id = LandMarksID::lm_toe1;
			else if (str == "heel0") id = LandMarksID::lm_heel0;
			else if (str == "heel1") id = LandMarksID::lm_heel1;
			else if (str == "elbow0_min") id = LandMarksID::lm_elbow0_min;
			else if (str == "elbow0_max") id = LandMarksID::lm_elbow0_max;
			else if (str == "elbow1_min") id = LandMarksID::lm_elbow1_min;
			else if (str == "elbow1_max") id = LandMarksID::lm_elbow1_max;
			else if (str == "knee0_min") id = LandMarksID::lm_knee0_min;
			else if (str == "knee0_max") id = LandMarksID::lm_knee0_max;
			else if (str == "knee1_min") id = LandMarksID::lm_knee1_min;
			else if (str == "knee1_max") id = LandMarksID::lm_knee1_max;
			if(id >= 0) fs >> _ground_truths[id];
			else {
				std::cout << "read ground truth failed" << std::endl;
				_ground_truths[id] = -1;
			}
		}
		fs.close();
	}
	void SkeletalSegmentation::write_landmark_error() {

		const string LANDMARK_NAME[] = { "neck_front", "neck_back", "chest_front", "nipple0", "nipple1", "gluteal0", "gluteal1",
			"shoulder0", "shoulder1", "toe0", "toe1", "heel0", "heel1",
			"elbow0_min", "elbow0_max", "elbow1_min", "elbow1_max",
			"knee0_min", "knee0_max", "knee1_min", "knee1_max" };
		string path = LANDMARK_ERROR_PATH + _mesh->mesh_file_name + ".err";
		FILE* fp = fopen(path.c_str(), "wb");
		int size = sizeof(double) * LandMarksID::lm_landmark_amount * (CurvatureType::c_cvt_type_amount + 1);
		fwrite((void*)_landmark_errors, size, 1, fp);
		fclose(fp);
		// print
		std::cout << std::endl;
		for (int i = 0; i < LandMarksID::lm_landmark_amount; ++i) {
			std::cout << LANDMARK_NAME[i] << ' ';
			for (int j = 0; j < CurvatureType::c_cvt_type_amount + 1; ++j) {
				std::cout << _landmark_errors[i][j] << ' ';
			}
			std::cout << std::endl;
		}
	}
	void SkeletalSegmentation::compute_total_landmark_error() {

		const int file_num = 24;
		string file_name[] = { "in", "in3a", "in6a", "2", "3", "4", "5", "10", "13", "14", "15", "17",
			"SPRING0001", "SPRING0002", "SPRING0007", "SPRING0008", "SPRING0009", "SPRING0014", "SPRING0015", "SPRING0017",
			"SPRING0052", "SPRING0053", "SPRING2012a", "SPRING1590a", };
		int cnt[LandMarksID::lm_landmark_amount][CurvatureType::c_cvt_type_amount + 1] = { 0 };
		double avg_errors[LandMarksID::lm_landmark_amount][CurvatureType::c_cvt_type_amount + 1] = { 0 };
		double temp_errors[LandMarksID::lm_landmark_amount][CurvatureType::c_cvt_type_amount + 1];
		for (int i = 0; i < file_num; ++i) {
			string path = LANDMARK_ERROR_PATH + file_name[i] + ".err";
			FILE* fp = fopen(path.c_str(), "rb");
			int size = sizeof(double) * LandMarksID::lm_landmark_amount * (CurvatureType::c_cvt_type_amount + 1);
			fread((void*)temp_errors, size, 1, fp);
			fclose(fp);
			for (int j = 0; j < LandMarksID::lm_landmark_amount; ++j) {
				for (int k = 0; k < CurvatureType::c_cvt_type_amount + 1; ++k) {
					if (temp_errors[j][k] > -std::numeric_limits<double>::max()) {
						//if (temp_errors[j][k] < 10) temp_errors[j][k] = 0;
						avg_errors[j][k] += temp_errors[j][k]; ++cnt[j][k];
					}
				}
			}
		}
		for (int j = 0; j < LandMarksID::lm_landmark_amount; ++j) {
			for (int k = 0; k < CurvatureType::c_cvt_type_amount + 1; ++k) {
				avg_errors[j][k] /= (double)cnt[j][k];
			}
		}
		const string LANDMARK_NAME[] = { "neck_front", "neck_back", "chest_front", "nipple0   ", "nipple1   ", "gluteal0", "gluteal1",
			"shoulder0", "shoulder1", "toe0      ", "toe1      ", "heel0   ", "heel1   ",
			"elbow0_min", "elbow0_max", "elbow1_min", "elbow1_max",
			"knee0_min", "knee0_max", "knee1_min", "knee1_max" };
		// print
		std::cout << "\t\tmin \t max \t mean \t gauss \t m+g \t hks" << std::endl;
		for (int i = 0; i < LandMarksID::lm_landmark_amount; ++i) {
			std::cout << LANDMARK_NAME[i] << '\t';
			for (int j = 0; j < CurvatureType::c_cvt_type_amount + 1; ++j) {
				std::cout << avg_errors[i][j] << '\t';
			}
			std::cout << cnt[i][CurvatureType::c_cvt_type_amount] << '\t';
			std::cout << std::endl;
		}
	}
	// split _mesh by slice
	void SkeletalSegmentation::split_mesh_by_slice()
	{
		// read hks
		string hks_path = "hks_" + _mesh->mesh_file_name + ".txt";
		FILE* fp = fopen(hks_path.c_str(), "rb");
		_hks.resize(_mesh->v.size());
		fread(&_hks[0], _mesh->v.size() * sizeof(double), 1, fp);
		fclose(fp);
		// read curvature
		string cvt_path = "curvature_" + _mesh->mesh_file_name + ".txt";
		fp = fopen(cvt_path.c_str(), "rb");
		for (int i = 0; i < CurvatureType::c_cvt_type_amount - 1; ++i) {
			_curvature[i].resize(_mesh->v.size());
			fread(&_curvature[i][0], _mesh->v.size() * sizeof(double), 1, fp);
		}
		_curvature[CurvatureType::c_mean_gauss].resize(_mesh->v.size());
		for (int i = 0; i < _mesh->v.size(); ++i) {
			_curvature[CurvatureType::c_mean_gauss][i] = _curvature[CurvatureType::c_mean][i] + _curvature[CurvatureType::c_gauss][i];
		}
		fclose(fp);
		// init
		bool find_body = true;
		//bool PRINT_TO_FILE = false;
		for (int idx = 0; idx < (int)_mesh->v.size(); ++idx) {
			_mesh->color_vertex(idx, _preset_color[6]);
		}
		// find leaves
		Int2DVec leaves;
		find_skeleton_leaves(leaves);
		for (int i = 0; i < PartType::p_part_amount - 1; ++i) {
			_slices_face[i].resize((int)leaves[i].size());
			_slices_vtx[i].resize((int)leaves[i].size());
			_slices_vidx[i].resize((int)leaves[i].size());
			for (int j = 0; j < (int)leaves[i].size(); ++j) {
				_skeleton[i].push_back(_voxelh->voxel_center(leaves[i][j]));
			}
		}
		for (int j = 0; j < (int)leaves[PartType::p_body].size(); ++j) {
			_skeleton[PartType::p_body].push_back(_voxelh->voxel_center(leaves[PartType::p_body][j]));
		}
		smooth_skeleton_points(0, PartType::p_part_amount - 2, SMOOTH_SKELETON_TIMES);
		// search loop
		for (int idx = 0; idx < PartType::p_part_amount - 1; idx++) {
			for (int i = 0; i < (int)leaves[idx].size(); i++) {
				if (i >= SLICE_INTERVAL && i < (int)leaves[idx].size() - SLICE_INTERVAL) {
					Plane plane = create_plane(_skeleton[idx][i], _skeleton[idx][i - SLICE_INTERVAL], _skeleton[idx][i + SLICE_INTERVAL]);
					_planes[idx].push_back(plane);
					search_loop(plane, _skeleton[idx][i], _slices_face[idx][i], _slices_vtx[idx][i]);
				}
				else _planes[idx].push_back(Plane());
			}
		}
		// split
		for (int i = 0; i < PartType::p_part_amount - 1; i++) {
			switch (i) {
			case 0: split_head_by_slice(); break;
			case 1: case 2: split_arm_by_slice(i); break;
			case 3: case 4: split_leg_by_slice(i); break;
			default: break;
			}
		}

		// refine
		if (EXTEND_BODY_SKE) {
			extend_body_skeleton(leaves, _cuts);
			_slices_face[PartType::p_body].resize((int)leaves[PartType::p_body].size());
			_slices_vtx[PartType::p_body].resize((int)leaves[PartType::p_body].size());
			_slices_vidx[PartType::p_body].resize((int)leaves[PartType::p_body].size());
			_skeleton[PartType::p_body].clear();
			for (int idx = 0; idx < (int)leaves[PartType::p_body].size(); idx++) {
				_skeleton[PartType::p_body].push_back(_voxelh->voxel_center(leaves[PartType::p_body][idx]));
			}
			smooth_skeleton_points(PartType::p_body, PartType::p_body, SMOOTH_SKELETON_TIMES);
			for (int i = 0; i < (int)leaves[PartType::p_body].size(); i++) {
				if (i >= SLICE_INTERVAL && i < (int)leaves[PartType::p_body].size() - SLICE_INTERVAL) {
					Plane plane = create_plane(_skeleton[PartType::p_body][i], _skeleton[PartType::p_body][i - SLICE_INTERVAL], _skeleton[PartType::p_body][i + SLICE_INTERVAL]);
					if (STRAIGHT_BODY_CUT && MANUAL_BODY_SEG && i == MANUAL_BODY_CUT_UP) plane.n = { 0, 1, 0 };
					if (STRAIGHT_BODY_CUT && MANUAL_BODY_SEG && i == MANUAL_BODY_CUT_DOWN) plane.n = { 0, 1, 0 };
					_planes[PartType::p_body].push_back(plane);
					search_loop(plane, _skeleton[PartType::p_body][i], _slices_face[PartType::p_body][i], _slices_vtx[PartType::p_body][i]);
				}
				else _planes[PartType::p_body].push_back(Plane());
			}
		}
	}
	void SkeletalSegmentation::smooth_skeleton_points(int start, int end, int loop_times) {
		const int interval = SMOOTH_SKELETON_INTERVAL;
		for (int i = 0; i < loop_times; ++i) {
			VtxVec temp[PartType::p_part_amount];
			for (int j = start; j <= end; ++j) temp[j] = _skeleton[j];
			for (int j = start; j <= end; ++j) {
				for (int k = interval; k < (int)_skeleton[j].size() - interval; ++k) {
					_skeleton[j][k] = (temp[j][k - interval] + temp[j][k + interval]) / (float)2;
				}
			}
		}
	}
	void SkeletalSegmentation::compute_parameter_by_slice(int pi, ParmVec& parms) {
		// compute scalars
		const Vtx2DVec& slice = _slices_vtx[pi];
		const int len = (int)slice.size();
		for (int i = SLICE_INTERVAL; i < len - SLICE_INTERVAL; i++) {
			int size = slice[i].size();
			if (i < SLICE_INTERVAL || i >= len - SLICE_INTERVAL || size <= 0) {
				parms[i].df_ske = parms[i].ecc = parms[i].ecc_ske = parms[i].avg_r = parms[i].max_r = parms[i].min_r = parms[i].avg_r_ske = parms[i].max_r_ske = parms[i].min_r_ske = -1;
				parms[i].angle = 0;
				for (int j = 0; j < CurvatureType::c_cvt_type_amount; ++j) {
					parms[i].cvt_min_avg[j] = 0, parms[i].cvt_min[j] = -std::numeric_limits<double>::max();
				}
			}
			else {
				// slice
				{
					const IntVec& slice_face = _slices_face[pi][i];
					std::set<int> vset;
					for (int t : slice_face) {
						int vtx = -1; float min_dist = DECIMAL_MAX;
						for (int v = 0; v < 3; ++v) {
							for (Vertex sv : slice[i]) {
								float dist = (_mesh->v[_mesh->t[t].v[v]] - sv).square_length();
								if (dist < min_dist) {
									min_dist = dist; vtx = _mesh->t[t].v[v];
								}
							}
						}
						vset.insert(vtx);
					}
					for (int v : vset) _slices_vidx[pi][i].push_back(v);
				}
				// center
				Vertex cen(0, 0, 0);
				for (auto it = slice[i].begin(); it != slice[i].end(); ++it) cen += *it;
				cen.x /= (float)size; cen.y /= (float)size; cen.z /= (float)size;
				const Vertex& cen_ske = _skeleton[pi][i];
				parms[i].center = cen;
				// radius
				parms[i].angle = 0; parms[i].avg_r = parms[i].avg_r_ske = parms[i].girth = 0; parms[i].max_r = parms[i].max_r_ske = -1;
				parms[i].min_r = parms[i].min_r_ske = std::numeric_limits<float>::max();
				for (int j = 0; j < size; j++) {
					const Vertex& vtx = slice[i][j];
					const Vertex& last_vtx = (j > 0 ? slice[i][j - 1] : slice[i][size - 1]);
					parms[i].girth += (vtx - last_vtx).length();
					// radius
					float radius = (cen - vtx).length();
					if (radius < parms[i].min_r) parms[i].min_r = radius;
					if (radius > parms[i].max_r) parms[i].max_r = radius;
					parms[i].avg_r += radius;
					float radius_ske = (cen_ske - vtx).length();
					if (radius_ske < parms[i].min_r_ske) parms[i].min_r_ske = radius_ske;
					if (radius_ske > parms[i].max_r_ske) parms[i].max_r_ske = radius_ske;
					parms[i].avg_r_ske += radius_ske;
				}
				const IntVec& slice_face = _slices_face[pi][i];
				const Vector& normal = _planes[pi][i].n;
				int counter = 0;
				for (auto it = slice_face.begin(); it != slice_face.end(); ++it) {
					for (int k = 0; k < 3; ++k) {
						int n = _mesh->tt(*it, k);
						if (n >= 0 && (_mesh->triangle_center(n) - _mesh->triangle_center(*it)).dot(normal) > 0) {
							float ag = _mesh->triangle_dihedral_angle(n, *it);
							if (ag <= 180) {
								parms[i].angle += ag; ++counter;
							}
						}
					}
				}
				parms[i].angle /= (float)counter;
				parms[i].avg_r /= (float)size;
				parms[i].avg_r_ske /= (float)size;
				float df = parms[i].max_r * parms[i].max_r - parms[i].min_r * parms[i].min_r;
				if (df < (float)0.0001) parms[i].ecc = -1;
				else parms[i].ecc = sqrt(df) / parms[i].max_r;
				float df_ske = parms[i].max_r_ske * parms[i].max_r_ske - parms[i].min_r_ske * parms[i].min_r_ske;
				if (df_ske < (float)0.0001) parms[i].ecc_ske = -1;
				else parms[i].ecc_ske = sqrt(df_ske) / parms[i].max_r_ske;
				parms[i].df_ske = parms[i].max_r_ske - parms[i].min_r_ske;

				// curvature
				for (int j = 0; j < CurvatureType::c_cvt_type_amount; ++j) {
					parms[i].cvt_min_avg[j] = 0, parms[i].cvt_min[j] = std::numeric_limits<double>::max();
				}
				int cnt[CurvatureType::c_cvt_type_amount] = { 0 };
				const IntVec& slice_vidx = _slices_vidx[pi][i];
				for (auto vidx : slice_vidx) {
					if (vidx >= 0) {
						for (int j = 0; j < CurvatureType::c_cvt_type_amount; ++j) {
							if (_curvature[j][vidx] < parms[i].cvt_min[j]) {
								parms[i].cvt_min[j] = _curvature[j][vidx];
							}
							++cnt[j]; parms[i].cvt_min_avg[j] += _curvature[j][vidx];
						}
					}
				}
				for (int j = 0; j < CurvatureType::c_cvt_type_amount; ++j) {
					if(cnt[j] > 0) parms[i].cvt_min_avg[j] /= (double)cnt[j];
				}
			}
		}
		_parms[pi] = parms;
		// write scalars to txt
		if (PRINT_TO_FILE) {
			double mul = 10000.0;
			std::string fname = ".\\parm"; fname += ('0' + pi); fname += ".txt";
			std::fstream fs(fname.c_str(), std::ios::out);
			const char* blank = "\t\t";
			for (int i = 0; i < len; i++) {
				fs << i << blank;
				//for (int j = 0; j < CurvatureType::c_cvt_type_amount; ++j) fs << parms[i].cvt_min_avg[j] << blank;
				//for (int j = 0; j < CurvatureType::c_cvt_type_amount; ++j) fs << parms[i].cvt_min[j] << blank;
				fs << parms[i].girth << blank << parms[i].angle << blank << parms[i].ecc << blank
					<< parms[i].avg_r << blank << parms[i].min_r << blank << parms[i].max_r << blank << parms[i].df_ske << std::endl;
			}
			fs.close();
		}
	}
	void SkeletalSegmentation::extend_body_skeleton(Int2DVec& leaves, const IntVec border[]) {
		Vertex leg0 = _voxelh->voxel_center(leaves[3][border[3][0] - SLICE_INTERVAL]);
		Vertex leg1 = _voxelh->voxel_center(leaves[4][border[4][0] - SLICE_INTERVAL]);
		Vertex body_seed = _voxelh->voxel_center(leaves[5][leaves[5].size() - 1]);
		Vertex leg_mid = leg0 + leg1; leg_mid *= (float)0.5;
		Vector body2leg = leg_mid - body_seed;
		int slice_num = (int)(body2leg.length() / _voxelh->vxsize);
		body2leg.normalize();
		Vector step = body2leg * _voxelh->vxsize;
		int last = NONE;
		for (int i = 0; i < slice_num - 1; ++i) {
			int curr = _voxelh->vertex_in_which_voxel(body_seed + step * (float)i);
			if (curr >= 0 && curr != last) {
				leaves[5].push_back(curr);
				last = curr;
			}
		}
	}
	void SkeletalSegmentation::split_head_by_slice() {
		// compute scalars
		int pi = PartType::p_head;
		auto& skerings = _slices_face[0];
		const int len = (int)skerings.size();
		ParmVec parms(len);
		compute_parameter_by_slice(pi, parms);
		int half = (int)(len / 2);
		// neck
		std::vector<float> cosines(len, -1); float maxcosine = -1; int neck = -1;
		const float adder = (float)(COS_INTERVAL*COS_INTERVAL);
		for (int i = half; i < len - SLICE_INTERVAL - COS_INTERVAL; i++) {
			float y0_y1 = (float)(parms[i - COS_INTERVAL].avg_r - parms[i].avg_r);
			//if (y0_y1 > 0) y0_y1 = -y0_y1;
			float y2_y1 = (float)(parms[i + COS_INTERVAL].avg_r - parms[i].avg_r);
			float len0 = adder + y0_y1*y0_y1; len0 = sqrt(len0);
			float len1 = adder + y2_y1*y2_y1; len1 = sqrt(len1);
			float cosine = (y0_y1*y2_y1 - adder) / (len0*len1);
			if (y2_y1 < 0 || y0_y1 >= 0) cosine = -1;
			cosines[i] = cosine;
			if (cosine > maxcosine) {
				maxcosine = cosine; neck = i;
			}
		}
		// neck
		float maxdiff = -1; int neck_diff = -1;
		for (int i = half; i < len - SLICE_INTERVAL - COS_INTERVAL; i++) {
			float y1 = parms[i].girth;
			if (y1 < 0) continue;
			float y0_y1 = (float)(parms[i - COS_INTERVAL].girth - y1);
			float y2_y1 = (float)(parms[i + COS_INTERVAL].girth - y1);
			if (y1 > parms[half].girth * (float)1.5 || parms[i].ecc >(float)0.9) {
				neck_diff = i - 1; break;
			}
			if (y2_y1 > 0 && y2_y1 > maxdiff) {
				if (y0_y1 <= 0)
				{
					maxdiff = y2_y1; neck_diff = i;
					if (parms[i + COS_INTERVAL].girth > 3 * y1)
						break;
				}
				else
				{
					for (int j = i - 1; j >= SLICE_INTERVAL; --j)
					{
						float diff = parms[i + 1].girth - parms[j].girth;
						if (diff >= 0)
						{
							if (diff > maxdiff)
							{
								maxdiff = diff; neck_diff = j;
							}
							break;
						}
					}
				}
			}
		}
		neck = neck_diff;
		//neck = (neck + neck_diff) / 2;
		//neck = neck < neck_diff ? neck : neck_diff;
		// head
		int head = -1;
		for (int i = neck - 1; i - 1 >= SLICE_INTERVAL; --i) {
			if (parms[i - 1].avg_r - parms[i].avg_r > 0) {
				head = i; break;
			}
		}

		head += HEAD_ADJUST;
		neck += NECK_ADJUST;

		// display
		if (PAINT_SEG_ON_MESH) {
			for (int k = 0; k < (int)skerings[neck].size(); k++)
				_mesh->color_face(skerings[neck][k], _preset_color[4]);
			for (int k = 0; k < (int)skerings[head].size(); k++)
				_mesh->color_face(skerings[head][k], _preset_color[0]);
		}
		if (PAINT_SEG_OVER_MESH) {
			paint_ring_vertices(_planes[pi][head].n, _slices_vtx[pi][head], _mesh->RED);
			paint_ring_vertices(_planes[pi][neck].n, _slices_vtx[pi][neck], _mesh->MYCOLOR[20]);
		}
		_cuts[PartType::p_head].resize(HeadCut::c_head_cut_amount);
		_cuts[PartType::p_head][HeadCut::c_head] = head; _cuts[PartType::p_head][HeadCut::c_neck] = neck;
		std::cout << "head info : " << head << " " << neck << std::endl;
	}
	void SkeletalSegmentation::split_arm_by_slice(int pi) {
		// compute scalars
		auto& skerings = _slices_face[pi];
		const int len = (int)skerings.size();
		ParmVec parms(len);
		compute_parameter_by_slice(pi, parms);
		int smallest = SLICE_INTERVAL;
		int cos_interval = COS_INTERVAL;
		// wrist
		int real_len = len - 2 * SLICE_INTERVAL;
		int half = (int)(len / 2);
		int oneforth = (int)SLICE_INTERVAL + (real_len / 10);
		int wrist = 0;
		for (int i = oneforth; i >= smallest; --i) {
			if (parms[i].girth - parms[i + 1].girth > 0) {
				wrist = i; break;
			}
		}
		if (wrist <= 0)
			wrist = smallest;

		int LR = 0;
		if (_skeleton[PartType::p_arm0][half].x < _skeleton[PartType::p_arm1][half].x) {
			LR = PartType::p_arm0;
		}
		else LR = PartType::p_arm1;

		//// arm
		//std::vector<float> cosines(len, -1);
		//float maxcosine = -1; int arm = -1;
		//const float adder = (float)(COS_INTERVAL * COS_INTERVAL);
		//for (int i = half; i < len - SLICE_INTERVAL - COS_INTERVAL; i++) {
		//	float y0_y1 = (float)(parms[i - COS_INTERVAL].avg_r - parms[i].avg_r);
		//	//if (y0_y1 > 0) y0_y1 = -y0_y1;
		//	float y2_y1 = (float)(parms[i + COS_INTERVAL].avg_r - parms[i].avg_r);
		//	float len0 = adder + y0_y1*y0_y1; len0 = sqrt(len0);
		//	float len1 = adder + y2_y1*y2_y1; len1 = sqrt(len1);
		//	float cosine = (y0_y1*y2_y1 - adder) / (len0*len1);
		//	if (y2_y1 < 0 || y0_y1 >= 0) cosine = -1;
		//	cosines[i] = cosine;
		//	if (cosine > maxcosine) {
		//		maxcosine = cosine; arm = i;
		//	}
		//}
		// arm
		std::vector<float> cosines(len, -1);
		Plane plane_up; plane_up.n = _skeleton[PartType::p_body][0] - _skeleton[PartType::p_body][_skeleton[PartType::p_body].size() - 1]; plane_up.p = _skeleton[PartType::p_body][0];
		if (point_plane_distance(plane_up, _skeleton[pi][len / 2]) > point_plane_distance(plane_up, _skeleton[pi][len - 1])) {
			std::cout << "special mode" << std::endl;
			cos_interval = COS_INTERVAL_SP;
		}
		float maxcosine = -1; int arm = -1;
		const float adder = (float)(cos_interval*cos_interval*parms[half].avg_r*parms[half].avg_r);
		for (int i = half; i < len - SLICE_INTERVAL - cos_interval; ++i) {
			if (parms[i].girth < parms[half].girth) continue;
			if (parms[i].girth > 2 * parms[half].girth || parms[i - cos_interval].girth > parms[SLICE_INTERVAL].girth && parms[i].girth > parms[i - cos_interval].girth + parms[half].girth) {
				std::cout << "break at " << i << std::endl;
				break;
			}
			float y0_y1 = (float)(parms[i - cos_interval].girth - parms[i].girth);
			if (y0_y1 > 0) {
				for (int j = i - 1; j >= SLICE_INTERVAL + cos_interval; --j)
				{
					float diff = parms[j].girth - parms[j - cos_interval].girth;
					if (diff >= 0)
					{
						y0_y1 = -diff;
						break;
					}
				}
				if (y0_y1 > 0) y0_y1 = -y0_y1;
			}
			float y2_y1 = (float)(parms[i + cos_interval].girth - parms[i].girth);
			float len0 = adder + y0_y1*y0_y1; len0 = sqrt(len0);
			float len1 = adder + y2_y1*y2_y1; len1 = sqrt(len1);
			float cosine = (y0_y1*y2_y1 - adder) / (len0*len1);
			if (y2_y1 < 0 || y2_y1 < -y0_y1 || cosine > 0) cosine = -1;
			cosines[i] = cosine;
			if (cosine > maxcosine) {
				maxcosine = cosine; arm = i;
			}
		}

		// arm
		float maxdiff = -1; int arm_diff = -1;
		//const float adder = (float)(COS_INTERVAL*COS_INTERVAL);
		for (int i = half; i < len - SLICE_INTERVAL - COS_INTERVAL; i++) {
			float y1 = parms[i].girth;
			float y0_y1 = (float)(parms[i - COS_INTERVAL].girth - y1);
			float y2_y1 = (float)(parms[i + COS_INTERVAL].girth - y1);
			if (y1 > parms[half].girth * (float)1.5 || parms[i].ecc > (float)0.9) {
				arm_diff = i - 1; break;
			}
			if (y2_y1 > 0 && y2_y1 > maxdiff) {
				if (y0_y1 <= 0)
				{
					maxdiff = y2_y1; arm_diff = i;
					if (parms[i + COS_INTERVAL].girth > 3 * y1)
						break;
				}
				else
				{
					for (int j = i - 1; j >= SLICE_INTERVAL; --j)
					{
						float diff = parms[i + 1].girth - parms[j].girth;
						if (diff >= 0)
						{
							if (diff > maxdiff)
							{
								maxdiff = diff; arm_diff = j;
							}
							break;
						}
					}
				}
			}
		}

		arm = arm_diff;

		//// elbow
		//int SEARCH_INTERVAL = ((float)(arm - wrist) / (float)10) + (float)0.5;
		//float maxecc = DECIMAL_MIN;
		//int mid = (arm + wrist) / 2;
		//int elbow = mid;
		//for (int i = mid - SEARCH_INTERVAL; i <= mid + SEARCH_INTERVAL; i++) {
		//	if (parms[i].ecc > 0 && parms[i].ecc > maxecc) {
		//		maxecc = parms[i].ecc; elbow = i;
		//	}
		//}
		// elbow
		int SEARCH_INTERVAL = ((float)(arm - wrist) / (float)15) + (float)0.5;
		int mid = (arm + wrist) / 2 + FIX_ARM;
		int elbow = mid;
		double min_cvt = parms[mid].cvt_min_avg[CurvatureType::c_mean];
		for (int i = mid - 1; i >= mid - SEARCH_INTERVAL; --i) {
			if (parms[i].cvt_min_avg[CurvatureType::c_mean] < min_cvt) {
				min_cvt = parms[i].cvt_min_avg[CurvatureType::c_mean]; elbow = i;
			}
		}
		for (int i = mid + 1; i <= mid + SEARCH_INTERVAL; ++i) {
			if (parms[i].cvt_min_avg[CurvatureType::c_mean] < min_cvt) {
				min_cvt = parms[i].cvt_min_avg[CurvatureType::c_mean]; elbow = i;
			}
		}
		//// elbow
		//int elbow = 0; int mid = (arm + wrist) / 2;
		//int thirdforth = (int)arm - ((arm - wrist) / 4);
		//for (int i = thirdforth; i >= SLICE_INTERVAL; --i) {
		//	if (parms[i].girth - parms[i + 1].girth > 0) {
		//		elbow = i; break;
		//	}
		//}
		//// elbow
		//elbow = wrist + len / 16; char state = 0;
		//for (int i = elbow; i <= arm - 1; ++i) {
		//	if (state == 0) {
		//		if (parms[i - 1].avg_r - parms[i].avg_r > 0) state = 1;
		//	}
		//	else if (state == 1) {
		//		if (parms[i - 1].avg_r - parms[i].avg_r < 0) {
		//			state = 2; elbow = i; break;
		//		}
		//	}
		//	else if (state == 2) {
		//	}
		//}

		if (LR == pi) {
			elbow += L_ELBOW_ADJUST;
			wrist += L_WRIST_ADJUST;
		}
		else {
			elbow += R_ELBOW_ADJUST;
			wrist += R_WRIST_ADJUST;
		}

		// display
		if (PAINT_SEG_ON_MESH) {
			for (int k = 0; k < (int)skerings[wrist].size(); k++)
				_mesh->color_face(skerings[wrist][k], _preset_color[0]);
			for (int k = 0; k < (int)skerings[elbow].size(); k++)
				_mesh->color_face(skerings[elbow][k], _preset_color[2]);
			for (int k = 0; k < (int)skerings[arm].size(); k++)
				_mesh->color_face(skerings[arm][k], _preset_color[1]);
		}
		if (PAINT_SEG_OVER_MESH) {
			if (!REFINE_ARM_SPLIT) paint_ring_vertices(_planes[pi][arm].n, _slices_vtx[pi][arm], _mesh->PURPLE);
			paint_ring_vertices(_planes[pi][elbow].n, _slices_vtx[pi][elbow], _mesh->MYCOLOR[21]);
			paint_ring_vertices(_planes[pi][wrist].n, _slices_vtx[pi][wrist], _mesh->MYCOLOR[24]);
		}

		//paint_ring_vertices(_planes[pi][(elbow + arm) / 2+2].n, _slices_vtx[pi][(elbow + arm) / 2+2], _mesh->RED);

		_cuts[pi].resize(ArmCut::c_arm_cut_amount);
		_cuts[pi][ArmCut::c_arm] = arm; _cuts[pi][ArmCut::c_elbow] = elbow; _cuts[pi][ArmCut::c_wrist] = wrist;

		// length
		std::cout << "arm : " << arm << " elbow : " << elbow << " wrist : " << wrist << " forearm thickness : " << parms[elbow].avg_r * 2 << " wrist radius : " << parms[wrist].avg_r << std::endl;
	}
	void SkeletalSegmentation::split_leg_by_slice(int pi) {

		// compute scalars
		auto& skerings = _slices_face[pi];
		const int len = (int)skerings.size();
		ParmVec parms(len);
		compute_parameter_by_slice(pi, parms);
		int smallest = SLICE_INTERVAL;
		// ankle
		int half = (int)(len / 2);
		int oneforth = (int)(len / 4);
		int ankle = 0;
		for (int i = oneforth; i >= smallest; --i) {
			if (parms[i].girth - parms[i + 1].girth > 0) {
				ankle = i; break;
			}
		}
		if (ankle <= 0)
			ankle = smallest;

		int LR = 0;
		if (_skeleton[PartType::p_leg0][half].x < _skeleton[PartType::p_leg1][half].x) {
			LR = PartType::p_leg0;
		}
		else LR = PartType::p_leg1;

		// leg
		std::vector<float> cosines(len, -1);
		float maxcosine = -1; int leg = -1;
		const float adder = (float)(COS_INTERVAL*COS_INTERVAL*parms[half].avg_r*parms[half].avg_r);
		for (int i = half; i < len - SLICE_INTERVAL - COS_INTERVAL; ++i) {
			if (parms[i].girth < parms[half].girth) continue;
			if (parms[i].girth > parms[i - COS_INTERVAL].girth + parms[half].girth && parms[i - COS_INTERVAL].girth > parms[half].girth) break;
			float y0_y1 = (float)(parms[i - COS_INTERVAL].girth - parms[i].girth);
			float y2_y1 = (float)(parms[i + COS_INTERVAL].girth - parms[i].girth);
			if (y0_y1 > 0) {
				for (int j = i - 1; j >= SLICE_INTERVAL + COS_INTERVAL; --j)
				{
					float diff = parms[j].girth - parms[j - COS_INTERVAL].girth;
					if (diff >= 0)
					{
						y0_y1 = -diff;
						break;
					}
				}
				if (y0_y1 > 0) y0_y1 = -y0_y1;
			}
			float len0 = adder + y0_y1*y0_y1; len0 = sqrt(len0);
			float len1 = adder + y2_y1*y2_y1; len1 = sqrt(len1);
			float cosine = (y0_y1*y2_y1 - adder) / (len0*len1);
			if (y2_y1 < 0 || y2_y1 < -y0_y1 || cosine > 0) cosine = -1;
			cosines[i] = cosine;
			if (cosine > maxcosine) {
				maxcosine = cosine; leg = i;
			}
		}
		// leg
		float maxdiff = -1; int leg_diff = -1;
		for (int i = half; i < len - SLICE_INTERVAL - COS_INTERVAL; ++i) {
			float y1 = parms[i].girth;
			if (y1 < 0) continue;
			float y0_y1 = (float)(parms[i - COS_INTERVAL].girth - y1);
			float y2_y1 = (float)(parms[i + COS_INTERVAL].girth - y1);
			if (y1 > parms[half].girth * (float)2 || parms[i].ecc >(float)0.9) {
				//leg_diff = i - 1; break;
			}
			if (y2_y1 > 0 && y2_y1 > maxdiff) {
				if (y0_y1 <= 0)
				{
					maxdiff = y2_y1; leg_diff = i;
					if (parms[i + COS_INTERVAL].girth > 3 * y1)
						;// break;
				}
				else
				{
					for (int j = i - 1; j >= SLICE_INTERVAL; --j)
					{
						float diff = parms[i + 1].girth - parms[j].girth;
						if (diff >= 0)
						{
							if (diff > maxdiff)
							{
								maxdiff = diff; leg_diff = j;
							}
							break;
						}
					}
				}
			}
		}

		leg = leg_diff;

		//// knee
		//int knee = ankle + len / 16; char state = 0;
		//for (int i = knee; i <= leg - 1; ++i) {
		//	if (state == 0) {
		//		if (parms[i - 1].girth - parms[i].girth > 0) state = 1;
		//	}
		//	else if (state == 1) {
		//		if (parms[i - 1].girth - parms[i].girth < 0) {
		//			state = 2; knee = i; break;
		//		}
		//	}
		//	else if (state == 2) {
		//	}
		//}
		//// knee
		//int SEARCH_INTERVAL = ((float)(leg - ankle) / (float)10) + (float)0.5;
		//int mid = (leg + ankle) / 2;
		//int knee = mid;
		//float maxecc = DECIMAL_MIN;
		//for (int i = mid - SEARCH_INTERVAL; i <= mid + SEARCH_INTERVAL; ++i) {
		//	if (parms[i].ecc > 0 && parms[i].ecc > maxecc) {
		//		maxecc = parms[i].ecc; knee = i;
		//	}
		//}
		// knee
		int SEARCH_INTERVAL = ((float)(leg - ankle) / (float)15) + (float)0.5;
		int mid = (leg + ankle) / 2 + FIX_LEG;
		int knee = mid;
		double min_cvt = parms[mid].cvt_min[CurvatureType::c_min];
		for (int i = mid - 1; i >= mid - SEARCH_INTERVAL; --i) {
			if (parms[i].cvt_min[CurvatureType::c_min] < min_cvt) {
				min_cvt = parms[i].cvt_min[CurvatureType::c_min]; knee = i;
			}
		}
		for (int i = mid + 1; i <= mid + SEARCH_INTERVAL; ++i) {
			if (parms[i].cvt_min[CurvatureType::c_min] < min_cvt) {
				min_cvt = parms[i].cvt_min[CurvatureType::c_min]; knee = i;
			}
		}
		knee = (float)mid;
		if (LR == pi) {
			knee += L_KNEE_ADJUST;
			ankle += L_ANKLE_ADJUST;
		}
		else {
			knee += R_KNEE_ADJUST;
			ankle += R_ANKLE_ADJUST;
		}

		// paint
		if (PAINT_SEG_ON_MESH) {
			for (int k = 0; k < (int)skerings[leg].size(); k++)
				_mesh->color_face(skerings[leg][k], _preset_color[4]);
			for (int k = 0; k < (int)skerings[knee].size(); k++)
				_mesh->color_face(skerings[knee][k], _preset_color[2]);
			for (int k = 0; k < (int)skerings[ankle].size(); k++)
				_mesh->color_face(skerings[ankle][k], _preset_color[5]);
		}
		if (PAINT_SEG_OVER_MESH) {
			paint_ring_vertices(_planes[pi][leg].n, _slices_vtx[pi][leg], _mesh->MYCOLOR[25]);
			paint_ring_vertices(_planes[pi][knee].n, _slices_vtx[pi][knee], _mesh->MYCOLOR[26]);
			paint_ring_vertices(_planes[pi][ankle].n, _slices_vtx[pi][ankle], _mesh->MYCOLOR[27]);
		}
		_cuts[pi].resize(LegCut::c_leg_cut_amount);
		_cuts[pi][LegCut::c_leg] = leg; _cuts[pi][LegCut::c_knee] = knee; _cuts[pi][LegCut::c_ankle] = ankle;

		// length
		std::cout << "leg : " << leg << " knee : " << knee << " ankle : " << ankle << " thigh thickness : " << parms[leg].avg_r*2 << " thigh len : " << (_skeleton[pi][leg] - _skeleton[pi][knee]).length()
			<< "  shank len : " << (_skeleton[pi][knee] - _skeleton[pi][ankle]).length() << std::endl;
	}
	void SkeletalSegmentation::split_body_by_slice() {
		// compute scalars
		int pi = PartType::p_body;
		auto& skerings = _slices_face[pi];
		const int len = (int)skerings.size();
		int half = len / 2;
		ParmVec parms(len);
		compute_parameter_by_slice(pi, parms);
		int interval = COS_INTERVAL;
		// find chest cut
		int chest = -1;
		{
			std::vector<float> cosines(len, -1);
			float maxcosine = -1;
			const float adder = (float)(interval*interval*parms[half].avg_r*parms[half].avg_r);
			for (int i = std::max(_body_border[0] + 2, interval + SLICE_INTERVAL); i < half; ++i) {
				if (abs(parms[i].girth - parms[i - interval].girth) > parms[half].girth) break;
				float y0_y1 = (float)(parms[i - interval].girth - parms[i].girth);
				float y2_y1 = (float)(parms[i + interval].girth - parms[i].girth);
				if (y2_y1 > 0) y2_y1 = -y2_y1;
				if (y2_y1 > 0) {
					for (int j = i + 1; j < half; ++j)
					{
						float diff = parms[j].girth - parms[j + interval].girth;
						if (diff >= 0)
						{
							y2_y1 = -diff;
							break;
						}
					}
					if (y2_y1 > 0) y2_y1 = -y2_y1;
				}
				float len0 = adder + y0_y1*y0_y1; len0 = sqrt(len0);
				float len1 = adder + y2_y1*y2_y1; len1 = sqrt(len1);
				float cosine = (y0_y1*y2_y1 - adder) / (len0*len1);
				if (y0_y1 < 0 || y0_y1 < -y2_y1 || cosine > 0) cosine = -1;
				cosines[i] = cosine;
				if (cosine > maxcosine) {
					maxcosine = cosine; chest = i;
				}
			}
		}
		// find hip cut
		int hip = -1;
		{
			std::vector<float> cosines(len, -1);
			float maxcosine = -1;
			const float adder = (float)(interval*interval*parms[half].avg_r*parms[half].avg_r);
			for (int i = half; i < std::min(_body_border[1] - 2, len - SLICE_INTERVAL - interval); ++i) {
				if (parms[i].girth > parms[i - interval].girth + parms[half].girth) break;
				float y0_y1 = (float)(parms[i - interval].girth - parms[i].girth);
				if (y0_y1 > 0) {
					for (int j = i - 1; j >= SLICE_INTERVAL + interval; --j)
					{
						float diff = parms[j].girth - parms[j - interval].girth;
						if (diff >= 0)
						{
							y0_y1 = -diff;
							break;
						}
					}
					if (y0_y1 > 0) y0_y1 = -y0_y1;
				}
				float y2_y1 = (float)(parms[i + interval].girth - parms[i].girth);
				float len0 = adder + y0_y1*y0_y1; len0 = sqrt(len0);
				float len1 = adder + y2_y1*y2_y1; len1 = sqrt(len1);
				float cosine = (y0_y1*y2_y1 - adder) / (len0*len1);
				if (y2_y1 < 0 || y2_y1 < -y0_y1 || cosine > 0) cosine = -1;
				cosines[i] = cosine;
				if (cosine > maxcosine) {
					maxcosine = cosine; hip = i;
				}
			}
		}
		// waist
		int waist = -1;
		{
			float mingirth = DECIMAL_MAX;
			for (int i = chest; i < std::min(_body_border[1] - 2, len - SLICE_INTERVAL - interval); ++i) {
				if (parms[i].girth > parms[i - interval].girth + parms[half].girth) break;
				if (parms[i].girth < mingirth) {
					mingirth = parms[i].girth; waist = i;
				}
			}
		}
		// info
		std::cout << "body info, chest : " << chest << ", waist : " << waist << ", hip : " << hip << ", border : " << _body_border[0] << " " << _body_border[1] << std::endl;
		
		if (MANUAL_BODY_SEG) {
			_cuts[PartType::p_body][BodyCut::c_up] = MANUAL_BODY_CUT_UP;
			_cuts[PartType::p_body][BodyCut::c_down] = MANUAL_BODY_CUT_DOWN;
			_cuts[PartType::p_body][BodyCut::c_waist] = (MANUAL_BODY_CUT_UP + MANUAL_BODY_CUT_DOWN) / 2;
		}

		int body_cut_up = _cuts[PartType::p_body][BodyCut::c_up];
		int body_cut_mid = _cuts[PartType::p_body][BodyCut::c_waist];
		int body_cut_down = _cuts[PartType::p_body][BodyCut::c_down];

		// paint
		if (PAINT_SEG_ON_MESH) {
			for (int k = 0; k < (int)skerings[body_cut_up].size(); k++)
				_mesh->color_face(skerings[body_cut_up][k], _preset_color[4]);
			for (int k = 0; k < (int)skerings[body_cut_mid].size(); k++)
				_mesh->color_face(skerings[body_cut_mid][k], _preset_color[6]);
			for (int k = 0; k < (int)skerings[body_cut_down].size(); k++)
				_mesh->color_face(skerings[body_cut_down][k], _preset_color[5]);
		}
		if (PAINT_SEG_OVER_MESH) {
			paint_ring_vertices(_planes[pi][body_cut_up].n, _slices_vtx[pi][body_cut_up], _mesh->MYCOLOR[1]);
			paint_ring_vertices(_planes[pi][body_cut_mid].n, _slices_vtx[pi][body_cut_mid], _mesh->MYCOLOR[2]);
			paint_ring_vertices(_planes[pi][body_cut_down].n, _slices_vtx[pi][body_cut_down], _mesh->MYCOLOR[8]);
		}
	}
	// slicing
	bool SkeletalSegmentation::trace_is_end(int current, int target, int v0, int v1)
	{
		for (int i = 0; i < 3; i++)
		{
			if (_mesh->tt(current, i) == target)
			{
				int p0 = 0, p1 = 0;
				find_connected_vertices(current, target, p0, p1);
				if ((v0 != p0 && v0 != p1) || (v1 != p0 && v1 != p1))
					return true;
			}
		}
		return false;
	}
	void SkeletalSegmentation::search_loop(const Plane& plane, int start, IntVec& ret) {
		// find seed BFS
		int face_num = (int)_mesh->t.size();
		ArrayQueue<int> Q(face_num);
		Q.push(_mesh->vt(start, 0));
		std::vector<bool> fvisited(_mesh->t.size(), false);
		fvisited[_mesh->vt(start, 0)] = true;
		int seed = NONE;
		while (!Q.empty())
		{
			int current = Q.front();
			Q.pop();
			int temp = face_cross_plane(current, plane);
			if (temp > 0 && temp < 3)
			{
				seed = current;
				break;
			}
			for (int i = 0; i < 3; i++)
			{
				int neighbor = _mesh->tt(current, i);
				if (neighbor >= 0 && !fvisited[neighbor])
				{
					fvisited[neighbor] = true;
					Q.push(neighbor);
				}
			}
		}
		if (seed == NONE)
			return;
		// find loop
		for (int i = 0; i < (int)_mesh->t.size(); i++)
			fvisited[i] = false;
		ArrayStack<int> S((int)_mesh->t.size());
		fvisited[seed] = true;
		S.push(seed);
		int v0 = 0, v1 = 0;
		bool result = false;
		int prev_node[2] = { 0 };
		int prev_face = NONE;
		float loop_len = 0;
		int count = 0;
		while (!S.empty())
		{
			int current = S.top();
			S.pop();
			for (int i = 0; i < 3; i++)
			{
				int neighbor = _mesh->tt(current, i);
				if (neighbor >= 0 && !fvisited[neighbor])
				{
					int temp = face_cross_plane(neighbor, plane);
					if (temp > 0 && temp < 3)
					{
						fvisited[neighbor] = true;
						S.push(neighbor);
						if (count == 0)
							break;
					}
				}
			}
			if (!S.empty())
			{
				int temp[2];
				temp[0] = NONE; temp[1] = NONE;
				if (count == 0)
				{
					find_connected_vertices(current, S.top(), v0, v1);
				}
			}
			ret.push_back(current);
			if (trace_is_end(current, seed, v0, v1))
			{
				result = true;
				break;
			}
			count++;
		}
		if (result)
		{
			for (int i = 0; i < (int)ret.size(); i++) {
				//_mesh->color_face(ret[i], _mesh->RED);
			}
		}
		else
			ret.clear();
	}
	void SkeletalSegmentation::search_loop(const Plane& plane, Vertex currske, IntVec& ret) {
		// find seed
		int start = 0; float mindist = std::numeric_limits<float>::max();
		for (int i = 0; i < (int)_mesh->v.size(); i++) {
			float dist = (_mesh->v[i] - currske).square_length();
			if (dist < mindist) {
				mindist = dist; start = i;
			}
		}
		// find seed BFS
		int face_num = (int)_mesh->t.size();
		ArrayQueue<int> Q(face_num);
		Q.push(_mesh->vt(start, 0));
		std::vector<bool> fvisited(_mesh->t.size(), false);
		fvisited[_mesh->vt(start, 0)] = true;
		int seed = NONE;
		while (!Q.empty())
		{
			int current = Q.front();
			Q.pop();
			int temp = face_cross_plane(current, plane);
			if (temp > 0 && temp < 3)
			{
				seed = current;
				break;
			}
			for (int i = 0; i < 3; i++)
			{
				int neighbor = _mesh->tt(current, i);
				if (neighbor >= 0 && !fvisited[neighbor])
				{
					fvisited[neighbor] = true;
					Q.push(neighbor);
				}
			}
		}
		if (seed == NONE)
			return;
		// find loop
		for (int i = 0; i < (int)_mesh->t.size(); i++) {
			fvisited[i] = false;
		}
		ArrayStack<int> S((int)_mesh->t.size());
		fvisited[seed] = true;
		S.push(seed);
		int v0 = 0, v1 = 0;
		bool result = false;
		int prev_node[2] = { 0 };
		int prev_face = NONE;
		float loop_len = 0;
		int count = 0;
		while (!S.empty())
		{
			int current = S.top();
			S.pop();
			for (int i = 0; i < 3; i++)
			{
				int neighbor = _mesh->tt(current, i);
				if (neighbor >= 0 && !fvisited[neighbor])
				{
					int temp = face_cross_plane(neighbor, plane);
					if (temp > 0 && temp < 3)
					{
						fvisited[neighbor] = true;
						S.push(neighbor);
						if (count == 0)
							break;
					}
				}
			}
			if (!S.empty())
			{
				int temp[2];
				temp[0] = NONE; temp[1] = NONE;
				if (count == 0)
				{
					find_connected_vertices(current, S.top(), v0, v1);
				}
			}
			ret.push_back(current);
			if (trace_is_end(current, seed, v0, v1))
			{
				result = true;
				break;
			}
			count++;
		}
		if (result)
		{
			for (int i = 0; i < (int)ret.size(); i++) {
				//_mesh->color_face(ret[i], _mesh->RED);
			}
		}
		else
			ret.clear();
	}
	void SkeletalSegmentation::search_loop(const Plane& plane, Vertex currske, IntVec& ret, VtxVec& retv) {
		// find seed
		int start = 0; float mindist = std::numeric_limits<float>::max();
		for (int i = 0; i < (int)_mesh->v.size(); ++i) {
			float dist = (_mesh->v[i] - currske).square_length();
			if (dist < mindist) {
				mindist = dist; start = i;
			}
		}
		// find seed BFS
		int face_num = (int)_mesh->t.size();
		ArrayQueue<int> Q(face_num);
		int close_seed = _mesh->vt(start, 0);
		mindist = std::numeric_limits<float>::max();
		for (int i = 0; i < (int)_mesh->vtn[start]; ++i) {
			float dist = (triangle_center(_mesh->vt(start, i)) - currske).square_length();
			if (dist < mindist) {
				mindist = dist; close_seed = _mesh->vt(start, i);
			}
		}
		Q.push(close_seed);
		std::vector<bool> fvisited(_mesh->t.size(), false);
		fvisited[_mesh->vt(start, 0)] = true;
		int seed = NONE;
		while (!Q.empty())
		{
			int current = Q.front();
			Q.pop();
			int temp = face_cross_plane(current, plane);
			if (temp > 0 && temp < 3)
			{
				seed = current;
				break;
			}
			for (int i = 0; i < 3; i++)
			{
				int neighbor = _mesh->tt(current, i);
				if (neighbor >= 0 && !fvisited[neighbor])
				{
					fvisited[neighbor] = true;
					Q.push(neighbor);
				}
			}
		}
		if (seed == NONE)
			return;
		// find loop
		for (int i = 0; i < (int)_mesh->t.size(); i++) {
			fvisited[i] = false;
		}
		ArrayStack<int> S((int)_mesh->t.size());
		fvisited[seed] = true;
		S.push(seed);
		int v0 = 0, v1 = 0;
		bool result = false;
		int count = 0;
		while (!S.empty()) {
			int current = S.top();
			S.pop();
			for (int i = 0; i < 3; i++) {
				int neighbor = _mesh->tt(current, i);
				if (neighbor >= 0 && !fvisited[neighbor])
				{
					int temp = face_cross_plane(neighbor, plane);
					if (temp > 0 && temp < 3)
					{
						fvisited[neighbor] = true;
						S.push(neighbor);
						if (count == 0)
							break;
					}
				}
			}
			int temp[2];
			temp[0] = NONE; temp[1] = NONE;
			if (!S.empty())
			{
				find_connected_vertices(current, S.top(), temp[0], temp[1]);
				if (count == 0)
				{
					find_connected_vertices(current, S.top(), v0, v1);
				}
			}
			ret.push_back(current);
			if (temp[0] != NONE && temp[1] != NONE)
				retv.push_back(plane_edge_intersection(plane, temp[0], temp[1]));
			if (trace_is_end(current, seed, v0, v1))
			{
				result = true;
				break;
			}
			count++;
		}
		if (result)
		{
			for (int i = 0; i < (int)ret.size(); i++) {
				//_mesh->color_face(ret[i], _mesh->RED);
			}
		}
		else {
			ret.clear(); retv.clear();
		}
	}
	// geodesic
	void SkeletalSegmentation::geodesic_dist_from_single_vertex(int seed, float range) {
		FltVec gd_dist((int)_mesh->v.size(), std::numeric_limits<float>::max());
		Vertex& seedv = _mesh->v[seed];
		std::vector<bool> visited((int)_mesh->v.size(), false);
		ArrayQueue<int> Q((int)_mesh->v.size());
		gd_dist[seed] = 0;
		visited[seed] = true;
		Q.push(seed);
		while (!Q.empty()) {
			int curr = Q.front(); Q.pop();
			Vertex& vtx = _mesh->v[curr];
			if (1/*vertex_distance_s(vtx, seedv) < range*range*/) {
				for (int i = 0; i < _mesh->vvn[curr]; i++) {
					int n = _mesh->vv(curr, i);
					if (!visited[n]) {
						visited[n] = true; Q.push(n);
					}
					//float dist = vertex_distance_s(vtx, _mesh->v[n]);
					float dist = (vtx - _mesh->v[n]).length();
					dist += gd_dist[curr];
					if (dist < gd_dist[n])
						gd_dist[n] = dist;
				}
			}
		}
	}
	float SkeletalSegmentation::geodesic_dist_slow(int seed, int dest) {
		// slow
		FltVec gd_dist((int)_mesh->v.size(), std::numeric_limits<float>::max());
		Vertex& seedv = _mesh->v[seed];
		std::vector<bool> visited((int)_mesh->v.size(), false);
		ArrayQueue<int> Q((int)_mesh->v.size());
		gd_dist[seed] = 0;
		visited[seed] = true;
		Q.push(seed);
		while (!Q.empty()) {
			int curr = Q.front(); Q.pop();
			Vertex& vtx = _mesh->v[curr];
			for (int i = 0; i < _mesh->vvn[curr]; i++) {
				int n = _mesh->vv(curr, i);
				if (!visited[n]) {
					visited[n] = true; Q.push(n);
				}
				float dist = (vtx - _mesh->v[n]).length();

				//float dist = 1;

				if (n == dest) return gd_dist[curr] + dist;
				dist += gd_dist[curr];
				if (dist < gd_dist[n])
					gd_dist[n] = dist;
			}
		}
		return gd_dist[dest];
	}
	float SkeletalSegmentation::geodesic_dist(int seed, int dest) {
		return geodesic_dist_slow(seed, dest);
		if (false) {
			// slow
			FltVec gd_dist((int)_mesh->v.size(), std::numeric_limits<float>::max());
			Vertex& seedv = _mesh->v[seed];
			std::vector<bool> visited((int)_mesh->v.size(), false);
			ArrayQueue<int> Q((int)_mesh->v.size());
			gd_dist[seed] = 0;
			visited[seed] = true;
			Q.push(seed);
			while (!Q.empty()) {
				int curr = Q.front(); Q.pop();
				Vertex& vtx = _mesh->v[curr];
				for (int i = 0; i < _mesh->vvn[curr]; i++) {
					int n = _mesh->vv(curr, i);
					if (!visited[n]) {
						visited[n] = true; Q.push(n);
					}
					float dist = (vtx - _mesh->v[n]).length();

					//float dist = 1;

					if (n == dest) return gd_dist[curr] + dist;
					dist += gd_dist[curr];
					if (dist < gd_dist[n])
						gd_dist[n] = dist;
				}
			}
			return gd_dist[dest];
		}
		else {
			// fast
			Vertex& goal = _mesh->v[dest];
			float ret = 0; int curr = seed;
			std::vector<bool> visited((int)_mesh->v.size(), false);
			visited[seed] = true;
			while (1) {
				bool flag = false; float mindist = DECIMAL_MAX; int next = -1;
				Vertex& vtx = _mesh->v[curr];
				for (int i = 0; i < _mesh->vvn[curr]; ++i) {
					int n = _mesh->vv(curr, i);
					if (!visited[n]) {
						flag = true;  float dist = (_mesh->v[n] - goal).length();
						if (n == dest) return ret + dist;
						if (dist < mindist) {
							mindist = dist; next = n;
						}
					}
				}
				if (!flag)
					return geodesic_dist_slow(seed, dest);
				ret += mindist; curr = next; visited[next] = true;
			}
			return ret;
		}
	}
	void SkeletalSegmentation::geodesic_dist() {
		int maxn = 20; FltVec vvdist((int)_mesh->v.size() * maxn);
		for (int i = 0; i < (int)_mesh->v.size(); i++) {
			for (int j = 0; j < _mesh->vvn[i]; j++) {
				int n = _mesh->vv(i, j);
				vvdist[i*maxn + j] = (_mesh->v[i] - _mesh->v[n]).length();
			}
		}
		for (int seed = 0; seed < (int)_mesh->v.size(); seed++) {
			if (seed % 100 == 0) std::cout << seed << '\n';
			FltVec gd_dist((int)_mesh->v.size(), std::numeric_limits<float>::max());
			Vertex& seedv = _mesh->v[seed];
			std::vector<bool> visited((int)_mesh->v.size(), false);
			ArrayQueue<int> Q((int)_mesh->v.size());
			gd_dist[seed] = 0;
			visited[seed] = true;
			Q.push(seed);
			while (!Q.empty()) {
				int curr = Q.front(); Q.pop();
				Vertex& vtx = _mesh->v[curr];
				for (int i = 0; i < _mesh->vvn[curr]; i++) {
					int n = _mesh->vv(curr, i);
					if (!visited[n]) {
						visited[n] = true; Q.push(n);
					}
					//float dist = vertex_distance_s(vtx, _mesh->v[n]);
					//float dist = sqrt(vertex_distance_s(vtx, _mesh->v[n]));
					float dist = vvdist[curr*maxn + i];
					dist += gd_dist[curr];
					if (dist < gd_dist[n])
						gd_dist[n] = dist;
				}
			}
		}
	}
	// geometric
	float SkeletalSegmentation::point_distance(int p0, int p1)
	{
		return (_mesh->v[p0] - _mesh->v[p1]).length();
		//result = (result >= 0 ? result : -result);
	}
	float SkeletalSegmentation::point_plane_distance(const Plane& plane, const Vertex& p)
	{
		return (p - plane.p).dot(plane.n);
		//result = (result >= 0 ? result : -result);
	}
	int SkeletalSegmentation::face_cross_plane(int face, const Plane& plane)
	{
		int result = 0;
		for (int i = 0; i < 3; i++)
		{
			float temp = point_plane_distance(plane, _mesh->v[_mesh->t[face].v[i]]);
			if (temp > 0)
				result += 1;
			else if (temp == 0)
				std::cout << "Point on plane !\n";
		}
		return result;
	}
	Vertex SkeletalSegmentation::plane_edge_intersection(const Plane& plane, int v0, int v1) {
		const Vertex& vtx0 = _mesh->v[v0], &vtx1 = _mesh->v[v1];
		Vector v = vtx1 - vtx0;
		float cosine = abs(plane.n.dot(v) / v.length());
		float d = abs(plane.n.dot(vtx0 - plane.p));
		float t = d / cosine;
		v.normalize();
		auto ret = vtx0 + v*t;
		return ret;
	}
	Plane SkeletalSegmentation::create_plane(const Vertex& center, const Vertex& ph, const Vertex& pt)
	{
		Plane result;
		//float small = 0;
		//result.p.x = center.x + small; result.p.y = center.y + small; result.p.z = center.z + small;
		result.p = center;
		result.n = ph - pt; result.n.normalize();
		return result;
	}
	Plane SkeletalSegmentation::create_plane_cross(const Vertex& p, const Vertex& na, const Vertex& nb)
	{
		Plane result;
		result.p = p;
		result.n = na.cross(nb);
		result.n.normalize();
		return result;
	}
	Vertex SkeletalSegmentation::triangle_center(int face) {
		Vertex ret(0,0,0);
		for (int i = 0; i < 3; i++)
			ret += _mesh->v[_mesh->t[face].v[i]];
		ret.x /= (float)3; ret.y /= (float)3; ret.z /= (float)3;
		return ret;
	}
	float SkeletalSegmentation::triangle_distance_s(int f0, int f1) {
		return (triangle_center(f0) - triangle_center(f1)).square_length();
	}
	Vertex SkeletalSegmentation::project_to_plane(const Vertex& v, const Plane& plane) {
		Vector d = plane.p - v;
		float dp = d.dot(plane.n);
		if (dp >= 0) {
			return v + plane.n*dp;
		}
		else {
			return v + plane.n*(-dp);
		}
	}
	void SkeletalSegmentation::find_connected_vertices(int f0, int f1, int& v0, int& v1)
	{
		bool flag = false;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (_mesh->t[f0].v[i] == _mesh->t[f1].v[j])
				{
					if (!flag)
					{
						v0 = _mesh->t[f0].v[i];
						flag = true;
						break;
					}
					else
					{
						v1 = _mesh->t[f0].v[i];
						return;
					}
				}
			}
		}
	}
	void SkeletalSegmentation::connect_two_vertex(Plane& plane, int start, int end) {
		//geodesic_dist_from_single_vertex(end, (float)300);
		int curr = start;
		Vertex endv = _mesh->v[end];
		while (1) {
			float mindist = std::numeric_limits<float>::max(); int minidx = 0; float tdist = 0;
			for (int i = 0; i < _mesh->vvn[curr]; i++) {
				int n = _mesh->vv(curr, i);
				float dist = (_mesh->v[n] - _mesh->v[end]).length();
				if (dist < mindist) {
					mindist = dist; minidx = n;
					tdist = (_mesh->v[n] - _mesh->v[end]).length();
				}
			}
			if (tdist < (float)0.1) {
				break;
			}
			curr = minidx;
			_mesh->color_vertex(curr, _preset_color[0]);
		}
	}
	// paint
	void SkeletalSegmentation::paint_ball(const int& index, const VtxVec& ring, float radius, int cuts, const Color& color) {

		if (cuts <= 0 || radius <= (float)0) return;
		Vertex point = _mesh->v[index];
		float mindist = DECIMAL_MAX;
		Vertex center;
		for (int i = 0; i < (int)ring.size(); ++i)
		{
			float dist = (ring[i] - point).square_length();
			if (dist < mindist)
			{
				mindist = dist;
				center = ring[i];
			}	
		}
		paint_ball(center, radius, cuts, color);
	}
	void SkeletalSegmentation::paint_ball(const Vertex& center, float radius, int cuts, const Color& color) {

		if (!PAINT_BALL) return;
		if (cuts <= 0 || radius <= (float)0) return;
		int h = 1 + (1 << cuts);
		int n = 4 * h - 8;
		Vtx2DVec vs(h, VtxVec(n));
		for (int i = 0; i < n; ++i) {
			vs[0][i] = { 0, radius, 0 };
			vs[h - 1][i] = { 0, -radius, 0 };
		}
		float inc = -PI / (h - 1);
		float inc_xz = 2 * PI / n;
		float angle = PI / 2;
		for (int i = 1; i < h - 1; ++i) {
			angle += inc;
			float r = radius * cos(angle);
			float y = radius * sin(angle);
			float angle_xz = 0;
			for (int j = 0; j < n; ++j) {
				float x = r * cos(angle_xz);
				float z = r * sin(angle_xz);
				vs[i][j] = { x, y, z };
				angle_xz += inc_xz;
			}
		}
		std::vector<VertexHandle> handles(h * n);
		for (int i = 0; i < h; ++i) {
			for (int j = 0; j < n; ++j) {
				handles[i * n + j] = _mesh->add_vertex(vs[i][j] + center);
				_mesh->color_vertex(handles[i * n + j], color);
			}
		}
		for (int high = 0; high < h - 1; ++high) {
			int low = high + 1;
			for (int i = 0; i < n; ++i) {
				int j = (i + 1 >= n ? 0 : i + 1);
				_mesh->add_face(handles[low * n + i], handles[high * n + i], handles[high * n + j]);
				_mesh->add_face(handles[low * n + i], handles[high * n + j], handles[low * n + j]);
			}
		}
	}
	void SkeletalSegmentation::paint_ring_vertices(Vector n, const VtxVec& vs, const Color& color) {

		float leno = 2, lenu = 5;
		int size = vs.size();
		auto th = vs[1] - vs[0];
		auto to = th.cross(n);
		to.normalize();
		Vertex center(0, 0, 0);
		for (int i = 0; i < size; ++i) {
			center += vs[i];
		}
		center /= size;
		auto up = n*lenu, dn = n*-lenu;
		bool clockwise = false;
		if ((center - vs[0]).dot(to) > 0) {
			clockwise = true; 
		}
		float leno_dir = (clockwise ? -leno : leno);
		to *= leno_dir;
		VertexHandle head[3] = {
			_mesh->add_vertex(vs[0] + up + to),
			_mesh->add_vertex(vs[0] + to),
			_mesh->add_vertex(vs[0] + dn + to)
		};
		for (int i = 0; i < 3; ++i)
			_mesh->color_vertex(head[i], color);
		VertexHandle curr[3], last[3] = { head[0], head[1], head[2] };
		for (int i = 1; i <= size; ++i) {
			if (i < size) {
				int ni = (i >= size - 1 ? 0 : i + 1);
				auto h = vs[ni] - vs[i];
				auto o = h.cross(n);
				o.normalize();
				o *= leno_dir;
				curr[0] = _mesh->add_vertex(vs[i] + up + o);
				curr[1] = _mesh->add_vertex(vs[i] + o);
				curr[2] = _mesh->add_vertex(vs[i] + dn + o);
				for (int j = 0; j < 3; ++j)
					_mesh->color_vertex(curr[j], color);
			}
			else {
				for (int j = 0; j < 3; ++j)
					curr[j] = head[j];
			}
			if (clockwise) {
				_mesh->add_face(curr[2], last[2], last[1]);
				_mesh->add_face(curr[2], last[1], curr[1]);
				_mesh->add_face(curr[1], last[1], last[0]);
				_mesh->add_face(curr[1], last[0], curr[0]);
			}
			else {
				_mesh->add_face(last[2], curr[2], curr[1]);
				_mesh->add_face(last[2], curr[1], last[1]);
				_mesh->add_face(last[1], curr[1], curr[0]);
				_mesh->add_face(last[1], curr[0], last[0]);
			}
			for (int j = 0; j < 3; ++j)
				last[j] = curr[j];
		}
	}
	// shape diameter function
	void SkeletalSegmentation::CGAL_SDF() {
		//// read input polyhedron
		//Polyhedron_3 polyhedron;
		//BuildTriangle<HalfedgeDS> builder(_mesh->v, _mesh->t);
		//polyhedron.delegate(builder);
		//// initialize indices of vertices, halfedges and faces
		//CGAL::set_halfedgeds_items_id(polyhedron);
		//
		////// create and read Polyhedron
		////Polyhedron _mesh;
		////std::ifstream input("data/cactus.off");
		////if (!input || !(input >> _mesh) || _mesh.empty()) {
		////	std::cerr << "Not a valid off file." << std::endl;
		////	return EXIT_FAILURE;
		////}
		//// create a property-map for segment-ids
		//typedef std::map<Polyhedron_3::Facet_const_handle, std::size_t> Facet_int_map;
		//Facet_int_map internal_segment_map;
		//boost::associative_property_map<Facet_int_map> segment_property_map(internal_segment_map);
		//// calculate SDF values and segment the _mesh using default parameters.
		//std::size_t number_of_segments = CGAL::segmentation_via_sdf_values(polyhedron, segment_property_map);
		//std::cout << "Number of segments: " << number_of_segments << std::endl;
		//// print segment-ids
		//int count = 0;
		//for (Polyhedron_3::Facet_const_iterator facet_it = polyhedron.facets_begin();
		//	facet_it != polyhedron.facets_end(); ++facet_it) {
		//	std::cout << segment_property_map[facet_it] << " ";
		//	_mesh->color_face(count, m_color[segment_property_map[facet_it]]);
		//	count++;
		//}
		//std::cout << std::endl;
	}
	void SkeletalSegmentation::read_sdf_from_file(string in_file, FltVec& ret) {
		// read binary
		std::FILE* fp = std::fopen(in_file.c_str(), "rb");
		std::fread(&ret[0], sizeof(float)*_mesh->t.size(), 1, fp);
		std::fclose(fp);
	}
	void SkeletalSegmentation::color_faces_BFS(int seed, const IntVec& border0, const IntVec& border1, const Color& color) {
		int face_num = (int)_mesh->t.size();
		CharVec labels(face_num, 0);
		for (int i = 0; i < (int)border0.size(); i++) {
			labels[border0[i]] = 1;
		}
		for (int i = 0; i < (int)border1.size(); i++) {
			labels[border1[i]] = 1;
		}
		ArrayQueue<int> Q(face_num);
		labels[seed] = 1; Q.push(seed);
		_mesh->color_face(seed, color);
		while (!Q.empty()) {
			int curr = Q.front(); Q.pop();
			for (int i = 0; i < 3; i++) {
				int n = _mesh->tt(curr, i);
				if (n >= 0 && labels[n] != 1) {
					labels[n] = 1; Q.push(n);
					_mesh->color_face(n, color);
				}
			}
		}
	}
	void SkeletalSegmentation::log_normalize_probability_matrix(std::vector<std::vector<sdfdecimal>>& probabilities) {
		const sdfdecimal epsilon = 5e-6;
		for (std::vector<std::vector<sdfdecimal> >::iterator it_i = probabilities.begin();
			it_i != probabilities.end(); ++it_i) {
			for (std::vector<sdfdecimal>::iterator it = it_i->begin(); it != it_i->end(); ++it) {
				sdfdecimal probability = (std::max)(*it,
					epsilon); // give every facet a little probability to be in any cluster
				probability = -log(probability);
				*it = (std::max)(probability, std::numeric_limits<sdfdecimal>::epsilon());
				// zero values are not accepted in max-flow as weights for edges which connects some vertex with Source or Sink (in boost::boykov..)
			}
		}
	}
	void SkeletalSegmentation::calculate_dihedral_angles(IntVec& corrs, SzVec& labels, Gmm2DVec& probability_matrix,
		sdfdecimal smooth_lambda, SzPrVec& a_edges, GmmVec& edge_weights) {
		//// associate each facet with an id
		//// important note: ids should be compatible with iteration order of facets:
		//// [0 <- facet_begin(),...., size_of_facets() -1 <- facet_end()]
		//// Why ? it is send to graph cut algorithm where other data associated with facets are also sorted according to iteration order.
		//std::map<face_descriptor, std::size_t> facet_index_map;
		//std::size_t facet_index = 0;
		//face_iterator facet_it, fend;
		//for (boost::tie(facet_it, fend) = faces(_mesh);
		//	facet_it != fend;
		//	++facet_it, ++facet_index) {
		//	facet_index_map[*facet_it] = facet_index;
		//}
		const sdfdecimal epsilon = 5e-6;
		//// add arm function
		//const int max_depth = 30;
		//auto func = [](const int valid_id, int& sp_id, SzVec& labs, Gmm2DVec& mat) mutable -> void {
		//	labs.push_back(labs[valid_id]);
		//	for (int j = 0; j < (int)mat.size(); ++j) {
		//		mat[j].push_back(mat[j][valid_id]);
		//	}
		//	sp_id = (int)(labs.size() - 1);
		//};
		//IntVec depth(_mesh->t.size(), 0);
		//ArrayQueue<int> Q((int)_mesh->t.size());
		//BoolVec visited(_mesh->t.size(), true), changed(_mesh->t.size(), false);
		//for (int i = 0; i < (int)_mesh->t.size(); ++i) {
		//	if (corrs[i] == -2) {
		//		visited[i] = false;
		//		for (int j = 0; j < 3; ++j) {
		//			auto n = _mesh->tt(i, j);
		//			if (n >= 0 && !changed[n] && corrs[n] >= 0) {
		//				func(corrs[n], corrs[i], labels, probability_matrix);
		//				changed[i] = visited[i] = true; depth[i] = 0; Q.push(i);
		//				break;
		//			}
		//		}
		//	}
		//}
		//while (!Q.empty()) {
		//	auto curr = Q.front(); Q.pop();
		//	for (int j = 0; j < 3; ++j) {
		//		auto n = _mesh->tt(curr, j);
		//		if (n >= 0 && !visited[n] && corrs[n] == -2) {
		//			func(corrs[curr], corrs[n], labels, probability_matrix);
		//			visited[n] = true;
		//			if ((depth[n] = depth[curr] + 1) <= max_depth)
		//				Q.push(n);
		//		}
		//	}
		//}
		BoolVec border(_mesh->t.size(), false);
		for (int i = 0; i < (int)_mesh->t.size(); ++i) {
			if (corrs[i] >= 0) {
				for (int j = 0; j < 3; ++j) {
					auto n = _mesh->tt(i, j);
					if (n >= 0 && corrs[n] == -2) {
						for (int k = 0; k < 3; ++k) {
							auto ni = _mesh->tt(i, k);
							if (ni >= 0)
								border[ni] = true;
						}
						border[i] = true;
						break;
					}
				}
			}
		}
		// edges and their weights. pair<std::size_t, std::size_t> stores facet-id pairs (see above) (may be using boost::tuple can be more suitable)
		IntVec need_adjust; sdfdecimal max_angle = 0;
		for (int i = 0; i < (int)_mesh->e.size(); ++i) {
			if (_mesh->et(i, 0) >= 0 && _mesh->et(i, 1) >= 0) {
				int index_f1 = corrs[_mesh->et(i, 0)];
				int index_f2 = corrs[_mesh->et(i, 1)];
				if (index_f1 >= 0 && index_f2 >= 0) {
					// if edge does not contain two neighbor facets then do not include it in graph-cut
					// weight
					sdfdecimal angle = calculate_dihedral_angle_of_edge(i);
					angle = (std::max)(angle, epsilon);
					angle = -log(angle);
					angle *= smooth_lambda;
					if (angle > max_angle)
						max_angle = angle;
					if (border[_mesh->et(i, 0)] || border[_mesh->et(i, 1)])
						need_adjust.push_back((int)edge_weights.size());
					edge_weights.push_back(angle);
					// edge
					a_edges.push_back(std::make_pair(index_f1, index_f2));
				}
			}
		}
		for (int i = 0; i < (int)need_adjust.size(); ++i) {
			edge_weights[need_adjust[i]] = max_angle * 3;
		}
	}
	sdfdecimal SkeletalSegmentation::calculate_dihedral_angle_of_edge(int edge) {
		// As far as I check: if, say, dihedral angle is 5, this returns 175,
		// if dihedral angle is -5, this returns -175.
		// Another words this function returns angle between planes.
		sdfdecimal n_angle = (sdfdecimal)_mesh->triangle_dihedral_angle(edge);
		n_angle /= (sdfdecimal)180.0;
		bool concave = n_angle > 0;
		sdfdecimal angle = 1 + ((concave ? -1 : +1) * n_angle);

		if (!concave) {
			angle *= CONVEX_FACTOR;
		}
		return angle;
	}
	void SkeletalSegmentation::do_graph_cut(IntVec& corrs, Gaussian_Mixture_Model& gmm, std::vector<std::size_t>& labels) {
		sdfdecimal smoothing_lambda = (sdfdecimal)SMOOTHING_LAMBDA;
		smoothing_lambda = (std::max)(0.0, smoothing_lambda); // min zero
		smoothing_lambda *= SMOOTHING_LAMBDA_MULTIPLIER; // scale it into meaningful range for graph-cut
		// fill center id
		gmm.fill_with_center_ids(labels);
		// fill matrix
		std::vector<std::vector<sdfdecimal>> probability_matrix;
		gmm.fill_with_probabilities(probability_matrix);
		log_normalize_probability_matrix(probability_matrix);
		// calculating edge weights
		std::vector<std::pair<std::size_t, std::size_t>> edges;
		std::vector<sdfdecimal> edge_weights;
		calculate_dihedral_angles(corrs, labels, probability_matrix, smoothing_lambda, edges, edge_weights);
		// apply graph cut
		Alpha_expansion_graph_cut_boykov_kolmogorov()(edges, edge_weights, probability_matrix, labels);
		//std::vector<std::size_t>::iterator label_it = labels.begin();
		//face_iterator facet_it, fend;
		//for (boost::tie(facet_it, fend) = faces(_mesh);
		//	facet_it != fend;
		//	++facet_it, ++label_it) {
		//	segment_pmap[*facet_it] = *label_it; // fill with cluster-ids
		//}
		//if (clusters_to_segments) {
		//	// assign a segment id for each facet
		//	std::size_t number_of_segments = assign_segments(number_of_centers, sdf_pmap,
		//		segment_pmap);
		//	return number_of_segments;
		//}
		//return number_of_centers;
	}
	void SkeletalSegmentation::do_GMM_arm(CharVec& labels, FltVec& sdf) {
		// GMM
		int part_idxs[2] = { 1, 3 };
		int cen_num = 2, real_cen_num = 2;
		int face_num = (int)_mesh->t.size();
		GmmVec centers(real_cen_num, 0); IntVec counts(real_cen_num, 0);
		GmmVec gmm_sdf; IntVec corrs(_mesh->t.size());
		for (int i = 0; i < face_num; i++) {
			corrs[i] = -1;
			for (int j = 0; j < cen_num; ++j) {
				if (labels[i] == part_idxs[j]) {
					gmm_sdf.push_back((sdfdecimal)sdf[i]);
					counts[j]++; centers[j] += (sdfdecimal)sdf[i];
					corrs[i] = (int)(gmm_sdf.size() - 1);
					break;
				}
			}
		}
		for (int i = 0; i < cen_num; i++) {
			centers[i] /= counts[i];
		}
		//centers[2] = (centers[0] + centers[1]) / 2;
		//K_means_clustering kmeans(real_cen_num, gmm_sdf);
		//std::vector<sdfdecimal> kmeans_centers;
		//kmeans.fill_with_centers(kmeans_centers);
		Gaussian_Mixture_Model gmm(real_cen_num, centers, gmm_sdf);
		std::vector<std::size_t> gmm_result;
		do_graph_cut(corrs, gmm, gmm_result);
		int count = 0;
		for (int i = 0; i < face_num; i++) {
			for (int j = 0; j < cen_num; ++j) {
				if (labels[i] == part_idxs[j]) {
					if (j == 1 && gmm_result[count] != 2) {
						labels[i] = part_idxs[gmm_result[count]];
					}
					++count; break;
				}
			}
		}
	}
	void SkeletalSegmentation::do_GMM_upbody(CharVec& labels, FltVec& sdf, IntVec& ret) {
		// GMM
		int body_id = 3, arm_id = 1, cen_num = 2;
		int face_num = (int)_mesh->t.size();
		GmmVec centers(cen_num, 0); IntVec counts(cen_num, 0);
		GmmVec gmm_sdf; IntVec corrs(_mesh->t.size());
		float min_sdf = std::numeric_limits<float>::max();
		float	max_sdf = std::numeric_limits<float>::min();
		for (int i = 0; i < face_num; ++i) {
			corrs[i] = -1;
			if (labels[i] == body_id) {
				gmm_sdf.push_back((sdfdecimal)sdf[i]);
				if (sdf[i] > max_sdf) max_sdf = sdf[i];
				if (sdf[i] < min_sdf) min_sdf = sdf[i];
				corrs[i] = (int)(gmm_sdf.size() - 1);
			}
			else if (labels[i] == arm_id) {
				corrs[i] = -2;
			}
		}
		std::vector<sdfdecimal> fences(cen_num + 1);
		float step = (max_sdf - min_sdf) / cen_num;
		fences[0] = (sdfdecimal)min_sdf; fences[cen_num] = (sdfdecimal)max_sdf;
		for (int i = 1; i < cen_num; ++i) {
			fences[i] = (sdfdecimal)(min_sdf + step * i);
		}
		for (int i = 0; i < (int)gmm_sdf.size(); ++i) {
			for (int j = 0; j < cen_num; ++j) {
				if (gmm_sdf[i] >= fences[j] && gmm_sdf[i] <= fences[j + 1]) {
					counts[j]++; centers[j] += gmm_sdf[i];
					break;
				}
			}
		}
		for (int i = 0; i < cen_num; ++i) {
			centers[i] /= counts[i];
		}
		K_means_clustering kmeans(cen_num, gmm_sdf);
		std::vector<sdfdecimal> kmeans_centers;
		kmeans.fill_with_centers(kmeans_centers);
		Gaussian_Mixture_Model gmm(cen_num, kmeans_centers, gmm_sdf);
		std::vector<std::size_t> gmm_result;
		do_graph_cut(corrs, gmm, gmm_result);
		int count = 0;
		for (int i = 0; i < face_num; ++i) {
			if (labels[i] == body_id) {
				if (gmm_result[count] == 0)
					ret.push_back(i);
				++count;
			}
		}
	}
	void SkeletalSegmentation::do_GMM_dnbody(CharVec& labels, FltVec& sdf, IntVec& ret) {
		// GMM
		int body_id = 3, cen_num = 2;
		int face_num = (int)_mesh->t.size();
		GmmVec centers(cen_num, 0); IntVec counts(cen_num, 0);
		GmmVec gmm_sdf; IntVec corrs(_mesh->t.size());
		float min_sdf = std::numeric_limits<float>::max();
		float	max_sdf = std::numeric_limits<float>::min();
		for (int i = 0; i < face_num; ++i) {
			corrs[i] = -1;
			if (labels[i] == body_id) {
				gmm_sdf.push_back((sdfdecimal)sdf[i]);
				if (sdf[i] > max_sdf) max_sdf = sdf[i];
				if (sdf[i] < min_sdf) min_sdf = sdf[i];
				corrs[i] = (int)(gmm_sdf.size() - 1);
			}
		}
		std::vector<sdfdecimal> fences(cen_num + 1);
		float step = (max_sdf - min_sdf) / cen_num;
		fences[0] = (sdfdecimal)min_sdf; fences[cen_num] = (sdfdecimal)max_sdf;
		for (int i = 1; i < cen_num; ++i) {
			fences[i] = (sdfdecimal)(min_sdf + step * i);
		}
		for (int i = 0; i < (int)gmm_sdf.size(); ++i) {
			for (int j = 0; j < cen_num; ++j) {
				if (gmm_sdf[i] >= fences[j] && gmm_sdf[i] <= fences[j + 1]) {
					counts[j]++; centers[j] += gmm_sdf[i];
					break;
				}
			}
		}
		for (int i = 0; i < cen_num; ++i) {
			centers[i] /= counts[i];
		}
		K_means_clustering kmeans(cen_num, gmm_sdf);
		std::vector<sdfdecimal> kmeans_centers;
		kmeans.fill_with_centers(kmeans_centers);
		Gaussian_Mixture_Model gmm(cen_num, kmeans_centers, gmm_sdf);
		std::vector<std::size_t> gmm_result;
		do_graph_cut(corrs, gmm, gmm_result);
		int count = 0;
		for (int i = 0; i < face_num; ++i) {
			if (labels[i] == body_id) {
				if (gmm_result[count] == 1)
					ret.push_back(i);
				++count;
			}
		}
	}
	void SkeletalSegmentation::do_GMM_all(CharVec& labels, FltVec& sdf) {
		// GMM
		int cen_num = 8;
		int face_num = (int)_mesh->t.size();
		GmmVec centers(cen_num, 0); IntVec counts(cen_num, 0);
		GmmVec gmm_sdf; IntVec corrs(_mesh->t.size());
		float min_sdf = std::numeric_limits<float>::max(), max_sdf = std::numeric_limits<float>::min();
		for (int i = 0; i < face_num; ++i) {
			gmm_sdf.push_back((sdfdecimal)sdf[i]);
			if (sdf[i] > max_sdf) max_sdf = sdf[i];
			if (sdf[i] < min_sdf) min_sdf = sdf[i];
			corrs[i] = i;
		}
		std::vector<sdfdecimal> fences(cen_num + 1); float step = (max_sdf - min_sdf) / cen_num;
		fences[0] = (sdfdecimal)min_sdf; fences[cen_num] = (sdfdecimal)max_sdf;
		for (int i = 1; i < cen_num; ++i) {
			fences[i] = (sdfdecimal)(min_sdf + step * i);
		}
		for (int i = 0; i < (int)gmm_sdf.size(); ++i) {
			for (int j = 0; j < cen_num; ++j) {
				if (gmm_sdf[i] >= fences[j] && gmm_sdf[i] <= fences[j + 1]) {
					counts[j]++; centers[j] += gmm_sdf[i];
					break;
				}
			}
		}
		for (int i = 0; i < cen_num; ++i) {
			centers[i] /= counts[i];
		}
		K_means_clustering kmeans(cen_num, gmm_sdf);
		std::vector<sdfdecimal> kmeans_centers;
		kmeans.fill_with_centers(kmeans_centers);
		Gaussian_Mixture_Model gmm(cen_num, kmeans_centers, gmm_sdf);
		std::vector<std::size_t> gmm_result;
		do_graph_cut(corrs, gmm, gmm_result);
		int count = 0;
		for (int i = 0; i < face_num; ++i) {
			labels[i] = (char)gmm_result[i];
		}
	}
	void SkeletalSegmentation::do_K_means() {
		int max_it = 100;
		//// k-means
		//for (int idx = 0; idx < max_it; ++idx) {
		//	float centers[2] = { 0 }; int counts[2] = { 0 };
		//	for (int i = 0; i < face_num; i++) {
		//		if (labels[i] == 5) {
		//			counts[1]++; centers[1] += sdf[i];
		//		}
		//		if (labels[i] == 1 || labels[i] == 2) {
		//			counts[0]++; centers[0] += sdf[i];
		//		}
		//	}
		//	centers[0] /= counts[0]; centers[1] /= counts[1];
		//	for (int i = 0; i < face_num; i++) {
		//		if (labels[i] == 1 || labels[i] == 2 || labels[i] == 5) {
		//			float diff[2] = { std::abs(sdf[i] - centers[0]), std::abs(sdf[i] - centers[1]) };
		//			if (labels[i] == 5 && diff[0] < diff[1]) {
		//				labels[i] = 1;
		//			}
		//			if ((labels[i] == 1 || labels[i] == 2) && diff[1] < diff[0]) {
		//				labels[i] = 5;
		//			}
		//		}
		//	}
		//}
	}

	// -------------------- Class Reshape --------------------

	// main
	void Reshape::deform_to_seat() {

		// init
		deform_to_sitting_pose();
		_mesh->write_to_ply(MESH_PATH + "_sit0_" + _mesh->mesh_file_name + ".ply");
		//return;

		// chair
		Chair& chair = _chair;
		string chair_path = MESH_PATH + "chair_2/";
		StrVec seat_name = { "seat.obj" };
		StrVec back_name = { "back.obj" };
		StrVec arm_names = { "arm1.obj", "arm2.obj" };
		StrVec leg_names = { "leg1.obj", "leg2.obj", "leg3.obj", "leg4.obj" };
		init_chair(chair_path, seat_name, back_name, arm_names, leg_names, { 28, 30, 25 });

		// rotate chair
		Vertex seat_cen = find_centroid(chair.seat.v);
		Vertex back_cen = find_centroid(chair.back.v);
		Vertex seat_contact = find_closest(chair.seat.v, back_cen);
		Vector chair_z = seat_cen - seat_contact; chair_z.y = 0; chair_z.normalize();
		for (int i = 0; i < chair.p.size(); ++i)
			chair.p[i]->rotate(Quaternion(chair_z, { 0, 0, -1 }));
		seat_cen = find_centroid(chair.seat.v);
		back_cen = find_centroid(chair.back.v);
		seat_contact = find_closest(chair.seat.v, back_cen);

		// compute ground
		Plane ground;
		{
			float ymin = DECIMAL_MAX;
			for (const auto& m : chair.legs) {
				for (auto v : m.v) {
					if (v.y < ymin) ymin = v.y;
				}
			}
			ground = { { 0, ymin, 0 }, { 0, 1, 0 } };
		}

		// compute chair seat width & depth & height
		Vector seat_parm, seat_max, seat_min;
		{
			float xmin = DECIMAL_MAX, xmax = DECIMAL_MIN;
			float ymin = DECIMAL_MAX, ymax = DECIMAL_MIN;
			float zmin = DECIMAL_MAX, zmax = DECIMAL_MIN;
			for (auto v : chair.seat.v) {
				if (v.x < xmin) xmin = v.x; if (v.x > xmax) xmax = v.x;
				if (v.y < ymin) ymin = v.y; if (v.y > ymax) ymax = v.y;
				if (v.z < zmin) zmin = v.z; if (v.z > zmax) zmax = v.z;
			}
			seat_parm = { xmax - xmin, ymax - ground.p.y, zmax - zmin };
			seat_max = { xmax, ymax, zmax };
			seat_min = { xmin, ymin, zmin };
		}

		// compute chair back parameters
		Vector chair_back_max, chair_back_min;
		{
			float xmin = DECIMAL_MAX, xmax = DECIMAL_MIN;
			float ymin = DECIMAL_MAX, ymax = DECIMAL_MIN;
			float zmin = DECIMAL_MAX, zmax = DECIMAL_MIN;
			for (auto v : chair.back.v) {
				if (v.x < xmin) xmin = v.x; if (v.x > xmax) xmax = v.x;
				if (v.y < ymin) ymin = v.y; if (v.y > ymax) ymax = v.y;
				if (v.z < zmin) zmin = v.z; if (v.z > zmax) zmax = v.z;
			}
			chair_back_max = { xmax, ymax, zmax };
			chair_back_min = { xmin, ymin, zmin };
		}

		// find man skeleton point
		auto find_man_ske_pt = [this](int part, int cut) {
			Vertex ret(0, 0, 0);
			for (auto t : _slices_face[part][this->_cuts[part][cut]])
				ret += triangle_center(t);
			return ret / _slices_face[part][this->_cuts[part][cut]].size();
		};

		// rotate man
		Vertex knee_cen[2] = { find_man_ske_pt(PartType::p_leg0, LegCut::c_knee),
			find_man_ske_pt(PartType::p_leg1, LegCut::c_knee) };
		Vertex crotch = _mesh->v[_landmarks.crotch];
		crotch = _mesh->joints[_mesh->j_hips];
		knee_cen[0] = _mesh->joints[_mesh->j_lknee];
		knee_cen[1] = _mesh->joints[_mesh->j_rknee];
		{
			Vector man_z = (knee_cen[0] - crotch).normalized() + (knee_cen[1] - crotch).normalized();
			man_z.y = 0; man_z.normalize();
			_mesh->rotate(Quaternion(man_z, { 0, 0, -1 }));
			crotch = _mesh->v[_landmarks.crotch];
			knee_cen[0] = find_man_ske_pt(PartType::p_leg0, LegCut::c_knee);
			knee_cen[1] = find_man_ske_pt(PartType::p_leg1, LegCut::c_knee);
		}
		
		// find knee points
		Vertex knee_inside[2];
		{
			float zmaxs[2] = { DECIMAL_MIN, DECIMAL_MIN };
			for (int i = 0; i < 2; ++i) {
				float d = abs((find_closest(_mesh->v, knee_cen[i]) - knee_cen[i]).y);
				int jn = (i == 0 ? _mesh->j_lknee : _mesh->j_rknee);
				int jn2 = (i == 0 ? _mesh->j_rknee : _mesh->j_lknee);
				knee_inside[i] = _mesh->joints[jn] + Vector(0, -d, d);
			}
		}

		// find foot bottom point
		Vertex foot_bottom[2];
		{
			float ymins[2] = { DECIMAL_MAX, DECIMAL_MAX };
			for (int i = 0; i < _mesh->t.size(); ++i) {
				if (_part_labels[i] == DetailPartType::foot0) {
					Vertex vtx = triangle_center(i);
					if (vtx.y < ymins[0]) { ymins[0] = vtx.y; foot_bottom[0] = vtx; }
				}
				if (_part_labels[i] == DetailPartType::foot1) {
					Vertex vtx = triangle_center(i);
					if (vtx.y < ymins[1]) { ymins[1] = vtx.y; foot_bottom[1] = vtx; }
				}
			}
		}

		// find hip points
		const float ratio = 0.1;
		Vertex hip_tail;
		{
			Vertex hip_joint = _mesh->joints[Mesh::j_hips];
			Plane hip_FB(hip_joint, Vector(0, -1, 1));
			Plane hip_LR(hip_joint, { 1, 0, 0 });
			IntVec hip_tail_candidates;
			for (int i = 0; i < _mesh->t.size(); ++i) {
				if (_part_labels[i] == DetailPartType::downbody
					&& point_plane_distance(hip_FB, _mesh->triangle_center(i)) > 0) {
					int flag = face_cross_plane(i, hip_LR);
					if (flag > 0 && flag < 3)
						hip_tail_candidates.push_back(i);
				}
			}
			float ymin = DECIMAL_MAX, ymax = DECIMAL_MIN;
			float zmin = DECIMAL_MAX, zmax = DECIMAL_MIN;
			for (auto i : hip_tail_candidates) {
				Vertex mid(0, 0, 0);
				int cnt = 0;
				for (int j = 0; j < 3; ++j) {
					int v0 = _mesh->t[i].v[j];
					int v1 = _mesh->t[i].v[(j + 1) % 3];
					float temp0 = point_plane_distance(hip_LR, _mesh->v[v0]);
					float temp1 = point_plane_distance(hip_LR, _mesh->v[v1]);
					if (temp0 > 0 && temp1 < 0 || temp0 < 0 && temp1 > 0) {
						mid += plane_edge_intersection(hip_LR, v0, v1);
						++cnt;
					}
				}
				mid /= cnt;
				if (mid.y > ymax) ymax = mid.y;
				if (mid.y < ymin) ymin = mid.y;
				if (mid.z > zmax) zmax = mid.z;
				if (mid.z < zmin) zmin = mid.z;
			}
			hip_tail = { hip_joint.x, ymin + (ymax - ymin)*ratio, zmax - (zmax - zmin)*ratio };
		}

		// ------------------------------ start deform ------------------------------

		// if shank length < seat height
		float sit_depth = 0;
		{
			Vector seat_dir = seat_cen - seat_contact;
			Vector back_dir = back_cen - seat_contact;
			Vector chair_in = (seat_dir.normalized() + back_dir.normalized()).normalized();
			Vertex knee_inside_mid = (knee_inside[0] + knee_inside[1]) / 2;
			Vertex foot_bottom_mid = (foot_bottom[0] + foot_bottom[1]) / 2;
			sit_depth = abs((hip_tail - knee_inside_mid).z);
			Vector trans = { 0, 0, 0 };
			if (knee_inside_mid.y - foot_bottom_mid.y < seat_parm.y) {
				// if seat depth < man leg length
				if (hip_tail.z - knee_inside_mid.z > seat_parm.z) {
					// man's hip tail point move to chair's seat-back contact point
					trans = seat_contact + chair_in * 10 - hip_tail;
				}
				else {
					// man's knee inside point move to chair's seat-front point
					Vertex seat_front_pt = seat_contact - Vector(0, 0, seat_parm.z);
					trans = seat_front_pt + chair_in * 10 - knee_inside_mid;
				}
			}
			else {
				// lift shank
				Vertex hip_joint_mid = (_mesh->joints[_mesh->j_lhip] + _mesh->joints[_mesh->j_rhip]) / 2;
				float diff = knee_inside_mid.y - foot_bottom_mid.y - seat_parm.y;
				if (diff > 0) {
					std::cout << "shank lifted" << std::endl;
					Vector from = knee_inside_mid - hip_joint_mid; from.x = 0;
					Vector to = from; to.y += diff; to.z = from.square_length() - to.y*to.y;
					if (to.z >= 0) {
						to.z = std::sqrt(to.z); to.z = -abs(to.z);
						sit_depth = abs((hip_tail - (hip_joint_mid + to)).z);
						Quaternion q(from, to);
						_mesh->apply_rotation_with_cor(_mesh->j_lknee, q);
						_mesh->apply_rotation_with_cor(_mesh->j_rknee, q);
						for (int i = 0; i < 2; ++i) {
							int jn = i == 0 ? _mesh->j_lankle : _mesh->j_rankle;
							int jn2 = i == 0 ? _mesh->j_lknee : _mesh->j_rknee;
							Vector dir = (_mesh->joints[jn] - _mesh->joints[jn2]).normalized();
							_mesh->apply_rotation_with_cor(_mesh->j_lknee, Quaternion(dir, { 0, -1, 0 }));
						}
					}
				}
				// man's hip tail point move to chair's seat-back contact point
				Vertex seat_front_pt = seat_contact - Vector(0, 0, seat_parm.z);
				trans = seat_front_pt + Vector(0, 0, sit_depth) + chair_in * 10 - hip_tail;
			}
			_mesh->translate(trans);
			hip_tail += trans;
		}

		// check if man width > seat width
		{
			// update knee points
			Vertex knee_LR_out[2], knee_LR_in[2];
			float zmaxs[2] = { DECIMAL_MIN, DECIMAL_MIN };
			for (int i = 0; i < 2; ++i) {
				float dy = abs((find_closest(_mesh->v, knee_cen[i]) - knee_cen[i]).y);
				int jn = (i == 0 ? _mesh->j_lknee : _mesh->j_rknee);
				int jn2 = (i == 0 ? _mesh->j_rknee : _mesh->j_lknee);
				knee_inside[i] = _mesh->joints[jn] + Vector(0, -dy, dy);
				Vector dir = _mesh->joints[jn] - _mesh->joints[jn2];
				knee_LR_out[i] = _mesh->joints[jn] + Vector(dir.x, 0, 0).normalized();
				knee_LR_in[i] = _mesh->joints[jn] + Vector(-dir.x, 0, 0).normalized();
			}

			// if man width > seat width
			if (abs(knee_LR_out[0].x - knee_LR_out[1].x) > seat_parm.x) {

			}
		}

		_mesh->write_to_ply(MESH_PATH + "_sit1_" + _mesh->mesh_file_name + ".ply");

		// back
		Vector chair_back_vec = { 0, 0, 0 };
		_mesh->joints[_mesh->j_hips] = (_mesh->joints[_mesh->j_lhip] + _mesh->joints[_mesh->j_rhip]) / 2;
		if (sit_depth >= (seat_parm.z * (float)0.75)) {

			Vector seat_dir = (seat_cen - seat_contact).normalized();
			Vector back_dir = (back_cen - seat_contact).normalized();
			Vector chair_x = seat_dir.cross(back_dir).normalized();
			Plane chair_yz = { back_cen, chair_x };
			Plane man_yz = { _mesh->joints[_mesh->j_back], chair_x };

			// find man back points
			Vertex waist_back_pt, shoulders_back_pt;
			{
				IntVec ts = find_plane_intersecting_faces(*_mesh, man_yz);
				float dymin[2] = { DECIMAL_MAX, DECIMAL_MAX };
				for (int i : ts) {
					Vertex vtx = _mesh->triangle_center(i);
					char p = _part_labels[i];
					bool is_body = (p == DetailPartType::downbody || p == DetailPartType::midbody || p == DetailPartType::upbody);
					if ((vtx - _mesh->joints[_mesh->j_back]).z > 0 && is_body) {
						float d = abs(vtx.y - _mesh->joints[_mesh->j_back].y);
						if (d < dymin[0]) { dymin[0] = d; waist_back_pt = vtx; }
						d = abs(vtx.y - _mesh->joints[_mesh->j_shoulders].y);
						if (d < dymin[1]) { dymin[1] = d; shoulders_back_pt = vtx; }
					}
				}
			}

			// find chair intersect point
			Vertex rot_cen = _mesh->joints[_mesh->j_hips];
			rot_cen.x = 0;
			waist_back_pt.x = 0;
			shoulders_back_pt.x = 0;
			Vertex back_contact, waist_touch_chair_pt, shoulders_touch_chair_pt;
			{
				IntVec ts = find_plane_intersecting_faces(chair.back, chair_yz);
				float zmin[3] = { DECIMAL_MAX, DECIMAL_MAX, DECIMAL_MAX };
				const int loop_times = 100;
				for (int i : ts) {
					for (int j = 0; j < 3; ++j) {
						Vertex v0 = chair.back.v[chair.back.t[i].v[j]];
						if (v0.z < zmin[0]) { zmin[0] = v0.z; back_contact = v0; }
					}
				}
				for (int it = 0; it < loop_times; ++it) {
					float radius[2] = { (waist_back_pt - rot_cen).length(), (shoulders_back_pt - rot_cen).length() };
					zmin[1] = zmin[2] = DECIMAL_MAX;
					bool flag = false;
					for (int i : ts) {
						for (int j = 0; j < 3; ++j) {
							Vertex v0 = chair.back.v[chair.back.t[i].v[j]];
							Vertex v1 = chair.back.v[chair.back.t[i].v[(j + 1) % 3]];
							v0.x = 0; v1.x = 0;
							for (int k = 0; k < 2; ++k) {
								float d0 = (v0 - rot_cen).length() - radius[k];
								float d1 = (v1 - rot_cen).length() - radius[k];
								if (d0 < 0 && d1 > 0 || d0 > 0 && d1 < 0) {
									Vertex temp = circle_line_intersect(rot_cen, radius[k], v0, v1);
									if (temp.z < zmin[k + 1]) {
										if (k == 0 && temp.y > _mesh->joints[_mesh->j_hips].y) {
											zmin[k + 1] = temp.z; temp.x = 0;
											waist_touch_chair_pt = temp;
										}
										if (k == 1 && temp.y > _mesh->joints[_mesh->j_hips].y) {
											zmin[k + 1] = temp.z; temp.x = 0;
											shoulders_touch_chair_pt = temp; flag = true;
										}
									}
								}
							}
						}
					}
					if (flag) break;
					shoulders_back_pt -= ((shoulders_back_pt - hip_tail) / (float)loop_times);
					std::cout << "loop for " << it << " times" << std::endl;
				}
			}

			std::cout << shoulders_touch_chair_pt.x << ' ' << shoulders_touch_chair_pt.y << ' ' << shoulders_touch_chair_pt.z << std::endl;
			std::cout << _mesh->joints[_mesh->j_back].x << ' ' << _mesh->joints[_mesh->j_back].y << ' ' << _mesh->joints[_mesh->j_back].z << std::endl;
			
			// estimate the shape of chair back
			if (waist_touch_chair_pt.is_nonzero()) std::cout << "found waist_touch_chair_pt" << std::endl;
			chair_back_vec = (shoulders_touch_chair_pt - back_contact);
			Vector chair_back_upper = shoulders_touch_chair_pt - waist_touch_chair_pt;
			float cosine = (back_contact - waist_touch_chair_pt).normalized().dot(chair_back_upper.normalized());
			if (chair_back_upper.y < 0) cosine = -1;
			if (cosine > (float)-0.82) { // if chair back is not straight
				std::cout << "chair back is not straight, cosine : " << cosine << std::endl;
			}
			else { // chair back is straight
				std::cout << "chair back is straight" << std::endl;
				if (shoulders_touch_chair_pt.is_nonzero() || waist_touch_chair_pt.is_nonzero()) {
					Vector from, to;
					if (shoulders_touch_chair_pt.is_nonzero()) {
						std::cout << "shoulders touches chair" << std::endl;
						from = shoulders_touch_chair_pt - rot_cen;
						to = shoulders_back_pt - rot_cen;
					}
					else {
						std::cout << "waist touches chair" << std::endl;
						from = waist_touch_chair_pt - rot_cen;
						to = waist_back_pt - rot_cen;
					}
					from.x = 0; to.x = 0;
					Quaternion q(from, to);
					Vertex hips_pos = _mesh->joints[_mesh->j_hips];
					_mesh->apply_rotation_with_cor(_mesh->j_lknee, q);
					_mesh->apply_rotation_with_cor(_mesh->j_rknee, q);
					_mesh->rotate(q.inverse());
					_mesh->translate(hips_pos - _mesh->joints[_mesh->j_hips]);
				}
				else std::cout << "chair back is too short" << std::endl;
			}
		}
		else std::cout << "sit depth too short" << std::endl;

		_mesh->write_to_ply(MESH_PATH + "_sit2_" + _mesh->mesh_file_name + ".ply");

		// arm
		Vertex p0, p1;
		{
			// elbow point
			float elbow_radius[2] = { 0, 0 };
			{
				Vertex vtx = find_closest(_mesh->v, _mesh->joints[_mesh->j_lelbow]);
				elbow_radius[0] = (vtx - _mesh->joints[_mesh->j_lelbow]).length();
				vtx = find_closest(_mesh->v, _mesh->joints[_mesh->j_relbow]);
				elbow_radius[1] = (vtx - _mesh->joints[_mesh->j_relbow]).length();
			}

			// deform
			for (int idx = 0; idx < chair.arm_num; ++idx) {
				const Mesh& armrest = chair.arms[idx];
				Vertex armrest_cen = find_centroid(armrest.v);
				float armrest_zmin = DECIMAL_MAX, armrest_zmax = DECIMAL_MIN;
				for (int i = 0; i < armrest.v.size(); ++i) {
					if (armrest.v[i].z > armrest_zmax) armrest_zmax = armrest.v[i].z;
					if (armrest.v[i].z < armrest_zmin) armrest_zmin = armrest.v[i].z;
				}
				float dl = (armrest_cen - _mesh->joints[_mesh->j_lelbow]).square_length();
				float dr = (armrest_cen - _mesh->joints[_mesh->j_relbow]).square_length();
				Mesh::JOINT_NAME elbow, wrist, shoulder;
				float elbow_lift_offset;
				Vertex knee_joint;
				Vector x_dir;
				if (dl < dr) {
					wrist = _mesh->j_lhand;
					elbow = _mesh->j_lelbow;
					shoulder = _mesh->j_lshoulder;
					elbow_lift_offset = elbow_radius[0];
					knee_joint = _mesh->joints[_mesh->j_lknee];
					x_dir = (_mesh->joints[_mesh->j_lknee] - _mesh->joints[_mesh->j_rknee]);
				}
				else {
					wrist = _mesh->j_rhand;
					elbow = _mesh->j_relbow;
					shoulder = _mesh->j_rshoulder;
					elbow_lift_offset = elbow_radius[1];
					knee_joint = _mesh->joints[_mesh->j_rknee];
					x_dir = (_mesh->joints[_mesh->j_rknee] - _mesh->joints[_mesh->j_lknee]);
				}
				x_dir.y = 0; x_dir.z = 0; x_dir.normalize();
				Plane armrest_yz = { armrest_cen, { 1, 0, 0 } };
				Plane elbow_xy = { (_mesh->joints[shoulder] + _mesh->joints[shoulder]) / 2, { 0, 0, 1 } };
				float elbow_bone_len = (_mesh->joints[elbow] - _mesh->joints[shoulder]).length();
				IntVec ts = find_plane_intersecting_faces(armrest, armrest_yz);
				IntVec armrest_upmost;
				for (int i : ts) {
					Vertex vtx = armrest.triangle_center(i);
					Plane cutter = { vtx, { 0, 0, 1 } };
					bool flag = true;
					for (int j : ts) {
						if (i != j && armrest.triangle_cross_plane(j, cutter) && armrest.triangle_center(j).y > vtx.y) {
							flag = false; break;
						}
					}
					if (flag) armrest_upmost.push_back(i);
				}
				Vertex target;
				float dmin = DECIMAL_MAX;
				for (int i : armrest_upmost) {
					if (armrest.triangle_cross_plane(i, elbow_xy)) {
						Vertex vtx = armrest.triangle_center(i) + Vector(0, elbow_lift_offset, 0);
						float d = (_mesh->joints[shoulder] - vtx).length();
						if (d < elbow_bone_len * (float)2.1 && d < dmin) {
							std::cout << "found armrest contact by cutter succeed" << std::endl;
							vtx.z = elbow_xy.p.z;
							target = vtx; dmin = d;
						}
					}
				}
				//p0 = target; p1 = elbow_xy.p;
				
				if (!target.is_nonzero()) {
					std::cout << "found armrest contact by cutter failed" << std::endl;
					for (int i : armrest_upmost) {
						Vertex vtx = armrest.triangle_center(i) + Vector(0, elbow_lift_offset, 0);
						float d = (_mesh->joints[shoulder] - vtx).length();
						if (d < elbow_bone_len * (float)1.1 && d < dmin) { dmin = d; target = vtx; }
					}
					if (!target.is_nonzero()) std::cout << "found armrest contact failded" << std::endl;
				}
				float shoulder_bone_len = (_mesh->joints[shoulder] - _mesh->joints[_mesh->j_shoulders]).length();
				if (target.is_nonzero() && shoulder_bone_len + elbow_bone_len > dmin) {
					std::cout << "elbow can reach armrest" << std::endl;
					float cosine = shoulder_bone_len*shoulder_bone_len + dmin*dmin - elbow_bone_len*elbow_bone_len;
					cosine /= 2 * shoulder_bone_len * dmin;
					Vector axis = (_mesh->joints[shoulder] - _mesh->joints[_mesh->j_shoulders]).cross(target - _mesh->joints[_mesh->j_shoulders]).normalized();
					Quaternion q(axis, acos(cosine), true);
					Vector vec0 = (q * (target - _mesh->joints[_mesh->j_shoulders]).normalized()).normalized();
					Vector vec1 = (q.inverse() * (target - _mesh->joints[_mesh->j_shoulders]).normalized()).normalized();
					Vector shoulder_bone_dir = (_mesh->joints[shoulder] - _mesh->joints[_mesh->j_shoulders]).normalized();
					Vector to;
					if (vec0.dot(shoulder_bone_dir) > vec1.dot(shoulder_bone_dir)) to = vec0;
					else to = vec1;
					//_mesh->apply_rotation_with_cor(shoulder, Quaternion(shoulder_bone_dir, to));
					Vector elbow_bone_dir = (_mesh->joints[elbow] - _mesh->joints[shoulder]).normalized();
					_mesh->apply_rotation_with_cor(elbow, Quaternion(elbow_bone_dir, target - _mesh->joints[shoulder]));
					// forearm
					target = { 0, 0, 0 }; dmin = DECIMAL_MAX;
					float forearm_len = (_mesh->joints[elbow] - _mesh->joints[wrist]).length();
					for (int i : armrest_upmost) {
						Vertex vtx = armrest.triangle_center(i) + Vector(0, elbow_lift_offset/8, 0);
						float d = abs(forearm_len - (_mesh->joints[elbow] - vtx).length());
						if (d < dmin) { target = vtx; dmin = d; }
					}
					Vector wrist_bone_dir = (_mesh->joints[wrist] - _mesh->joints[elbow]).normalized();
					to = { 0, 0, -1 };
					if (target.is_nonzero()) {
						to = (target - _mesh->joints[elbow]).normalized();
						std::cout << "forearm move" << std::endl;
					}
					_mesh->apply_rotation_with_cor(wrist, Quaternion(wrist_bone_dir, to));
				}
				else {
					std::cout << "elbow cannot reach armrest" << std::endl;
					// upper arm
					float d = (_mesh->joints[elbow] - _mesh->joints[shoulder]).length() * (float)1;
					Vector elbow_bone_dir = (_mesh->joints[elbow] - _mesh->joints[shoulder]).normalized();
					Vector to = (knee_joint + Vector(0, -10*d, 0) + (x_dir * d * 5) - _mesh->joints[shoulder]).normalized();
					_mesh->apply_rotation_with_cor(elbow, Quaternion(elbow_bone_dir, to));
					// forearm
					float dy = abs((find_closest(_mesh->v, knee_joint) - knee_joint).y);
					Vector wrist_bone_dir = (_mesh->joints[wrist] - _mesh->joints[elbow]).normalized();
					to = (knee_joint + Vector(0, dy, 0) + (x_dir * -2*dy) - _mesh->joints[elbow]).normalized();
					to.z = -abs(to.z);
					_mesh->apply_rotation_with_cor(wrist, Quaternion(wrist_bone_dir, to));
				}
			}
		}

		//_mesh->paint_ball(p0, BALL_RADIUS, BALL_CUTS, _mesh->RED);
		//_mesh->paint_ball(p1, BALL_RADIUS, BALL_CUTS, _mesh->BLUE);
		//_mesh->write_to_ply(MESH_PATH + "_sit_" + _mesh->mesh_file_name + ".ply");
		//return;

		// head stay straight
		{
			Vector head_dir = (_mesh->joints[_mesh->j_head] - _mesh->joints[_mesh->j_shoulders]).normalized();
			_mesh->apply_rotation_with_cor(_mesh->j_head, Quaternion(head_dir, { 0, 1, 0 }));
		}

		// elasticity
		auto triangle_is_intersect = [](const Mesh& m0, int t0, const Mesh& m1, int t1) {
			for (int i = 0; i < 3; ++i) {
				Vertex a = m0.v[m0.t[t0].v[i]]; a.y = a.z;
				Vertex b = m0.v[m0.t[t0].v[(i + 1) % 3]]; b.y = b.z;
				for (int j = 0; j < 3; ++j) {
					Vertex c = m1.v[m1.t[t1].v[i]]; c.y = c.z;
					Vertex d = m1.v[m1.t[t1].v[(i + 1) % 3]]; d.y = d.z;
					float u = (c.x - a.x)*(b.y - a.y) - (b.x - a.x)*(c.y - a.y);
					float v = (d.x - a.x)*(b.y - a.y) - (b.x - a.x)*(d.y - a.y);
					float w = (a.x - c.x)*(d.y - c.y) - (d.x - c.x)*(a.y - c.y);
					float z = (b.x - c.x)*(d.y - c.y) - (d.x - c.x)*(b.y - c.y);
					if (u*v <= DECIMAL_SMALLEST && w*z <= DECIMAL_SMALLEST)
						return true;
				}
			}
			return false;
		};
		auto vertex_in_triangle = [](Vertex p, const Mesh& m, int t) {
			Vertex a = m.v[m.t[t].v[0]]; a.y = a.z;
			Vertex b = m.v[m.t[t].v[1]]; b.y = b.z;
			Vertex c = m.v[m.t[t].v[2]]; c.y = c.z;
			p.y = p.z;

			float signOfTrig = (b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x);
			float signOfAB = (b.x - a.x)*(p.y - a.y) - (b.y - a.y)*(p.x - a.x);
			float signOfCA = (a.x - c.x)*(p.y - c.y) - (a.y - c.y)*(p.x - c.x);
			float signOfBC = (c.x - b.x)*(p.y - c.y) - (c.y - b.y)*(p.x - c.x);

			bool d1 = (signOfAB * signOfTrig > 0);
			bool d2 = (signOfCA * signOfTrig > 0);
			bool d3 = (signOfBC * signOfTrig > 0);

			return d1 && d2 && d3;
		};
		// seat
		if (CHAIR_DEFORM) {
			IntVec ts;
			for (int j = 0; j < _mesh->t.size(); ++j) {
				if (_mesh->triangle_center(j).y < _mesh->joints[_mesh->j_lknee].y
					&& (_part_labels[j] == DetailPartType::thigh0 || _part_labels[j] == DetailPartType::thigh1
					|| _part_labels[j] == DetailPartType::downbody))
					ts.push_back(j);
			}
			Mass_Spring_System mass_spring(&chair.seat, 10, 1.1);
			IntVec upmost_surface = mass_spring.upmost_surface();
			VtxVec ply;
			for (auto i : upmost_surface) ply.push_back(mass_spring.get_centers()[i]);
			write_ply(ply, chair_path + "out_chair_seat_upmost.ply");
			IntVec seat_contact_voxels;
			for (int i : upmost_surface) {
				Vertex pt = mass_spring.get_centers()[i];
				if (1 || pt.z > seat_min.z + seat_parm.z * (float)0.01) {
					for (int j : ts) {
						if (vertex_in_triangle(pt, *_mesh, j)) {
							if ((pt - _mesh->triangle_center(j)).length() < 10) {
								seat_contact_voxels.push_back(i); break;
							}
						}
					}
				}
			}
			ply.clear();
			for (auto i : seat_contact_voxels) ply.push_back(mass_spring.get_centers()[i]);
			write_ply(ply, chair_path + "out_chair_seat_pts.ply");

			// deform
			float weight = 50;
			float mass = 10;
			float h = 0.01; // time interval (in seconds)
			float stiffness = 0.05;
			float damper = 0.1;
			mass /= (float)mass_spring.get_centers().size();
			mass /= h;
			Vector force = { 0, -weight*(float)9.8, 0 };
			force /= (float)seat_contact_voxels.size();
			VtxVec points = mass_spring.deform(seat_contact_voxels, force, mass, stiffness, damper, h, 1);
		}
		// back
		chair_back_vec.x = 0;
		if (CHAIR_DEFORM && chair_back_vec.is_nonzero())
		{
			// human back
			IntVec ts;
			for (int i = 0; i < _mesh->t.size(); ++i) {
				char part = _part_labels[i];
				if (part == DetailPartType::upbody || part == DetailPartType::midbody || part == DetailPartType::downbody
					//|| part==DetailPartType::neck || part == DetailPartType::head
					//|| part == DetailPartType::arm0 || part == DetailPartType::arm1
					) {
					ts.push_back(i);
				}
			}
			// contacts
			Quaternion rot(chair_back_vec.normalized(), { 0, 0, 1 });
			_mesh->rotate(rot);
			chair.back.rotate(rot);
			Mass_Spring_System mass_spring(&chair.back, 15, 1.1);
			IntVec upmost_surface = mass_spring.upmost_surface();
			VtxVec ply;
			for (auto i : upmost_surface) ply.push_back(mass_spring.get_centers()[i]);
			write_ply(ply, chair_path + "out_chair_back_upmost.ply");
			write_ply(mass_spring.get_centers(), chair_path + "out_chair_back.ply");
			IntVec back_contact_voxels;
			for (int i : upmost_surface) {
				Vertex pt = mass_spring.get_centers()[i];
				if (1 || pt.z > seat_min.z + seat_parm.z * (float)0.02) {
					for (int j : ts) {
						if (vertex_in_triangle(pt, *_mesh, j)) {
							if ((pt - _mesh->triangle_center(j)).length() < 10) {
								back_contact_voxels.push_back(i); break;
							}
						}
					}
				}
			}
			ply.clear();
			for (auto i : back_contact_voxels) ply.push_back(mass_spring.get_centers()[i]);
			write_ply(ply, chair_path + "out_chair_back_pts.ply");
			
			// deform
			float weight = 30;
			float mass = 1;
			float h = 0.01; // time interval (in seconds)
			float stiffness = 0.01;
			float damper = 0.1;
			mass /= (float)mass_spring.get_centers().size();
			mass /= h;
			Vector force = { 0, -weight*(float)9.8, 0 };
			force /= (float)back_contact_voxels.size();
			VtxVec points = mass_spring.deform(back_contact_voxels, force, mass, stiffness, damper, h, 1);
			//write_ply(points, chair_path + "out_chair_back_pts.ply");

			_mesh->rotate(rot.inverse());
			chair.back.rotate(rot.inverse());
		}

		// write
		_mesh->write_to_ply(MESH_PATH + "_sit9_" + _mesh->mesh_file_name + ".ply");
		chair.write_to_ply(chair_path + "out_chair_" + _mesh->mesh_file_name + ".ply");
	}
	void Reshape::init_chair(const string& chair_path, StrVec seat_name, StrVec back_name, StrVec arm_names, StrVec leg_names, Vector scale_factor) {

		Chair& chair = _chair;

		// seat
		chair.has_seat = false;
		if (back_name.size() > 0) {
			chair.has_seat = true;
			chair.seat.read_from_file(chair_path + seat_name[0]); chair.p.push_back(&chair.seat);
		}
		
		// back
		chair.has_back = false;
		if (back_name.size() > 0) {
			chair.has_back = true;
			chair.back.read_from_file(chair_path + back_name[0]); chair.p.push_back(&chair.back);
		}

		// arm
		chair.arm_num = arm_names.size();
		chair.arms.resize(chair.arm_num);
		for (int i = 0; i < chair.arm_num; ++i) {
			chair.arms[i].read_from_file(chair_path + arm_names[i]);
			chair.p.push_back(&chair.arms[i]);
		}

		// leg
		chair.leg_num = leg_names.size();
		chair.legs.resize(chair.leg_num);
		for (int i = 0; i < chair.leg_num; ++i) {
			chair.legs[i].read_from_file(chair_path + leg_names[i]);
			chair.p.push_back(&chair.legs[i]);
		}

		// chair init color
		for (int i = 0; i < chair.p.size(); ++i) {
			for (auto& v : chair.p[i]->vcolor) {
				v.r = v.g = v.b = v.a = 255;
			}
		}

		// chair scale
		for (int i = 0; i < chair.p.size(); ++i)
			chair.p[i]->scale(scale_factor);
	}
	void Reshape::deform_to_sitting_pose() {

		// segmenting human body
		run();

		//// sitting ske
		//Vertex sit_ske[18] = {
		//	Vector(0., 0.5, 0.), Vector(-0.02, 0.15, 0.), Vector(-0.02, 0., 0.), Vector(0., 0.7, 0.), // shoulders back hip head
		//	Vector(-0.1, 0., 0.), Vector(-0.2, -0.2, -0.5), Vector(-0.2, -0.5, -0.5), Vector(-0.15, -0.8, -0.1), // L hip knee ankle foot
		//	Vector(0.1, 0., 0.), Vector(0.2, -0.2, -0.5), Vector(0.2, -0.5, -0.5), Vector(0.15, -0.8, -0.1), // R hip knee ankle foot
		//	Vector(-0.2, 0.5, 0.), Vector(-0.3, 0.15, 0.), Vector(-0.3, 0., -0.5), // L shoulder elbow wrist
		//	Vector(0.2, 0.5, 0.), Vector(0.3, 0.15, 0.), Vector(0.3, 0., -0.5) // R shoulder elbow wrist
		//};

		//// sitting ske
		//Vertex sit_ske[18] = {
		//	Vector(0., 0.5, 0.), Vector(0., 0.15, 0.), Vector(0., 0., 0.), Vector(0., 0.7, 0.), // shoulders back hip head
		//	Vector(-0.1, 0., 0.), Vector(-0.2, -0.205, -0.5), Vector(-0.2, -0.5, -0.5), Vector(-0.15, -0.8, -0.1), // L hip knee ankle foot
		//	Vector(0.1, 0., 0.), Vector(0.2, -0.205, -0.5), Vector(0.2, -0.5, -0.5), Vector(0.15, -0.8, -0.1), // R hip knee ankle foot
		//	Vector(-0.2, 0.5, 0.), Vector(-0.3, 0.15, 0.), Vector(-0.3, 0., -0.5), // L shoulder elbow wrist
		//	Vector(0.2, 0.5, 0.), Vector(0.3, 0.15, 0.), Vector(0.3, 0., -0.5) // R shoulder elbow wrist
		//};

		// sitting ske
		Vertex sit_ske[18] = {
			Vector(0., 0.5, 0.02), Vector(0., 0.15, 0.01), Vector(0., 0., 0.), Vector(0., 0.7, 0.), // shoulders back hip head
			Vector(-0.1, 0., 0.), Vector(-0.2, -0.2, -0.5), Vector(-0.2, -0.5, -0.5), Vector(-0.15, -0.8, -0.1), // L hip knee ankle foot
			Vector(0.1, 0., 0.), Vector(0.2, -0.2, -0.5), Vector(0.2, -0.5, -0.5), Vector(0.15, -0.8, -0.1), // R hip knee ankle foot
			Vector(-0.2, 0.48, 0.), Vector(-0.3, 0.15, 0.), Vector(-0.3, -0., -0.5), // L shoulder elbow wrist
			Vector(0.2, 0.48, 0.), Vector(0.3, 0.15, 0.), Vector(0.3, -0., -0.5) // R shoulder elbow wrist
		};

		Vertex ap[2] = { _mesh->v[_landmarks.armpit[0]], _mesh->v[_landmarks.armpit[1]] };
		Vertex ske = _skeleton[PartType::p_body][_skeleton[PartType::p_body].size() - 1];
		Plane plane_FB = create_plane_cross(ske, ap[0] - ske, ap[1] - ske);
		if (plane_FB.n.z > 0) plane_FB.n *= -1;

		Quaternion q = Quaternion(Vector(0, 0, -1), plane_FB.n);
		for (int i = 0; i < 18; ++i) {
			sit_ske[i] -= Vector(0., 0.5, 0.);
			sit_ske[i] = q * sit_ske[i];
		}

		int order[] = {
			Mesh::j_head, Mesh::j_back, Mesh::j_hips,
			Mesh::j_lshoulder, Mesh::j_lelbow, Mesh::j_lhand,
			Mesh::j_rshoulder, Mesh::j_relbow, Mesh::j_rhand,
			Mesh::j_lknee, Mesh::j_lankle,
			Mesh::j_rknee, Mesh::j_rankle,
		};

		//int order[] = {
		//	//Mesh::j_head, Mesh::j_back, Mesh::j_hips,
		//	//Mesh::j_lshoulder, Mesh::j_lelbow, Mesh::j_lhand,
		//	//Mesh::j_rshoulder, Mesh::j_relbow, Mesh::j_rhand,
		//	Mesh::j_lelbow, Mesh::j_lhand,
		//	Mesh::j_relbow, Mesh::j_rhand,
		//	Mesh::j_lknee, Mesh::j_lankle,
		//	Mesh::j_rknee, Mesh::j_rankle,
		//};

#pragma pack(1)
		struct COR_PART { int i; float x, y, z; };
#pragma pack()
		string path = _mesh->mesh_file_name + ".cors";
		FILE* fp = fopen(path.c_str(), "rb");
		if (nullptr != fp) {
			int cors_size = 0;
			fread(&cors_size, sizeof(int), 1, fp);
			std::vector<COR_PART> buff(cors_size);
			fread(&buff[0], sizeof(COR_PART)*cors_size, 1, fp);
			fclose(fp);
			for (int i = 0; i < cors_size; ++i) {
				_mesh->cors[buff[i].i] = Vertex(buff[i].x, buff[i].y, buff[i].z);
			}
		}
		else {
			_mesh->cors = compute_optimized_center_of_rotation();
			fp = fopen(path.c_str(), "wb");
			int cors_size = _mesh->cors.size();
			fwrite(&cors_size, sizeof(int), 1, fp);
			std::vector<COR_PART> buff;
			for (int i = 0; i < (int)_mesh->v.size(); ++i) {
				if (_mesh->cors.count(i)) {
					COR_PART part;
					part.i = i;
					part.x = _mesh->cors[i].x;
					part.y = _mesh->cors[i].y;
					part.z = _mesh->cors[i].z;
					buff.push_back(part);
				}
			}
			fwrite(&buff[0], sizeof(COR_PART)*cors_size, 1, fp);
			fclose(fp);
		}

		for (int i = 0; i < sizeof(order) / sizeof(int); ++i) {
			int joint_id = order[i];
			Vector from = _mesh->joints[joint_id] - _mesh->joints[_mesh->joint_parent[joint_id]];
			Vector to = sit_ske[joint_id] - sit_ske[_mesh->joint_parent[joint_id]];
			_mesh->apply_rotation_with_cor(joint_id, Quaternion(from, to));
			//_mesh->apply_rotation(joint_id, Quaternion(from, to));
		}
		//_mesh->apply_deformation();
		//_mesh->apply_deformation_with_cor();
	}
	// chair
	void Reshape::Chair::write_to_ply(const string& path) {
		std::vector<Vertex> vs;
		std::vector<Triangle> ts;
		for (int i = 0; i < this->p.size(); ++i) {
			int size = vs.size();
			for (auto v : this->p[i]->v) vs.push_back(v);
			for (auto t : this->p[i]->t) ts.push_back(Triangle(size + t.v[0], size + t.v[1], size + t.v[2]));
		}
		Mesh::write_to_ply(vs, ts, path);
	}
	// skinning
	void Reshape::subdivide_mesh(float epsilon, VtxVec& vertices_out, std::vector<Triangle>& triangles_out,
		IntVec& triangle_neighbors_out, std::vector<double>& bone_weights_out) {

		const int joint_num = Mesh::JOINT_NAME::j_number;
		auto vertices = _mesh->v;
		auto triangles = _mesh->t;
		auto tts = _mesh->tt_data;
		auto skinning_weights = _mesh->weights;

		//for (int e = 0; e < (int)mesh.e.size(); ++e) {
		//	Edge edge = mesh.e[e];
		//	int v0 = edge.v[0];
		//	int v1 = edge.v[1];
		//	float l2Distance = 0;
		//	for (int j = 0; j < joint_num; ++j) {
		//		l2Distance += std::pow(mesh.weights[v0 * joint_num + j] - mesh.weights[v1 * joint_num + j], 2);
		//	}
		//	if (l2Distance > epsilon) {

		//		// vertex
		//		int v2 = mesh.ev(e, 0);
		//		int v3 = mesh.ev(e, 1);
		//		Vertex new_v = (mesh.v[v0] + mesh.v[v1]) * (float)0.5;
		//		int new_vidx = (int)(mesh.v.size());
		//		mesh.v.push_back(new_v);

		//		// bone weight
		//		for (int j = 0; j < joint_num; ++j) {
		//			mesh.weights.push_back((mesh.weights[v0 * joint_num + j] + mesh.weights[v1 * joint_num + j]) * (float)0.5);
		//		}

		//		// edge
		//		mesh.e[e].v[0] = new_vidx;
		//		mesh.e.push_back({ v0, new_vidx });
		//		mesh.e.push_back({ v2, new_vidx });
		//		mesh.e.push_back({ new_vidx, v3 });

		//		// triangle
		//		int v2 = mesh.e
		//		triangles.push_back({ newVIdx, beta, gamma });

		//		triangles.erase(triangles.begin() + tri);
		//		--tri;
		//		break;
		//	}
		//}

		auto triangle_neighbor = [&triangles, &tts](int face, int v0, int v1, int& opposite_vertex, int& neighbor_index) {

			for (int i = 0; i < 3; ++i) {
				int n = tts[3 * face + i];
				if (n < 0) continue;
				int vs[3] = { 0 };
				vs[0] = triangles[n].v[0];
				vs[1] = triangles[n].v[1];
				vs[2] = triangles[n].v[2];
				for (int j = 0; j < 3; ++j) {
					if (vs[j] == v0 && vs[(j + 1) % 3] == v1 || vs[j] == v1 && vs[(j + 1) % 3] == v0) {
						opposite_vertex = vs[(j + 2) % 3];
						neighbor_index = i;
						return n;
					}
				}
			}
			return (int)-1;
		};

		for (int tri = 0; tri < (int)triangles.size(); ++tri) {
			if (tri % 3000 == 0)
				std::cout << tri << std::endl;
			auto triangle = triangles[tri];
			for (int i = 0; i < 3; ++i) {
				int v0 = triangle.v[i];
				int v1 = triangle.v[(i + 1) % 3];
				double diff = 0;
				for (int j = 0; j < joint_num; ++j) {
					diff += (double)std::pow(skinning_weights[v0 * joint_num + j] - skinning_weights[v1 * joint_num + j], 2);
				}
				diff = std::sqrt(diff);
				if (diff >(double)epsilon) {
					int v2 = triangle.v[(i + 2) % 3];
					Vertex new_v = (vertices[v0] + vertices[v1]) * (float)0.5;
					int vx = (int)(vertices.size());
					vertices.push_back(new_v);

					for (int j = 0; j < joint_num; ++j) {
						double neww = (skinning_weights[v0 * joint_num + j] + skinning_weights[v1 * joint_num + j]) * (double)0.5;
						skinning_weights.push_back(neww);
					}

					triangles.push_back({ v0, vx, v2 });
					triangles.push_back({ vx, v1, v2 });

					triangles.erase(triangles.begin() + tri);
					--tri;
					break;
				}
			}
		}
		vertices_out.swap(vertices);
		triangles_out.swap(triangles);
		triangle_neighbors_out.swap(tts);
		bone_weights_out.swap(skinning_weights);
		return;

		for (int tri = 0; tri < (int)triangles.size(); ++tri) {
			Triangle triangle = triangles[tri];
			for (int i = 0; i < 3; ++i) {
				int v0 = triangle.v[i];
				int v1 = triangle.v[(i + 1) % 3];
				if (v0 < 0 || v1 < 0)
					std::cout << "v0 or v1 error" << std::endl;

				double diff = 0;
				double w0[18], w1[18];
				for (int j = 0; j < joint_num; ++j) {
					w0[j] = skinning_weights[v0 * joint_num + j];
					w1[j] = skinning_weights[v1 * joint_num + j];
					diff += (double)std::pow(skinning_weights[v0 * joint_num + j] - skinning_weights[v1 * joint_num + j], 2);
				}
				diff = std::sqrt(diff);

				// subdivide
				if (diff >(double)epsilon) {

					if (tri % 3000 == 0)
						std::cout << tri << std::endl;

					int v2 = triangle.v[(i + 2) % 3];
					if (v2 < 0)
						std::cout << "v2 error" << std::endl;

					Vertex new_v = (vertices[v0] + vertices[v1]) * (float)0.5;
					int vx = (int)(vertices.size());
					vertices.push_back(new_v);

					double wx[18];
					for (int j = 0; j < joint_num; ++j) {
						double neww = (skinning_weights[v0 * joint_num + j] + skinning_weights[v1 * joint_num + j]) * (double)0.5;
						wx[j] = neww;
						skinning_weights.push_back(neww);
					}

					int temp, temp2;
					int v3 = -1;
					int tri01 = triangle_neighbor(tri, v0, v1, v3, temp);
					int tri02 = triangle_neighbor(tri, v0, v2, temp, temp2);
					int tri12 = triangle_neighbor(tri, v1, v2, temp, temp2);
					int index12 = -1;
					triangle_neighbor(tri12, v1, v2, temp, index12);
					int tri03 = triangle_neighbor(tri01, v0, v3, temp, temp2);
					int tri13 = triangle_neighbor(tri01, v1, v3, temp, temp2);
					int index13 = -1;
					triangle_neighbor(tri13, v1, v3, temp, index13);
					if (tri01 < 0 || tri02 < 0 || tri12 < 0 || tri03 < 0 || tri13 < 0)
						std::cout << "triangle_neighbor error" << std::endl;

					triangles[tri] = { v0, vx, v2 };
					triangles[tri01] = { v0, v3, vx };
					int trix12 = triangles.size();
					triangles.push_back({ vx, v1, v2 });
					int trix13 = triangles.size();
					triangles.push_back({ vx, v3, v1 });

					tts.resize(tts.size() + 6);
					tts[3 * tri + 0] = tri01; tts[3 * tri + 1] = trix12; tts[3 * tri + 2] = tri02;
					tts[3 * tri01 + 0] = tri03; tts[3 * tri01 + 1] = trix13; tts[3 * tri01 + 2] = tri;
					tts[3 * trix12 + 0] = trix13; tts[3 * trix12 + 1] = tri12; tts[3 * trix12 + 2] = tri;
					tts[3 * trix13 + 0] = tri01; tts[3 * trix13 + 1] = tri13; tts[3 * trix13 + 2] = trix12;
					tts[3 * tri12 + index12] = trix12;
					tts[3 * tri13 + index13] = trix13;

					--tri;
					break;
				}
			}
		}
		vertices_out.swap(vertices);
		triangles_out.swap(triangles);
		triangle_neighbors_out.swap(tts);
		bone_weights_out.swap(skinning_weights);
	}
	std::map<int, Vertex> Reshape::compute_optimized_center_of_rotation(float epsilon, float sigma) {

		VtxVec vertices;
		std::vector<Triangle> triangles;
		IntVec triangle_neighbors;
		std::vector<double> skinning_weights;

		subdivide_mesh(epsilon, vertices, triangles, triangle_neighbors, skinning_weights);

		const int jn = Mesh::JOINT_NAME::j_number;
		int vn = vertices.size();
		int tn = triangles.size();
		int real_vn = (int)_mesh->v.size();

		// precalculate
		FltVec triangle_areas(tn);
		std::vector<double> triangle_weights(tn * jn, 0);
		VtxVec triangle_centroids(tn);
		for (int i = 0; i < tn; ++i) {
			Vertex v0 = vertices[triangles[i].v[0]];
			Vertex v1 = vertices[triangles[i].v[1]];
			Vertex v2 = vertices[triangles[i].v[2]];
			triangle_centroids[i] = (v0 + v1 + v2) / (float)3;
			float a = (v0 - v1).length();
			float b = (v2 - v1).length();
			float c = (v0 - v2).length();
			float p = (a + b + c) / 2;
			float area = p*(p - a)*(p - b)*(p - c);
			if (area < std::numeric_limits<float>::min()) triangle_areas[i] = 0;
			else triangle_areas[i] = std::sqrt(area);
			for (int j = 0; j < jn; ++j) {
				for (int k = 0; k < 3; ++k) {
					triangle_weights[i*jn + j] += skinning_weights[triangles[i].v[k] * jn + j];
				}
				triangle_weights[i*jn + j] /= (double)3;
			}
		}

		// compute center of rotation
		std::map<int, Vertex> results;
		IntVec nonzero_widx(jn);
		const double zero = std::numeric_limits<double>::min();
		int cnt = 0;
		//printf("Computing optimized centers of rotation for {0} vertices in {1} triangles, subdivided to {2} vertices in {3} triangles, over {4} bones...\n", numUnsubdividedVertices, triangles.size(), subdividedMesh.vertices.size(), subdividedMesh.triangles.size(), boneIndex);
		//fmt::print("Computing optimized centers of rotation for {0} vertices in {1} triangles, subdivided to {2} vertices in {3} triangles, over {4} bones...\n", numUnsubdividedVertices, triangles.size(), subdividedMesh.vertices.size(), subdividedMesh.triangles.size(), boneIndex);
		for (int i = 0; i < real_vn; ++i) {

			if (i % (vn / 40) == 0) {
				float pct = float(i) / float(vn);
				//fmt::print("{0}%\n", pct*100);
				//printf("{0}%\n", pct * 100);
			}

			// nonzero weights
			nonzero_widx.clear();
			for (int j = 0; j < jn; ++j) {
				if (skinning_weights[i*jn + j] > zero) nonzero_widx.push_back(j);
			}

			Vertex cor(0, 0, 0);
			float total_weight = 0;
			const float sigmaSq = sigma*sigma;

			for (int tri = 0; tri < tn; ++tri) {

				float similarity = 0;

				for (int j = 0; j < (int)nonzero_widx.size(); ++j) {
					for (int k = j + 1; k < (int)nonzero_widx.size(); ++k) {
						float wPj = (float)skinning_weights[i*jn + nonzero_widx[j]];
						float wPk = (float)skinning_weights[i*jn + nonzero_widx[k]];
						float wVj = (float)triangle_weights[tri*jn + nonzero_widx[j]];
						float wVk = (float)triangle_weights[tri*jn + nonzero_widx[k]];

						similarity += wPj*wPk*wVj*wVk * std::exp(-(std::pow(wPj*wVk - wPk*wVj, 2)) / sigmaSq);
					}
				}

				if (triangle_areas[tri] > 0) {

					float weight = similarity * triangle_areas[tri];
					total_weight += weight;

					cor += triangle_centroids[tri] * weight;
				}
			}

			if (total_weight > 0) {
				results[i] = cor / total_weight;
			}
			else ++cnt;
			std::cout << i << "/" << real_vn << " " << (total_weight > 0) << std::endl;
		}
		std::cout << "error number : " << cnt << std::endl;
		return results;
	}
	// functions
	Vertex Reshape::find_centroid(const VtxVec& vs) {
		Vertex ret(0, 0, 0);
		for (auto v : vs) ret += v;
		ret /= vs.size();
		return ret;
	};
	Vertex Reshape::find_closest(const VtxVec& vs, Vertex target) {
		Vertex ret(0, 0, 0);
		float dmin = DECIMAL_MAX;
		for (auto v : vs) {
			if ((v - target).length() < dmin) {
				dmin = (v - target).length(); ret = v;
			}
		}
		return ret;
	};
	Reshape::IntVec Reshape::find_plane_intersecting_faces(const Mesh& m, const Plane& plane) {

		IntVec ret;
		for (int i = 0; i < m.t.size(); ++i) {
			if (m.triangle_cross_plane(i, plane)) {
				ret.push_back(i);
			}
		}
		return ret;
	}
	Vertex Reshape::circle_line_intersect(Vertex cen, float r, Vertex v0, Vertex v1) {

		Vertex in, out;
		float d0 = (v0 - cen).length() - r;
		float d1 = (v1 - cen).length() - r;
		if (d0 < 0 && d1 > 0) { in = v0; out = v1; }
		if (d0 > 0 && d1 < 0) { in = v1; out = v0; }
		in -= cen; out -= cen;
		Vector line = (out - in).normalized();
		Vector cross = (in - out).cross({ 1, 0, 0 }).normalized();
		float divider = line.y*cross.z - line.z*cross.y;
		if (abs(divider) < DECIMAL_SMALLEST)
			std::cout << "error in circle_line_intersect" << std::endl;
		float cross_t = (line.y*in.z - line.z*in.y) / divider;
		Vertex intersect = cross * cross_t;
		float len = r*r - intersect.square_length();
		if (len <= 0)
			std::cout << "error in circle_line_intersect" << std::endl;
		len = std::sqrt(len);
		return cen + intersect + (line * len);
	}
}