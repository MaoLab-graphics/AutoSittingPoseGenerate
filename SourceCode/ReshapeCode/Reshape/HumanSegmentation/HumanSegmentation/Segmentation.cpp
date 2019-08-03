// Segmentation

#include "Segmentation.h"
#include "Mesh.h"
#include "Voxel.h"
#include "Skeletonization.h"
#include <string>
#include <time.h>
#include <fstream>
#include <set>

namespace ManSeg {

	// -------------------- Class Segmentation --------------------

	Segmentation::Segmentation(Mesh* in_mesh) : _human(in_mesh) {
		_preset_color[0] = _human->RED; _preset_color[1] = _human->YELLOW; _preset_color[2] = _human->GREEN;
		_preset_color[3] = _human->CYAN; _preset_color[4] = _human->BLUE; _preset_color[5] = _human->PURPLE;
		_preset_color[6] = _human->WHITE;
		_preset_color[7] = _human->RED; _preset_color[8] = _human->YELLOW; _preset_color[9] = _human->GREEN;
		_preset_color[10] = _human->CYAN; _preset_color[11] = _human->BLUE; _preset_color[12] = _human->PURPLE;
	}
	Segmentation::Segmentation(Mesh* in_mesh, VoxelHandler* vxh) : _human(in_mesh), _voxelh(vxh) {
		_preset_color[0] = _human->RED; _preset_color[1] = _human->YELLOW; _preset_color[2] = _human->GREEN;
		_preset_color[3] = _human->CYAN; _preset_color[4] = _human->BLUE; _preset_color[5] = _human->PURPLE;
		_preset_color[6] = _human->WHITE;
		_preset_color[7] = _human->RED; _preset_color[8] = _human->YELLOW; _preset_color[9] = _human->GREEN;
		_preset_color[10] = _human->CYAN; _preset_color[11] = _human->BLUE; _preset_color[12] = _human->PURPLE;
	}
	void Segmentation::write_off(string off_path) {
		std::fstream fs(off_path, std::ios::out);
		fs << "OFF\n" << (int)_human->v.size() << ' ' << (int)_human->t.size() << ' ' << 0 << '\n';
		for (int i = 0; i < (int)_human->v.size(); ++i) {
			fs << _human->v[i].x << ' ' << _human->v[i].y << ' ' << _human->v[i].z << '\n';
		}
		for (int i = 0; i < (int)_human->t.size(); ++i) {
			fs << 3 << ' ' << _human->t[i].v[0] << ' ' << _human->t[i].v[1] << ' ' << _human->t[i].v[2] << ' '
				<< (int)_human->tcolor[i].r << ' ' << (int)_human->tcolor[i].g << ' ' << (int)_human->tcolor[i].b << '\n';
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

	void SkeletalSegmentation::do_human_segmentation() {
		//split_mesh_by_border_voxel();
		//split_mesh_by_SS_code();
		if (FIND_LANDMARKS) read_landmark_ground_truth();
		read_HKS_and_curvature("");
		split_mesh_by_slice();
		if (REFINE_SEG) refine_split_by_slice();
		if (FIND_LANDMARKS || FIND_FEATURE_POINTS) find_landmarks();
		if (FIND_LANDMARKS) write_landmark_error();
		print_time_message("split end");
	}
	void SkeletalSegmentation::do_human_segmentation(const string& data_path) {
		//split_mesh_by_border_voxel();
		//split_mesh_by_SS_code();
		if (FIND_LANDMARKS) read_landmark_ground_truth();
		read_HKS_and_curvature(data_path);
		split_mesh_by_slice();
		if (REFINE_SEG) refine_split_by_slice();
		if (FIND_LANDMARKS || FIND_FEATURE_POINTS) find_landmarks();
		if (FIND_LANDMARKS) write_landmark_error();
		print_time_message("split end");
	}
	void SkeletalSegmentation::write_smooth_skeleton() {
		// init
		for (int idx = 0; idx < (int)_human->v.size(); idx++) {
			_human->color_vertex(idx, _preset_color[6]);
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
	void SkeletalSegmentation::read_HKS_and_curvature(const string& path) {

		// read hks
		string hks_path = path + "hks_" + _human->mesh_file_name + ".txt";
		FILE* fp = fopen(hks_path.c_str(), "rb");
		_hks.resize(_human->v.size());
		fread(&_hks[0], _human->v.size() * sizeof(double), 1, fp);
		fclose(fp);
		// read curvature
		string cvt_path = path + "curvature_" + _human->mesh_file_name + ".txt";
		fp = fopen(cvt_path.c_str(), "rb");
		for (int i = 0; i < CurvatureType::c_cvt_type_amount - 1; ++i) {
			_curvature[i].resize(_human->v.size());
			fread(&_curvature[i][0], _human->v.size() * sizeof(double), 1, fp);
		}
		_curvature[CurvatureType::c_mean_gauss].resize(_human->v.size());
		for (int i = 0; i < _human->v.size(); ++i) {
			_curvature[CurvatureType::c_mean_gauss][i] = _curvature[CurvatureType::c_mean][i] + _curvature[CurvatureType::c_gauss][i];
		}
		fclose(fp);
	}
	// refine segmentation
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
		int face_num = (int)_human->t.size();
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
				int n = _human->tt(curr, i);
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
					int n = _human->tt(curr, j);
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
				int vidx = _human->t[curr].v[j];
				float dist = (_human->v[vidx] - vtx).square_length();
				if (dist < mindist) {
					mindist = dist; armpt[i - 1] = vidx;
				}
			}
			_human->color_vertex(armpt[i - 1], _preset_color[0]);
		}
		// rotate plane
		for (int i = 1; i < 3; i++) {
			Vertex armv = _human->v[armpt[i - 1]];
			Vertex armv2 = _human->v[armpt[2 - i]];
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
					int fv = _human->t[cut_ret[j]].v[k];
					float dist = (_human->v[fv] - legpt[i - 1]).square_length();
					if (dist > maxdist) {
						if (point_plane_distance(cut, _human->v[fv]) > 0) {
							maxidx = fv; maxdist = dist;
						}
					}
				}
			}
			//_human->color_vertex(maxidx, _preset_color[0]);
			//connect_two_vertex(cut, maxidx, armpt[i - 1]);
		}
	}
	void SkeletalSegmentation::split_body_by_contour() {

		if (MANUAL_BODY_SEG) {

			_cuts[PartType::p_body].resize(BodyCut::c_body_cut_amount);
			_cuts[PartType::p_body][BodyCut::c_up] = MANUAL_BODY_CUT_UP;
			_cuts[PartType::p_body][BodyCut::c_down] = MANUAL_BODY_CUT_DOWN;
			_cuts[PartType::p_body][BodyCut::c_waist] = (MANUAL_BODY_CUT_UP + MANUAL_BODY_CUT_DOWN) / 2;

			int body_cut_up = _cuts[PartType::p_body][BodyCut::c_up];
			int body_cut_mid = _cuts[PartType::p_body][BodyCut::c_waist];
			int body_cut_down = _cuts[PartType::p_body][BodyCut::c_down];

			if (PAINT_SEG_OVER_MESH) {
				//paint_ring_vertices(_planes[PartType::p_body][body_cut_up].n, _slices_vtx[PartType::p_body][body_cut_up], _human->MYCOLOR[21]);
				//paint_ring_vertices(_planes[PartType::p_body][body_cut_mid].n, _slices_vtx[PartType::p_body][body_cut_mid], _human->MYCOLOR[22]);
				//paint_ring_vertices(_planes[PartType::p_body][body_cut_down].n, _slices_vtx[PartType::p_body][body_cut_down], _human->MYCOLOR[28]);
				paint_ring_vertices(_planes[PartType::p_body][body_cut_up].n, _slices_vtx[PartType::p_body][body_cut_up], _human->MYCOLOR[15]);
				//paint_ring_vertices(_planes[PartType::p_body][body_cut_mid].n, _slices_vtx[PartType::p_body][body_cut_mid], _human->MYCOLOR[22]);
				paint_ring_vertices(_planes[PartType::p_body][body_cut_down].n, _slices_vtx[PartType::p_body][body_cut_down], _human->YELLOW);
			}
		}

		int size = (int)_slices_vtx[PartType::p_body].size();
		int interval = COS_INTERVAL * 3;
		_body_border[0] = 0;
		_body_border[1] = size - 1;

		// find body cut
		const Vertex ap[2] = { _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]] };
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
				cos_front[i] = acos(cosine)*180.0f / PI;
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
				cos_back[i] = acos(cosine)*180.0f / PI;
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
			//paint_ring_vertices(_planes[PartType::p_body][body_cut_up].n, _slices_vtx[PartType::p_body][body_cut_up], _human->MYCOLOR[21]);
			//paint_ring_vertices(_planes[PartType::p_body][body_cut_mid].n, _slices_vtx[PartType::p_body][body_cut_mid], _human->MYCOLOR[22]);
			//paint_ring_vertices(_planes[PartType::p_body][body_cut_down].n, _slices_vtx[PartType::p_body][body_cut_down], _human->MYCOLOR[28]);
			paint_ring_vertices(_planes[PartType::p_body][body_cut_up].n, _slices_vtx[PartType::p_body][body_cut_up], _human->MYCOLOR[15]);
			//paint_ring_vertices(_planes[PartType::p_body][body_cut_mid].n, _slices_vtx[PartType::p_body][body_cut_mid], _human->MYCOLOR[22]);
			paint_ring_vertices(_planes[PartType::p_body][body_cut_down].n, _slices_vtx[PartType::p_body][body_cut_down], _human->YELLOW);
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

		int face_num = (int)_human->t.size();
		_part_labels.resize(face_num);
		for (auto it = _part_labels.begin(); it != _part_labels.end(); ++it) *it = -1;
		CharVec& labels = _part_labels;

		//// label limbs
		//int face_num = (int)_human->t.size();
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
		//			int n = _human->tt(curr, i);
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
		//	tcolors[0] = _human->WHITE; tcolors[1] = _human->RED;
		//	tcolors[10] = _human->WHITE; tcolors[11] = _human->BLUE; tcolors[12] = _human->WHITE;
		//	tcolors[20] = _human->WHITE; tcolors[21] = _human->BLUE; tcolors[22] = _human->WHITE;
		//	tcolors[30] = _human->WHITE; tcolors[31] = _human->GREEN; tcolors[32] = _human->WHITE;
		//	tcolors[40] = _human->WHITE; tcolors[41] = _human->GREEN; tcolors[42] = _human->WHITE;
		//	tcolors[50] = _human->CYAN; tcolors[51] = _human->WHITE; tcolors[52] = _human->YELLOW;
		//	for (int idx = 0; idx < (int)_human->t.size(); ++idx) {
		//		_human->color_face(idx, tcolors[labels[idx]]);
		//	}
		//}
		if (PAINT_MESH_USING_LABEL)
		{
			Color tcolors[256];
			tcolors[0] = _human->MYCOLOR[11]; tcolors[1] = _human->MYCOLOR[1];
			tcolors[10] = _human->MYCOLOR[3]; tcolors[11] = _human->MYCOLOR[0]; tcolors[12] = _human->MYCOLOR[2];
			tcolors[20] = _human->MYCOLOR[3]; tcolors[21] = _human->MYCOLOR[0]; tcolors[22] = _human->MYCOLOR[2];
			tcolors[30] = _human->MYCOLOR[7]; tcolors[31] = _human->MYCOLOR[2]; tcolors[32] = _human->MYCOLOR[9];
			tcolors[40] = _human->MYCOLOR[7]; tcolors[41] = _human->MYCOLOR[2]; tcolors[42] = _human->MYCOLOR[9];
			tcolors[50] = _human->MYCOLOR[2]; tcolors[51] = _human->MYCOLOR[7]; tcolors[52] = _human->MYCOLOR[5];

			tcolors[0] = _human->MYCOLOR[11]; tcolors[1] = _human->RED;
			tcolors[10] = _human->YELLOW; tcolors[11] = _human->CYAN; tcolors[12] = _human->GREEN;
			tcolors[20] = _human->YELLOW; tcolors[21] = _human->CYAN; tcolors[22] = _human->GREEN;
			tcolors[30] = _human->CYAN; tcolors[31] = _human->GREEN; tcolors[32] = _human->BLUE;
			tcolors[40] = _human->CYAN; tcolors[41] = _human->GREEN; tcolors[42] = _human->BLUE;
			tcolors[50] = _human->MYCOLOR[11]; tcolors[51] = _human->MYCOLOR[11]; tcolors[52] = _human->MYCOLOR[11];

			tcolors[0] = _human->MYCOLOR[11]; tcolors[1] = _human->RED;
			tcolors[10] = _human->MYCOLOR[13]; tcolors[11] = _human->BLUE; tcolors[12] = _human->MYCOLOR[12];
			tcolors[20] = _human->MYCOLOR[13]; tcolors[21] = _human->BLUE; tcolors[22] = _human->MYCOLOR[12];
			tcolors[30] = _human->GREEN; tcolors[31] = _human->MYCOLOR[16]; tcolors[32] = _human->MYCOLOR[17];
			tcolors[40] = _human->GREEN; tcolors[41] = _human->MYCOLOR[16]; tcolors[42] = _human->MYCOLOR[17];
			tcolors[50] = _human->CYAN; tcolors[51] = _human->MYCOLOR[15]; tcolors[52] = _human->YELLOW;

			for (int idx = 0; idx < (int)_human->t.size(); ++idx) {
				_human->color_face(idx, tcolors[labels[idx]]);
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

			//for (int idx = 0; idx < (int)_human->v.size(); ++idx) {
			//	Vertex& vtx = _human->v[idx];
			//	int cnt = 0; int rgb[3] = { 0 };
			//	auto lbl = labels[_human->vt(idx, 0)];
			//	vtx.r = tcolors[lbl][0]; vtx.g = tcolors[lbl][1]; vtx.b = tcolors[lbl][2];
			//	for (int i = 0; i < _human->vtn[idx]; ++i) {
			//		int t = _human->vt(idx, i);
			//		if (t >= 0) {
			//			if (labels[t] != lbl) {
			//				vtx.r = vtx.g = vtx.b = 255; break;
			//			}
			//			//auto& tri = _human->t[t];
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
			//	for (int idx = 0; idx < (int)_human->v.size(); ++idx) {
			//		Vertex& vtx = _human->v[idx];
			//		int cnt = 0; int rgb[3] = { 0 };
			//		for (int i = 0; i < _human->vvn[idx]; ++i) {
			//			int v = _human->vv(idx, i);
			//			if (v >= 0) {
			//				Vertex& nv = _human->v[v];
			//				++cnt; rgb[0] += nv.r; rgb[1] += nv.g; rgb[2] += nv.b;
			//			}
			//		}
			//		if (cnt > 0) {
			//			rgb[0] /= cnt; rgb[1] /= cnt; rgb[2] /= cnt;
			//		}
			//		vtx.r = rgb[0]; vtx.g = rgb[1]; vtx.b = rgb[2];
			//	}
			//}

			//for (int idx = 0; idx < (int)_human->v.size(); ++idx) {
			//	Vertex& vtx = _human->v[idx];
			//	Color vcolor; vcolor[0] = vtx.r; vcolor[1] = vtx.g; vcolor[2] = vtx.b;
			//	_human->color_vertex(idx, vcolor);
			//}
		}

		//// Color face
		//Color colors[] = { _human->WHITE, _human->MYCOLOR[5], _human->MYCOLOR[1], _human->MYCOLOR[7], _human->MYCOLOR[3], _human->MYCOLOR[0],
		//	_human->MYCOLOR[6], _human->MYCOLOR[5], _human->MYCOLOR[2], _human->MYCOLOR[4] };
		//for (int part_idx = 5; part_idx >= 0; --part_idx) {
		//	for (int idx = 0; idx < (int)_human->t.size(); ++idx) {
		//		if (labels[idx] == part_idx)
		//			_human->color_face(idx, colors[part_idx]);
		//	}
		//}
		//for (int idx = 0; idx < (int)_human->t.size(); ++idx) {
		//	if (labels[idx] == 6)
		//		_human->color_face(idx, colors[6]);
		//}
	}
	void SkeletalSegmentation::cut_ring_by_plane(const IntVec& ring, const Plane& plane, Color upcolor, Color dncolor) {

		for (int i = 0; i < (int)ring.size(); ++i) {
			int t = ring[i];
			auto& tri = _human->t[t];
			for (int j = 0; j < 3; ++j) {
				int v0 = tri.v[j]; Vertex& vtx = _human->v[v0]; RGBA& vc = _human->vcolor[v0];
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
				if (point_plane_distance(plane, _human->v[v0]) * point_plane_distance(plane, _human->v[v1]) < 0) {
					vs[2 * cnt] = v0; vs[2 * cnt + 1] = v1;
					intersect[cnt++] = plane_edge_intersection(plane, v0, v1);
				}
				else {
					mid = (_human->v[v0] + _human->v[v1]) / 2;
				}
			}
			int vn = _human->v.size();
			if (vs[1] == vs[2]) {
				//auto vh0 = _human->add_vertex(intersect[0]);
				//auto vh1 = _human->add_vertex(intersect[1]);
				//auto vh2 = _human->add_vertex(mid);
				//_human->add_face(vh0, _human->vh[vs[1]], vh1);
				//_human->add_face(_human->vh[vs[0]], vh0, vh2);
				//_human->add_face(vh2, vh1, _human->vh[vs[3]]);

				_human->v.push_back(intersect[0]);
				_human->v.push_back(intersect[1]);
				_human->v.push_back(mid);
				_human->t.push_back(Triangle(vn, vs[1], vn + 1));
				_human->t.push_back(Triangle(vs[0], vn, vn + 2));
				_human->t.push_back(Triangle(vn + 2, vn + 1, vs[3]));
			}
			else if (vs[0] == vs[3]) {
				//auto vh0 = _human->add_vertex(intersect[1]);
				//auto vh1 = _human->add_vertex(intersect[0]);
				//auto vh2 = _human->add_vertex(mid);
				//_human->add_face(vh0, _human->vh[vs[0]], vh1);
				//_human->add_face(_human->vh[vs[2]], vh0, vh2);
				//_human->add_face(vh2, vh1, _human->vh[vs[1]]);

				_human->v.push_back(intersect[1]);
				_human->v.push_back(intersect[0]);
				_human->v.push_back(mid);
				_human->t.push_back(Triangle(vn, vs[0], vn + 1));
				_human->t.push_back(Triangle(vs[2], vn, vn + 2));
				_human->t.push_back(Triangle(vn + 2, vn + 1, vs[1]));
			}
			else {
				std::cout << "error in cut_ring_by_plane : unexpected situation" << std::endl;
			}
			tri.v[0] = vn; tri.v[1] = vn + 1; tri.v[2] = vn + 2;
			_human->vcolor.push_back({ upcolor });
			_human->vcolor.push_back({ upcolor });
			if (point_plane_distance(plane, mid) > 0) {
				_human->vcolor.push_back({ upcolor });
			}
			else {
				_human->vcolor.push_back({ dncolor });
			}
		}
	}
	void SkeletalSegmentation::refine_split_by_feature_point() {

		int face_num = (int)_human->t.size();
		for (auto it = _part_labels.begin(); it != _part_labels.end(); ++it) *it = -1;
		CharVec& labels = _part_labels;

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
				if (i < PartType::p_leg0 && j == 0) {
					label_faces_BFS(_shoulder_face[i - PartType::p_arm0], _shoulder_cutter[i - PartType::p_arm0], i * 10 + j, labels, -2);
				}
				else {
					int cut = _cuts[i][j];
					label_faces_BFS(_slices_face[i][cut], _planes[i][cut], i * 10 + j, labels, -2);
				}
			}
		}

		// body label
		int bodyidx = PartType::p_body;
		int cut = _cuts[bodyidx][BodyCut::c_down];
		label_faces_BFS(_slices_face[bodyidx][cut], _planes[bodyidx][cut], DetailPartType::midbody, labels, -1);
		cut = _cuts[bodyidx][BodyCut::c_up];
		label_faces_BFS(_slices_face[bodyidx][cut], _planes[bodyidx][cut], DetailPartType::upbody, labels, DetailPartType::midbody);
		for (int i = 0; i < labels.size(); ++i) {
			if (labels[i] < 0) labels[i] = DetailPartType::downbody;
		}

		if (PAINT_MESH_USING_LABEL)
		{
			Color tcolors[256];

			tcolors[0] = _human->MYCOLOR[11]; tcolors[1] = _human->RED;
			tcolors[10] = _human->MYCOLOR[13]; tcolors[11] = _human->BLUE; tcolors[12] = _human->MYCOLOR[12];
			tcolors[20] = _human->MYCOLOR[13]; tcolors[21] = _human->BLUE; tcolors[22] = _human->MYCOLOR[12];
			tcolors[30] = _human->GREEN; tcolors[31] = _human->MYCOLOR[16]; tcolors[32] = _human->MYCOLOR[17];
			tcolors[40] = _human->GREEN; tcolors[41] = _human->MYCOLOR[16]; tcolors[42] = _human->MYCOLOR[17];
			tcolors[50] = _human->CYAN; tcolors[51] = _human->MYCOLOR[15]; tcolors[52] = _human->YELLOW;

			for (int idx = 0; idx < (int)_human->t.size(); ++idx) {
				_human->color_face(idx, tcolors[labels[idx]]);
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
							if ((idx == PartType::p_arm0 || idx == PartType::p_arm1) && i == 0) {
								cut_ring_by_plane(_shoulder_face[idx - PartType::p_arm0], _shoulder_cutter[idx - PartType::p_arm0], tcolors[colori[cnt][0]], tcolors[colori[cnt][1]]);
							}
							else {
								int cut = _cuts[idx][i];
								cut_ring_by_plane(_slices_face[idx][cut], _planes[idx][cut], tcolors[colori[cnt][0]], tcolors[colori[cnt][1]]);
							}
							++cnt;
						}
					}
				}
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
		for (int i = 0; i < (int)_human->v.size(); ++i) {
			bool flag = true;
			for (int j = 0; j < _human->vvn[i]; ++j) {
				int n = _human->vv(i, j);
				if (n != i && hks[i] <= hks[n]) flag = false;
				//for (int k = 0; k < _human->vvn[n]; ++k) {
				//	int nn = _human->vv(n, k);
				//	if (nn != i && hks[i] <= hks[nn]) flag = false;
				//}
			}
			if (flag) {
				feature_points.push_back(i);
				//paint_ball(_human->v[i], BALL_RADIUS, BALL_CUTS, _human->WHITE);
				//_human->color_vertex(i, _human->PURPLE);
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
			int vidx = this->_human->t[face].v[0]; int ret = vidx;
			float mindist = this->point_plane_distance(plane, this->_human->v[vidx]);
			for (int i = 1; i < 3; ++i) {
				vidx = this->_human->t[face].v[i];
				float dist = this->point_plane_distance(plane, this->_human->v[vidx]);
				if (mindist > dist) { mindist = dist; ret = vidx; }
			}
			return ret;
		};

		auto find_feature = [closest_vertex_in_triangle, this](int part, int cut, int size, int* dest, int* out) {
			IntVec& slice = _slices_face[part][cut];
			for (int i = 0; i < size; ++i) {
				float min_dist = DECIMAL_MAX; int vtx = -1, face = -1;
				for (auto it = slice.begin(); it != slice.end(); ++it) {
					//float dist = (_human->triangle_center(*it) - _human->v[dest[i]]).square_length();
					//if (dist < min_dist) { min_dist = dist; face = *it; }
					for (int j = 0; j < 3; ++j) {
						int v = _human->t[*it].v[j];
						float dist = this->geodesic_dist(v, dest[i]);
						//float dist = (_human->v[v] - _human->v[dest[i]]).square_length();
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
				float dist = (_human->triangle_center(*it) - _human->v[dest]).square_length();
				if (dist < min_dist) { min_dist = dist; face = *it; }
			}
			out[0] = closest_vertex_in_triangle(part, cut, face);
			float max_dist = DECIMAL_MIN; face = -1;
			for (auto it = slice.begin(); it != slice.end(); ++it) {
				float dist = (_human->triangle_center(*it) - _human->v[dest]).square_length();
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
					Vertex tri_cen = this->_human->triangle_center(*it);
					int temp = this->face_cross_plane(*it, plane0);
					if (temp > 0 && temp < 3) {
						bool flag = (this->point_plane_distance(plane1, tri_cen) > 0);
						if (i % 2 == 0 && flag || i % 2 == 1 && !flag) {
							out[i] = _human->t[*it].v[0];
						}
					}
				}
				//this->_human->color_vertex(out[i], _human->BLUE);
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
					Vertex tri_cen = this->_human->triangle_center(*it);
					int temp = this->face_cross_plane(*it, plane0);
					if (temp > 0 && temp < 3) {
						bool flag = (this->point_plane_distance(plane1, tri_cen) > 0);
						if (i % 2 == 0 && flag || i % 2 == 1 && !flag) {
							out[i] = _human->t[*it].v[0];
						}
					}
				}
				//this->_human->color_vertex(out[i], _human->BLUE);
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
		find_feature_plane2(PartType::p_head, _cuts[PartType::p_head][HeadCut::c_neck], _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]], _landmarks.neck);
		//paint_ball(_landmarks.neck[0], neck_ring, BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[0]);
		//paint_ball(_landmarks.neck[1], neck_ring, BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[0]);
		//paint_ball(_landmarks.neck[2], neck_ring, BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[0]);
		//paint_ball(_landmarks.neck[3], neck_ring, BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[0]);
		find_neck();

		// find shoulder
		find_shoulder();

		// chest
		int chest = _cuts[PartType::p_body][BodyCut::c_up];
		find_feature_plane2(PartType::p_body, chest, _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]], _landmarks.chest);
		//paint_ball(_landmarks.chest[2], upbody_ring, BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[1]);
		//paint_ball(_landmarks.chest[3], upbody_ring, BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[1]);
		find_chest();

		refine_split_by_feature_point();

		return;

		// shoulder
		if (REFINE_SHOULDER) {
			if (_landmarks.shoulder[0] >= 0) paint_ball(_landmarks.shoulder[0], arm_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[9]);
			if (_landmarks.shoulder[1] >= 0) paint_ball(_landmarks.shoulder[1], arm_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[9]);
		}

		// armpit
		paint_ball(_landmarks.armpit[0], arm_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[9]);
		paint_ball(_landmarks.armpit[1], arm_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[9]);
		// crotch
		paint_ball(_landmarks.crotch, leg_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[5]);
		// waist
		int waist = _cuts[PartType::p_body][BodyCut::c_waist];
		//find_feature(PartType::p_body, waist, 2, _landmarks.armpit, _landmarks.waist);
		find_feature_plane2(PartType::p_body, waist, _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]], _landmarks.waist);
		paint_ball(_landmarks.waist[0], midbody_ring, BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[2]);
		paint_ball(_landmarks.waist[1], midbody_ring, BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[2]);
		// hip
		int hip = _cuts[PartType::p_body][BodyCut::c_down];
		//find_feature(PartType::p_body, hip, 2, _landmarks.armpit, _landmarks.hip);
		find_feature_plane2(PartType::p_body, hip, _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]], _landmarks.hip);
		paint_ball(_landmarks.hip[0], dnbody_ring, BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[8]);
		paint_ball(_landmarks.hip[1], dnbody_ring, BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[8]);
		// back high point
		//find_feature_plane2(PartType::p_body, (waist + navel_cut) / 2, _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]], _landmarks.waist_high_back);
		//paint_ball(_human->v[_landmarks.waist_high_back[2]], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[2]);
		// back low point
		//find_feature_plane2(PartType::p_body, (hip + navel_cut) / 2, _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]], _landmarks.waist_low_back);
		//paint_ball(_human->v[_landmarks.waist_low_back[2]], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[8]);
		//paint_ball(_human->v[_landmarks.waist_low_back[3]], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[8]);
		// leg
		//find_feature_plane(PartType::p_leg0, _cuts[PartType::p_leg0][0], _landmarks.leg0);
		//find_feature_plane(PartType::p_leg1, _cuts[PartType::p_leg1][0], _landmarks.leg1);
		int leg0[2] = { 0 }, leg1[2] = { 0 };
		find_feature(PartType::p_leg0, _cuts[PartType::p_leg0][0], 2, _landmarks.hip, leg0);
		find_feature(PartType::p_leg1, _cuts[PartType::p_leg1][0], 2, _landmarks.hip, leg1);
		_landmarks.leg[0] = (point_distance(leg0[0], _landmarks.crotch) > point_distance(leg0[1], _landmarks.crotch) ? leg0[0] : leg0[1]);
		_landmarks.leg[1] = (point_distance(leg1[0], _landmarks.crotch) > point_distance(leg1[1], _landmarks.crotch) ? leg1[0] : leg1[1]);
		paint_ball(_landmarks.leg[0], leg_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[5]);
		paint_ball(_landmarks.leg[1], leg_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[5]);
		// knee
		//int knee_dest0[2] = { _landmarks.leg[0], _landmarks.crotch };
		//int knee_dest1[2] = { _landmarks.leg[1], _landmarks.crotch };
		//find_feature(PartType::p_leg0, _cuts[PartType::p_leg0][1], 2, knee_dest0, _landmarks.knee0);
		//find_feature(PartType::p_leg1, _cuts[PartType::p_leg1][1], 2, knee_dest1, _landmarks.knee1);
		//find_feature(PartType::p_leg0, _cuts[PartType::p_leg0][1], 1, &_landmarks.heel[0], &_landmarks.knee0[2]);
		//find_feature(PartType::p_leg1, _cuts[PartType::p_leg1][1], 1, &_landmarks.heel[1], &_landmarks.knee1[2]);
		find_feature_plane(PartType::p_leg0, _cuts[PartType::p_leg0][1], _landmarks.knee0);
		find_feature_plane(PartType::p_leg1, _cuts[PartType::p_leg1][1], _landmarks.knee1);
		paint_ball(_landmarks.knee0[0], knee_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[6]);
		paint_ball(_landmarks.knee0[1], knee_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[6]);
		paint_ball(_landmarks.knee0[2], knee_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[6]);
		paint_ball(_landmarks.knee0[3], knee_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[6]);
		paint_ball(_landmarks.knee1[0], knee_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[6]);
		paint_ball(_landmarks.knee1[1], knee_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[6]);
		paint_ball(_landmarks.knee1[2], knee_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[6]);
		paint_ball(_landmarks.knee1[3], knee_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[6]);
		// ankle
		//find_feature(PartType::p_leg0, _cuts[PartType::p_leg0][2], 2, _landmarks.knee0, _landmarks.ankle0);
		//find_feature(PartType::p_leg1, _cuts[PartType::p_leg1][2], 2, _landmarks.knee1, _landmarks.ankle1);
		find_feature_plane(PartType::p_leg0, _cuts[PartType::p_leg0][2], _landmarks.ankle0);
		find_feature_plane(PartType::p_leg1, _cuts[PartType::p_leg1][2], _landmarks.ankle1);
		paint_ball(_landmarks.ankle0[0], ankle_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[7]);
		paint_ball(_landmarks.ankle0[1], ankle_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[7]);
		paint_ball(_landmarks.ankle0[2], ankle_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[7]);
		paint_ball(_landmarks.ankle0[3], ankle_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[7]);
		paint_ball(_landmarks.ankle1[0], ankle_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[7]);
		paint_ball(_landmarks.ankle1[1], ankle_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[7]);
		paint_ball(_landmarks.ankle1[2], ankle_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[7]);
		paint_ball(_landmarks.ankle1[3], ankle_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[7]);
		// head
		find_feature(PartType::p_head, _cuts[PartType::p_head][0], 2, _landmarks.neck, _landmarks.head);
		find_feature_plane2(PartType::p_head, _cuts[PartType::p_head][0], _human->v[_landmarks.head[0]], _human->v[_landmarks.head[1]], _landmarks.head);
		paint_ball(_landmarks.head[0], head_ring, BALL_RADIUS, BALL_CUTS, _human->RED);
		paint_ball(_landmarks.head[1], head_ring, BALL_RADIUS, BALL_CUTS, _human->RED);
		// elbow
		int elbow_dest0[2] = { _landmarks.shoulder[0], _landmarks.armpit[0] };
		int elbow_dest1[2] = { _landmarks.shoulder[1], _landmarks.armpit[1] };
		//find_feature(PartType::p_arm0, _cuts[PartType::p_arm0][1], elbow_dest0, _landmarks.elbow0);
		//find_feature(PartType::p_arm1, _cuts[PartType::p_arm1][1], elbow_dest1, _landmarks.elbow1);
		//find_feature_near_far(PartType::p_arm0, _cuts[PartType::p_arm0][1], _landmarks.armpit[0], _landmarks.elbow0);
		//find_feature_near_far(PartType::p_arm1, _cuts[PartType::p_arm1][1], _landmarks.armpit[1], _landmarks.elbow1);
		find_feature_plane(PartType::p_arm0, _cuts[PartType::p_arm0][1], _landmarks.elbow0);
		find_feature_plane(PartType::p_arm1, _cuts[PartType::p_arm1][1], _landmarks.elbow1);
		paint_ball(_landmarks.elbow0[0], elbow_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[3]);
		paint_ball(_landmarks.elbow0[1], elbow_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[3]);
		paint_ball(_landmarks.elbow0[2], elbow_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[3]);
		paint_ball(_landmarks.elbow0[3], elbow_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[3]);
		paint_ball(_landmarks.elbow1[0], elbow_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[3]);
		paint_ball(_landmarks.elbow1[1], elbow_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[3]);
		paint_ball(_landmarks.elbow1[2], elbow_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[3]);
		paint_ball(_landmarks.elbow1[3], elbow_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[3]);
		// wrist
		find_feature(PartType::p_arm0, _cuts[PartType::p_arm0][2], 2, _landmarks.elbow0, _landmarks.wrist0);
		find_feature(PartType::p_arm1, _cuts[PartType::p_arm1][2], 2, _landmarks.elbow1, _landmarks.wrist1);
		find_feature_plane(PartType::p_arm0, _cuts[PartType::p_arm0][2], _landmarks.wrist0);
		find_feature_plane(PartType::p_arm1, _cuts[PartType::p_arm1][2], _landmarks.wrist1);
		paint_ball(_landmarks.wrist0[2], wrist_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[4]);
		paint_ball(_landmarks.wrist0[3], wrist_ring[0], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[4]);
		paint_ball(_landmarks.wrist1[2], wrist_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[4]);
		paint_ball(_landmarks.wrist1[3], wrist_ring[1], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[4]);

		//int foot0 = -1, foot1 = -1;// vertex index
		//IntVec hips, heads;
		//for (auto it = feature_points.begin(); it != feature_points.end(); ++it) {
		//	int f = _human->vt(*it, 0);
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
		//				int v = _human->t[face].v[m];
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
		//	//_human->color_face(face, _human->RED);
		//}
		//// len
		//std::cout << "head len : " << point_plane_distance(_planes[0][_cuts[0][0]], _human->v[heads[0]]) << std::endl;
		//std::cout << "3 hip len : " << point_plane_distance(_planes[3][_cuts[3][0]], _human->v[hips[1]]) << std::endl;
		//std::cout << "4 hip len : " << point_plane_distance(_planes[4][_cuts[4][0]], _human->v[hips[1]]) << std::endl;
		//std::cout << 1 << " arm len : " << (_skeleton[1][_cuts[1][0]] - _skeleton[1][_cuts[1][1]]).length()
		//	<< "  forearm len : " << (_skeleton[1][_cuts[1][1]] - _skeleton[1][_cuts[1][2]]).length() << std::endl;
		//std::cout << 2 << " arm len : " << (_skeleton[2][_cuts[2][0]] - _skeleton[2][_cuts[2][1]]).length()
		//	<< "  forearm len : " << (_skeleton[2][_cuts[2][1]] - _skeleton[2][_cuts[2][2]]).length() << std::endl;
		//std::cout << "neck to hip : " << (_skeleton[5][MANUAL_BODY_CUT_DOWN] - _skeleton[0][_cuts[0][0]]).length() << std::endl;
		//std::cout << "shoulder to hip : " << (_skeleton[5][MANUAL_BODY_CUT_DOWN] - _skeleton[0][shoulder]).length() << std::endl;
		//std::cout << "3 foot len : " << point_plane_distance(_planes[3][_cuts[3][2]], _human->v[foot0]) << std::endl;
		//std::cout << "4 foot len : " << point_plane_distance(_planes[4][_cuts[4][2]], _human->v[foot1]) << std::endl;
	}
	void SkeletalSegmentation::HKS_paint_features() {
		FILE* fp = fopen("hks.txt", "rb");
		std::vector<double> hks(_human->v.size());
		fread(&hks[0], _human->v.size() * sizeof(double), 1, fp);
		fclose(fp);
		IntVec features;
		for (int i = 0; i < (int)_human->v.size(); ++i) {
			bool flag = true;
			for (int j = 0; j < _human->vvn[i]; ++j) {
				int n = _human->vv(i, j);
				if (n != i && hks[i] <= hks[n]) flag = false;
				/*for (int k = 0; k < mesh.vvn[n]; ++k) {
				int nn = mesh.vv(n, k);
				if (nn != i && hks[i] <= hks[nn]) flag = false;
				}*/
			}
			if (flag) features.push_back(i);
			//if (flag) _human->color_vertex(i, _human->RED);
			//else _human->color_vertex(i, _human->WHITE);
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
					int v = _human->t[f].v[j];
					for (auto f_op : _slices_face[leg_id_op][_cuts[leg_id_op][leg_cut]]) {
						for (int k = 0; k < 3; ++k) {
							int v_op = _human->t[f_op].v[k];
							float dist = (_human->v[v] - _human->v[v_op]).square_length();
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
		paint_ball(_human->v[_landmarks.crotch], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[25]);
		////
		//Vertex cen(0, 0, 0);
		//for (auto v : pts) {
		//	cen += _human->v[v];
		//}
		//cen /= pts.size(); min_dist = DECIMAL_MAX;
		//for (int i = 0; i < 2; ++i) {
		//	int leg_id = PartType::p_leg0 + i;
		//	for (auto f : _slices_face[leg_id][_cuts[leg_id][leg_cut]]) {
		//		for (int j = 0; j < 3; ++j) {
		//			int v = _human->t[f].v[j];
		//			float dist = (cen - _human->v[v]).square_length();
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
		//	Vertex tri_cen = _human->triangle_center(*it);
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
					int v = _human->t[*it].v[j];
					for (auto k : arm_slice_vtx) {
						float dist2 = (k - _human->v[v]).length();
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
			paint_ball(_human->v[_landmarks.armpit[i]], BALL_RADIUS, BALL_CUTS, _human->MYCOLOR[29]);
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
		for (int i = 0; i < _human->t.size(); ++i)
		{
			if (_human->t[i].v[0] < 0 || _human->t[i].v[1] < 0 || _human->t[i].v[2] < 0) continue;
			const Vertex& v0 = _human->v[_human->t[i].v[0]];
			const Vertex& v1 = _human->v[_human->t[i].v[1]];
			const Vertex& v2 = _human->v[_human->t[i].v[2]];
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
		//	int f = _human->vt(*it, 0);
		//	*pit = labels_detail[f];
		//	if (*pit == DetailPartType::upbody) {
		//		float dist = (_skeleton[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_arm]] - _human->v[*it]).square_length();
		//		float dist2 = (_skeleton[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_arm]] - _human->v[*it]).square_length();
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
		//	int f = _human->vt(*it, 0);
		//	*pit = labels_detail[f];
		//	if (*pit == DetailPartType::upbody || *pit == DetailPartType::arm0 || *pit == DetailPartType::arm1) {
		//		float dist, dist2;
		//		int pos = closest_skept(PartType::p_head, _human->v[*it], dist);
		//		int pos2 = closest_skept(PartType::p_body, _human->v[*it], dist2);
		//		if (dist > dist2) pos = pos2 + (int)_skeleton[PartType::p_head].size();
		//		dist = (_skeleton[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_arm]] - _human->v[*it]).square_length();
		//		dist2 = (_skeleton[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_arm]] - _human->v[*it]).square_length();
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
		//		float dist = (*it - _human->v[_landmarks.armpit[i]]).square_length();
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
		//		int f = _human->vt(*it, 0);
		//		*pit = labels_detail[f];
		//		if (*pit == DetailPartType::upbody || *pit == DetailPartType::arm0 || *pit == DetailPartType::arm1) {
		//			float dist = (_human->v[*it] - shoulder_seed[i]).square_length();
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
		const Vertex ap[2] = { _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]] };
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
			BoolVec visited(_human->v.size(), false);
			ArrayQueue<int> Q(_human->v.size());
			IntVec points;
			for (int i = 0; i < _human->v.size(); ++i) {
				int n = i;
				float temp = point_plane_distance(border[idx][0], _human->v[n]);
				float temp2 = point_plane_distance(border[idx][1], _human->v[n]);
				float temp3 = point_plane_distance(border[idx][2], _human->v[n]);
				char label = _part_labels[_human->vt(n, 0)];
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
			//	for (int i = 0; i < _human->vvn[curr]; i++) {
			//		int n = _human->vv(curr, i);
			//		float temp = point_plane_distance(border[idx][0], _human->v[n]);
			//		float temp2 = point_plane_distance(border[idx][1], _human->v[n]);
			//		float temp3 = point_plane_distance(border[idx][2], _human->v[n]);
			//		char label = _part_labels[_human->vt(n, 0)];
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
				Vertex necks[] = { _human->v[_landmarks.neck[0]], _human->v[_landmarks.neck[1]] };
				Vertex neck = point_plane_distance(arm_mid[idx], necks[0]) < point_plane_distance(arm_mid[idx], necks[1]) ? necks[0] : necks[1];
				float min_dist = DECIMAL_MAX;
				Vertex side(0, 0, 0);
				for (auto v : points) {
					float dist = point_plane_distance(arm_mid[idx], _human->v[v]);
					if (dist < min_dist) { min_dist = dist; side = _human->v[v]; }
				}
				border[idx][3].p = side;
				border[idx][3].n = (axis.cross((neck - side).normalized())).normalized();
				float cosine = _planes[PartType::p_body][SLICE_INTERVAL].n.dot(border[idx][3].n);
				cosine /= (_planes[PartType::p_body][SLICE_INTERVAL].n.length() * border[idx][3].n.length());
				if (cosine < 0) border[idx][3].n *= -1;
				for (auto v : points) {
					float temp = point_plane_distance(border[idx][3], _human->v[v]);
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
			//	int tri = intersect_triangle(_human->v[_landmarks.armpit[idx]], border[idx][2].n);
			//	if (tri >= 0) get_candidates_geodesic(_human->t[tri].v[0], _parms[PartType::p_arm0 + idx][_cuts[PartType::p_arm0 + idx][ArmCut::c_arm]].avg_r * 2, candidates);
			//}
			for (auto v : candidates) {
				_human->color_vertex(v, _human->RED);
			}
			if (candidates.size() > 0) extract_landmark_from_candidates(candidates, LandMarksID::lm_shoulder0 + idx, _landmarks.shoulder[idx]);
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
					if (PAINT_SEG_OVER_MESH) paint_ring_vertices(_planes[i + 1][_cuts[i + 1][ArmCut::c_arm]].n, arm_ring[i], _human->PURPLE);
					float maxdist = DECIMAL_MIN;
					Vertex shoulder_vtx(0, 0, 0);
					for (auto it = arm_ring[i].begin(); it != arm_ring[i].end(); ++it) {
						float dist = (*it - _human->v[_landmarks.armpit[i]]).square_length();
						if (dist > maxdist) {
							maxdist = dist; shoulder_vtx = *it;
						}
					}
					//if (PAINT_BALL) paint_ball(shoulder_vtx, BALL_RADIUS, BALL_CUTS, _human->PURPLE);
				}
				else {
					//if (PAINT_BALL) paint_ball(_human->v[_landmarks.shoulder[i]], BALL_RADIUS, BALL_CUTS, _human->BLUE);
					int ske_idx = _cuts[i + 1][ArmCut::c_arm];
					Vertex ske_head = _skeleton[i + 1][ske_idx - SLICE_INTERVAL];
					Vertex ske_mid = _skeleton[i + 1][ske_idx];
					Vertex ske_tail = _skeleton[i + 1][ske_idx + SLICE_INTERVAL];
					Vector axis = (ske_head - ske_mid).cross(ske_tail - ske_mid);
					Vertex cen = _human->v[_landmarks.armpit[i]] + (_skeleton[i + 1][ske_idx - 1] - _skeleton[i + 1][ske_idx]);
					Plane plane = create_plane_cross(cen, axis, _human->v[_landmarks.shoulder[i]] - cen);
					_shoulder_cutter[i] = plane;
					IntVec ring_face;
					arm_ring[i].clear();
					search_loop(plane, _human->v[_landmarks.shoulder[i]], ring_face, arm_ring[i]);
					_shoulder_face[i] = ring_face;
					if (PAINT_SEG_OVER_MESH) paint_ring_vertices(plane.n, arm_ring[i], _human->PURPLE);
				}
			}
		}
		else if (PAINT_SEG_OVER_MESH) {
			paint_ring_vertices(_planes[PartType::p_arm0][_cuts[PartType::p_arm0][ArmCut::c_arm]].n, arm_ring[0], _human->PURPLE);
			paint_ring_vertices(_planes[PartType::p_arm1][_cuts[PartType::p_arm1][ArmCut::c_arm]].n, arm_ring[1], _human->PURPLE);
		}
	}
	void SkeletalSegmentation::find_chest() {

		_landmarks.nipple[0] = _landmarks.nipple[1] = -1;
		IntVec& feature_points = _hks_points;
		CharVec& labels = _part_labels;

		const Vertex ap[2] = { _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]] };
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
		for (int i = 0; i < _human->v.size(); ++i) {
			int f = _human->vt(i, 0);
			if (labels[f] == DetailPartType::upbody) {
				Vertex v = _human->v[i];
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
		//	int f = _human->vt(i, 0);
		//	Vertex v = _human->v[i];
		//	if (labels[f] == part_id && i != _landmarks.shoulder[0] && i != _landmarks.shoulder[1]
		//		&& point_plane_distance(plane_down, v) > 0 && point_plane_distance(plane_up, v) > 0 && point_plane_distance(plane_FB, v) > 0) {
		//		paint_ball(v, BALL_RADIUS, BALL_CUTS, _human->YELLOW);
		//	}
		//}
		//// curvature
		//float max_val[2] = { DECIMAL_MIN, DECIMAL_MIN };
		//int max_idx[2] = { -1, -1 };
		//for (int i = 0; i < _human->v.size(); ++i) {
		//	int f = _human->vt(i, 0);
		//	if (labels[f] == DetailPartType::upbody) {
		//		Vertex v = _human->v[i];
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
		if (point_plane_distance(plane_FB, _human->v[_landmarks.chest[2]]) > 0) {
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
		const Vertex sd[2] = { _human->v[_landmarks.shoulder[0]], _human->v[_landmarks.shoulder[1]] };
		Plane plane_arm[2]; plane_arm[0].p = ap[0]; plane_arm[1].p = ap[1];
		plane_arm[0].n = ((sd[0] - ap[0]).cross((sd[0] - ap[0]).cross(ap[1] - ap[0]))).normalized();
		plane_arm[1].n = ((sd[1] - ap[1]).cross((sd[1] - ap[1]).cross(ap[0] - ap[1]))).normalized();
		if (plane_arm[0].n.dot((ap[1] - ap[0]).normalized()) < 0) plane_arm[0].n *= -1;
		if (plane_arm[1].n.dot((ap[0] - ap[1]).normalized()) < 0) plane_arm[1].n *= -1;
		for (int i = 0; i < _human->v.size(); ++i) {
			int f = _human->vt(i, 0);
			if (labels[f] == DetailPartType::upbody) {
				Vertex v = _human->v[i];
				if (point_plane_distance(plane_down, v) > 0 && point_plane_distance(plane_chest, v) > 0
					&& point_plane_distance(plane_arm[0], v) > 0 && point_plane_distance(plane_arm[1], v) > 0) {
					if (point_plane_distance(plane_LR, v) > 0) {
						_human->color_vertex(i, _human->BLUE);
					}
					else {
						_human->color_vertex(i, _human->GREEN);
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
			for (int v = 0; v < _human->v.size(); ++v) {
				if (labels[_human->vt(v, 0)] == part_id) {
					float dist = (center[i] - _human->v[v]).square_length();
					if (dist > max_dist) {
						max_dist = dist; toe = _human->v[v];
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
		//		int f = _human->vt(v, 0);
		//		if (labels[f] == part_id) {
		//			float dist = (center[i] - _human->v[v]).square_length();
		//			if (point_plane_distance(plane_toe[i], _human->v[v]) < 0 && point_plane_distance(plane_ankle[i], _human->v[v]) > 0 && dist < min_dist) {
		//				hks_heel[i] = v; min_dist = dist;
		//			}
		//			if (point_plane_distance(plane_toe[i], _human->v[v]) > 0 && dist > max_dist) {
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
		//	for (int v = 0; v < _human->v.size(); ++v) {
		//		int f = _human->vt(v, 0);
		//		if (labels[f] == part_id) {
		//			float val = _curvature_max[v];
		//			if (point_plane_distance(plane_toe[i], _human->v[v]) > 0) {
		//				if (val > max_val_toe) {
		//					max_val_toe = val; cvt_toe[i] = v;
		//				}
		//			}
		//			if (point_plane_distance(plane_heel[i], _human->v[v]) < 0 && point_plane_distance(plane_ankle[i], _human->v[v]) > 0) {
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
			for (int v = 0; v < _human->v.size(); ++v) {
				int f = _human->vt(v, 0);
				if (labels[f] == part_id) {
					if (point_plane_distance(plane_toe[i], _human->v[v]) > 0) {
						//candidates[0].push_back(v);
					}
					if (point_plane_distance(plane_heel[i], _human->v[v]) < 0 && point_plane_distance(plane_ankle[i], _human->v[v]) > 0) {
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
		//		paint_ball(_human->v[_landmarks.toe[i]], BALL_RADIUS, BALL_CUTS, _human->YELLOW);
		//	if (_landmarks.heel[i] >= 0)
		//		paint_ball(_human->v[_landmarks.heel[i]], BALL_RADIUS, BALL_CUTS, _human->YELLOW);
		//}
	}
	void SkeletalSegmentation::find_hip() {

		_landmarks.gluteal[0] = _landmarks.gluteal[1] = -1;
		IntVec& feature_points = _hks_points;
		CharVec& labels = _part_labels;

		Plane plane_up = _planes[PartType::p_body][_cuts[PartType::p_body][BodyCut::c_down]]; plane_up.n *= -1;
		Plane plane_dn = plane_up; plane_dn.p += Vector(0, -1500, 0); plane_dn.n *= -1;
		const Vertex ap[2] = { _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]] };
		Vertex ske = _skeleton[PartType::p_body][_skeleton[PartType::p_body].size() - 1];
		Plane plane_LR; plane_LR.p = ske; plane_LR.n = (ap[1] - ap[0]).normalized();
		Plane plane_FB = create_plane_cross(ske, ap[0] - ske, ap[1] - ske);
		if (plane_FB.n.z > 0) plane_FB.n *= -1;
		Vertex side_vtx_max(0, 0, 0), side_vtx_min(0, 0, 0);
		float side_dist_max = DECIMAL_MIN, side_dist_min = DECIMAL_MAX;
		for (int i = 0; i < _human->v.size(); ++i) {
			int f = _human->vt(i, 0);
			if (labels[f] == DetailPartType::downbody) {
				float dist = point_plane_distance(plane_LR, _human->v[i]);
				if (dist > side_dist_max) {
					side_vtx_max = _human->v[i]; side_dist_max = dist;
				}
				if (dist < side_dist_min) {
					side_vtx_min = _human->v[i]; side_dist_min = dist;
				}
			}
		}
		Plane plane_side[2]; plane_side[0].n = plane_LR.n*-1; plane_side[1].n = plane_LR.n;
		plane_side[0].p = side_vtx_max + plane_side[0].n * (side_dist_max * (float)0.3);
		plane_side[1].p = side_vtx_min + plane_side[1].n * (side_dist_min * (float)0.3);

		IntVec candidates[2];
		for (int i = 0; i < _human->v.size(); ++i) {
			int f = _human->vt(i, 0);
			if (labels[f] == DetailPartType::downbody) {
				Vertex v = _human->v[i];
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
		//		int f = _human->vt(*it, 0);
		//		if (labels_detail[f] == DetailPartType::midbody) {
		//			_landmarks.navel[i] = *it;
		//		}
		//	}
		//}
		//int navel_cut = -1;
		//for (int i = 0; i < (int)_slices_face[PartType::p_body].size(); ++i) {
		//	for (auto it = _slices_face[PartType::p_body][i].begin(); it != _slices_face[PartType::p_body][i].end(); ++it) {
		//		for (int j = 0; j < 3; ++j) {
		//			int v = _human->t[*it].v[j];
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
		const Vertex ap[2] = { _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]] };
		Vertex neck_cen = _parms[PartType::p_head][_cuts[PartType::p_head][HeadCut::c_neck]].center;
		Plane plane_FB = create_plane_cross(neck_cen, ap[0] - neck_cen, ap[1] - neck_cen);
		if (plane_FB.n.z > 0) plane_FB.n *= -1;

		int neck_front = -1, neck_back = -1;
		if (point_plane_distance(plane_FB, _human->v[_landmarks.neck[2]]) > 0) {
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
				if (point_plane_distance(plane_up, _human->v[v]) > 0)
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
					Vertex tri_cen = this->_human->triangle_center(*it);
					int temp = this->face_cross_plane(*it, plane0);
					if (temp > 0 && temp < 3) {
						bool flag = (this->point_plane_distance(plane1, tri_cen) > 0);
						if (i % 2 == 0 && flag || i % 2 == 1 && !flag) {
							int vtx = -1; float min_dist = DECIMAL_MAX;
							for (int v = 0; v < 3; ++v) {
								for (Vertex sv : slice_vtx) {
									float dist = (_human->v[_human->t[*it].v[v]] - sv).square_length();
									if (dist < min_dist) {
										min_dist = dist; vtx = _human->t[*it].v[v];
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
		min_dist = dist; heel = _human->v[v];
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
		int v = _human->t[t].v[k];
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
						int v = _human->t[t].v[k];
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

		FltVec gd_dist((int)_human->v.size(), DECIMAL_MAX);
		Vertex& seedv = _human->v[vtx_seed];
		std::vector<bool> visited((int)_human->v.size(), false);
		ArrayQueue<int> Q((int)_human->v.size());
		gd_dist[vtx_seed] = 0;
		visited[vtx_seed] = true;
		Q.push(vtx_seed);
		while (!Q.empty()) {
			int curr = Q.front(); Q.pop();
			if (gd_dist[curr] > max_dist * (float)1.5) continue;
			for (int i = 0; i < _human->vvn[curr]; i++) {
				int n = _human->vv(curr, i);
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
		for (int i = 0; i < _human->v.size(); ++i) {
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
			_human->MYCOLOR[20], _human->RED, _human->MYCOLOR[21], _human->YELLOW, _human->YELLOW, _human->CYAN, _human->CYAN,
			_human->PURPLE, _human->PURPLE, _human->MYCOLOR[30], _human->MYCOLOR[30], _human->BLUE, _human->BLUE,
			_human->GREEN, _human->GREEN, _human->GREEN, _human->GREEN,
			_human->MYCOLOR[26], _human->MYCOLOR[26], _human->MYCOLOR[26], _human->MYCOLOR[26]
		};

		for (auto v : candidates) {
			_human->color_vertex(v, _human->RED);
		}
		// plane
		const Vertex ap[2] = { _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]] };
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
					//paint_ball(_human->v[out_idx], BALL_RADIUS, BALL_CUTS, _human->RED);
				}
			}
		}
		else {
			for (int i = 0; i < cvt_amount; ++i) {
				if (index_min[i] >= 0) {
					ret[i] = index_min[i];
					//paint_ball(_human->v[out_idx], BALL_RADIUS, BALL_CUTS, _human->BLUE);
				}
			}
		}
		// hks
		float mind = DECIMAL_MAX;
		for (auto c : candidates) {
			for (auto v : _hks_points) {
				if (c == v) {
					if (landmark_id == LandMarksID::lm_shoulder0 || landmark_id == LandMarksID::lm_shoulder1) {
						const Vertex ap[2] = { _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]] };
						Plane arm_mid = { (ap[0] + ap[1]) / 2, (ap[1] - ap[0]).normalized() };
						float dist = point_plane_distance(arm_mid, _human->v[c]);
						if (_human->v[c].y < mind) {
							std::cout << landmark_id << "shoulder candidate " << std::endl;
							mind = dist; ret[cvt_amount] = c;
							break;
						}
					}
					else ret[cvt_amount] = v;
				}
			}
		}

		Color colors[] = { _human->BLUE, _human->RED, _human->GREEN, _human->CYAN, _human->PURPLE, _human->YELLOW };

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
				if (point_plane_distance(plane_LR, _human->v[_ground_truths[offset]]) > 0) {
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
				for (int v = 0; v < _human->v.size(); ++v) {
					float dist = point_distance(v, _ground_truths[offset]);
					if (dist < min_dist && _part_labels[_human->vt(v, 0)] <= DetailPartType::hand1
						&& _part_labels[_human->vt(v, 0)] >= DetailPartType::arm0) {
						min_dist = dist; min_vtx = v;
					}
				}
				int temp[2] = { -1, -1 };
				if (_part_labels[_human->vt(min_vtx, 0)] < DetailPartType::arm1) {
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
				int t1 = _human->vt(gt1, 0), t2 = _human->vt(gt2, 0);
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
			else paint_ball(_human->v[gt], BALL_RADIUS, BALL_CUTS, _human->BLACK);

			for (int i = 0; i < ret.size(); ++i) {
				if (ret[i] >= 0 && gt >= 0) {
					Vertex v1 = _human->v[ret[i]], v2 = _human->v[gt];
					double dist1 = (double)v1.x - (double)v2.x;
					double dist2 = (double)v1.y - (double)v2.y;
					double dist3 = (double)v1.z - (double)v2.z;
					double error = sqrt(dist1*dist1 + dist2*dist2 + dist3*dist3);
					_landmark_errors[landmark_id][i] = error;
					paint_ball(_human->v[ret[i]], BALL_RADIUS, BALL_CUTS, colors[i]);
				}
				else _landmark_errors[landmark_id][i] = INF_ERR;
			}
		}

		if (FIND_FEATURE_POINTS) {
			for (int i = 0; i < ret.size(); ++i) {
				if (ret[i] >= 0) {
					paint_ball(_human->v[ret[i]], BALL_RADIUS, BALL_CUTS, landmark_colors[landmark_id]);
				}
			}
		}

		if (landmark_id == LandMarksID::lm_heel0 || landmark_id == LandMarksID::lm_heel1) {
			out_idx = ret[CurvatureType::c_cvt_type_amount] >= 0 ? ret[CurvatureType::c_cvt_type_amount] : ret[CurvatureType::c_min];
		}
		if (landmark_id == LandMarksID::lm_shoulder0 || landmark_id == LandMarksID::lm_shoulder1) {
			out_idx = ret[CurvatureType::c_cvt_type_amount] >= 0 ? ret[CurvatureType::c_cvt_type_amount] : ret[CurvatureType::c_min];
			std::cout << "hks >= 0 ?" << (ret[CurvatureType::c_cvt_type_amount] >= 0) << std::endl;
			paint_ball(_human->v[out_idx], BALL_RADIUS, BALL_CUTS, landmark_colors[landmark_id]);
		}
		if (landmark_id == LandMarksID::lm_shoulder0) {
			//out_idx = ret[CurvatureType::c_min];
			std::cout << out_idx << std::endl;
		}
		if (landmark_id == LandMarksID::lm_shoulder1) {
			//out_idx = ret[CurvatureType::c_min];
			std::cout << out_idx << std::endl;
		}
	}
	void SkeletalSegmentation::read_landmark_ground_truth() {

		string path = LANDMARK_GROUND_TRUTH_PATH + "gt_" + _human->mesh_file_name + ".txt";
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
			if (id >= 0) fs >> _ground_truths[id];
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
		string path = LANDMARK_ERROR_PATH + _human->mesh_file_name + ".err";
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
	// split mesh by slice
	void SkeletalSegmentation::split_mesh_by_slice() {
		// init
		bool find_body = true;
		//bool PRINT_TO_FILE = false;
		for (int idx = 0; idx < (int)_human->v.size(); ++idx) {
			_human->color_vertex(idx, _preset_color[6]);
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

		// write skeleton

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
								float dist = (_human->v[_human->t[t].v[v]] - sv).square_length();
								if (dist < min_dist) {
									min_dist = dist; vtx = _human->t[t].v[v];
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
						int n = _human->tt(*it, k);
						if (n >= 0 && (_human->triangle_center(n) - _human->triangle_center(*it)).dot(normal) > 0) {
							float ag = _human->triangle_dihedral_angle(n, *it);
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
				parms[i].df_ske = parms[i].max_r_ske / parms[i].min_r_ske;
				parms[i].df_ske = parms[i].min_r;

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
					if (cnt[j] > 0) parms[i].cvt_min_avg[j] /= (double)cnt[j];
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
					<< parms[i].avg_r << blank << parms[i].min_r << blank << parms[i].max_r << blank << parms[i].cvt_min_avg[CurvatureType::c_mean] << std::endl;
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
				_human->color_face(skerings[neck][k], _preset_color[4]);
			for (int k = 0; k < (int)skerings[head].size(); k++)
				_human->color_face(skerings[head][k], _preset_color[0]);
		}
		if (PAINT_SEG_OVER_MESH) {
			//paint_ring_vertices(_planes[pi][head].n, _slices_vtx[pi][head], _human->RED);
			//paint_ring_vertices(_planes[pi][neck].n, _slices_vtx[pi][neck], _human->MYCOLOR[20]);
			paint_ring_vertices(_planes[pi][head].n, _slices_vtx[pi][head], _human->RED);
			paint_ring_vertices(_planes[pi][neck].n, _slices_vtx[pi][neck], _human->CYAN);
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
			if (y1 > parms[half].girth * (float)2 || parms[i].ecc > (float)100.9) {
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
		int SEARCH_INTERVAL = ((float)(arm - wrist) / (float)8) + (float)0.5;
		int mid = (arm + wrist) / 2;
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

		float maxecc = DECIMAL_MAX;
		for (int i = mid - SEARCH_INTERVAL; i <= mid + SEARCH_INTERVAL; ++i) {
			if (parms[i].df_ske > 0 && parms[i].df_ske < maxecc) {
				maxecc = parms[i].df_ske; //elbow = i;
			}
		}

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
				_human->color_face(skerings[wrist][k], _preset_color[0]);
			for (int k = 0; k < (int)skerings[elbow].size(); k++)
				_human->color_face(skerings[elbow][k], _preset_color[2]);
			for (int k = 0; k < (int)skerings[arm].size(); k++)
				_human->color_face(skerings[arm][k], _preset_color[1]);
		}
		if (PAINT_SEG_OVER_MESH) {
			//if (!REFINE_SHOULDER) paint_ring_vertices(_planes[pi][arm].n, _slices_vtx[pi][arm], _human->PURPLE);
			//paint_ring_vertices(_planes[pi][elbow].n, _slices_vtx[pi][elbow], _human->MYCOLOR[21]);
			//paint_ring_vertices(_planes[pi][wrist].n, _slices_vtx[pi][wrist], _human->MYCOLOR[24]);
			if (!REFINE_SHOULDER) paint_ring_vertices(_planes[pi][arm].n, _slices_vtx[pi][arm], _human->MYCOLOR[13]);
			paint_ring_vertices(_planes[pi][elbow].n, _slices_vtx[pi][elbow], _human->BLUE);
			paint_ring_vertices(_planes[pi][wrist].n, _slices_vtx[pi][wrist], _human->MYCOLOR[12]);
		}

		//paint_ring_vertices(_planes[pi][(elbow + arm) / 2+2].n, _slices_vtx[pi][(elbow + arm) / 2+2], _human->RED);

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
			if (y1 > parms[half].girth * (float)2 || parms[i].ecc >(float)100.9) {
				leg_diff = i - 1; break;
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
		// knee
		int SEARCH_INTERVAL = ((float)(leg - ankle) / (float)8) + (float)0.5;
		int mid = (leg + ankle) / 2;
		int knee = mid;
		float maxecc = DECIMAL_MAX;
		for (int i = mid - SEARCH_INTERVAL; i <= mid + SEARCH_INTERVAL; ++i) {
			if (parms[i].df_ske > 0 && parms[i].df_ske < maxecc) {
				maxecc = parms[i].df_ske; knee = i;
			}
		}
		// knee
		//int SEARCH_INTERVAL = ((float)(leg - ankle) / (float)15) + (float)0.5;
		//int mid = (leg + ankle) / 2;
		int knee_cvt = mid;
		double min_cvt = parms[mid].cvt_min_avg[CurvatureType::c_mean];
		for (int i = mid - 1; i >= mid - SEARCH_INTERVAL; --i) {
			if (parms[i].cvt_min_avg[CurvatureType::c_mean] < min_cvt) {
				min_cvt = parms[i].cvt_min_avg[CurvatureType::c_mean]; knee_cvt = i;
			}
		}
		for (int i = mid + 1; i <= mid + SEARCH_INTERVAL; ++i) {
			if (parms[i].cvt_min_avg[CurvatureType::c_mean] < min_cvt) {
				min_cvt = parms[i].cvt_min_avg[CurvatureType::c_mean]; knee_cvt = i;
			}
		}
		knee = knee_cvt;
		//knee = (float)mid;
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
				_human->color_face(skerings[leg][k], _preset_color[4]);
			for (int k = 0; k < (int)skerings[knee].size(); k++)
				_human->color_face(skerings[knee][k], _preset_color[2]);
			for (int k = 0; k < (int)skerings[ankle].size(); k++)
				_human->color_face(skerings[ankle][k], _preset_color[5]);
		}
		if (PAINT_SEG_OVER_MESH) {
			//paint_ring_vertices(_planes[pi][leg].n, _slices_vtx[pi][leg], _human->MYCOLOR[25]);
			//paint_ring_vertices(_planes[pi][knee].n, _slices_vtx[pi][knee], _human->MYCOLOR[26]);
			//paint_ring_vertices(_planes[pi][ankle].n, _slices_vtx[pi][ankle], _human->MYCOLOR[27]);
			paint_ring_vertices(_planes[pi][leg].n, _slices_vtx[pi][leg], _human->GREEN);
			paint_ring_vertices(_planes[pi][knee].n, _slices_vtx[pi][knee], _human->MYCOLOR[16]);
			paint_ring_vertices(_planes[pi][ankle].n, _slices_vtx[pi][ankle], _human->MYCOLOR[17]);
		}
		_cuts[pi].resize(LegCut::c_leg_cut_amount);
		_cuts[pi][LegCut::c_leg] = leg; _cuts[pi][LegCut::c_knee] = knee; _cuts[pi][LegCut::c_ankle] = ankle;

		// length
		std::cout << "leg : " << leg << " knee : " << knee << " ankle : " << ankle << " thigh thickness : " << parms[leg].avg_r * 2 << " thigh len : " << (_skeleton[pi][leg] - _skeleton[pi][knee]).length()
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
				_human->color_face(skerings[body_cut_up][k], _preset_color[4]);
			for (int k = 0; k < (int)skerings[body_cut_mid].size(); k++)
				_human->color_face(skerings[body_cut_mid][k], _preset_color[6]);
			for (int k = 0; k < (int)skerings[body_cut_down].size(); k++)
				_human->color_face(skerings[body_cut_down][k], _preset_color[5]);
		}
		if (PAINT_SEG_OVER_MESH) {
			paint_ring_vertices(_planes[pi][body_cut_up].n, _slices_vtx[pi][body_cut_up], _human->MYCOLOR[1]);
			paint_ring_vertices(_planes[pi][body_cut_mid].n, _slices_vtx[pi][body_cut_mid], _human->MYCOLOR[2]);
			paint_ring_vertices(_planes[pi][body_cut_down].n, _slices_vtx[pi][body_cut_down], _human->MYCOLOR[8]);
		}
	}
	// slicing
	bool SkeletalSegmentation::trace_is_end(int current, int target, int v0, int v1)
	{
		for (int i = 0; i < 3; i++)
		{
			if (_human->tt(current, i) == target)
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
		int face_num = (int)_human->t.size();
		ArrayQueue<int> Q(face_num);
		Q.push(_human->vt(start, 0));
		std::vector<bool> fvisited(_human->t.size(), false);
		fvisited[_human->vt(start, 0)] = true;
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
				int neighbor = _human->tt(current, i);
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
		for (int i = 0; i < (int)_human->t.size(); i++)
			fvisited[i] = false;
		ArrayStack<int> S((int)_human->t.size());
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
				int neighbor = _human->tt(current, i);
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
				//_human->color_face(ret[i], _human->RED);
			}
		}
		else
			ret.clear();
	}
	void SkeletalSegmentation::search_loop(const Plane& plane, Vertex currske, IntVec& ret) {
		// find seed
		int start = 0; float mindist = std::numeric_limits<float>::max();
		for (int i = 0; i < (int)_human->v.size(); i++) {
			float dist = (_human->v[i] - currske).square_length();
			if (dist < mindist) {
				mindist = dist; start = i;
			}
		}
		// find seed BFS
		int face_num = (int)_human->t.size();
		ArrayQueue<int> Q(face_num);
		Q.push(_human->vt(start, 0));
		std::vector<bool> fvisited(_human->t.size(), false);
		fvisited[_human->vt(start, 0)] = true;
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
				int neighbor = _human->tt(current, i);
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
		for (int i = 0; i < (int)_human->t.size(); i++) {
			fvisited[i] = false;
		}
		ArrayStack<int> S((int)_human->t.size());
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
				int neighbor = _human->tt(current, i);
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
				//_human->color_face(ret[i], _human->RED);
			}
		}
		else
			ret.clear();
	}
	void SkeletalSegmentation::search_loop(const Plane& plane, Vertex currske, IntVec& ret, VtxVec& retv) {
		// find seed
		int start = 0; float mindist = std::numeric_limits<float>::max();
		for (int i = 0; i < (int)_human->v.size(); ++i) {
			float dist = (_human->v[i] - currske).square_length();
			if (dist < mindist) {
				mindist = dist; start = i;
			}
		}
		// find seed BFS
		int face_num = (int)_human->t.size();
		ArrayQueue<int> Q(face_num);
		int close_seed = _human->vt(start, 0);
		mindist = std::numeric_limits<float>::max();
		for (int i = 0; i < (int)_human->vtn[start]; ++i) {
			float dist = (triangle_center(_human->vt(start, i)) - currske).square_length();
			if (dist < mindist) {
				mindist = dist; close_seed = _human->vt(start, i);
			}
		}
		Q.push(close_seed);
		std::vector<bool> fvisited(_human->t.size(), false);
		fvisited[_human->vt(start, 0)] = true;
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
				int neighbor = _human->tt(current, i);
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
		for (int i = 0; i < (int)_human->t.size(); i++) {
			fvisited[i] = false;
		}
		ArrayStack<int> S((int)_human->t.size());
		fvisited[seed] = true;
		S.push(seed);
		int v0 = 0, v1 = 0;
		bool result = false;
		int count = 0;
		while (!S.empty()) {
			int current = S.top();
			S.pop();
			for (int i = 0; i < 3; i++) {
				int neighbor = _human->tt(current, i);
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
				//_human->color_face(ret[i], _human->RED);
			}
		}
		else {
			ret.clear(); retv.clear();
		}
	}
	// geodesic
	void SkeletalSegmentation::geodesic_dist_from_single_vertex(int seed, float range) {
		FltVec gd_dist((int)_human->v.size(), std::numeric_limits<float>::max());
		Vertex& seedv = _human->v[seed];
		std::vector<bool> visited((int)_human->v.size(), false);
		ArrayQueue<int> Q((int)_human->v.size());
		gd_dist[seed] = 0;
		visited[seed] = true;
		Q.push(seed);
		while (!Q.empty()) {
			int curr = Q.front(); Q.pop();
			Vertex& vtx = _human->v[curr];
			if (1/*vertex_distance_s(vtx, seedv) < range*range*/) {
				for (int i = 0; i < _human->vvn[curr]; i++) {
					int n = _human->vv(curr, i);
					if (!visited[n]) {
						visited[n] = true; Q.push(n);
					}
					//float dist = vertex_distance_s(vtx, _human->v[n]);
					float dist = (vtx - _human->v[n]).length();
					dist += gd_dist[curr];
					if (dist < gd_dist[n])
						gd_dist[n] = dist;
				}
			}
		}
	}
	float SkeletalSegmentation::geodesic_dist_slow(int seed, int dest) {
		// slow
		FltVec gd_dist((int)_human->v.size(), std::numeric_limits<float>::max());
		Vertex& seedv = _human->v[seed];
		std::vector<bool> visited((int)_human->v.size(), false);
		ArrayQueue<int> Q((int)_human->v.size());
		gd_dist[seed] = 0;
		visited[seed] = true;
		Q.push(seed);
		while (!Q.empty()) {
			int curr = Q.front(); Q.pop();
			Vertex& vtx = _human->v[curr];
			for (int i = 0; i < _human->vvn[curr]; i++) {
				int n = _human->vv(curr, i);
				if (!visited[n]) {
					visited[n] = true; Q.push(n);
				}
				float dist = (vtx - _human->v[n]).length();

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
			FltVec gd_dist((int)_human->v.size(), std::numeric_limits<float>::max());
			Vertex& seedv = _human->v[seed];
			std::vector<bool> visited((int)_human->v.size(), false);
			ArrayQueue<int> Q((int)_human->v.size());
			gd_dist[seed] = 0;
			visited[seed] = true;
			Q.push(seed);
			while (!Q.empty()) {
				int curr = Q.front(); Q.pop();
				Vertex& vtx = _human->v[curr];
				for (int i = 0; i < _human->vvn[curr]; i++) {
					int n = _human->vv(curr, i);
					if (!visited[n]) {
						visited[n] = true; Q.push(n);
					}
					float dist = (vtx - _human->v[n]).length();

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
			Vertex& goal = _human->v[dest];
			float ret = 0; int curr = seed;
			std::vector<bool> visited((int)_human->v.size(), false);
			visited[seed] = true;
			while (1) {
				bool flag = false; float mindist = DECIMAL_MAX; int next = -1;
				Vertex& vtx = _human->v[curr];
				for (int i = 0; i < _human->vvn[curr]; ++i) {
					int n = _human->vv(curr, i);
					if (!visited[n]) {
						flag = true;  float dist = (_human->v[n] - goal).length();
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
		int maxn = 20; FltVec vvdist((int)_human->v.size() * maxn);
		for (int i = 0; i < (int)_human->v.size(); i++) {
			for (int j = 0; j < _human->vvn[i]; j++) {
				int n = _human->vv(i, j);
				vvdist[i*maxn + j] = (_human->v[i] - _human->v[n]).length();
			}
		}
		for (int seed = 0; seed < (int)_human->v.size(); seed++) {
			if (seed % 100 == 0) std::cout << seed << '\n';
			FltVec gd_dist((int)_human->v.size(), std::numeric_limits<float>::max());
			Vertex& seedv = _human->v[seed];
			std::vector<bool> visited((int)_human->v.size(), false);
			ArrayQueue<int> Q((int)_human->v.size());
			gd_dist[seed] = 0;
			visited[seed] = true;
			Q.push(seed);
			while (!Q.empty()) {
				int curr = Q.front(); Q.pop();
				Vertex& vtx = _human->v[curr];
				for (int i = 0; i < _human->vvn[curr]; i++) {
					int n = _human->vv(curr, i);
					if (!visited[n]) {
						visited[n] = true; Q.push(n);
					}
					//float dist = vertex_distance_s(vtx, _human->v[n]);
					//float dist = sqrt(vertex_distance_s(vtx, _human->v[n]));
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
		return (_human->v[p0] - _human->v[p1]).length();
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
			float temp = point_plane_distance(plane, _human->v[_human->t[face].v[i]]);
			if (temp > 0)
				result += 1;
			else if (temp == 0)
				std::cout << "Point on plane !\n";
		}
		return result;
	}
	Vertex SkeletalSegmentation::plane_edge_intersection(const Plane& plane, int v0, int v1) {
		const Vertex& vtx0 = _human->v[v0], &vtx1 = _human->v[v1];
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
		Vertex ret(0, 0, 0);
		for (int i = 0; i < 3; i++)
			ret += _human->v[_human->t[face].v[i]];
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
				if (_human->t[f0].v[i] == _human->t[f1].v[j])
				{
					if (!flag)
					{
						v0 = _human->t[f0].v[i];
						flag = true;
						break;
					}
					else
					{
						v1 = _human->t[f0].v[i];
						return;
					}
				}
			}
		}
	}
	void SkeletalSegmentation::connect_two_vertex(Plane& plane, int start, int end) {
		//geodesic_dist_from_single_vertex(end, (float)300);
		int curr = start;
		Vertex endv = _human->v[end];
		while (1) {
			float mindist = std::numeric_limits<float>::max(); int minidx = 0; float tdist = 0;
			for (int i = 0; i < _human->vvn[curr]; i++) {
				int n = _human->vv(curr, i);
				float dist = (_human->v[n] - _human->v[end]).length();
				if (dist < mindist) {
					mindist = dist; minidx = n;
					tdist = (_human->v[n] - _human->v[end]).length();
				}
			}
			if (tdist < (float)0.1) {
				break;
			}
			curr = minidx;
			_human->color_vertex(curr, _preset_color[0]);
		}
	}
	// paint
	void SkeletalSegmentation::paint_ball(const int& index, const VtxVec& ring, float radius, int cuts, const Color& color) {

		if (cuts <= 0 || radius <= (float)0) return;
		Vertex point = _human->v[index];
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
				handles[i * n + j] = _human->add_vertex(vs[i][j] + center);
				_human->color_vertex(handles[i * n + j], color);
			}
		}
		for (int high = 0; high < h - 1; ++high) {
			int low = high + 1;
			for (int i = 0; i < n; ++i) {
				int j = (i + 1 >= n ? 0 : i + 1);
				_human->add_face(handles[low * n + i], handles[high * n + i], handles[high * n + j]);
				_human->add_face(handles[low * n + i], handles[high * n + j], handles[low * n + j]);
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
			_human->add_vertex(vs[0] + up + to),
			_human->add_vertex(vs[0] + to),
			_human->add_vertex(vs[0] + dn + to)
		};
		for (int i = 0; i < 3; ++i)
			_human->color_vertex(head[i], color);
		VertexHandle curr[3], last[3] = { head[0], head[1], head[2] };
		for (int i = 1; i <= size; ++i) {
			if (i < size) {
				int ni = (i >= size - 1 ? 0 : i + 1);
				auto h = vs[ni] - vs[i];
				auto o = h.cross(n);
				o.normalize();
				o *= leno_dir;
				curr[0] = _human->add_vertex(vs[i] + up + o);
				curr[1] = _human->add_vertex(vs[i] + o);
				curr[2] = _human->add_vertex(vs[i] + dn + o);
				for (int j = 0; j < 3; ++j)
					_human->color_vertex(curr[j], color);
			}
			else {
				for (int j = 0; j < 3; ++j)
					curr[j] = head[j];
			}
			if (clockwise) {
				_human->add_face(curr[2], last[2], last[1]);
				_human->add_face(curr[2], last[1], curr[1]);
				_human->add_face(curr[1], last[1], last[0]);
				_human->add_face(curr[1], last[0], curr[0]);
			}
			else {
				_human->add_face(last[2], curr[2], curr[1]);
				_human->add_face(last[2], curr[1], last[1]);
				_human->add_face(last[1], curr[1], curr[0]);
				_human->add_face(last[1], curr[0], last[0]);
			}
			for (int j = 0; j < 3; ++j)
				last[j] = curr[j];
		}
	}
	void SkeletalSegmentation::color_faces_BFS(int seed, const IntVec& border0, const IntVec& border1, const Color& color) {
		int face_num = (int)_human->t.size();
		CharVec labels(face_num, 0);
		for (int i = 0; i < (int)border0.size(); i++) {
			labels[border0[i]] = 1;
		}
		for (int i = 0; i < (int)border1.size(); i++) {
			labels[border1[i]] = 1;
		}
		ArrayQueue<int> Q(face_num);
		labels[seed] = 1; Q.push(seed);
		_human->color_face(seed, color);
		while (!Q.empty()) {
			int curr = Q.front(); Q.pop();
			for (int i = 0; i < 3; i++) {
				int n = _human->tt(curr, i);
				if (n >= 0 && labels[n] != 1) {
					labels[n] = 1; Q.push(n);
					_human->color_face(n, color);
				}
			}
		}
	}
}