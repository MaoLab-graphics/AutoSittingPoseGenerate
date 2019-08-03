// Operations for mesh

#include "Mesh.h"
#include <sstream>

namespace ManSeg
{
	// -------------------- Triangle Mesh --------------------

	int TriangleMesh::vv(int vertex_index, int neighbor_index) const {
		return vv_data[vv_seed[vertex_index] + neighbor_index];
	}
	int TriangleMesh::vt(int vertex_index, int neighbor_index) const {
		return vt_data[vt_seed[vertex_index] + neighbor_index];
	}
	int TriangleMesh::tt(int face_index, int neighbor_index) const {
		return tt_data[3 * face_index + neighbor_index];
	}
	int TriangleMesh::ev(int edge_index, int neighbor_index) const {
		return ev_data[2 * edge_index + neighbor_index];
	}
	int TriangleMesh::et(int edge_index, int neighbor_index) const {
		return et_data[2 * edge_index + neighbor_index];
	}
	float TriangleMesh::triangle_area(int face) const {
		float a = (v[t[face].v[0]] - v[t[face].v[1]]).length();
		float b = (v[t[face].v[1]] - v[t[face].v[2]]).length();
		float c = (v[t[face].v[2]] - v[t[face].v[0]]).length();
		float p = (a + b + c) / 2;
		return std::sqrt(p*(p-a)*(p-b)*(p-c));
	}
	Vertex TriangleMesh::triangle_center(int face) const {
		Vertex ret; ret.x = 0; ret.y = 0; ret.z = 0;
		for (int i = 0; i < 3; i++)
			ret += v[t[face].v[i]];
		ret.x /= (float)3; ret.y /= (float)3; ret.z /= (float)3;
		return ret;
	}
	Vertex TriangleMesh::triangle_center(const Triangle& face) const {
		Vertex ret; ret.x = 0; ret.y = 0; ret.z = 0;
		for (int i = 0; i < 3; i++)
			ret += v[face.v[i]];
		ret.x /= (float)3; ret.y /= (float)3; ret.z /= (float)3;
		return ret;
	}
	Vector TriangleMesh::triangle_normal(int face, bool invert) const {
		Vector v0 = v[t[face].v[1]] - v[t[face].v[0]];
		Vector v1 = v[t[face].v[2]] - v[t[face].v[0]];
		Vector ret = v0.cross(v1);
		if (invert) ret *= -1;
		ret.normalize();
		return ret;
	}
	float TriangleMesh::triangle_dihedral_angle(int edge, bool rad) const {
		const Vertex& pa = v[e[edge].v[0]]; const Vertex& pb = v[e[edge].v[1]];
		const Vertex& pc = v[ev(edge, 0)]; const Vertex& pd = v[ev(edge, 1)];
		const Vector ab = pb - pa;
		const Vector ac = pc - pa;
		const Vector ad = pd - pa;
		const Vector abad = ab.cross(ad);
		const float x = (ab.cross(ac)).dot(abad);
		const float l_ab = ab.length();
		const float y = l_ab * ac.dot(abad);
		if (rad == false) return std::atan2(y, x) * 180 / PI;
		else return std::atan2(y, x);
	}
	float TriangleMesh::triangle_dihedral_angle(int f0, int f1, bool rad) const {
		const Vector n0 = triangle_normal(f0, false);
		const Vector n1 = triangle_normal(f1, false);
		const float cosine = -(n0.dot(n1));
		if (rad == false) return std::acos(cosine) * 180 / PI;
		else return std::acos(cosine);
	}
	float TriangleMesh::triangle_dihedral_angle(const Vector& n0, const Vector& n1, bool rad) {
		const float cosine = -(n0.dot(n1));
		if (rad == false) return std::acos(cosine) * 180 / PI;
		else return std::acos(cosine);
	}
	int TriangleMesh::triangle_neighbor(int face, int v0, int v1, int& opposite_vertex) const {
		for (int i = 0; i < 3; ++i) {
			int n = tt(face, i);
			const int* vs = this->t[n].v;
			for (int j = 0; j < 3; ++j) {
				if (vs[j] == v0 && vs[(j + 1) % 3] == v1 || vs[j] == v1 && vs[(j + 1) % 3] == v0) {
					opposite_vertex = vs[(j + 2) % 3];
					return n;
				}
			}
		}
		return -1;
	}
	float TriangleMesh::vertex_plane_distance(const Plane& plane, const Vertex& p) {
		
		return (p - plane.p).dot(plane.n);
	}
	float TriangleMesh::vertex_triangle_distance(int face, const Vertex& p) {
		Vector a = v[t[face].v[1]] - v[t[face].v[0]];
		Vector b = v[t[face].v[2]] - v[t[face].v[0]];
		Vector n = a.cross(b).normalized();
		return n.dot(p - v[t[face].v[0]]);
	}
	Vertex TriangleMesh::project_to_plane(const Vertex& v, const Plane& plane) {
		Vector d = plane.p - v;
		float dp = d.dot(plane.n);
		if (dp >= 0) {
			return v + plane.n*dp;
		}
		else {
			return v + plane.n*(dp);
		}
	}
	bool TriangleMesh::triangle_cross_plane(int face, const Plane& plane) const {

		int result = 0;
		for (int i = 0; i < 3; i++)
		{
			float temp = vertex_plane_distance(plane, this->v[this->t[face].v[i]]);
			if (temp > 0)
				result += 1;
			else if (temp == 0)
				std::cout << "Point on plane !\n";
		}
		return result > 0 && result < 3;
	}
	bool TriangleMesh::intersect_triangle(Vertex orig, Vertex dir, int face, float& t) const {

		dir.normalize();

		// vertices of triangle
		const Vertex& v0 = this->v[this->t[face].v[0]];
		const Vertex& v1 = this->v[this->t[face].v[1]];
		const Vertex& v2 = this->v[this->t[face].v[2]];
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
		float u = T.dot(P);
		if (u < 0.0f || u > det)
			return false;
		// Q
		Vertex Q = T.cross(E1);
		// Calculate v and make sure u + v <= 1
		float v = dir.dot(Q);
		if (v < 0.0f || u + v > det)
			return false;
		// Calculate t, scale parameters, ray intersects triangle
		t = E2.dot(Q);
		float fInvDet = 1.0f / det;
		t *= fInvDet;
		u *= fInvDet;
		v *= fInvDet;
		return true;
	}
	void TriangleMesh::translate(Vector trans) {
		for (Vertex& vtx : this->v) {
			vtx += trans;
		}
	}
	void TriangleMesh::rotate(Quaternion rot) {
		for (Vertex& vtx : this->v) {
			vtx = rot * vtx;
		}
	}
	void TriangleMesh::scale(Vector factor) {
		for (Vertex& vtx : this->v) {
			vtx.x *= factor.x;
			vtx.y *= factor.y;
			vtx.z *= factor.z;
		}
	}
	void TriangleMesh::write_to_ply(const std::vector<Vertex>& vs, const std::vector<Triangle>& ts, const string& path) {

		std::stringstream ss;
		string num;
		ss << vs.size(); ss >> num; ss.clear();
		string header = "ply\nformat binary_little_endian 1.0\n";
		header += "element vertex "; header += num; header += "\n";
		header += "property float x\nproperty float y\nproperty float z\n";
		header += "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";
		ss << ts.size(); ss >> num; ss.clear();
		header += "element face "; header += num; header += "\n";
		header += "property list uchar int vertex_indices\nend_header\n";
#pragma pack(1)
		struct SimpleVertex { float x, y, z; unsigned char r, g, b, a; };
		struct SimpleFace { unsigned char n; int v[3]; };
#pragma pack()
		std::vector<SimpleVertex> svs(vs.size());
		for (int i = 0; i < vs.size();++i) {
			svs[i].x = vs[i].x; svs[i].y = vs[i].y; svs[i].z = vs[i].z;
			svs[i].r = 255; svs[i].g = 255; svs[i].b = 255; svs[i].a = 255;
		}
		std::vector<SimpleFace> sfs(ts.size());
		for (int i = 0; i < ts.size(); ++i) {
			sfs[i].n = 3;
			for (int j = 0; j < 3; ++j)
				sfs[i].v[j] = ts[i].v[j];
		}
		FILE* fp = fopen(path.c_str(), "wb");
		fwrite(header.c_str(), header.size(), 1, fp);
		fwrite(&svs[0], sizeof(SimpleVertex)*svs.size(), 1, fp);
		fwrite(&sfs[0], sizeof(SimpleFace)*sfs.size(), 1, fp);
		fclose(fp);
	}
	void TriangleMesh::write_to_ply(const std::vector<Vertex>& vs, const std::vector<Triangle>& ts, const std::vector<RGBA>& vcs, const string& path) {

		std::stringstream ss;
		string num;
		ss << vs.size(); ss >> num; ss.clear();
		string header = "ply\nformat binary_little_endian 1.0\n";
		header += "element vertex "; header += num; header += "\n";
		header += "property float x\nproperty float y\nproperty float z\n";
		header += "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";
		ss << ts.size(); ss >> num; ss.clear();
		header += "element face "; header += num; header += "\n";
		header += "property list uchar int vertex_indices\nend_header\n";
#pragma pack(1)
		struct SimpleVertex { float x, y, z; unsigned char r, g, b, a; };
		struct SimpleFace { unsigned char n; int v[3]; };
#pragma pack()
		std::vector<SimpleVertex> svs(vs.size());
		for (int i = 0; i < vs.size(); ++i) {
			svs[i].x = vs[i].x; svs[i].y = vs[i].y; svs[i].z = vs[i].z;
			svs[i].r = vcs[i].r; svs[i].g = vcs[i].g; svs[i].b = vcs[i].b; svs[i].a = 255;
		}
		std::vector<SimpleFace> sfs(ts.size());
		for (int i = 0; i < ts.size(); ++i) {
			sfs[i].n = 3;
			for (int j = 0; j < 3; ++j)
				sfs[i].v[j] = ts[i].v[j];
		}
		FILE* fp = fopen(path.c_str(), "wb");
		fwrite(header.c_str(), header.size(), 1, fp);
		fwrite(&svs[0], sizeof(SimpleVertex)*svs.size(), 1, fp);
		fwrite(&sfs[0], sizeof(SimpleFace)*sfs.size(), 1, fp);
		fclose(fp);
	}
	void TriangleMesh::write_to_ply(const string& path) {
		TriangleMesh::write_to_ply(v, t, vcolor, path);
	}
	void TriangleMesh::paint_ball(const Vertex& center, float radius, int cuts, const Color& color) {

		if (cuts <= 0 || radius <= (float)0) return;
		int h = 1 + (1 << cuts);
		int n = 4 * h - 8;
		std::vector<std::vector<Vertex>> vs(h, std::vector<Vertex>(n));
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
		int size = v.size();
		for (int i = 0; i < h; ++i) {
			for (int j = 0; j < n; ++j) {
				this->v.push_back(vs[i][j] + center);
				this->vcolor.push_back({ color });
			}
		}
		for (int high = 0; high < h - 1; ++high) {
			int low = high + 1;
			for (int i = 0; i < n; ++i) {
				int j = (i + 1 >= n ? 0 : i + 1);
				this->t.push_back({ size + low * n + i, size + high * n + i, size + high * n + j });
				this->t.push_back({ size + low * n + i, size + high * n + j, size + low * n + j });
			}
		}
	}

	// -------------------- Deformable Mesh --------------------

	void DeformableTriangleMesh::read_skeleton_data(const string& path) {

#pragma pack(1)
		struct SkeletonVertex { float x, y, z; };
#pragma pack()

		int ske_size = 0;
		int size = 0;
		FILE* fp = fopen(path.c_str(), "rb");
		fread(&ske_size, sizeof(int), 1, fp);
		this->joints.resize(ske_size);
		fread(&size, sizeof(int), 1, fp);
		this->weights.resize(size * ske_size);
		if (size != (int)v.size()) {
			std::cout << "read_skeleton_data error : vertex size != mesh.size()" << std::endl;
			return;
		}
		std::vector<SkeletonVertex> ske(ske_size);
		fread(&ske[0], sizeof(SkeletonVertex) * ske_size, 1, fp);
		for (int i = 0; i < ske_size; ++i) {
			this->joints[i].x = (float)ske[i].x;
			this->joints[i].y = (float)ske[i].y;
			this->joints[i].z = (float)ske[i].z;
		}
		std::vector<double> ws((ske_size - 1) * size);
		fread(&ws[0], sizeof(double) * (ske_size - 1) * size, 1, fp);
		fclose(fp);

		this->oj = this->joints;

		ov.resize(size);
		for (int i = 0; i < size; ++i) {
			ov[i] = v[i];
			this->weights[i * ske_size] = 0;
			for (int j = 1; j < ske_size; ++j) {
				this->weights[i * ske_size + j] = ws[i * (ske_size - 1) + j - 1];
			}
		}

		joint_parent.resize(ske_size);
		joint_children.resize(ske_size);
		joint_rotation.resize(ske_size);
		for (int i = 0; i < ske_size; ++i) {
			joint_rotation[i] = Quaternion(Vector(0,0,0), 0);
		}

		if (ske_size == 18) {
			string ske_names[] = { "shoulders","back", "hips", "head",
				"lthigh", "lknee", "lankle", "lfoot",
				"rthigh", "rknee", "rankle", "rfoot",
				"lshoulder", "lelbow", "lhand",
				"rshoulder", "relbow", "rhand" };
			
			joint_parent[j_shoulders] = -1;
			joint_parent[j_back] = j_shoulders;
			joint_parent[j_hips] = j_back;
			joint_parent[j_head] = j_shoulders;
			joint_parent[j_lhip] = j_hips;
			joint_parent[j_lknee] = j_lhip;
			joint_parent[j_lankle] = j_lknee;
			joint_parent[j_lfoot] = j_lankle;
			joint_parent[j_rhip] = j_hips;
			joint_parent[j_rknee] = j_rhip;
			joint_parent[j_rankle] = j_rknee;
			joint_parent[j_rfoot] = j_rankle;
			joint_parent[j_lshoulder] = j_shoulders;
			joint_parent[j_lelbow] = j_lshoulder;
			joint_parent[j_lhand] = j_lelbow;
			joint_parent[j_rshoulder] = j_shoulders;
			joint_parent[j_relbow] = j_rshoulder;
			joint_parent[j_rhand] = j_relbow;

			joint_children[j_shoulders].push_back(j_head);
			joint_children[j_shoulders].push_back(j_back);
			joint_children[j_shoulders].push_back(j_lshoulder);
			joint_children[j_shoulders].push_back(j_rshoulder);
			joint_children[j_back].push_back(j_hips);
			joint_children[j_hips].push_back(j_lhip);
			joint_children[j_hips].push_back(j_rhip);
			joint_children[j_head].push_back(-1);
			joint_children[j_lhip].push_back(j_lknee);
			joint_children[j_lknee].push_back(j_lankle);
			joint_children[j_lankle].push_back(j_lfoot);
			joint_children[j_lfoot].push_back(-1);
			joint_children[j_rhip].push_back(j_rknee);
			joint_children[j_rknee].push_back(j_rankle);
			joint_children[j_rankle].push_back(j_rfoot);
			joint_children[j_rfoot].push_back(-1);
			joint_children[j_lshoulder].push_back(j_lelbow);
			joint_children[j_lelbow].push_back(j_lhand);
			joint_children[j_lhand].push_back(-1);
			joint_children[j_rshoulder].push_back(j_relbow);
			joint_children[j_relbow].push_back(j_rhand);
			joint_children[j_rhand].push_back(-1);
		}
		else std::cout << "read_skeleton_data error : ske_size != 18" << std::endl;
	}
	void DeformableTriangleMesh::apply_rotation(int ske_id, const Quaternion& rot) {

		joint_rotation[ske_id] = rot;
		apply_deformation();
	}
	void DeformableTriangleMesh::apply_rotation_with_cor_helper(int ske_id, const Quaternion& rot, std::map<int, Vertex>& cors, bool change_cors, bool correct) {

		if (correct && (ske_id == JOINT_NAME::j_lankle || ske_id == JOINT_NAME::j_rankle || ske_id == JOINT_NAME::j_lhand || ske_id == JOINT_NAME::j_rhand)) {
			JOINT_NAME mid = j_lknee, up = j_lhip, down = (JOINT_NAME)ske_id;
			if (ske_id == JOINT_NAME::j_rankle) mid = j_rknee, up = j_rhip;
			if (ske_id == JOINT_NAME::j_lhand) mid = j_lelbow, up = j_lshoulder;
			if (ske_id == JOINT_NAME::j_rhand) mid = j_relbow, up = j_rshoulder;
			Vector to = rot * (this->joints[down] - this->joints[mid]);
			Vertex target = this->joints[mid] + to;
			to.normalize();
			//Plane plane = { this->joints[mid], (this->joints[up] - this->joints[mid]).cross(to).normalized() };
			//Vertex p = project_to_plane(this->joints[down], plane);
			//Plane plane2 = { p, (this->joints[up] - this->joints[mid]).normalized() };
			//Vertex p2 = project_to_plane(this->joints[mid], plane2);

			Plane plane = { this->joints[mid], (this->joints[up] - this->joints[mid]).normalized() };
			Vertex p0 = project_to_plane(this->joints[down], plane);
			Vertex p1 = project_to_plane(target, plane);

			Vector l0 = (p0 - this->joints[mid]).normalized();
			Vector l1 = (p1 - this->joints[mid]).normalized();

			auto ret = plane.n.dot(l0);
			ret = plane.n.dot(l1);

			Quaternion q = Quaternion(l0, l1);
			if (q.qs >= 0 || q.qs < 0) {
				joint_rotation[mid] = q;
				apply_deformation_with_cor(cors, change_cors);
			}

			joint_rotation[ske_id] = Quaternion((this->joints[down] - this->joints[mid]).normalized(), (target - this->joints[mid]).normalized());
			apply_deformation_with_cor(cors, change_cors);

			/*Vector from = this->joints[down] - p2; from.normalize();
			Quaternion q(from, (p - p2).normalized());
			joint_rotation[mid] = q;
			apply_deformation_with_cor(cors, change_cors);
			from = (this->joints[down] - this->joints[mid]).normalized();
			to = (target - this->joints[mid]).normalized();
			joint_rotation[ske_id] = Quaternion(from, to);
			apply_deformation_with_cor(cors, change_cors);*/
		}
		else if (correct && (ske_id == JOINT_NAME::j_back)) {
			Vertex hips_pos = this->joints[this->j_hips];
			joint_rotation[this->j_lknee] = rot.inverse();
			apply_deformation_with_cor(cors, change_cors);
			joint_rotation[this->j_rknee] = rot.inverse();
			apply_deformation_with_cor(cors, change_cors);
			rotate(rot);
			translate(hips_pos - this->joints[this->j_hips]);
		}
		else {
			joint_rotation[ske_id] = rot;
			apply_deformation_with_cor(cors, change_cors);
		}
	}
	void DeformableTriangleMesh::apply_rotation_with_cor(int ske_id, const Quaternion& rot, bool change_cors) {
		
		apply_rotation_with_cor_helper(ske_id, rot, this->cors, change_cors);
	}
	void DeformableTriangleMesh::apply_rotation_with_cor(int ske_id, const Vector& dir, bool change_cors, bool correct) {

		Vector from = joints[ske_id] - joints[joint_parent[ske_id]];
		Quaternion rot(from, dir);
		apply_rotation_with_cor_helper(ske_id, rot, this->cors, change_cors, correct);
	}
	void DeformableTriangleMesh::apply_rotation_with_cor(int ske_id, const Quaternion& rot, std::map<int, Vertex>& cors, bool change_cors) {

		apply_rotation_with_cor_helper(ske_id, rot, cors, change_cors);
	}
	void DeformableTriangleMesh::compute_joint_position(int joint_id, std::vector<Vertex>& joint_pos, std::vector<bool>& compute_status) {

		if (compute_status[joint_id]) return;
		int parent_id = joint_parent[joint_id];
		if (!compute_status[parent_id])
			compute_joint_position(parent_id, joint_pos, compute_status);
		joint_pos[joint_id] = joint_pos[parent_id] + joint_rotation[joint_id] * (joints[joint_id] - joints[parent_id]);
		compute_status[joint_id] = true;
	}
	void DeformableTriangleMesh::apply_deformation() {

		int joint_size = (int)joints.size();
		std::vector<Quaternion> rots(joint_size, Quaternion({ 0, 0, 0 }, 0));
		for (int i = 1; i < joint_size; ++i) {
			int curr = i;
			while (curr > 0) {
				if (!(Quaternion(Vector(0, 0, 0), 0) == joint_rotation[curr]))
					rots[i] = rots[i] * joint_rotation[curr];
				curr = joint_parent[curr];
			}
		}
		for (int i = 1; i < joint_size; ++i) {
			joint_rotation[i] = rots[i];
		}

		std::vector<Vertex> joint_pos(joint_size);
		std::vector<bool> ske_compute_status(joint_size, false);
		ske_compute_status[0] = true;
		joint_pos[0] = joints[0];
		for (int i = 1; i < joint_size; ++i) {
			compute_joint_position(i, joint_pos, ske_compute_status);
		}
		for (int i = 0; i < (int)v.size(); ++i) {
			Vertex p(0, 0, 0);
			for (int j = 1; j < joint_size; ++j) {
				int parent_id = joint_parent[j];
				Vertex q = joint_rotation[j] * (v[i] - joints[parent_id]) + joint_pos[parent_id];
				p += q * (float)weights[i * joint_size + j];
			}
			v[i] = p;
		}
		for (int i = 0; i < joint_size; ++i) {
			joints[i] = joint_pos[i];
			joint_rotation[i] = Quaternion(Vector(0, 0, 0), 0);
		}

		//int ske_size = (int)joint_rotation.size();
		//std::vector<Quaternion> rots(ske_size);
		//std::vector<Vector> moves(ske_size);
		//rots[0] = Quaternion(Vector(0, 0, 0), 0);
		//moves[0] = Vector(0, 0, 0);
		//for (int i = 1; i < ske_size; ++i) {
		//	int curr = i;
		//	IntVec vec;
		//	while (curr > 0) {
		//		vec.push_back(curr);
		//		curr = joint_parent[curr];
		//	}
		//	Quaternion rot(Vector(0, 0, 0), 0);
		//	Vector move(0, 0, 0);
		//	for (auto j : vec) {
		//		if (joint_rotation[j] == Quaternion(Vector(0, 0, 0), 0))
		//			continue;
		//		rot = rot * joint_rotation[j];
		//		Vertex parent = skeleton[joint_parent[j]];
		//		Vector tmove = parent - rot * parent;
		//		move = rot * move + tmove;
		//	}
		//	rots[i] = rot;
		//	moves[i] = move;
		//	skeleton[i] = rot * skeleton[i] + move;
		//}
		//// deformation
		//for (int i = 0; i < (int)v.size(); ++i) {
		//	Vertex p(0, 0, 0);
		//	for (int j = 1; j < ske_size; ++j) {
		//		Vertex q = rots[j] * v[i] + moves[j];
		//		p += q * (float)weights[i * ske_size + j];
		//	}
		//	v[i] = p;
		//}
	}
	void DeformableTriangleMesh::apply_deformation_with_cor(std::map<int, Vertex>& cors, bool change_cors) {

		int joint_size = (int)joints.size();
		std::vector<Quaternion> rots(joint_size, Quaternion({ 0, 0, 0 }, 0));
		for (int i = 1; i < joint_size; ++i) {
			int curr = i;
			while (curr > 0) {
				if (!(Quaternion(Vector(0, 0, 0), 0) == joint_rotation[curr]))
					rots[i] = rots[i] * joint_rotation[curr];
				curr = joint_parent[curr];
			}
		}
		for (int i = 1; i < joint_size; ++i) {
			joint_rotation[i] = rots[i];
		}

		std::vector<Vertex> joint_pos(joint_size);
		std::vector<bool> compute_status(joint_size, false);
		compute_status[0] = true;
		joint_pos[0] = joints[0];
		for (int i = 1; i < joint_size; ++i) {
			compute_joint_position(i, joint_pos, compute_status);
		}
		for (int i = 0; i < (int)v.size(); ++i) {
			if (cors.count(i) > 0) {

				Quaternion optimized_rot = joint_rotation[0] * (float)weights[i * joint_size + 0];
				for (int j = 1; j < joint_size; ++j) {
					if (optimized_rot.dot(joint_rotation[j]) >= 0)
						optimized_rot = optimized_rot + (joint_rotation[j] * (float)weights[i * joint_size + j]);
					else
						optimized_rot = optimized_rot + (joint_rotation[j] * (float)weights[i * joint_size + j]) * (float)-1;
				}
				optimized_rot = optimized_rot.normalized();

				const auto& rest_cor = cors.at(i);
				Vertex posed_cor(0, 0, 0);
				for (int j = 1; j < joint_size; ++j) {
					int parent_id = joint_parent[j];
					Vertex q = joint_rotation[j] * (rest_cor - joints[parent_id]) + joint_pos[parent_id];
					posed_cor += q * (float)weights[i * joint_size + j];
				}

				v[i] = (posed_cor + optimized_rot * (v[i] - rest_cor));
				if (change_cors)
					cors[i] = posed_cor;
			}
			else {
				Vertex p(0, 0, 0);
				for (int j = 1; j < joint_size; ++j) {
					int parent_id = joint_parent[j];
					Vertex q = joint_rotation[j] * (v[i] - joints[parent_id]) + joint_pos[parent_id];
					p += q * (float)weights[i * joint_size + j];
				}
				v[i] = p;
			}
		}
		for (int i = 0; i < joint_size; ++i) {
			joints[i] = joint_pos[i];
			joint_rotation[i] = Quaternion(Vector(0, 0, 0), 0);
		}

		//int ske_size = (int)joint_rotation.size();
		//std::vector<Quaternion> rots(ske_size);
		//std::vector<Vector> moves(ske_size);
		//rots[0] = Quaternion(Vector(0, 0, 0), 0);
		//moves[0] = Vector(0, 0, 0);
		//for (int i = 1; i < ske_size; ++i) {
		//	int curr = i;
		//	IntVec vec;
		//	while (curr > 0) {
		//		vec.push_back(curr);
		//		curr = joint_parent[curr];
		//	}
		//	for (int j = 0; j < vec.size() / 2; ++j) {
		//		vec[j] ^= vec[(int)vec.size() - 1 - j];
		//		vec[(int)vec.size() - 1 - j] ^= vec[j];
		//		vec[j] ^= vec[(int)vec.size() - 1 - j];
		//	}
		//	Quaternion rot(Vector(0, 0, 0), 0);
		//	Vector move(0, 0, 0);
		//	for (auto j : vec) {
		//		if (joint_rotation[j] == Quaternion(Vector(0, 0, 0), 0))
		//			continue;
		//		rot = rot * joint_rotation[j];
		//		Vertex parent = skeleton[joint_parent[j]];
		//		Vector tmove = parent - rot * parent;
		//		move = rot * move + tmove;
		//	}
		//	rots[i] = rot;
		//	moves[i] = move;
		//	skeleton[i] = rot * skeleton[i] + move;
		//}

		//for (int i = 0; i < (int)v.size(); ++i) {
		//	if (cors.count(i)) {

		//		Quaternion optimized_rot = joint_rotation[0] * (float)weights[i * ske_size + 0];
		//		for (int j = 1; j < ske_size; ++j) {
		//			if (optimized_rot.dot(joint_rotation[j]) >= 0) optimized_rot = optimized_rot + (rots[j] * (float)weights[i * ske_size + j]);
		//			else optimized_rot = optimized_rot - (rots[j] * (float)weights[i * ske_size + j]);
		//			//optimized_rot = optimized_rot * joint_rotation[j];
		//		}
		//		optimized_rot = optimized_rot.normalized();

		//		const auto& rest_cor = cors.at(i);
		//		Vertex posed_cor(0, 0, 0);
		//		for (int j = 1; j < ske_size; ++j) {
		//			Vertex q = rots[j] * rest_cor + moves[j];
		//			posed_cor += q * (float)weights[i * ske_size + j];
		//		}

		//		v[i] = (posed_cor + optimized_rot * (v[i] - rest_cor));
		//		cors[i] = posed_cor;
		//	}
		//}
	}
	Vertex DeformableTriangleMesh::transform_by_joint(JOINT_NAME jn, const Vertex& v, const Quaternion& q) {

		return this->joints[jn] + q * (v - this->joints[jn]);
	}
	void DeformableTriangleMesh::translate(Vector trans) {
		this->TriangleMesh::translate(trans);
		for (Vertex& vtx : this->joints) {
			vtx += trans;
		}
		for (int i = 0; i < (int)v.size(); ++i) {
			if (cors.count(i)) {
				cors[i] += trans;
			}
		}
	}
	void DeformableTriangleMesh::rotate(Quaternion rot) {
		this->TriangleMesh::rotate(rot);
		for (Vertex& vtx : this->joints) {
			vtx = rot * vtx;
		}
		for (int i = 0; i < (int)v.size(); ++i) {
			if (cors.count(i)) {
				cors[i] = rot * cors[i];
			}
		}
	}
	void DeformableTriangleMesh::scale(Vector factor) {
		this->TriangleMesh::scale(factor);
		for (Vertex& vtx : this->joints) {
			vtx.x *= factor.x;
			vtx.y *= factor.y;
			vtx.z *= factor.z;
		}
		for (int i = 0; i < (int)v.size(); ++i) {
			if (cors.count(i)) {
				cors[i].x *= factor.x;
				cors[i].y *= factor.y;
				cors[i].z *= factor.z;
			}
		}
	}

	// -------------------- OpenMesh Triangle Mesh --------------------

	Plane Plane::create_plane(const Vertex& center, const Vertex& ph, const Vertex& pt)
	{
		Plane result;
		//float small = 0;
		//result.p.x = center.x + small; result.p.y = center.y + small; result.p.z = center.z + small;
		result.p = center;
		result.n = ph - pt; result.n.normalize();
		return result;
	}
	void OpenMeshTriangleMesh::color_vertex(int index, const Color& color) {
		vcolor[index].r = color[0]; vcolor[index].g = color[1]; vcolor[index].b = color[2];
		openmesh_mesh.set_color(vh[index], color);
	}
	void OpenMeshTriangleMesh::color_vertex(VertexHandle& vtxh, const Color& color) {
		openmesh_mesh.set_color(vtxh, color);
	}
	void OpenMeshTriangleMesh::color_face(int index, const Color& color) {
		tcolor[index].r = color[0]; tcolor[index].g = color[1]; tcolor[index].b = color[2];
		// color vertices
		//for (int i = 0; i < 3; i++)
		//	openmesh_mesh.set_color(vh[t[index].v[i]], color);
		for (int i = 0; i < 3; i++)
			this->color_vertex(t[index].v[i], color);
		//// color face
		//openmesh_mesh.set_color(th[index], color);
	}
	void OpenMeshTriangleMesh::update_all_vertex() {
		auto v_end = openmesh_mesh.vertices_end();
		for (auto v_it = openmesh_mesh.vertices_begin(); v_it != v_end; ++v_it) {
			int index = v_it->idx();
			if (index < 0 || index >= (int)v.size()) continue;
			ManSegMesh::Point p; p[0] = (float)v[index].x; p[1] = (float)v[index].y; p[2] = (float)v[index].z;
			openmesh_mesh.set_point(*v_it, p);
		}
	}
	void OpenMeshTriangleMesh::transfer_vertex_coord(OpenMeshTriangleMesh& mesh) {
		if (mesh.v.size() != this->v.size()) {
			std::cout << "transfer_vertex_coord : vertex number not equal !" << std::endl;
			return;
		}
		for (int i = 0; i < (int)mesh.v.size(); ++i) {
			this->v[i].x = mesh.v[i].x;
			this->v[i].y = mesh.v[i].y;
			this->v[i].z = mesh.v[i].z;
		}
	}
	bool OpenMeshTriangleMesh::read_from_file(const string& input_file_name) {

		// settings
		mesh_setting();
		io_setting();
		// read mesh with vertex colors
		if (!OpenMesh::IO::read_mesh(openmesh_mesh, input_file_name, openmesh_io_option))
		{
			std::cerr << "Error loading mesh from file " << std::endl;
			return false;
		}
		// initiate data
		mesh_data_init();
		return true;
	}
	bool OpenMeshTriangleMesh::write_to_file(const string& output_file_name) const {
		// write mesh
		try
		{
			if (!OpenMesh::IO::write_mesh(openmesh_mesh, output_file_name, openmesh_io_option))
			{
				std::cerr << "Cannot write mesh to file << std::endl";
				return false;
			}
		}
		catch (std::exception& x)
		{
			std::cerr << x.what() << std::endl;
			return false;
		}
		return true;
	}
	void OpenMeshTriangleMesh::mesh_setting() {
		// request vertex colors property, so the mesh reader can use color information if available
		openmesh_mesh.request_vertex_colors();
		// assure we have vertex colors property
		if (!openmesh_mesh.has_vertex_colors())
		{
			std::cerr << "ERROR: Standard vertex property 'Color' not available!\n";
		}
		//// request face colors property, so the mesh reader can use color information if available
		//openmesh_mesh.request_face_colors();
		//// assure we have face colors property
		//if (!openmesh_mesh.has_face_colors())
		//{
		//	std::cerr << "ERROR: Standard face property 'Color' not available!\n";
		//}
	}
	void OpenMeshTriangleMesh::io_setting() {
		openmesh_io_option = OpenMesh::IO::Options::Default;
		openmesh_io_option += OpenMesh::IO::Options::Binary;
		openmesh_io_option += OpenMesh::IO::Options::VertexColor;
		//openmesh_io_option += OpenMesh::IO::Options::FaceColor;
	}
	void OpenMeshTriangleMesh::mesh_data_init() {
		// color
		BLACK[0] = 0; BLACK[1] = 0; BLACK[2] = 0;
		WHITE[0] = 255; WHITE[1] = 255; WHITE[2] = 255;
		RED[0] = 255; RED[1] = 0; RED[2] = 0;
		GREEN[0] = 0; GREEN[1] = 255; GREEN[2] = 0;
		BLUE[0] = 0; BLUE[1] = 0; BLUE[2] = 255;
		PURPLE[0] = 255; PURPLE[1] = 0; PURPLE[2] = 255;
		YELLOW[0] = 255; YELLOW[1] = 255; YELLOW[2] = 0;
		CYAN[0] = 0; CYAN[1] = 255; CYAN[2] = 255;
		MYCOLOR[20][0] = 247; MYCOLOR[20][1] = 0; MYCOLOR[20][2] = 98; //ÌÒºì
		MYCOLOR[21][0] = 255; MYCOLOR[21][1] = 201; MYCOLOR[21][2] = 15; //³È
		MYCOLOR[22][0] = 251; MYCOLOR[22][1] = 251; MYCOLOR[22][2] = 141; //»Æ
		MYCOLOR[23][0] = 200; MYCOLOR[23][1] = 251; MYCOLOR[23][2] = 150; //Ç³ÂÌ
		MYCOLOR[24][0] = 2; MYCOLOR[24][1] = 198; MYCOLOR[24][2] = 3; //ÂÌ
		MYCOLOR[25][0] = 91; MYCOLOR[25][1] = 253; MYCOLOR[25][2] = 255; //Çà
		MYCOLOR[26][0] = 45; MYCOLOR[26][1] = 166; MYCOLOR[26][2] = 222; //ÇàÀ¶
		MYCOLOR[27][0] = 47; MYCOLOR[27][1] = 94; MYCOLOR[27][2] = 234; //À¶
		MYCOLOR[28][0] = 45; MYCOLOR[28][1] = 166; MYCOLOR[28][2] = 222; //ÇàÀ¶
		//MYCOLOR[9][0] = 228; MYCOLOR[9][1] = 169; MYCOLOR[9][2] = 29;
		//MYCOLOR[10][0] = 255; MYCOLOR[10][1] = 255; MYCOLOR[10][2] = 139;
		//MYCOLOR[11][0] = 85; MYCOLOR[11][1] = 120; MYCOLOR[11][2] = 255;

		MYCOLOR[29][0] = 233; MYCOLOR[29][1] = 65; MYCOLOR[29][2] = 0; //Éî³È
		MYCOLOR[30][0] = 151; MYCOLOR[30][1] = 0; MYCOLOR[30][2] = 255; //Éî×Ï

		MYCOLOR[0][0] = 255; MYCOLOR[0][1] = 198; MYCOLOR[0][2] = 138; //Ç³³È
		MYCOLOR[1][0] = 255; MYCOLOR[1][1] = 155; MYCOLOR[1][2] = 160; //Ç³ºì
		MYCOLOR[2][0] = 162; MYCOLOR[2][1] = 255; MYCOLOR[2][2] = 162; //Ç³ÂÌ
		MYCOLOR[3][0] = 255; MYCOLOR[3][1] = 251; MYCOLOR[3][2] = 50; //»Æ
		MYCOLOR[4][0] = 228; MYCOLOR[4][1] = 142; MYCOLOR[4][2] = 233; //Æ·ºì
		MYCOLOR[5][0] = 130; MYCOLOR[5][1] = 136; MYCOLOR[5][2] = 218; //µ­À¶
		MYCOLOR[6][0] = 250; MYCOLOR[6][1] = 253; MYCOLOR[6][2] = 151; //Ç³»Æ
		MYCOLOR[7][0] = 161; MYCOLOR[7][1] = 255; MYCOLOR[7][2] = 255; //ÇàÉ«
		MYCOLOR[8][0] = 205; MYCOLOR[8][1] = 123; MYCOLOR[8][2] = 125; // Éîºì
		MYCOLOR[9][0] = 180; MYCOLOR[9][1] = 145; MYCOLOR[9][2] = 241;//×Ï
		MYCOLOR[10][0] = 42; MYCOLOR[10][1] = 133; MYCOLOR[10][2] = 42;//ÉîÂÌ
		MYCOLOR[11][0] = 230; MYCOLOR[11][1] = 230; MYCOLOR[11][2] = 230;

		MYCOLOR[12][0] = 229; MYCOLOR[12][1] = 108; MYCOLOR[12][2] = 10;//head
		MYCOLOR[13][0] = 112; MYCOLOR[13][1] = 48; MYCOLOR[13][2] = 161;//up arm
		MYCOLOR[14][0] = 255; MYCOLOR[14][1] = 0; MYCOLOR[14][2] = 102;//hand
		MYCOLOR[15][0] = 229; MYCOLOR[15][1] = 86; MYCOLOR[15][2] = 181;//waist
		MYCOLOR[16][0] = 70; MYCOLOR[16][1] = 135; MYCOLOR[16][2] = 114;//thigh
		MYCOLOR[17][0] = 149; MYCOLOR[17][1] = 54; MYCOLOR[17][2] = 52;//foot

		// vertices
		v.resize(openmesh_mesh.n_vertices());
		vh.resize(openmesh_mesh.n_vertices());
		vv_seed.resize(openmesh_mesh.n_vertices());
		vt_seed.resize(openmesh_mesh.n_vertices());
		vvn.resize(openmesh_mesh.n_vertices());
		vtn.resize(openmesh_mesh.n_vertices());
		vcolor.resize(openmesh_mesh.n_vertices());
		auto v_end = openmesh_mesh.vertices_end();
		int count_vvidx = 0, count_vfidx = 0;
		for (auto v_it = openmesh_mesh.vertices_begin(); v_it != v_end; ++v_it)
		{
			int index = v_it->idx();
			// x, y, z
			auto p = openmesh_mesh.point(*v_it);
			auto rgba = openmesh_mesh.color(*v_it);
			vcolor[index].r = rgba[0];
			vcolor[index].g = rgba[1];
			vcolor[index].b = rgba[2];
			vcolor[index].a = 255;
			v[index].x = p[0];
			v[index].y = p[1];
			v[index].z = p[2];
			// handle
			vh[index] = *v_it;
			// vertex neighbor
			int count_vv = 0;
			vv_seed[index] = count_vvidx;
			for (auto vv_it = openmesh_mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
			{
				int indexvv = vv_it->idx();
				vv_data.push_back(indexvv);
				count_vvidx++;
				count_vv++;
			}
			vvn[index] = count_vv;
			// triangle neighbor
			int count_vf = 0;
			vt_seed[index] = count_vfidx;
			for (auto vf_it = openmesh_mesh.vf_iter(*v_it); vf_it.is_valid(); ++vf_it)
			{
				int indexvf = vf_it->idx();
				vt_data.push_back(indexvf);
				count_vfidx++;
				count_vf++;
			}
			vtn[index] = count_vf;
		}
		// edges
		e.resize(openmesh_mesh.n_edges());
		ev_data.resize(2 * openmesh_mesh.n_edges());
		et_data.resize(2 * openmesh_mesh.n_edges());
		auto e_end = openmesh_mesh.edges_end();
		for (auto e_it = openmesh_mesh.edges_begin(); e_it != e_end; ++e_it)
		{
			int index_edge = e_it->idx();
			// vertex neighbor
			int count_ef = 0;
			auto he0 = openmesh_mesh.halfedge_handle(*e_it, 0);
			auto he1 = openmesh_mesh.halfedge_handle(*e_it, 1);
			et_data[2*index_edge] = openmesh_mesh.face_handle(he0).idx();
			et_data[2*index_edge + 1] = openmesh_mesh.face_handle(he1).idx();
			e[index_edge].v[0] = openmesh_mesh.to_vertex_handle(he0).idx();
			e[index_edge].v[1] = openmesh_mesh.from_vertex_handle(he0).idx();
			ev_data[2*index_edge] = openmesh_mesh.to_vertex_handle(openmesh_mesh.next_halfedge_handle(he0)).idx();
			ev_data[2*index_edge + 1] = openmesh_mesh.to_vertex_handle(openmesh_mesh.next_halfedge_handle(he1)).idx();
		}
		// faces
		t.resize(openmesh_mesh.n_faces());
		th.resize(openmesh_mesh.n_faces());
		tt_data.resize(3 * openmesh_mesh.n_faces());
		tcolor.resize(openmesh_mesh.n_faces());
		auto f_end = openmesh_mesh.faces_end();
		for (auto f_it = openmesh_mesh.faces_begin(); f_it != f_end; ++f_it)
		{
			int indexf = f_it->idx();
			// OpenMesh face handle
			th[indexf] = *f_it;
			// 3 vertices of triangle
			int counter = 0;
			for (auto fv_it = openmesh_mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			{
				int indexfv = fv_it->idx();
				t[indexf].v[counter] = indexfv;
				counter++;
			}
			// 3 triangle neighbor
			counter = 0;
			tt_data[3 * indexf] = tt_data[3 * indexf + 1] = tt_data[3 * indexf + 2] = -1;
			for (auto ff_it = openmesh_mesh.ff_iter(*f_it); ff_it.is_valid(); ++ff_it)
			{
				int indexff = ff_it->idx();
				tt_data[3 * indexf + counter] = indexff;
				counter++;
			}
		}
	}
	VertexHandle OpenMeshTriangleMesh::add_vertex(const Vertex& vtx) {
		ManSegMesh::Point p; p[0] = (float)vtx.x; p[1] = (float)vtx.y; p[2] = (float)vtx.z;
		return openmesh_mesh.add_vertex(p);
	}
	FaceHandle OpenMeshTriangleMesh::add_face(const VertexHandle& vh0, const VertexHandle& vh1, const VertexHandle& vh2) {
		return openmesh_mesh.add_face(vh0, vh1, vh2);
	}
	void OpenMeshTriangleMesh::paint_ball_openmesh(const Vertex& center, float radius, int cuts, const Color& color) {

		if (cuts <= 0 || radius <= (float)0) return;
		int h = 1 + (1 << cuts);
		int n = 4 * h - 8;
		std::vector<std::vector<Vertex>> vs(h, std::vector<Vertex>(n));
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
				handles[i * n + j] = this->add_vertex(vs[i][j] + center);
				this->color_vertex(handles[i * n + j], color);
			}
		}
		for (int high = 0; high < h - 1; ++high) {
			int low = high + 1;
			for (int i = 0; i < n; ++i) {
				int j = (i + 1 >= n ? 0 : i + 1);
				this->add_face(handles[low * n + i], handles[high * n + i], handles[high * n + j]);
				this->add_face(handles[low * n + i], handles[high * n + j], handles[low * n + j]);
			}
		}
	}

	// -------------------- Vector --------------------

	Vector Vector::operator+(const Vector& rhs) const {
		Vertex ret = *this;
		ret.x = this->x + rhs.x;
		ret.y = this->y + rhs.y;
		ret.z = this->z + rhs.z;
		return ret;
	}
	Vector Vector::operator*(float rhs) const {
		Vertex ret = *this;
		ret.x = this->x * rhs;
		ret.y = this->y * rhs;
		ret.z = this->z * rhs;
		return ret;
	}
	Vector Vector::operator/(float rhs) const {
		Vertex ret = *this;
		ret.x = this->x / rhs;
		ret.y = this->y / rhs;
		ret.z = this->z / rhs;
		return ret;
	}
	Vector Vector::operator-(const Vector& rhs) const {
		Vertex ret = *this;
		ret.x = this->x - rhs.x;
		ret.y = this->y - rhs.y;
		ret.z = this->z - rhs.z;
		return ret;
	}
	Vector& Vector::operator+=(const Vector& rhs) {
		this->x = this->x + rhs.x;
		this->y = this->y + rhs.y;
		this->z = this->z + rhs.z;
		return *this;
	}
	Vector& Vector::operator-=(const Vector& rhs) {
		this->x = this->x - rhs.x;
		this->y = this->y - rhs.y;
		this->z = this->z - rhs.z;
		return *this;
	}
	Vector& Vector::operator*=(float rhs) {
		this->x = this->x * rhs;
		this->y = this->y * rhs;
		this->z = this->z * rhs;
		return *this;
	}
	Vector& Vector::operator/=(float rhs) {
		this->x = this->x / rhs;
		this->y = this->y / rhs;
		this->z = this->z / rhs;
		return *this;
	}
	void Vector::normalize() {
		float divisor = this->x * this->x + this->y * this->y + this->z * this->z;
		if (divisor < DECIMAL_SMALLEST) return;
		divisor = sqrt(divisor);
		this->x = this->x / divisor;
		this->y = this->y / divisor;
		this->z = this->z / divisor;
	}
	Vector Vector::normalized() const {
		float divisor = this->x * this->x + this->y * this->y + this->z * this->z;
		if (divisor < DECIMAL_SMALLEST) return *this;
		divisor = sqrt(divisor);
		Vector ret = *this;
		ret.x = this->x / divisor;
		ret.y = this->y / divisor;
		ret.z = this->z / divisor;
		return ret;
	}
	float Vector::dot(const Vector& rhs) const {
		return this->x*rhs.x + this->y*rhs.y + this->z*rhs.z;
	}
	Vector Vector::cross(const Vector& rhs) const {
		Vertex ret = *this;
		ret.x = this->y*rhs.z - this->z*rhs.y;
		ret.y = this->z*rhs.x - this->x*rhs.z;
		ret.z = this->x*rhs.y - this->y*rhs.x;
		return ret;
	}
	float Vector::length() const {
		return sqrt(this->x*this->x + this->y*this->y + this->z*this->z);
	}
	float Vector::square_length() const {
		return this->x*this->x + this->y*this->y + this->z*this->z;
	}
	bool Vector::is_nonzero() const {
		if (this->x <= -DECIMAL_SMALLEST || this->x >= DECIMAL_SMALLEST)
			return true;
		if (this->y <= -DECIMAL_SMALLEST || this->y >= DECIMAL_SMALLEST)
			return true;
		if (this->z <= -DECIMAL_SMALLEST || this->z >= DECIMAL_SMALLEST)
			return true;
		return false;
	}
	Vector Vector::create_vector_from_angles(float xz_angle, float xy_angle) {

		xz_angle = xz_angle * PI / 180.f;
		xy_angle = xy_angle * PI / 180.f;
		float len = abs(cos(xy_angle));
		Vector dir = Vector(len*cos(xz_angle), sin(xy_angle), len*sin(xz_angle));
		return dir.normalized();
	}

	// -------------------- Quaternion --------------------

	Quaternion::Quaternion(const Vector& axis, float angle, bool angle_in_rad) {
		if (!angle_in_rad) angle = angle * PI / 180 / 2;
		else angle /= 2;
		qv = axis*sin(angle);
		qs = cos(angle);
	}
	//minimum rotation constructor
	Quaternion::Quaternion(Vector from, Vector to) : qs(1.) {
		from.normalize();
		to.normalize();
		float angle = -acos(from.dot(to)) / 2;
		qs = cos(angle);
		qv = (from.cross(to)).normalized() * sin(angle);
	}
	Quaternion Quaternion::operator+(const Quaternion& q) const {
		Quaternion ret;
		ret.qv = qv + q.qv;
		ret.qs = qs + q.qs;
		return ret;
	}
	Quaternion Quaternion::operator*(const Quaternion& q) const {
		Quaternion ret;
		ret.qv = q.qv*qs + qv*q.qs + q.qv.cross(qv);
		ret.qs = q.qs*qs - q.qv.dot(qv);
		return ret;
	}
	Quaternion Quaternion::operator*(float rhs) const {
		Quaternion ret;
		ret.qv = qv * rhs;
		ret.qs = qs * rhs;
		return ret;
	}
	Vector Quaternion::operator*(const Vector& v) const {
		Quaternion p(v, 180);
		p.qs = 0;
		Vector ret = v;
		Vector temp = ((*this) * p * this->inverse()).qv;
		ret.x = temp.x; ret.y = temp.y; ret.z = temp.z;
		return ret;
	}
	bool Quaternion::operator==(const Quaternion& q) const {
		if (this->qs != q.qs) return false;
		if (this->qv.x != q.qv.x) return false;
		if (this->qv.y != q.qv.y) return false;
		if (this->qv.z != q.qv.z) return false;
		return true;
	}
	Quaternion Quaternion::normalized() const {
		Quaternion ret = *this;
		float len = std::sqrt(this->dot(*this));
		ret.qv.x /= len; ret.qv.y /= len; ret.qv.z /= len; ret.qs /= len;
		return ret;
	}
	Quaternion Quaternion::inverse() const {
		Quaternion ret = *this;
		ret.qv *= -1;
		return ret;
	}
	float Quaternion::dot(const Quaternion& q) const {
		return qs*q.qs + qv.dot(q.qv);
	}
	Vector Quaternion::rotate(const Vector& v, const Vector& axis, float angle) {
		Quaternion p(v, 180);
		p.qs = 0;
		Quaternion q(axis, angle);
		Vector temp = (q * p * q.inverse()).qv;
		Vector ret = v;
		ret.x = temp.x; ret.y = temp.y; ret.z = temp.z;
		return ret;
	}

	// -------------------- RGBA --------------------
	Color RGBA::HSV2RGB(int H, float S, float V) {

		// HSV to RGB
		float C = S * V;
		float t = (float)H / 60.0f;
		int t2 = (int)(t / 2) * 2;
		float t3 = abs(t - (float)t2 - 1.0f);
		float X = C * (1.0f - t3);
		float m = V - C;
		float R = 0, G = 0, B = 0;
		if (H < 0);
		else if (H < 60) { R = C; G = X; }
		else if (H < 120) { R = X; G = C; }
		else if (H < 180) { G = C; B = X; }
		else if (H < 240) { G = X; B = C; }
		else if (H < 300) { R = X; B = C; }
		else if (H < 360) { R = C; B = X; }
		Color color; color[0] = color[1] = color[2] = 0;
		color[0] = (int)((R + m) * 255);
		color[1] = (int)((G + m) * 255);
		color[2] = (int)((B + m) * 255);
		return color;
	}
}