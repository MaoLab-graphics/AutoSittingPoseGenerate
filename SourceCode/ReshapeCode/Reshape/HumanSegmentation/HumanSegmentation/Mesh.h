// Operations for mesh

#pragma once

#ifndef MESH_H
#define MESH_H

#include "Global.h"
#include "OpenMesh.h"

namespace ManSeg {

	static const string MESH_PATH = "..\\..\\..\\..\\..\\..\\HumanModel\\";

	// -------------------- Types --------------------

	typedef OMTriangleMesh ManSegMesh;
	typedef ManSegMesh::VertexHandle VertexHandle;
	typedef ManSegMesh::FaceHandle FaceHandle;

	// -------------------- Struct Vertex / Vector --------------------

	struct Vector {
	public:
		float x, y, z;
	public:
		Vector() { x = y = z = 0; }
		Vector(float px, float py, float pz) { x = px; y = py; z = pz; }
		static Vector create_vector_from_angles(float xz_angle, float xy_angle);
	public:
		Vector operator+(const Vector& rhs) const;
		Vector operator-(const Vector& rhs) const;
		Vector operator*(float rhs) const;
		Vector operator/(float rhs) const;
		Vector& operator+=(const Vector& rhs);
		Vector& operator-=(const Vector& rhs);
		Vector& operator*=(float rhs);
		Vector& operator/=(float rhs);
	public:
		bool is_nonzero() const;
		void normalize();
		Vector normalized() const;
		float dot(const Vector& rhs) const;
		Vector cross(const Vector& rhs) const;
		float length() const;
		float square_length() const; // length^2
	};

	// -------------------- Struct Quaternion --------------------

	struct Quaternion {
		Vector qv;
		float qs;
		Quaternion() = default;
		// axis, angle (0 ~ 360)
		Quaternion(const Vector& axis, float angle, bool angle_in_rad = false);
		// minimum rotation constructor
		Quaternion(Vector from, Vector to);
		Quaternion operator+(const Quaternion& q) const;
		Quaternion operator*(const Quaternion& q) const;
		Quaternion operator*(float rhs) const;
		Vector operator*(const Vector& v) const;
		bool operator==(const Quaternion& q) const;
		Quaternion normalized() const;
		Quaternion inverse() const;
		float dot(const Quaternion& q) const;
		static Vector rotate(const Vector& v, const Vector& axis, float angle);
	};

	// -------------------- Struct Plane --------------------

	struct Plane
	{
		Vertex p; // point on plane
		Vector n; // normal of plane
		Plane() = default;
		Plane(const Vertex& center, const Vector& normal) { p = center; n = normal.normalized(); }
		static Plane create_plane(const Vertex& center, const Vertex& ph, const Vertex& pt);
	};

	// -------------------- Struct Half Edge --------------------

	struct Edge {
		int v[2]; // head, tail
		Edge() = default;
		Edge(int v0, int v1) { v[0] = v0; v[1] = v1; }
	};

	// -------------------- Struct Triangle --------------------

	struct Triangle {
		int v[3]; // indices of vertices
		Triangle() = default;
		Triangle(int v0, int v1, int v2) { v[0] = v0; v[1] = v1; v[2] = v2; }
	};
	typedef Triangle Face;

	// -------------------- Struct RGBA --------------------

	struct RGBA {
		unsigned char r, g, b, a;
		RGBA() { r = g = b = 0; a = 255; }
		RGBA(const Color& color) { r = color[0]; g = color[1]; b = color[2]; a = 255; }
		RGBA(unsigned char pr, unsigned char pg, unsigned char pb) { r = pr; g = pg; b = pb; a = 255; }
		static Color HSV2RGB(int H, float S, float V);
	};

	// -------------------- Struct Triangle Mesh --------------------

	struct TriangleMesh {
	public:
		std::vector<Vertex> v; // vertices
		std::vector<Edge> e; // edges
		std::vector<Triangle> t; // triangles / faces
		std::vector<int> vvn; // vertex neighbor amount of each vertex
		std::vector<int> vtn; // triangle neighbor amount of each vertex
	public:
		std::vector<RGBA> vcolor;
		std::vector<RGBA> tcolor;
	public:
		int vv(int vertex_index, int neighbor_index) const; // vertex neighbors of each vertex
		int vt(int vertex_index, int neighbor_index) const; // triangle neighbors of each vertex
		int tt(int face_index, int neighbor_index) const; // triangle neighbors of each triangle
		int ev(int edge_index, int neighbor_index) const; // next's head & opposite's next's head
		int et(int edge_index, int neighbor_index) const; // this edge's adjacent face & opposite's adjacent face
	public:
		void translate(Vector t);
		void rotate(Quaternion r);
		void scale(Vector s);
		static void write_to_ply(const std::vector<Vertex>& vs, const std::vector<Triangle>& ts, const string& path);
		static void write_to_ply(const std::vector<Vertex>& vs, const std::vector<Triangle>& ts, const std::vector<RGBA>& vcs, const string& path);
		void write_to_ply(const string& path);
		void paint_ball(const Vertex& center, float radius, int cuts, const Color& color);
	public:
		float triangle_area(int face) const;
		Vertex triangle_center(int face) const;
		Vertex triangle_center(const Triangle& face) const;
		Vector triangle_normal(int face, bool invert = false) const;
		float triangle_dihedral_angle(int edge, bool rad = false) const;
		float triangle_dihedral_angle(int f0, int f1, bool rad = false) const;
		static float triangle_dihedral_angle(const Vector& n0, const Vector& n1, bool rad = false);
		int triangle_neighbor(int face, int v0, int v1, int& opposite_vertex) const;
		static float vertex_plane_distance(const Plane& plane, const Vertex& p);
		bool intersect_triangle(Vertex orig, Vertex dir, int face, float& t) const;
		float vertex_triangle_distance(int face, const Vertex& p);
		static Vertex project_to_plane(const Vertex& v, const Plane& plane);
		bool triangle_cross_plane(int face, const Plane& plane) const;
	protected:
		typedef std::vector<int> IntVec;
	protected:
		IntVec vv_data; // vertex neighbors of each vertex, access using vv()
		IntVec vv_seed; // vertex neighbor start position on vv_data
		IntVec vt_data; // triangle neighbors of each vertex, access using vt()
		IntVec vt_seed; // triangle neighbor start position on vt_data
	public:
		IntVec tt_data; // triangle neighbors of each triangle, access using tt()
	protected:
		IntVec ev_data; // vertex neighbors of each edge, access using ev()
		IntVec et_data; // triangle neighbors of each edge, access using et()
	};

	// -------------------- Struct Deformable Mesh --------------------

	struct DeformableTriangleMesh : public TriangleMesh {
	public:
		void translate(Vector t);
		void rotate(Quaternion r);
		void scale(Vector s);
	public:
		void read_skeleton_data(const string& path);
		void apply_rotation(int ske_id, const Quaternion& rot);
		
		void apply_rotation_with_cor(int ske_id, const Vector& dir, bool change_cors = true, bool correct = true);
		void apply_rotation_with_cor(int ske_id, const Quaternion& rot, bool change_cors = true);
		void apply_rotation_with_cor(int ske_id, const Quaternion& rot, std::map<int, Vertex>& cors, bool change_cors = true);
		void apply_rotation_with_cor_helper(int ske_id, const Quaternion& rot, std::map<int, Vertex>& cors, bool change_cors, bool correct = true);
		enum JOINT_NAME {
			j_shoulders, j_back, j_hips, j_head,
			j_lhip, j_lknee, j_lankle, j_lfoot,
			j_rhip, j_rknee, j_rankle, j_rfoot,
			j_lshoulder, j_lelbow, j_lhand,
			j_rshoulder, j_relbow, j_rhand, j_number
		};
		Vertex transform_by_joint(JOINT_NAME jn, const Vertex& v, const Quaternion& q);
		double get_weight(int vidx, JOINT_NAME skeidx) { return weights[vidx * JOINT_NAME::j_number + skeidx]; }
	private:
		void apply_deformation();
		void apply_deformation_with_cor(std::map<int, Vertex>& cors, bool change_cors);
		void compute_joint_position(int joint_id, std::vector<Vertex>& joint_pos, std::vector<bool>& compute_status);
	public:
		std::map<int, Vertex> cors;
		std::vector<Vertex> joints; // joint coordinates
		std::vector<int> joint_parent; // maintain skeleton structure
		std::vector<double> weights; // skinning weights, vertex number * joint number (v0.b0, v0.b1... v0.bn, v1.b0... vn.bn)
	private:
		std::vector<std::vector<int>> joint_children; // maintain skeleton structure
		std::vector<Quaternion> joint_rotation; // joint transformation
		std::vector<Vertex> ov; // original vertices
		std::vector<Vertex> oj; // original joint coordinates
	};

	// -------------------- Struct OpenMesh Triangle Mesh --------------------

	struct OpenMeshTriangleMesh : public DeformableTriangleMesh {
	public:
		VertexHandle add_vertex(const Vertex& vtx);
		FaceHandle add_face(const VertexHandle& vh0, const VertexHandle& vh1, const VertexHandle& vh2);
		void color_vertex(int index, const Color& color);
		void color_vertex(VertexHandle& vtxh, const Color& color);
		void color_face(int index, const Color& color);
		void update_all_vertex();
		void transfer_vertex_coord(OpenMeshTriangleMesh& mesh);
		void paint_ball_openmesh(const Vertex& center, float radius, int cuts, const Color& color);
	public:
		bool read_from_file(const string& input_file_name);
		bool write_to_file(const string& output_file_name) const;
	public:
		std::vector<VertexHandle> vh; // OpenMesh handle
		std::vector<FaceHandle> th; // OpenMesh handle
	public:
		string mesh_file_name;
	private:
		ManSegMesh openmesh_mesh;
		Options openmesh_io_option;
	private:
		void mesh_setting();
		void io_setting();
		void mesh_data_init();
	public:
		Color BLACK;
		Color WHITE;
		Color RED;
		Color GREEN;
		Color BLUE;
		Color PURPLE;
		Color CYAN;
		Color YELLOW;
		Color MYCOLOR[50];
	};
}

#endif