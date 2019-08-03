// Viewer

#include "Mesh.h"
#include "GL\glut.h"

namespace ManSeg {

	class GLCamera
	{
	public:
		GLCamera() {}
		GLCamera(const Vector& pos, const Vector& target, const Vector& up);
		void setModelViewMatrix();
		void setShape(float viewAngle, float aspect, float Near, float Far);
		void slide(float du, float dv, float dn);
		void roll(float angle);
		void yaw(float angle);
		void pitch(float angle);
		float getDist();
	private:
		Vector m_pos;
		Vector m_target;
		Vector m_up;
		Vector u, v, n;
	};

	class Viewer {
	public:
		typedef std::vector<int> IntVec;
	public:
		static int mouse_state;
		static GLfloat prev_mouse_x;
		static GLfloat prev_mouse_y;
		static float rotate_x;
		static float rotate_y;
		static float move_x;
		static float move_y;
		static float move_z;
		static float scale;
		static GLCamera camera;
		static Mesh mesh;
		static Mesh original_mesh;
		static GLuint face_list, wire_list;
		static int show_state;
		static int cut_plane;
		static std::vector<Vector> body_ske;
		static std::vector<IntVec> cut_rings;
		static GLuint tex_id;
	private:
		static void init_display();
		static void smooth_skeleton_points(int loop_times);
		static void compute_intersection();
		static void read_skeleton(string ply_path);
		static void redraw();
		static void move_cut_plane(int val);
		static void rotate_camera_x(float angle);
		static void rotate_camera_y(float angle);
		static void reshade(GLint w, GLint h); // resize window
		static void keyboard(int key, int x, int y);
		static void mouse(int button, int state, int x, int y);
		static void on_mouse_move(int x, int y);
		static void display();
	public:
		static void run(string mesh_path, int* argc, char** argv);
	};

}