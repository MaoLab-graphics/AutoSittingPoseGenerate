// Viewer

#include "Viewer.h"
#include <iostream>
#include "Segmentation.h"

static int INTERVAL = 3;

namespace ManSeg {

	int Viewer::mouse_state = 0;
	GLfloat Viewer::prev_mouse_x = 0.0;
	GLfloat Viewer::prev_mouse_y = 0.0;
	float Viewer::rotate_x = 0.0f;
	float Viewer::rotate_y = 0.0f;
	float Viewer::move_x = 0.0f;
	float Viewer::move_y = 0.0f;
	float Viewer::move_z = 0.0f;
	float Viewer::scale = 0.001;
	int Viewer::show_state = 1;
	int Viewer::cut_plane = 0;
	GLuint Viewer::face_list;
	GLuint Viewer::wire_list;
	Mesh Viewer::mesh;
	Mesh Viewer::original_mesh;
	std::vector<Vector> Viewer::body_ske;
	std::vector<std::vector<int>> Viewer::cut_rings;
	GLCamera Viewer::camera(Vector(0, 0, 10), Vector(0, 0, 0), Vector(0, 1, 0));
	GLuint Viewer::tex_id;

	GLCamera::GLCamera(const Vector &pos, const Vector &target, const Vector &up)
	{
		m_pos = pos;
		m_target = target;
		m_up = up;
		n = Vector(pos.x - target.x, pos.y - target.y, pos.z - target.z);
		u = Vector(up.cross(n).x, up.cross(n).y, up.cross(n).z);
		v = Vector(n.cross(u).x, n.cross(u).y, n.cross(u).z);


		n.normalize();
		u.normalize();
		v.normalize();

		setModelViewMatrix();
	}
	void GLCamera::setModelViewMatrix()
	{
		double m[16];
		m[0] = u.x; m[4] = u.y; m[8] = u.z; m[12] = -m_pos.dot(u);
		m[1] = v.x; m[5] = v.y; m[9] = v.z; m[13] = -m_pos.dot(v);
		m[2] = n.x; m[6] = n.y; m[10] = n.z; m[14] = -m_pos.dot(n);
		m[3] = 0;  m[7] = 0;  m[11] = 0;  m[15] = 1.0;
		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixd(m);     //用M矩阵替换原视点矩阵  
	}
	void  GLCamera::setShape(float viewAngle, float aspect, float Near, float Far)
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();                                   //设置当前矩阵模式为投影矩阵并归一化  
		gluPerspective(viewAngle, aspect, Near, Far);        //对投影矩阵进行透视变换  
	}
	void GLCamera::slide(float du, float dv, float dn)
	{
		//std::cout<<"u.x:"<<u.x<<std::endl;  
		m_pos.x = m_pos.x + du*u.x + dv*v.x + dn*n.x;
		m_pos.y = m_pos.y + du*u.y + dv*v.y + dn*n.y;
		m_pos.z = m_pos.z + du*u.z + dv*v.z + dn*n.z;
		m_target.x = m_target.x + du*u.x + dv*v.x + dn*n.x;
		m_target.y = m_target.x + du*u.y + dv*v.y + dn*n.y;
		m_target.z = m_target.x + du*u.z + dv*v.z + dn*n.z;
		setModelViewMatrix();
	}
	void GLCamera::roll(float angle)
	{
		float cs = cos(angle*3.14159265 / 180);
		float sn = sin(angle*3.14159265 / 180);
		Vector t(u);
		Vector s(v);
		u.x = cs*t.x - sn*s.x;
		u.y = cs*t.y - sn*s.y;
		u.z = cs*t.z - sn*s.z;

		v.x = sn*t.x + cs*s.x;
		v.y = sn*t.y + cs*s.y;
		v.z = sn*t.z + cs*s.z;

		setModelViewMatrix();          //每次计算完坐标轴变化后调用此函数更新视点矩阵  
	}
	void GLCamera::pitch(float angle)
	{
		float cs = cos(angle*3.14159265 / 180);
		float sn = sin(angle*3.14159265 / 180);
		Vector t(v);
		Vector s(n);

		v.x = cs*t.x - sn*s.x;
		v.y = cs*t.y - sn*s.y;
		v.z = cs*t.z - sn*s.z;

		n.x = sn*t.x + cs*s.x;
		n.y = sn*t.y + cs*s.y;
		n.z = sn*t.z + cs*s.z;


		setModelViewMatrix();
	}
	void GLCamera::yaw(float angle)
	{
		float cs = cos(angle*3.14159265 / 180);
		float sn = sin(angle*3.14159265 / 180);
		Vector t(n);
		Vector s(u);

		n.x = cs*t.x - sn*s.x;
		n.y = cs*t.y - sn*s.y;
		n.z = cs*t.z - sn*s.z;

		u.x = sn*t.x + cs*s.x;
		u.y = sn*t.y + cs*s.y;
		u.z = sn*t.z + cs*s.z;

		setModelViewMatrix();
	}
	float GLCamera::getDist()
	{
		float dist = pow(m_pos.x, 2) + pow(m_pos.y, 2) + pow(m_pos.z, 2);
		return pow(dist, 0.5);
	}

	void Viewer::init_display() {
		glClearColor(0.0, 0.0, 0.0, 0.0);
		glClearDepth(2.0);
		glShadeModel(GL_SMOOTH);
		glShadeModel(GL_FLAT);
		// ------------------- Lighting  
		{
			GLfloat sun_light_position[] = { 10000.0f, 10000.0f, 10000.0f, 10000.0f };
			GLfloat sun_light_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
			GLfloat sun_light_diffuse[] = { 0.1f, 0.1f, 0.1f, 1.0f };
			GLfloat sun_light_specular[] = { 0.0f, 0.0f, 0.0f, 1.0f };

			glLightfv(GL_LIGHT0, GL_POSITION, sun_light_position); //指定第0号光源的位置 
			glLightfv(GL_LIGHT0, GL_AMBIENT, sun_light_ambient); //GL_AMBIENT表示各种光线照射到该材质上，
			//经过很多次反射后最终遗留在环境中的光线强度（颜色）
			glLightfv(GL_LIGHT0, GL_DIFFUSE, sun_light_diffuse); //漫反射后~~
			glLightfv(GL_LIGHT0, GL_SPECULAR, sun_light_specular);//镜面反射后~~~
		}
		glEnable(GL_LIGHT0); //第一个光源，而GL_LIGHT1表示第二个光源
		glEnable(GL_LIGHTING); // 如果enbale那么就使用当前的光照参数去推导顶点的颜色
		glEnable(GL_DEPTH_TEST); //用来开启深度缓冲区的功能，启动后OPengl就可以跟踪Z轴上的像素，那么它只有在前面没有东西的情况下才会绘制这个像素，在绘制3d时，最好启用，视觉效果会比较真实

		// Tex
		int tex_h = 256, tex_w = 256;
		unsigned char* pixels = new unsigned char[tex_h * tex_w * 3];
		for (int i = 0; i < tex_h; ++i) {
			for (int j = 0; j < tex_w; ++j) {
				pixels[i * tex_w * 3 + j * 3 + 0] = 255;
				pixels[i * tex_w * 3 + j * 3 + 1] = 0;
				pixels[i * tex_w * 3 + j * 3 + 2] = 0;
			}
		}
		glGenTextures(1, &tex_id);
		glBindTexture(GL_TEXTURE_2D, tex_id);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex_w, tex_h, 0,
			GL_BGR_EXT, GL_UNSIGNED_BYTE, pixels);
		glBindTexture(GL_TEXTURE_2D, tex_id);

		// ------------------- Display List  
		face_list = glGenLists(1);
		wire_list = glGenLists(1);

		// 绘制 wire 
		glNewList(wire_list, GL_COMPILE);
		glLineWidth(1.0f);
		glColor3f(1.0f, 0.0f, 0.0f);
		glBegin(GL_LINES);
		for (int i = 0; i < (int)mesh.e.size(); ++i) {
			for (int j = 1; j >= 0; --j) {
				auto v = mesh.v[mesh.e[i].v[j]];
				glVertex3f(v.x, v.y, v.z);
			}
		}
		glEnd();
		glEndList();

		// 绘制flat
		glBindTexture(GL_TEXTURE_2D, tex_id);
		glNewList(face_list, GL_COMPILE);
		for (int i = 0; i < (int)mesh.t.size(); ++i) {
			glBegin(GL_TRIANGLES);
			auto n = mesh.triangle_normal(i);
			for (int j = 0; j < 3; ++j) {
				glNormal3f(n.x, n.y, n.z);
				auto v = mesh.v[mesh.t[i].v[j]];
				//glColor3f((GLfloat)v.r / 255.0f, (GLfloat)v.g / 255.0f, (GLfloat)v.b / 255.0f);
				float p = i / (int)mesh.t.size();
				if (j == 0) glTexCoord2f(0.0f, 0.0f);
				else if (j == 1) glTexCoord2f(1.0f, 0.0f);
				else glTexCoord2f(0.0f, 1.0f);
				glVertex3f(v.x, v.y, v.z);
			}
			glEnd();
		}
		glEndList();
	}
	void Viewer::read_skeleton(string ply_path) {
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
		body_ske.resize(size);
		for (int i = 0; i < size; ++i) {
			body_ske[i].x = vec[i].x;
			body_ske[i].y = vec[i].y;
			body_ske[i].z = vec[i].z;
		}
		fclose(fp);
	}
	void Viewer::compute_intersection() {
		read_skeleton("mesh/body_ske.ply");
		smooth_skeleton_points(100);
		int size = body_ske.size();
		cut_rings.resize(size);
		SkeletalSegmentation seg(&mesh);
		int interval = INTERVAL;
		for (int i = interval; i < size - interval; ++i) {
			auto plane = Plane::create_plane(body_ske[i], body_ske[i - interval], body_ske[i + interval]);
			seg.search_loop(plane, body_ske[i], cut_rings[i]);
		}
	}
	void Viewer::smooth_skeleton_points(int loop_times) {
		const int interval = 1;
		for (int i = 0; i < loop_times; ++i) {
			auto temp = body_ske;
			for (int j = interval; j < (int)body_ske.size() - interval; ++j) {
				body_ske[j] = (temp[j - interval] + temp[j + interval]) / (float)2;
			}
		}
	}
	void Viewer::redraw()
	{
		// 绘制flat
		glDeleteLists(face_list, 1);
		glNewList(face_list, GL_COMPILE);
		for (int i = 0; i < (int)mesh.t.size(); ++i) {
			glBegin(GL_TRIANGLES);
			auto n = mesh.triangle_normal(i);
			for (int j = 0; j < 3; ++j) {
				glNormal3f(n.x, n.y, n.z);
				auto v = mesh.v[mesh.t[i].v[j]];
				auto vc = mesh.vcolor[mesh.t[i].v[j]];
				glColor3f((GLfloat)vc.r / 255.0f, (GLfloat)vc.g / 255.0f, (GLfloat)vc.b / 255.0f);
				glVertex3f(v.x, v.y, v.z);
			}
			glEnd();
		}
		glEndList();
	}
	void Viewer::move_cut_plane(int val) {
		int interval = INTERVAL;
		int prev_plane = cut_plane;
		cut_plane -= val;
		if (cut_plane < interval) cut_plane = interval;
		if (cut_plane >= (int)body_ske.size() - interval)
			cut_plane = (int)body_ske.size() - 1 - interval;
		for (int i = 0; i < (int)mesh.v.size(); ++i) {
			RGBA& ov = original_mesh.vcolor[i]; RGBA& v = mesh.vcolor[i];
			v.r = ov.r; v.g = ov.g; v.b = ov.b;
		}
		for (int i = 0; i < (int)cut_rings[cut_plane].size(); ++i) {
			Triangle& t = mesh.t[cut_rings[cut_plane][i]];
			for (int j = 0; j < 3; ++j) {
				RGBA& v = mesh.vcolor[t.v[j]];
				v.r = (unsigned char)255; v.g = (unsigned char)0; v.b = (unsigned char)255;
			}
		}
		redraw();
	}
	void Viewer::reshade(GLint w, GLint h)
	{
		// resize
		glViewport(0, 0, static_cast<GLsizei>(w), static_cast<GLsizei>(h));
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		if (w > h)
			glOrtho(-static_cast<GLdouble>(w) / h, static_cast<GLdouble>(w) / h, -1.0, 1.0, -100.0, 100.0);
		else
			glOrtho(-1.0, 1.0, -static_cast<GLdouble>(h) / w, static_cast<GLdouble>(h) / w, -100.0, 100.0);
		//camera.setShape(45.0, (GLfloat)w / (GLfloat)h, 0.1, 100.0);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}
	void Viewer::keyboard(int key, int x, int y) {
		switch (key) {
		case GLUT_KEY_F1:
			show_state = 1;
			std::cout << "切换显示模式为：Flat" << std::endl;
			break;
		case GLUT_KEY_F2:
			show_state = 2;
			std::cout << "切换显示模式为：Flat Lines" << std::endl;
			break;
		case GLUT_KEY_F3:
			show_state = 3;
			std::cout << "切换显示模式为：Wire" << std::endl;
			break;
		case GLUT_KEY_UP:
			move_cut_plane(1);
			break;
		case GLUT_KEY_DOWN:
			move_cut_plane(-1);
			break;
		case GLUT_KEY_LEFT:
			move_cut_plane(5);
			break;
		case GLUT_KEY_RIGHT:
			move_cut_plane(-5);
			break;
		default:
			break;
		}
		glutPostRedisplay();
	}
	void Viewer::mouse(int button, int state, int x, int y)
	{
		if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
			mouse_state = 1;
			prev_mouse_x = x;
			prev_mouse_y = y;
		}
		if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
			mouse_state = 2;
			prev_mouse_x = x;
			prev_mouse_y = y;
		}
		if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN) {
			mouse_state = 3;
			prev_mouse_x = x;
			prev_mouse_y = y;
		}
		if (state == GLUT_UP)
			mouse_state = 0;
		glutPostRedisplay();
	}
	void Viewer::rotate_camera_x(float angle)
	{
		float d = camera.getDist();
		int cnt = 100;
		float theta = angle / cnt;
		float slide_d = -2 * d*sin(theta*3.14159265 / 360);
		camera.yaw(theta / 2);
		for (; cnt != 0; --cnt)
		{
			camera.slide(slide_d, 0, 0);
			camera.yaw(theta);
		}
		camera.yaw(-theta / 2);
	}
	void Viewer::rotate_camera_y(float angle)
	{
		float d = camera.getDist();
		int cnt = 100;
		float theta = angle / cnt;
		float slide_d = 2 * d*sin(theta*3.14159265 / 360);
		camera.pitch(theta / 2);
		for (; cnt != 0; --cnt)
		{
			camera.slide(0, slide_d, 0);
			camera.pitch(theta);
		}
		camera.pitch(-theta / 2);
	}
	void Viewer::on_mouse_move(int x, int y) {
		GLfloat dx = x - prev_mouse_x;
		GLfloat dy = y - prev_mouse_y;
		if (mouse_state == 1) {
			//x对应y是因为对应的是法向量
			rotate_y += dx;
			rotate_x += dy;
			prev_mouse_x = x;
			prev_mouse_y = y;
			glutPostRedisplay();
			//rotate_camera_x(dx);
			//rotate_camera_y(dy);
		}
		else if (mouse_state == 2) {
			//camera.pitch(dy);
			scale -= 0.00002*(y - prev_mouse_y);
			prev_mouse_x = x;
			prev_mouse_y = y;
			glutPostRedisplay();
		}
		else if (mouse_state == 3) {
			//camera.slide(dx, dy, 0);
			move_x -= 0.005*(x - prev_mouse_x);
			move_y -= 0.005*(y - prev_mouse_y);
			prev_mouse_x = x;
			prev_mouse_y = y;
			glutPostRedisplay();
		}
	}
	void Viewer::display()
	{
		//要清除之前的深度缓存
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();
		//camera.setModelViewMatrix();
		//与显示相关的函数
		glRotatef(rotate_x, 1.0f, 0.0f, 0.0f); // 让物体旋转的函数 第一个参数是角度大小，后面的参数是旋转的法向量
		glRotatef(rotate_y, 0.0f, 1.0f, 0.0f);
		glTranslatef(move_x, move_y, move_z);
		glScalef(scale, scale, scale); // 缩放

		glBindTexture(GL_TEXTURE_2D, tex_id);
		//每次display都要使用glcalllist回调函数显示想显示的顶点列表
		if (show_state == 1)
			glCallList(face_list);
		else if (show_state == 2) {
			glCallList(face_list);
			glCallList(wire_list);
		}
		else if (show_state == 3)
			glCallList(wire_list);

		glutSwapBuffers(); //这是Opengl中用于实现双缓存技术的一个重要函数
	}
	void Viewer::run(string mesh_path, int* argc, char** argv) {
		glutInit(argc, argv);
		glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH); // GLUT_Double 表示使用双缓存而非单缓存
		glutInitWindowPosition(100, 100);
		glutInitWindowSize(800, 500);
		glutCreateWindow("Mesh Viewer");
		original_mesh.read_from_file(mesh_path);
		mesh.read_from_file(mesh_path);
		compute_intersection();
		init_display();
		glutMouseFunc(mouse);
		glutMotionFunc(on_mouse_move); // mouse
		glutSpecialFunc(keyboard);
		glutReshapeFunc(reshade);
		glutDisplayFunc(display);
		glutMainLoop();
	}
}