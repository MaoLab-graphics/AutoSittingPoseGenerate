// Reshape

#include "Reshape.h"
#include "Mesh.h"
#include "Voxel.h"
#include "MassSpringSystem.h"
#include "Skeletonization.h"
#include "DeformSolver.h"
#include <string>
#include <time.h>
#include <fstream>
#include <set>
#include <sstream>
#include <iostream>

using namespace std;
namespace ManSeg {

	// -------------------- Class Reshape --------------------

	// main
	void Reshape::deform_to_seat(int humanNum,int chairNum) {

		Vector HUMAN_ROT_AXIS = { 0, 1, 0 };
		float HUMAN_ROT_ANGLE = 0.f;//8 -8
		bool NEED_CHAIR_ROTATE = true;
		float ARM_ON_THIGH_LIFT_MUL_L = 0.0f;//girl2 0 2
		float ARM_ON_THIGH_LIFT_MUL_R = 0.0f;//girl2 0 1
		float FOREARM_LIFT_MUL = 1.0f;
		float BACK_TOUCH_PORTION = 0.075f;//0.75f;
		Vertex BODY_TRANS_ADJUST = { 0, 5, 0 };
		bool ARM_ON_THIGH = false;
		bool ARM_USE_STEP_2 = false;
		bool ARM_STEP_2_NO_BACKWARD = true;
		float ARM_TOUCH_MUL = 2.0f;
		Vertex BACK_INTERSECTION_ADJUST = { 0, 0, -2.0f };
		float SEAT_POSSIBLE_TOUCH_AREA_MUL = 0.8f;//.8
		float BACK_POSSIBLE_TOUCH_AREA_MUL = 0.25f;//.25
		int SEAT_DEFORM_MAX_LOOP = 100000;
		int BACK_DEFORM_MAX_LOOP = 500;

		bool TRANSLATE_HUMAN = true;
		bool DEFORM_HEAD = true;
		bool DEFORM_BACK = true;
		bool DEFORM_LEG = true;
		bool DEFORM_ARM = true;
		bool DEFORM_BACK_BY_ENERGY = false;
		bool DEFORM_ARM_BY_ENERGY = false;
		// init, default sitting pose
		deform_to_sitting_pose();

		//return;
		ifstream srcFile1("para1.txt", ios::in); 
		ifstream srcFile2("para2.txt", ios::in);
		int p1[10][4] = { 0 };
		int p2[32][4] = { 0 };
		for (int i = 0; i < 8;i++)
			for (int j = 0; j<4; j++)
			srcFile1 >> p1[i][j];
		for (int i = 0; i < 24; i++)
			for (int j = 0; j<4; j++)
				srcFile2 >> p2[i][j];
		srcFile1.close();
		srcFile2.close();
		// 1:60 2:80 4:139 6:135 14:150 15:190
		float SEAT_WEIGHT = 100;
		bool COUNT_BACK_CONTACT_AREA = true;

		// chair, read chair, scale
		Chair& chair = _chair;
		stringstream ss;
		ss << chairNum;
		string chairString = ss.str();
		string chair_path = "..\\..\\..\\..\\..\\..\\ChairModel\\chair" + chairString+"\\";
		cout << chair_path;
		StrVec seat_name = { "seat.obj" };
		StrVec back_name = { "back.obj" };
		StrVec arm_names = { "arm0.obj", "arm1.obj" };
	    StrVec leg_names = { "leg.obj","leg1.obj", "leg2.obj", "leg3.obj", "leg4.obj" };
		int fgo;
		Vector chair_names;
		if (chairNum < 8)
		{
			chair_names.x = p1[chairNum - 1][0];
			chair_names.y = p1[chairNum - 1][1];
			chair_names.z = p1[chairNum - 1][2];
			fgo = p1[chairNum - 1][3];
		}
		else
		{
			chair_names.x = 25;
			chair_names.y = 25;
			chair_names.z =25;
			fgo = 1;
		}
		init_chair(chair_path, seat_name, back_name, arm_names, leg_names, {}, chair_names, fgo);
		
		// rotate chair
		Vertex seat_cen = find_centroid(chair.seat.v);
		Vertex back_cen = find_centroid(chair.back.v);
		Vertex seat_contact = find_closest(chair.seat.v, back_cen);
		Vector chair_z = seat_cen - seat_contact; chair_z.y = 0; chair_z.normalize();
		Mesh a;

		if (NEED_CHAIR_ROTATE) {
			for (int i = 0; i < chair.p.size(); ++i)
				chair.p[i]->rotate(Quaternion(chair_z, { 0, 0, -1 }));
			seat_cen = find_centroid(chair.seat.v);
			back_cen = find_centroid(chair.back.v);
			seat_contact = find_closest(chair.seat.v, back_cen);
		}
		
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
			if (chair.is_wheel_chair) {
				float prev_ymax = ymax;
				for (auto v : chair.feet[0].v) {
					if (v.y < ymin) ymin = v.y; if (v.y > ymax) ymax = v.y;
				}
				seat_parm.y = prev_ymax - ymin;
			}
		}

		// compute chair back parameters
		Vector chair_back_max, chair_back_min;
		if (chair.has_back) {
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
		Vertex crotch = _human->v[_landmarks.crotch];
		{
			Vector man_z = (knee_cen[0] - crotch).normalized() + (knee_cen[1] - crotch).normalized();
			man_z.y = 0; man_z.normalize();
			_human->rotate(Quaternion(man_z, { 0, 0, -1 }));
			crotch = _human->v[_landmarks.crotch];
			knee_cen[0] = find_man_ske_pt(PartType::p_leg0, LegCut::c_knee);
			knee_cen[1] = find_man_ske_pt(PartType::p_leg1, LegCut::c_knee);
		}
		if (humanNum == 1 && (chairNum == 4 || chairNum == 6))
			HUMAN_ROT_ANGLE = -8.f;
		_human->rotate(Quaternion(HUMAN_ROT_AXIS, HUMAN_ROT_ANGLE));
		
		// find knee
		Vertex knee_inside[2];
		{
			float zmaxs[2] = { DECIMAL_MIN, DECIMAL_MIN };
			for (int i = 0; i < 2; ++i) {
				float d = abs((find_closest(_human->v, knee_cen[i]) - knee_cen[i]).y);
				int jn = (i == 0 ? _human->j_lknee : _human->j_rknee);
				int jn2 = (i == 0 ? _human->j_rknee : _human->j_lknee);
				knee_inside[i] = _human->joints[jn] + Vector(0, -d, d);
			}
		}

		// find foot bottom
		Vertex foot_bottom[2];
		{
			float ymins[2] = { DECIMAL_MAX, DECIMAL_MAX };
			for (int i = 0; i < _human->t.size(); ++i) {
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

		// find hip
		const float ratio = 0.1;
		Vertex hip_tail;
		{
			Vertex hip_joint = _human->joints[Mesh::j_hips];
			Plane hip_FB(hip_joint, Vector(0, -1, 1));
			Plane hip_LR(hip_joint, { 1, 0, 0 });
			IntVec hip_tail_candidates;
			for (int i = 0; i < _human->t.size(); ++i) {
				if (_part_labels[i] == DetailPartType::downbody
					&& point_plane_distance(hip_FB, _human->triangle_center(i)) > 0) {
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
					int v0 = _human->t[i].v[j];
					int v1 = _human->t[i].v[(j + 1) % 3];
					float temp0 = point_plane_distance(hip_LR, _human->v[v0]);
					float temp1 = point_plane_distance(hip_LR, _human->v[v1]);
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

		// move & leg
		Vector adjust_disp = BODY_TRANS_ADJUST;
		if (chair.is_wheel_chair) adjust_disp += Vector(0, 0, -20.0f);
		float sit_depth = 0;
		{
			Vector seat_dir = seat_cen - seat_contact;
			Vector back_dir = back_cen - seat_contact;
			Vector chair_in = (seat_dir.normalized() + back_dir.normalized()).normalized();
			Vertex knee_inside_mid = (knee_inside[0] + knee_inside[1]) * 0.5f;
			Vertex foot_bottom_mid = (foot_bottom[0] + foot_bottom[1]) * 0.5f;
			float shank_length = abs(knee_inside_mid.y - foot_bottom_mid.y);
			sit_depth = abs((hip_tail - knee_inside_mid).z);
			Vector trans = { 0, 0, 0 };


			bool need_lift_leg = (knee_inside_mid.y - foot_bottom_mid.y > seat_parm.y);
			if (need_lift_leg)
			{
				std::cout << "lift leg yes" << std::endl;
			}
			else
			{
				std::cout << "lift leg no" << std::endl;
			}
			// lounger
			

			// lift shank
			if (need_lift_leg) {
				Vertex hip_joint_mid = (_human->joints[_human->j_lhip] + _human->joints[_human->j_rhip]) / 2;
				float diff = knee_inside_mid.y - foot_bottom_mid.y - seat_parm.y;
				if (diff > 0) {
					std::cout << "shank lifted :" << diff << std::endl;
					Vector from = knee_inside_mid - hip_joint_mid; from.x = 0;
					std::cout << "knee mid z :" << knee_inside_mid.z << std::endl;
					std::cout << "hip_joint mid z :" << hip_joint_mid.z << std::endl;
					std::cout << "knee mid - hip mid z:" << from.z << std::endl;
					Vector to = from; to.y += diff; to.z = from.square_length() - to.y*to.y;
					std::cout << "to.y:" << to.y << std::endl;
					std::cout << (_human->joints[_human->j_rknee] - _human->joints[_human->j_rhip]).y << std::endl;
					to.y = (_human->joints[_human->j_rknee] - _human->joints[_human->j_rhip]).y + diff;
					if (to.z >= 0) {
						to.z = std::sqrt(to.z); to.z = -abs(to.z); to.x =( _human->joints[_human->j_lknee] - _human->joints[_human->j_lhip]).x;
						sit_depth = abs((hip_tail - (hip_joint_mid + to)).z);
						if (DEFORM_LEG) _human->apply_rotation_with_cor(_human->j_lknee, to);
						to.x = (_human->joints[_human->j_rknee] - _human->joints[_human->j_rhip]).x;
						if (DEFORM_LEG) _human->apply_rotation_with_cor(_human->j_rknee, to);
						for (int i = 0; i < 2; ++i) {
							int jn = i == 0 ? _human->j_lankle : _human->j_rankle;
							if (DEFORM_LEG) _human->apply_rotation_with_cor(jn, { 0, -1, 0 });
						}
					}
				}
			}


			// move man's hip tail point
			Vertex seat_front_pt = seat_contact - Vector(0, 0, seat_parm.z);
			float move_z = std::min(seat_parm.z, sit_depth);
			trans = seat_front_pt + Vector(0, 0, move_z) + chair_in * 10 - hip_tail + adjust_disp;
			
			if (!chair.has_back)
				trans = seat_cen + Vector(0, 0.5f * seat_parm.y, move_z - 0.5f * seat_parm.z) - hip_tail + adjust_disp;
			if (chair.is_lounger)
				trans = seat_contact + chair_in * 10 - hip_tail + adjust_disp;
			if (humanNum < 3)
			{
				trans.z += p2[(humanNum - 1) * 8 + chairNum - 1][0];
				trans.y -= p2[(humanNum - 1) * 8 + chairNum - 1][1];
			}

			if (TRANSLATE_HUMAN) _human->translate(trans);
			hip_tail += trans;

			// wheel chair
		
		}

		{
			// update knee
			Vertex knee_LR_out[2], knee_LR_in[2];
			float zmaxs[2] = { DECIMAL_MIN, DECIMAL_MIN };
			for (int i = 0; i < 2; ++i) {
				float dy = abs((find_closest(_human->v, knee_cen[i]) - knee_cen[i]).y);
				int jn = (i == 0 ? _human->j_lknee : _human->j_rknee);
				int jn2 = (i == 0 ? _human->j_rknee : _human->j_lknee);
				knee_inside[i] = _human->joints[jn] + Vector(0, -dy, dy);
				Vector dir = _human->joints[jn] - _human->joints[jn2];
				knee_LR_out[i] = _human->joints[jn] + Vector(dir.x, 0, 0).normalized();
				knee_LR_in[i] = _human->joints[jn] + Vector(-dir.x, 0, 0).normalized();
			}

			// if man width > seat width
		
		}
	
		// back
		Vector chair_back_vec = { 0, 0, 0 };
		_human->joints[_human->j_hips] = (_human->joints[_human->j_lhip] + _human->joints[_human->j_rhip]) / 2;
		if (DEFORM_BACK_BY_ENERGY && DEFORM_BACK) {
			IntVec vs, es;
			BoolVec target_v(_human->v.size(), false);
			for (int i = 0; i < (int)_human->v.size(); ++i) {
				Vector n;
				bool isbody = false;
				for (int j = 0; j < _human->vtn[i]; ++j) {
					int index = _human->vt(i, j);
					if (index >= 0) {
						char p = _part_labels[index];
						if (p == DetailPartType::upbody || p == DetailPartType::midbody || p == DetailPartType::downbody) {
							isbody = true;
						}
						n += _human->triangle_normal(index);
					}
				}
				if (isbody && n.z > 0) {
					vs.push_back(i);
					target_v[i] = true;
				}
			}
			for (int i = 0; i < (int)_human->e.size(); ++i) {
				if (target_v[_human->e[i].v[0]] || target_v[_human->e[i].v[1]]) {
					es.push_back(i);
				}
			}
			DeformSolver deformer(_human, &_chair.back, vs, es);
			deformer.solve("back", true, false);
		}
		if (!DEFORM_BACK_BY_ENERGY) {

			// if sit depth is not too short
			if (chair.has_back && sit_depth >= (seat_parm.z * (float)BACK_TOUCH_PORTION) && DEFORM_BACK) {

				Vector seat_dir = (seat_cen - seat_contact).normalized();
				Vector back_dir = (back_cen - seat_contact).normalized();
				Vector chair_x = seat_dir.cross(back_dir).normalized();
				Plane chair_yz = { back_cen, chair_x };
				Plane man_yz = { _human->joints[_human->j_back], chair_x };

				// find man back
				Vertex waist_back_pt, shoulders_back_pt;
				{
					IntVec ts = find_plane_intersecting_faces(*_human, man_yz);
					float dymin[2] = { DECIMAL_MAX, DECIMAL_MAX };
					for (int i : ts) {
						Vertex vtx = _human->triangle_center(i);
						char p = _part_labels[i];
						bool is_body = (p == DetailPartType::downbody || p == DetailPartType::midbody || p == DetailPartType::upbody);
						if ((vtx - _human->joints[_human->j_back]).z > 0 && is_body) {
							float d = abs(vtx.y - _human->joints[_human->j_back].y);
							if (d < dymin[0]) { dymin[0] = d; waist_back_pt = vtx; }
							d = abs(vtx.y - _human->joints[_human->j_shoulders].y);
							if (d < dymin[1]) { dymin[1] = d; shoulders_back_pt = vtx; }
						}
					}
				}

				// find chair intersect point
				Vertex rot_cen = _human->joints[_human->j_hips];
				rot_cen.x = 0;
				waist_back_pt.x = 0;
				shoulders_back_pt.x = 0;
				Vertex back_contact, waist_touch_chair_pt, shoulders_touch_chair_pt;
				{
					Vertex adjust = BACK_INTERSECTION_ADJUST;
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
											if (k == 0 && temp.y > _human->joints[_human->j_hips].y) {
												zmin[k + 1] = temp.z; temp.x = 0;
												waist_touch_chair_pt = temp + adjust;
											}
											if (k == 1 && temp.y > _human->joints[_human->j_hips].y) {
												zmin[k + 1] = temp.z; temp.x = 0;
												shoulders_touch_chair_pt = temp + adjust; flag = true;
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
				std::cout << _human->joints[_human->j_back].x << ' ' << _human->joints[_human->j_back].y << ' ' << _human->joints[_human->j_back].z << std::endl;
			

				// back angle score
			   if (chair.has_back && DEFORM_BACK)
				{
					Vector chair_back_down = waist_touch_chair_pt - back_contact;
					float angle = (seat_contact - back_contact).normalized().dot(chair_back_down.normalized());
					if (chair_back_down.y < 0) angle = -1;
					if (angle < -0.17365f && angle > -0.7071f);
					else if (angle > -0.17365f) {
						std::cout << "Seat-back Angle - 10" << std::endl;
					}
					else {
						std::cout << "Seat-back Angle - 20" << std::endl;
					}
				}
				// estimate the shape of chair back
				if (waist_touch_chair_pt.is_nonzero()) std::cout << "found waist_touch_chair_pt" << std::endl;
				chair_back_vec = (shoulders_touch_chair_pt - back_contact);
				Vector chair_back_upper = shoulders_touch_chair_pt - waist_touch_chair_pt;
				float cosine = (back_contact - waist_touch_chair_pt).normalized().dot(chair_back_upper.normalized());
				if (chair_back_upper.y < 0) cosine = -1;
				if (cosine >(float)-0.82) { // if chair back is not straight
					std::cout << "chair back is not straight, cosine : " << cosine << std::endl;
				}
				if (1) { // chair back is straight
					std::cout << "chair back is straight" << std::endl;
					if (shoulders_touch_chair_pt.is_nonzero() || waist_touch_chair_pt.is_nonzero()) {
						Vector from, to;
						if (shoulders_touch_chair_pt.is_nonzero()) {
							std::cout << "shoulders touches chair" << std::endl;
							from = rot_cen - shoulders_back_pt;
							to = rot_cen - shoulders_touch_chair_pt;
						}
						else {
							std::cout << "waist touches chair" << std::endl;
							from = rot_cen - waist_back_pt;
							to = rot_cen - waist_touch_chair_pt;
						}
						from.x = 0;
						to.x = 0;
						if (DEFORM_BACK) _human->apply_rotation_with_cor(_human->j_back, Quaternion(from, to));
					}
					else std::cout << "chair back is too short" << std::endl;
				}
			}
			else std::cout << "sit depth too short" << std::endl;
		}

		// arm
		bool arm_score_flag = false;
		float score_support_arm = 0;
		if (DEFORM_ARM_BY_ENERGY && DEFORM_ARM) {
			Vertex armrest_cen = find_centroid(chair.arms[0].v);
			float dl = (armrest_cen - _human->joints[_human->j_lelbow]).square_length();
			float dr = (armrest_cen - _human->joints[_human->j_relbow]).square_length();
			Mesh* armrest[2] = { &chair.arms[0], &chair.arms[1] };
			if (dl > dr) {
				armrest[0] = &chair.arms[1];
				armrest[1] = &chair.arms[0];
			}
			for (int idx = 0; idx < chair.arm_num-1; ++idx) {
				IntVec vs, es;
				BoolVec target_v(_human->v.size(), false);
				for (int i = 0; i < (int)_human->v.size(); ++i) {
					if (idx == 0 && _human->v[i].x > 0) continue;
					if (idx == 1 && _human->v[i].x < 0) continue;
					for (int j = 0; j < _human->vtn[i]; ++j) {
						int index = _human->vt(i, j);
						if (index >= 0) {
							char p = _part_labels[index];
							if (p == DetailPartType::hand0 || p == DetailPartType::hand1
								|| p == DetailPartType::forearm0 || p == DetailPartType::forearm1) {
								vs.push_back(i);
								target_v[i] = true;
								break;
							}
						}
					}
				}
				for (int i = 0; i < (int)_human->e.size(); ++i) {
					if (target_v[_human->e[i].v[0]] || target_v[_human->e[i].v[1]]) {
						es.push_back(i);
					}
				}
				DeformSolver deformer(_human, armrest[idx], vs, es);
				deformer.solve(idx == 0 ? "larm" : "rarm", true, true);
			}
		}
		

		if (!DEFORM_ARM_BY_ENERGY) {
			// elbow point
			float elbow_radius[2] = { 0, 0 };
			{
				Vertex vtx = find_closest(_human->v, _human->joints[_human->j_lelbow]);
				elbow_radius[0] = (vtx - _human->joints[_human->j_lelbow]).length();
				vtx = find_closest(_human->v, _human->joints[_human->j_relbow]);
				elbow_radius[1] = (vtx - _human->joints[_human->j_relbow]).length();
			}

			// deform
			bool need_on_thigh[2] = { true, true };
			for (int idx = 0; idx < chair.arm_num; ++idx) {
				const Mesh& armrest = chair.arms[idx];
				Vertex armrest_cen = find_centroid(armrest.v);
				float armrest_zmin = DECIMAL_MAX, armrest_zmax = DECIMAL_MIN;
				for (int i = 0; i < armrest.v.size(); ++i) {
					if (armrest.v[i].z > armrest_zmax) armrest_zmax = armrest.v[i].z;
					if (armrest.v[i].z < armrest_zmin) armrest_zmin = armrest.v[i].z;
				}
				float dl = (armrest_cen - _human->joints[_human->j_lelbow]).square_length();
				float dr = (armrest_cen - _human->joints[_human->j_relbow]).square_length();
				Mesh::JOINT_NAME elbow, wrist, shoulder;
				float elbow_lift_offset;
				Vertex knee_joint;
				Vector x_dir;
				if (dl < dr) {
					wrist = _human->j_lhand;
					elbow = _human->j_lelbow;
					shoulder = _human->j_lshoulder;
					elbow_lift_offset = elbow_radius[0];
					x_dir = (_human->joints[_human->j_lknee] - _human->joints[_human->j_rknee]);
				}
				else {
					wrist = _human->j_rhand;
					elbow = _human->j_relbow;
					shoulder = _human->j_rshoulder;
					elbow_lift_offset = elbow_radius[1];
					x_dir = (_human->joints[_human->j_rknee] - _human->joints[_human->j_lknee]);
				}
				knee_joint = _human->joints[elbow];
				x_dir.y = 0; x_dir.z = 0; x_dir.normalize();
				Plane armrest_yz = { armrest_cen, { 1, 0, 0 } };
				Plane elbow_xy = { (_human->joints[shoulder] + _human->joints[shoulder]) / 2, { 0, 0, 1 } };
				float elbow_bone_len = (_human->joints[elbow] - _human->joints[shoulder]).length();
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
				// step 1
				Vertex target;
				float dmin = DECIMAL_MAX;
				for (int i : armrest_upmost) {
					if (armrest.triangle_cross_plane(i, elbow_xy)) {
						Vertex vtx = armrest.triangle_center(i) + Vector(0, elbow_lift_offset, 0);
						float d = (_human->joints[shoulder] - vtx).length();
						if (d < elbow_bone_len * ARM_TOUCH_MUL && d < dmin) {
							std::cout << "found armrest contact by cutter succeed" << std::endl;
							vtx.z = elbow_xy.p.z;
							target = vtx; dmin = d;
						}
					}
				}
				//p0 = target; p1 = elbow_xy.p;

				// step 2
				if (ARM_USE_STEP_2 || !target.is_nonzero()) {
					std::cout << "found armrest contact by cutter failed" << std::endl;
					dmin = DECIMAL_MAX;
					for (int i : armrest_upmost) {
						Vertex vtx = armrest.triangle_center(i) + Vector(0, elbow_lift_offset, 0);
						float d = abs((_human->joints[shoulder] - vtx).length() - elbow_bone_len);
						Vector vec = vtx - _human->joints[shoulder];
						if (vec.length() < elbow_bone_len * ARM_TOUCH_MUL && d < dmin && (!ARM_STEP_2_NO_BACKWARD || vec.z < 0)) { dmin = d; target = vtx; }
					}
					if (!target.is_nonzero()) std::cout << "found armrest contact failded" << std::endl;
				}
				float shoulder_bone_len = (_human->joints[shoulder] - _human->joints[_human->j_shoulders]).length();
				if (!ARM_ON_THIGH && target.is_nonzero() /*&& shoulder_bone_len + elbow_bone_len > dmin*/) {
					if (wrist == _human->j_lhand) need_on_thigh[0] = false;
					else need_on_thigh[1] = false;
					std::cout << "elbow can reach armrest" << std::endl;
					float cosine = shoulder_bone_len*shoulder_bone_len + dmin*dmin - elbow_bone_len*elbow_bone_len;
					cosine /= 2 * shoulder_bone_len * dmin;
					Vector axis = (_human->joints[shoulder] - _human->joints[_human->j_shoulders]).cross(target - _human->joints[_human->j_shoulders]).normalized();
					Quaternion q(axis, acos(cosine), true);
					Vector vec0 = (q * (target - _human->joints[_human->j_shoulders]).normalized()).normalized();
					Vector vec1 = (q.inverse() * (target - _human->joints[_human->j_shoulders]).normalized()).normalized();
					Vector shoulder_bone_dir = (_human->joints[shoulder] - _human->joints[_human->j_shoulders]).normalized();
					Vector to;
					if (vec0.dot(shoulder_bone_dir) > vec1.dot(shoulder_bone_dir)) to = vec0;
					else to = vec1;
					//_human->apply_rotation_with_cor(shoulder, Quaternion(shoulder_bone_dir, to));
					if (DEFORM_ARM) _human->apply_rotation_with_cor(elbow, target - _human->joints[shoulder]);
					// forearm
					target = { 0, 0, 0 }; dmin = DECIMAL_MAX;
					float forearm_len = (_human->joints[elbow] - _human->joints[wrist]).length();
					for (int i : armrest_upmost) {
						Vertex vtx = armrest.triangle_center(i) + Vector(0, elbow_lift_offset * FOREARM_LIFT_MUL, 0);
						float d = abs(forearm_len - (_human->joints[elbow] - vtx).length());
						if (d < dmin && (vtx - _human->joints[elbow]).z < 0) { target = vtx; dmin = d; }
					}
					to = { 0, 0, -1 };
					if (target.is_nonzero()) {
						to = (target - _human->joints[elbow]).normalized();
						std::cout << "forearm move" << std::endl;
					}
					if (DEFORM_ARM) _human->apply_rotation_with_cor(wrist, to);

				}
			}

			for (int idx = 0; idx < 2; ++idx) {
				if (need_on_thigh[idx]) {
					std::cout << "elbow cannot reach armrest" << std::endl;

					Mesh::JOINT_NAME wrist = Mesh::JOINT_NAME(_human->j_lhand + idx * (_human->j_rhand - _human->j_lhand));
					Mesh::JOINT_NAME elbow = Mesh::JOINT_NAME(_human->j_lelbow + idx * (_human->j_relbow - _human->j_lelbow));
					Mesh::JOINT_NAME shoulder = Mesh::JOINT_NAME(_human->j_lshoulder + idx * (_human->j_rshoulder - _human->j_lshoulder));
					Mesh::JOINT_NAME knee = Mesh::JOINT_NAME(_human->j_lknee + idx * (_human->j_rknee - _human->j_lknee));
					Vector x_dir = (_human->joints[_human->j_lknee] - _human->joints[_human->j_rknee]);
					if (idx == 1) x_dir = x_dir * -1;
					// upper arm
					float d = (_human->joints[elbow] - _human->joints[shoulder]).length() * (float)1;
					Vector elbow_bone_dir = (_human->joints[elbow] - _human->joints[shoulder]).normalized();
					Vector to = (_human->joints[knee] + Vector(0, -10 * d, 0) + (x_dir * d * 5) - _human->joints[shoulder]).normalized();
					//_human->apply_rotation_with_cor(elbow, Quaternion(elbow_bone_dir, to));
					// forearm
					float dy = abs((find_closest(_human->v, _human->joints[knee]) - _human->joints[knee]).y);
					float mul = wrist == _human->j_lhand ? ARM_ON_THIGH_LIFT_MUL_L : ARM_ON_THIGH_LIFT_MUL_R;
					//mul *= 3.9; // 8_chair11
				//	mul *= 2.9;
					to = (_human->joints[knee] + Vector(0, dy * mul, 0) - _human->joints[elbow]).normalized();
					to.z = -abs(to.z);
					if (DEFORM_ARM) _human->apply_rotation_with_cor(wrist, to);
				}
			}
		}
		Vector tt;
		tt.x = 0;
		tt.y = 0;
		tt.z = 0;
		if (humanNum < 3)
		{
			tt.y = p2[(humanNum - 1) * 8 + +chairNum - 1][2];
			tt.z = p2[(humanNum - 1) * 8 + +chairNum - 1][3];
		}
	 	_human->translate(tt);

		// head stay straight
		if (!chair.is_lounger) {
			if (DEFORM_HEAD) _human->apply_rotation_with_cor(_human->j_head, { 0, 1, 0 });
		}

		// elasticity
		float possible_contact_area = 0, contact_area = 0;
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
		float seat_voxel_edge_len = 10;
		bool c1 = false;
		bool c = false;
		if (CHAIR_SEAT_DEFORM) {
			float total_area = 0;
			IntVec ts;
			for (int j = 0; j < _human->t.size(); ++j) {
				if (_human->triangle_center(j).y < _human->joints[_human->j_lknee].y
					&& (_part_labels[j] == DetailPartType::thigh0 || _part_labels[j] == DetailPartType::thigh1
					|| _part_labels[j] == DetailPartType::downbody)) {
					total_area += _human->triangle_area(j);
					ts.push_back(j);
				}
			}
			total_area *= SEAT_POSSIBLE_TOUCH_AREA_MUL;
			possible_contact_area += total_area;
			std::cout << total_area << std::endl;

			Mass_Spring_System mass_spring(&chair.seat, seat_voxel_edge_len, 1.1);
			IntVec upmost_surface = mass_spring.upmost_surface();
			VtxVec ply;
			for (auto i : upmost_surface) ply.push_back(mass_spring.get_centers()[i]);
			IntVec seat_contact_voxels;
			float area = 0;
			for (int i : upmost_surface) {
				Vertex pt = mass_spring.get_centers()[i];
				if (1 || pt.z > seat_min.z + seat_parm.z * (float)0.01) {
					for (int j : ts) {
						if (vertex_in_triangle(pt, *_human, j)) {
							//if ((pt - _human->triangle_center(j)).length() < 10) {
							area += seat_voxel_edge_len * seat_voxel_edge_len;
							seat_contact_voxels.push_back(i); break;
							//}
						}
					}
				}
			}
			contact_area += area;
			std::cout << area << std::endl;
			ply.clear();
			for (auto i : seat_contact_voxels) ply.push_back(mass_spring.get_centers()[i]);

			// deform
			float weight = SEAT_WEIGHT;
			float mass = 300;
			float h = 0.001; // time interval (in seconds)
			float stiffness = 300.f;//300
			float damper = 0.1;
			mass /= (float)mass_spring.get_centers().size();
			//mass /= h;
			Vector force = { 0, -weight*(float)9.8, 0 };
			force /= (float)seat_contact_voxels.size();
		}
		else {
			float total_area = 0;
			for (int j = 0; j < _human->t.size(); ++j) {
				if (_human->triangle_center(j).y < _human->joints[_human->j_lknee].y
					&& (_part_labels[j] == DetailPartType::thigh0 || _part_labels[j] == DetailPartType::thigh1
					|| _part_labels[j] == DetailPartType::downbody)) {
					total_area += _human->triangle_area(j);
				}
			}
			total_area *= SEAT_POSSIBLE_TOUCH_AREA_MUL;
			possible_contact_area += total_area;
			std::cout << "chair seat not deform  total:"<< total_area << std::endl;
		}
		// back
		float back_voxel_edge_len = 15;
		chair_back_vec.x = 0;
		if (chair.has_back && CHAIR_BACK_DEFORM && chair_back_vec.is_nonzero())
		{
			// human back
			float total_area = 0;
			IntVec ts;
			for (int i = 0; i < _human->t.size(); ++i) {
				char part = _part_labels[i];
				if (part == DetailPartType::upbody || part == DetailPartType::midbody || part == DetailPartType::downbody
					|| part == DetailPartType::neck || part == DetailPartType::head
					//|| part == DetailPartType::arm0 || part == DetailPartType::arm1
					) {
					ts.push_back(i);
				}
				if (part == DetailPartType::upbody || part == DetailPartType::midbody || part == DetailPartType::downbody) {
					total_area += _human->triangle_area(i);
				}
			}
			total_area *= BACK_POSSIBLE_TOUCH_AREA_MUL;
			std::cout << total_area << std::endl;
			possible_contact_area += total_area;
			// contacts
			Quaternion rot(chair_back_vec.normalized(), { 0, 0, 1 });
			_human->rotate(rot);
			chair.back.rotate(rot);
			Mass_Spring_System mass_spring(&chair.back, back_voxel_edge_len, 1.1);
			IntVec upmost_surface = mass_spring.upmost_surface();
			VtxVec ply;
			for (auto i : upmost_surface) ply.push_back(mass_spring.get_centers()[i]);
			IntVec back_contact_voxels;
			float area = 0;
			for (int i : upmost_surface) {
				Vertex pt = mass_spring.get_centers()[i];
				if (1 || pt.z > seat_min.z + seat_parm.z * (float)0.02) {
					for (int j : ts) {
						if (vertex_in_triangle(pt, *_human, j)) {
							if ((pt - _human->triangle_center(j)).length() < 30) {
								area += back_voxel_edge_len * back_voxel_edge_len;
								back_contact_voxels.push_back(i); break;
							}
						}
					}
				}
			}
			if (COUNT_BACK_CONTACT_AREA) contact_area += area;
			std::cout << area << std::endl;
			ply.clear();
			for (auto i : back_contact_voxels) ply.push_back(mass_spring.get_centers()[i]);

			// deform
			float weight = SEAT_WEIGHT * 0.6f;
			float mass = 1;
			float h = 0.01; // time interval (in seconds)
			float stiffness = 500; //0.01
			float damper = 0.1;
			mass /= (float)mass_spring.get_centers().size();
			mass /= h;
			Vector force = { 0, -weight*(float)9.8, 0 };
			force /= (float)back_contact_voxels.size();
			_human->rotate(rot.inverse());
			chair.back.rotate(rot.inverse());
		}
		else {
			float total_area = 0;
			for (int i = 0; i < _human->t.size(); ++i) {
				char part = _part_labels[i];
				if (part == DetailPartType::upbody || part == DetailPartType::midbody || part == DetailPartType::downbody) {
					total_area += _human->triangle_area(i);
				}
			}
			total_area *= BACK_POSSIBLE_TOUCH_AREA_MUL;
			std::cout <<"chair back not deform total:"<< total_area << std::endl;
			possible_contact_area += total_area;
		}


		// write
		_human->write_to_ply("..\\..\\..\\..\\..\\..\\OutPutModel\\OutHuamnModel_human" + to_string(humanNum)+"_chair" +to_string(chairNum)+ ".ply");
		chair.write_to_ply("..\\..\\..\\..\\..\\..\\OutPutModel/OutChairModel_human" + to_string(humanNum) + "_chair" + to_string(chairNum) + ".ply");
	
	}
	void Reshape::init_chair(const string& chair_path, StrVec seat_name, StrVec back_name, StrVec arm_names, StrVec leg_names, StrVec foot_names, Vector scale_factor,int flag) {

		Chair& chair = _chair;

		chair.is_lounger = false;

		// seat
		chair.has_seat = false;
		if (seat_name.size() > 0) {
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
		int j;
		if (flag)
		{
			chair.leg_num = 4;
			j = 1;
		}
		else
		{
			chair.leg_num = 1;
			j = 0;
		}
		//chair.leg_num = leg_names.size();
		chair.legs.resize(chair.leg_num);
		for (int i = 0; i < chair.leg_num; ++i) {
			chair.legs[i].read_from_file(chair_path + leg_names[j++]);
			chair.p.push_back(&chair.legs[i]);
		}

		// foot
		chair.foot_num = foot_names.size();
		chair.feet.resize(chair.foot_num);
		chair.is_wheel_chair = false;
		for (int i = 0; i < chair.foot_num; ++i) {
			chair.is_wheel_chair = true;
			chair.feet[i].read_from_file(chair_path + foot_names[i]);
			chair.p.push_back(&chair.feet[i]);
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
		do_human_segmentation();

		// sitting ske
		Vertex sit_ske[18] = {
			Vector(0., 0.5, 0.02), Vector(0., 0.15, 0.01), Vector(0., 0., 0.), Vector(0., 0.7, 0.), // shoulders back hip head
			Vector(-0.1, 0., 0.), Vector(-0.2, -0.2, -0.5), Vector(-0.2, -0.5, -0.5), Vector(-0.15, -0.8, -0.1), // L hip knee ankle foot
			Vector(0.1, 0., 0.), Vector(0.2, -0.2, -0.5), Vector(0.2, -0.5, -0.5), Vector(0.15, -0.8, -0.1), // R hip knee ankle foot
			Vector(-0.2, 0.48, 0.), Vector(-0.3, 0.15, 0.), Vector(-0.3, 0., -0.5), // L shoulder elbow wrist
			Vector(0.2, 0.48, 0.), Vector(0.3, 0.15, 0.), Vector(0.3, 0., -0.5) // R shoulder elbow wrist
		};


		string ske_names[] = { "shoulders", "back", "hips", "head",
			"lthigh", "lknee", "lankle", "lfoot",
			"rthigh", "rknee", "rankle", "rfoot",
			"lshoulder", "lelbow", "lhand",
			"rshoulder", "relbow", "rhand" };
		for (int i = 0; i < _human->joints.size(); ++i) {
			auto v = _human->joints[i];
			std::cout << ske_names[i] << " " << v.x << " " << v.y << " " << v.z << std::endl;
		}

		Vertex ap[2] = { _human->v[_landmarks.armpit[0]], _human->v[_landmarks.armpit[1]] };
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
		string path = _human->mesh_file_name + ".cors";
		FILE* fp = fopen(path.c_str(), "rb");
		if (nullptr != fp) {
			int cors_size = 0;
			fread(&cors_size, sizeof(int), 1, fp);
			std::vector<COR_PART> buff(cors_size);
			fread(&buff[0], sizeof(COR_PART)*cors_size, 1, fp);
			fclose(fp);
			for (int i = 0; i < cors_size; ++i) {
				_human->cors[buff[i].i] = Vertex(buff[i].x, buff[i].y, buff[i].z);
			}
		}
		else {
			_human->cors = compute_optimized_center_of_rotation();
			fp = fopen(path.c_str(), "wb");
			int cors_size = _human->cors.size();
			fwrite(&cors_size, sizeof(int), 1, fp);
			std::vector<COR_PART> buff;
			for (int i = 0; i < (int)_human->v.size(); ++i) {
				if (_human->cors.count(i)) {
					COR_PART part;
					part.i = i;
					part.x = _human->cors[i].x;
					part.y = _human->cors[i].y;
					part.z = _human->cors[i].z;
					buff.push_back(part);
				}
			}
			fwrite(&buff[0], sizeof(COR_PART)*cors_size, 1, fp);
			fclose(fp);
		}

		for (int i = 0; i < sizeof(order) / sizeof(int); ++i) {
			int joint_id = order[i];
			Vector to = sit_ske[joint_id] - sit_ske[_human->joint_parent[joint_id]];
			_human->apply_rotation_with_cor(joint_id, to, true, false);
			//_human->apply_rotation(joint_id, Quaternion(from, to));
		}
		//_human->apply_deformation();
		//_human->apply_deformation_with_cor();
	}
	// chair
	void Reshape::Chair::write_to_ply(const string& path) {
		std::vector<Vertex> vs;
		std::vector<Triangle> ts;
		std::vector<RGBA> vc, tc;
		for (int i = 0; i < this->p.size(); ++i) {
			int size = vs.size();
			for (auto v : this->p[i]->v) vs.push_back(v);
			for (auto t : this->p[i]->t) ts.push_back(Triangle(size + t.v[0], size + t.v[1], size + t.v[2]));
			for (auto v : this->p[i]->vcolor) vc.push_back(v);
			for (auto t : this->p[i]->tcolor) tc.push_back(t);
		}
		Mesh::write_to_ply(vs, ts, vc, path);
	}
	// skinning
	void Reshape::subdivide_mesh(float epsilon, VtxVec& vertices_out, std::vector<Triangle>& triangles_out,
		IntVec& triangle_neighbors_out, std::vector<double>& bone_weights_out) {

		const int joint_num = Mesh::JOINT_NAME::j_number;
		auto vertices = _human->v;
		auto triangles = _human->t;
		auto tts = _human->tt_data;
		auto skinning_weights = _human->weights;

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
		int real_vn = (int)_human->v.size();

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
		if (vs.size() > 0) ret /= vs.size();
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