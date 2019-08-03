// MassSpringSystem.h

#pragma once

#ifndef DEFORM_SOLVER_H
#define DEFORM_SOLVER_H

#include "Mesh.h"
#include <random>
#include <time.h>

namespace ManSeg {

	class DeformSolver {
	public:
		DeformSolver(Mesh* m, Mesh* c, std::vector<int> r, std::vector<int> er) : _human(m), _chair(c), _man_range(r), _man_edge_range(er) {}
	private:
		Mesh* _human;
		Mesh* _chair;
		std::vector<int> _man_range;
		std::vector<int> _man_edge_range;
		float _max_chair_height;
		std::vector<Mesh::JOINT_NAME> _bones;
	private:
		std::vector<Vector> deform_human(std::vector<float>& angles) {
			std::vector<Vector> dirs(_bones.size());
			for (int i = 0; i < (int)_bones.size(); ++i) {
				dirs[i] = Vector::create_vector_from_angles(angles[i * 2], angles[i * 2 + 1]);
				if (_bones[i] == Mesh::j_back) {
					dirs[i] *= -1;
					Vector from = _human->joints[_human->j_hips] - _human->joints[_human->j_back];
					Vector to = dirs[i];
					from.x = 0;
					to.x = 0;
					_human->apply_rotation_with_cor(_human->j_back, Quaternion(from, to), false);
				}
				_human->apply_rotation_with_cor(_bones[i], dirs[i], false);
			}
			return dirs;
		}
		float compute_energy(std::vector<float>& angles, std::vector<Vector>& dirs, bool detect_collision, bool armrest_energy) {
			
			// store current human
			auto vs = _human->v;
			auto js = _human->joints;
			dirs = deform_human(angles);
			
			// energy
			float energy = 0;
			int cnt = 0;
			if (armrest_energy) {
				for (auto pi : _man_range) {
					float height = _human->v[pi].y - _max_chair_height;
					if (height < 0) {
						energy += height*height;
					}
				}
			}
			for (auto pi : _man_range) {
				if (cnt++ % 10 != 0) continue;
				float mind = DECIMAL_MAX;
				for (int i = 0; i < (int)_chair->v.size(); ++i) {
					//float d = abs(_chair->vertex_triangle_distance(i, _human->v[pi]));
					float d = (_chair->v[i] - _human->v[pi]).square_length();
					if (d < mind)
						mind = d;
				}
				if (mind < DECIMAL_MAX)
					energy += mind;
			}
			if (detect_collision) {
				float t = 0;
				for (auto ei : _man_edge_range) {
					//if (cnt++ % 3 != 0) continue;
					Vertex v0 = _human->v[_human->e[ei].v[0]];
					Vertex v1 = _human->v[_human->e[ei].v[1]];
					for (int ti = 0; ti < (int)_chair->t.size(); ++ti) {
						float d0 = abs(_chair->vertex_triangle_distance(ti, v0));
						float d1 = abs(_chair->vertex_triangle_distance(ti, v1));
						if (d0*d1 < 0) {
							if (_chair->intersect_triangle(v0, v1 - v0, ti, t)) {
								// undeform human
								_human->v = vs;
								_human->joints = js;
								return DECIMAL_MAX;
								//energy += 1000000;
								//break;
							}
						}
					}
				}
			}
			// undeform human
			_human->v = vs;
			_human->joints = js;
			return energy;
		}
		bool collision_happen(std::vector<float>& angles) {

			// store current human
			auto vs = _human->v;
			auto js = _human->joints;
			deform_human(angles);

			// energy
			float t = 0;
			for (auto ei : _man_edge_range) {
				//if (cnt++ % 3 != 0) continue;
				Vertex v0 = _human->v[_human->e[ei].v[0]];
				Vertex v1 = _human->v[_human->e[ei].v[1]];
				for (int ti = 0; ti < (int)_chair->t.size(); ++ti) {
					float d0 = _chair->vertex_triangle_distance(ti, v0);
					float d1 = _chair->vertex_triangle_distance(ti, v1);
					if (d0*d1 < 0) {
						if (_chair->intersect_triangle(v0, v1 - v0, ti, t)) {
							_human->v = vs;
							_human->joints = js;
							return true;
						}
					}
				}
			}
			// undeform human
			_human->v = vs;
			_human->joints = js;
			return false;
		}
	public:
		void solve(std::string man_part, bool detect_collision, bool armrest_energy) {

			int SEARCH_SEED_TIMES = 20;
			int MAX_LOOP_HILL_CLIMB = 0;
			int SEARCH_TIMES_HILL_CLIMB = 100;
			int SEARCH_RETRY_TIMES_HILL_CLIMB = 3;
			float SEARCH_RANGE_HILL_CLIMB = 5.f;

			float max_chair_height = DECIMAL_MIN;
			for (auto p : _chair->v) {
				if (p.y > max_chair_height)
					max_chair_height = p.y;
			}
			_max_chair_height = max_chair_height;

			srand(time(0));

			// x:left
			int mode = 0;
			if (man_part.compare("larm") == 0) mode = 0;
			if (man_part.compare("rarm") == 0) mode = 1;
			if (man_part.compare("back") == 0) mode = 2;

			if (mode != 2) {
				SEARCH_SEED_TIMES = 1000;
				MAX_LOOP_HILL_CLIMB = 100;
			}

			const int BONE_NUM[3] = { 2, 2, 1 };
			const float ANGLE_MIN[3][4] = { { 180, -80, -135, -45 }, { -90, -80, -135, -45 }, { 90, 60, 0, 0 } };
			const float ANGLE_MAX[3][4] = { { 270, 15, -45, 45 }, { 0, 15, -45, 45 }, { 90.000001f, 80, 0, 0 } };
			Mesh::JOINT_NAME BONE[3][2] = { { Mesh::j_lelbow, Mesh::j_lhand }, { Mesh::j_relbow, Mesh::j_rhand }, { Mesh::j_back } };

			for (int i = 0; i < BONE_NUM[mode]; ++i) {
				_bones.push_back(BONE[mode][i]);
			}

			std::vector<Vertex> vs = _human->v;
			std::vector<Vertex> js = _human->joints;

			std::vector<Vector> dirs(BONE_NUM[mode]);
			std::vector<Vector> answer(BONE_NUM[mode]);

			std::vector<float> angles(BONE_NUM[mode] * 2);
			std::vector<float> seed_angles(BONE_NUM[mode] * 2);
			float min_energy = DECIMAL_MAX;
			for (int idx = 0; idx < SEARCH_SEED_TIMES; ++idx) {

				if (idx % 100 == 0)
					std::cout << idx << " " << min_energy << std::endl;

				// create angle
				for (int i = 0; i < (int)angles.size(); ++i) {
					float angle = (float)rand() / RAND_MAX;
					angle = (ANGLE_MAX[mode][i] - ANGLE_MIN[mode][i])*angle + ANGLE_MIN[mode][i];
					if (mode == 2 && i == 1) {
						angle = (ANGLE_MAX[mode][i] - ANGLE_MIN[mode][i])*(float)idx / (float)SEARCH_SEED_TIMES + ANGLE_MIN[mode][i];
					}
					//angle = 360.f*angle;
					angles[i] = angle;
				}
				// energy
				float energy = compute_energy(angles, dirs, false, armrest_energy);
				if (energy < min_energy && energy > 0.1f) {
					if (detect_collision && collision_happen(angles)) {
						std::cout << "collision !" << std::endl;
						if (mode != 2)
							--idx;
					}
					else {
						min_energy = energy;
						seed_angles = angles;
						answer = dirs;
					}
				}
			}

			// all
			float range_start[4] = { 0 };
			float range[4] = { 0 };
			float init_range[4] = { 0 };
			for (int i = 0; i < seed_angles.size(); ++i) {
				//range[i] = (ANGLE_MAX[mode][i] - ANGLE_MIN[mode][i]) / 5.f;
				range[i] = SEARCH_RANGE_HILL_CLIMB;
				range_start[i] = -range[i] / 2.f;
				init_range[i] = range[i];
			}
			int count = 0;
			for (int loop_idx = 0; loop_idx < MAX_LOOP_HILL_CLIMB; ++loop_idx) {
				std::vector<float> prev_seed_angles = seed_angles;
				bool flag = false;
				for (int idx = 0; idx < SEARCH_TIMES_HILL_CLIMB; ++idx) {
					// create angle
					for (int i = 0; i < (int)angles.size(); ++i) {
						while (1) {
							float d = (float)rand() / RAND_MAX;
							d = range_start[i] + range[i] * d;
							angles[i] = prev_seed_angles[i] + d;
							if (angles[i] >= ANGLE_MIN[mode][i] && angles[i] <= ANGLE_MAX[mode][i])
								break;
						}
					}
					// energy
					float energy = compute_energy(angles, dirs, false, armrest_energy);
					if (energy < min_energy && energy > 0.1f) {
						if (detect_collision && collision_happen(angles)) {
							//std::cout << "collision !" << std::endl;
							//--idx;
						}
						else {
							flag = true;
							seed_angles = angles;
							min_energy = energy;
							answer = dirs;
						}
					}
				}
				// break
				if (!flag) {
					++count;
					if (count > SEARCH_RETRY_TIMES_HILL_CLIMB)
						break;
				}
				else {
					count = 0;
				}
				std::cout << min_energy << std::endl;
			}

			//// in order
			//float range_start[4] = { 0 };
			//float range[4] = { 0 };
			//float init_range[4] = { 0 };
			//for (int i = 0; i < (int)seed_angles.size(); ++i) {
			//	//range[i] = (ANGLE_MAX[mode][i] - ANGLE_MIN[mode][i]) / 5.f;
			//	range[i] = 5.f;
			//	range_start[i] = -range[i] / 2.f;
			//	//init_range[i] = (ANGLE_MAX[mode][i] - ANGLE_MIN[mode][i]) / 5.f;
			//	init_range[i] = 5.f;
			//}
			//int count = 0;
			//while (1) {
			//	bool flag = false;
			//	for (int it = 0; it < (int)seed_angles.size(); ++it) {
			//		std::vector<float> prev_seed_angles = seed_angles;
			//		for (int idx = 0; idx < 30; ++idx) {
			//			// create angle
			//			while (1) {
			//				float d = (float)rand() / RAND_MAX;
			//				d = range_start[it] + range[it] * d;
			//				angles[it] = prev_seed_angles[it] + d;
			//				if (angles[it] >= ANGLE_MIN[mode][it] && angles[it] <= ANGLE_MAX[mode][it])
			//					break;
			//			}
			//			// energy
			//			float energy = compute_energy(angles, BONE_NUM[mode], BONE[mode], quats, false);
			//			if (energy < min_energy && energy > 0.1f) {
			//				if (detect_collision && collision_happen(angles, BONE_NUM[mode], BONE[mode])) {
			//					//std::cout << "collision !" << std::endl;
			//					//--idx;
			//				}
			//				else {
			//					flag = true;
			//					seed_angles = angles;
			//					min_energy = energy;
			//					answer = quats;
			//				}
			//			}
			//		}
			//	}
			//	// break
			//	if (!flag) {
			//		++count;
			//		if (count > 5)
			//			break;
			//	}
			//	else {
			//		count = 0;
			//	}
			//	std::cout << min_energy << std::endl;
			//}

			// deform
			for (int i = 0; i < BONE_NUM[mode]; ++i) {
				_human->apply_rotation_with_cor(BONE[mode][i], answer[i]);
				std::cout << "angle : " << seed_angles[i * 2] << std::endl;
				std::cout << "angle : " << seed_angles[i * 2 + 1] << std::endl;
			}
		}

	};
}

#endif