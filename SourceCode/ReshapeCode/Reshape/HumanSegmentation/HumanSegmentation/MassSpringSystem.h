// MassSpringSystem.h

#pragma once

#ifndef MASS_SPRING_SYSTEM_H
#define MASS_SPRING_SYSTEM_H

#include "Voxel.h"
#include <vector>
#include <queue>

namespace ManSeg {

	// -------------------- Struct Mass_Spring_System --------------------

	class Mass_Spring_System : public VoxelHandler {
		typedef std::vector<Vertex> VtxVec;
		typedef std::vector<int> IntVec;
		struct Spring { int opposite; float len; };
	public:
		Mass_Spring_System(Mesh* in_mesh, float voxel_size, float multiplier = 1)
			: VoxelHandler(in_mesh, voxel_size, multiplier) {
			system_init();
		}
	private:
		bool is_volume_object(int index) {
			return is_boundary(index) || is_inside(index);
		}
		void find_links(int idx) {

			Voxel vx = voxels[_index_map[idx]];
			int x = vx.x, y = vx.y, z = vx.z;
			Vertex cen = _centers[idx];
			int coord[3] = { x, y, z };
			int adder = 1;
			for (int i = 0; i < 6; ++i) {
				if (i == 3) ++adder;
				coord[i % 3] += adder;
				int n = voxel_index(coord[0], coord[1], coord[2]);
				if (n >= 0 && is_volume_object(n)) {
					int ni = voxels[n].label;
					if (ni >= 0) {
						float len = (cen - _centers[ni]).length();
						_links[idx*MAX_LINK + _link_num[idx]] = { ni, len };
						_links[ni*MAX_LINK + _link_num[ni]] = { idx, len };
						++_link_num[idx];
						++_link_num[ni];
					}
					else std::cout << "label < 0 !" << std::endl;
				}
				coord[i % 3] -= adder;
			}
			int index0[4] = { _index_map[idx], voxel_index(x + 1, y, z), voxel_index(x, y + 1, z), voxel_index(x, y, z + 1) };
			int index1[4] = { voxel_index(x + 1, y + 1, z + 1), voxel_index(x, y + 1, z + 1), voxel_index(x + 1, y, z + 1), voxel_index(x + 1, y + 1, z) };
			for (int i = 0; i < 4; ++i) {
				int curr = index0[i];
				int n = index1[i];
				if (curr >= 0 && n >= 0 && is_volume_object(curr) && is_volume_object(n)) {
					int ci = voxels[curr].label;
					int ni = voxels[n].label;
					if (ci >= 0 && ni >= 0) {
						float len = (_centers[ci] - _centers[ni]).length();
						_links[ci*MAX_LINK + _link_num[ci]] = { ni, len };
						_links[ni*MAX_LINK + _link_num[ni]] = { ci, len };
						++_link_num[ci];
						++_link_num[ni];
					}
					else std::cout << "label < 0 !" << std::endl;
				}
			}
		}
		void system_init() {
			
			for (int i = 0; i < (int)voxels.size(); i++) {
				voxels[i].label = -1;
				if (is_volume_object(i)) {
					voxels[i].label = (int)_index_map.size();
					_index_map.push_back(i);
				}
			}
			int size = _index_map.size();
			_centers.resize(size);
			for (int i = 0; i < size; ++i) {
				_centers[i] = voxel_center(_index_map[i]);
			}
			_links.resize(size * MAX_LINK);
			_link_num.resize(size, 0);
			for (int i = 0; i < size; ++i) {
				find_links(i);
			}
			float maxlen = DECIMAL_MIN;
			for (auto l : _links) {
				if (l.len > maxlen) maxlen = l.len;
			}
			//std::cout << "volume voxel number : " << size << std::endl;
			//std::cout << "total voxel number : " << voxels.size() << std::endl;
			//std::cout << "max spring length : " << maxlen << std::endl;
		}
	public:
		IntVec upmost_surface() {

			IntVec ret;
			_is_upmost.resize(_centers.size(), false);
			_upmost.resize(vxlen[2] * vxlen[0], -1);
			for (int iz = 0; iz < vxlen[2]; ++iz) {
				for (int ix = 0; ix < vxlen[0]; ++ix) {
					for (int iy = vxlen[1] - 1; iy >= 0; --iy) {
						int index = voxel_index(ix, iy, iz);
						if (index >= 0 && is_volume_object(index)) {
							if (voxels[index].label < 0) std::cout << "!!!" << std::endl;
							ret.push_back(voxels[index].label);
							_is_upmost[voxels[index].label] = true;
							_upmost[iz * vxlen[0] + ix] = index;
							break;
						}
					}
				}
			}
			return ret;
		}
		IntVec downmost_surface() {

			IntVec ret;
			_is_downmost.resize(_centers.size(), false);
			for (int iz = 0; iz < vxlen[2]; ++iz) {
				for (int ix = 0; ix < vxlen[0]; ++ix) {
					for (int iy = 0; iy < vxlen[1]; ++iy) {
						int index = voxel_index(ix, iy, iz);
						if (voxels[index].label >= 0) {
							ret.push_back(voxels[index].label);
							_is_downmost[voxels[index].label] = true;
							break;
						}
					}
				}
			}
			return ret;
		}
		void compute_acceleration(VtxVec& A, const VtxVec& P, const VtxVec& V) {

			for (int i = 0; i < _centers.size(); ++i) {
				// external force
				Vector force = _external_forces[i];
				// damper
				force -= V[i] * _damper;
				// tension
				Vector tension = { 0, 0, 0 };
				for (int j = 0; j < _link_num[i]; ++j) {
					const Spring& spring = _links[i * MAX_LINK + j];
					Vector vec = P[spring.opposite] - P[i];
					float len = vec.length();
					if (len >= DECIMAL_SMALLEST) {
						float val = (len - spring.len) / len;
						tension += vec * val;
					}
				}
				force += tension * _stiffness;
				A[i] = force / _mass;
			}
		}
		VtxVec deform(const IntVec& pts, Vector force, float mass, float stiffness, float damper, float time_interval, int loop_times, float& score) {
			
			upmost_surface(); downmost_surface();

			int size = _centers.size();
			_mass = mass; _stiffness = stiffness; _damper = damper;
			_external_forces.resize(size, { 0, 0, 0 });
			for (int i : pts) _external_forces[i] = force;

			VtxVec P = _centers, V(size, { 0, 0, 0 }), A(size, { 0, 0, 0 });
			VtxVec Pmid(size, { 0, 0, 0 }), Vmid(size, { 0, 0, 0 }), Amid(size, { 0, 0, 0 });
			float t = 0, h = time_interval, h_half = h / 2;

			Vertex  prev = P[pts[0]];
			for (int i = 0; i < loop_times; ++i) {

				if (i % (int)((float)1/h) == 0) {
					Vertex p = P[pts[0]];
					std::cout << i << ' ' << p.x << ' ' << p.y << ' ' << p.z << std::endl;
					if (i > 0 && (p - prev).length() < .001f) break;
					prev = p;
				}	

				// A at t
				compute_acceleration(A, P, V);
				// V at t+h/2
				for (int j = 0; j < size; ++j) {
					if(!_is_downmost[j])
						Vmid[j] = V[j] + A[j] * h_half;
				}
				// A at t+h/2
				for (int j = 0; j < size; ++j) {
					Pmid[j] = P[j] + V[j] * h_half;
				}
				compute_acceleration(Amid, Pmid, Vmid);
				// P at t+h
				for (int j = 0; j < size; ++j) {
					Vertex adder = Vmid[j] * h;
					P[j] += adder;
				}
				// V at t+h
				for (int j = 0; j < size; ++j) {
					if (!_is_downmost[j])
						V[j] = V[j] + Amid[j] * h;
				}
			}

			//// deform mesh
			//for (int i = 0; i < (int)mesh->v.size(); ++i) {
			//	Vertex& vtx = mesh->v[i];
			//	int vx = vertex_in_which_voxel(vtx);
			//	if (vx >= 0 && voxels[vx].label >= 0) {
			//		int idx = voxels[vx].label;
			//		vtx += (P[idx] - _centers[idx]);
			//	}
			//}

			// deform mesh
			VtxVec trans(mesh->t.size(), { 0, 0, 0 });
			for (int i = 0; i < (int)mesh->t.size(); ++i) {
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
				// translation
				int cnt = 0;
				for (int iz = mini[2]; iz <= maxi[2]; ++iz) {
					for (int iy = mini[1]; iy <= maxi[1]; ++iy) {
						for (int ix = mini[0]; ix <= maxi[0]; ++ix) {
							int vx = voxel_index(ix, iy, iz);
							if (vx >= 0 && voxels[vx].label >= 0) {
								++cnt; trans[i] += (P[voxels[vx].label] - _centers[voxels[vx].label]);
							}
						}
					}
				}
				if (cnt <= 0) std::cout << "error when deform mesh using mass-spring system" << std::endl;
				trans[i] /= (float)cnt;
			}
			std::vector<float> disp(mesh->v.size(), 0);
			float maxt = DECIMAL_MIN, mint = DECIMAL_MAX, avgt = 0;
			int cnt = 0;
			for (int i = 0; i < (int)mesh->v.size(); ++i) {
				if (mesh->vtn[i] > 0) {
					Vertex move = { 0, 0, 0 };
					for (int j = 0; j < mesh->vtn[i]; ++j) {
						move += trans[mesh->vt(i, j)];
					}
					move /= (float)mesh->vtn[i];
					disp[i] = move.length();
					if (disp[i] > maxt) maxt = disp[i];
					if (disp[i] < mint) mint = disp[i];
					avgt += disp[i];
					++cnt;
					mesh->v[i] += move;
				}
			}
			avgt /= (float)cnt;
			for (int i = 0; i < (int)mesh->v.size(); ++i) {
				if (mesh->vtn[i] > 0) {
					float h = disp[i];
					if (h < avgt) h = 0;
					else {
						h = (std::log(disp[i]) - std::log(avgt)) / (std::log(maxt) - std::log(avgt));
						//h *= 3.0f;
						//h = (1 / (std::pow(2.0f, 3.0f) - 1.0f)) * (std::pow(2.0f, h) - 1.0f);
					}
					if (h < 0.0) h = 0.0;
					if (h > 1.0) h = 1.0;
					// HSV
					int H = 240.0 - (240.0*(h));
					mesh->color_vertex(i, RGBA::HSV2RGB(H, 1.0f, 1.0f));
				}
			}

			// score
			std::vector<float> gradients;
			for (int iz = 0; iz < vxlen[2]; ++iz) {
				for (int ix = 0; ix < vxlen[0]; ++ix) {
					int vx = _upmost[iz * vxlen[0] + ix];
					if (vx >= 0) {
						Vector move = P[voxels[vx].label] - _centers[voxels[vx].label];
						float t0 = move.length();
						if (t0 > 0.000001f) {
							float max_len = DECIMAL_MIN;
							for (int i = 0; i < 4; ++i) {
								int jx = (i % 2) * 2 - 1 + ix;
								int jz = (i / 2) * 2 - 1 + iz;
								if (jz >= 0 && jx >= 0 && jz < vxlen[2] && jx < vxlen[0]) {
									int vxn = _upmost[jz * vxlen[0] + jx];
									if (vxn >= 0) {
										Vector moven = P[voxels[vxn].label] - _centers[voxels[vxn].label];
										float t1 = moven.length();
										float dt = t1 - t0;
										Vector dv = _centers[voxels[vxn].label] - _centers[voxels[vx].label];
										Vector grad = { 0, 0, 0 };
										if (dv.x >= DECIMAL_SMALLEST || dv.x <= -DECIMAL_SMALLEST) grad.x = dt / dv.x;
										if (dv.y >= DECIMAL_SMALLEST || dv.y <= -DECIMAL_SMALLEST) grad.y = dt / dv.y;
										if (dv.z >= DECIMAL_SMALLEST || dv.z <= -DECIMAL_SMALLEST) grad.z = dt / dv.z;
										if (grad.length() > max_len) max_len = grad.length();
									}
								}
							}
							if (max_len != DECIMAL_MIN) gradients.push_back(max_len);
						}
					}
				}
			}
			if (gradients.size() > 0) {
				float avg_grad = 0;
				for (float grad : gradients) {
					avg_grad += grad;
				}
				avg_grad /= gradients.size();
				float cov = 0, max_diff = DECIMAL_MIN;
				for (float grad : gradients) {
					float diff = grad - avg_grad;
					diff *= diff;
					cov += diff;
					if (diff > max_diff) max_diff = diff;
				}
				cov /= gradients.size();
				cov = std::sqrt(cov);
				max_diff = std::sqrt(max_diff);
				score = 100 * (max_diff - cov) / max_diff;
			}

			// score
			float score_disp = 0, max_disp = DECIMAL_MIN;
			int count_disp = 0;
			for (int iz = 0; iz < vxlen[2]; ++iz) {
				for (int ix = 0; ix < vxlen[0]; ++ix) {
					int vx = _upmost[iz * vxlen[0] + ix];
					if (vx >= 0) {
						float d = (P[voxels[vx].label] - _centers[voxels[vx].label]).length();
						score_disp += d;
						++count_disp;
						if (d > max_disp)
							max_disp = d;
					}
				}
			}
			score = score_disp;
			std::cout << "disp score : " << score_disp / max_disp / (float)count_disp << "\ndisp sum : " << score_disp << std::endl;


			// score
			std::vector<int> depth(vxlen[0] * vxlen[2], -2);
			std::queue<int> bfs_queue;
			for (int i = 0; i < vxlen[0] * vxlen[2]; ++i) {
				int vx = _upmost[i];
				if (vx >= 0) {
					if (!_external_forces[voxels[vx].label].is_nonzero()
						/*&& (P[voxels[vx].label] - _centers[voxels[vx].label]).length() <= DECIMAL_SMALLEST*/) {
						for (int j = 0; j < 4; ++j) {
							int jx = (j % 2) * 2 - 1 + int(i % vxlen[0]);
							int jz = (j / 2) * 2 - 1 + int(i / vxlen[0]);
							if (jz >= 0 && jx >= 0 && jz < vxlen[2] && jx < vxlen[0]) {
								int vxn = _upmost[jz * vxlen[0] + jx];
								if (vxn >= 0 && (_external_forces[voxels[vxn].label].is_nonzero()
									/*|| (P[voxels[vxn].label] - _centers[voxels[vxn].label]).length() > DECIMAL_SMALLEST*/)) {
									bfs_queue.push(i);
									depth[i] = 0;
									break;
								}
							}
						}
					}
					else depth[i] = -1;
				}
			}
			while (!bfs_queue.empty()) {
				int curr = bfs_queue.front();
				bfs_queue.pop();
				for (int j = 0; j < 4; ++j) {
					int jx = (j % 2) * 2 - 1 + int(curr % vxlen[0]);
					int jz = (j / 2) * 2 - 1 + int(curr / vxlen[0]);
					if (jz >= 0 && jx >= 0 && jz < vxlen[2] && jx < vxlen[0]) {
						int index = jz * vxlen[0] + jx;
						if (depth[index] == -1) {
							depth[index] = depth[curr] + 1;
							bfs_queue.push(index);
						}
					}
				}
			}
			float max_grad = DECIMAL_MIN, sum_grad = 0;
			float max_grad_diff = DECIMAL_MIN, sum_grad_diff = 0;
			int count_grad = 0, count_grad_diff = 0;
			for (int i = 0; i < vxlen[0] * vxlen[2]; ++i) {
				if (depth[i] >= 0) {
					int vx = _upmost[i];
					float d = (P[voxels[vx].label] - _centers[voxels[vx].label]).length() + 0.0f * depth[i];
					float maxg = DECIMAL_MIN, maxgdiff = DECIMAL_MIN;
					for (int j = 0; j < 4; ++j) {
						int jx = (j % 2) * 2 - 1 + int(i % vxlen[0]);
						int jz = (j / 2) * 2 - 1 + int(i / vxlen[0]);
						int idxn = jz * vxlen[0] + jx;
						if (jz >= 0 && jx >= 0 && jz < vxlen[2] && jx < vxlen[0] && depth[idxn] > depth[i]) {
							int vxn = _upmost[idxn];
							float dn = (P[voxels[vxn].label] - _centers[voxels[vxn].label]).length();
							float grad = dn + 0.0f * depth[idxn] - d;
							if (grad > maxg)
								maxg = grad;
							for (int k = 0; k < 4; ++k) {
								int kx = (k % 2) * 2 - 1 + jx;
								int kz = (k / 2) * 2 - 1 + jz;
								int idxnn = kz * vxlen[0] + kx;
								if (kz >= 0 && kx >= 0 && kz < vxlen[2] && kx < vxlen[0] && depth[idxnn] > depth[idxn]) {
									int vxnn = _upmost[idxnn];
									float dnn = (P[voxels[vxnn].label] - _centers[voxels[vxnn].label]).length();
									float grad_diff = dnn - dn - grad;
									if (grad_diff > maxgdiff)
										maxgdiff = grad_diff;
								}
							}
						}
					}
					if (maxg != DECIMAL_MIN) {
						sum_grad += maxg;
						++count_grad;
					}
					if (maxg > max_grad)
						max_grad = maxg;
					if (maxgdiff != DECIMAL_MIN) {
						sum_grad_diff += maxgdiff;
						++count_grad_diff;
					}
					if (maxgdiff > max_grad_diff)
						max_grad_diff = maxgdiff;
				}
			}

			float grad_score = sum_grad / max_grad / (float)count_grad;
			if (max_grad == DECIMAL_MIN)
				std::cout << "grad error" << std::endl;
			else if (max_grad < DECIMAL_SMALLEST && max_grad > -DECIMAL_SMALLEST)
				std::cout << "max grad is 0" << std::endl;
			else
				std::cout << "grad score : " << grad_score /*<< "\nsum : " << sum_grad << " grad : " << max_grad << " count : " << count_grad*/ << std::endl;

			return P;
		}
	public:
		VtxVec& get_centers() { return _centers; }
	private:
		const static int MAX_LINK = 20;
		std::vector<Spring> _links; // springs
		IntVec _link_num; // how many springs are linked to each mass
		IntVec _index_map;
		float _mass;
		float _stiffness;
		float _damper;
		VtxVec _external_forces;
		std::vector<bool> _is_upmost;
		std::vector<int> _upmost;
		std::vector<bool> _is_downmost;
		VtxVec _centers; // masses (constructed form voxel centers)
	};
}

#endif