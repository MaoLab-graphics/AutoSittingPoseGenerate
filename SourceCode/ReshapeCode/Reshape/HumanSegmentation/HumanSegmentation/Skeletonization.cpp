// Skeletonization.cpp

#include "Skeletonization.h"
#include "Mesh.h"
#include "Voxel.h"
#include <list>

namespace ManSeg {
	// constructor
	SkeletonGenerator::SkeletonGenerator(Mesh* in_mesh, VoxelHandler* vxh, bool cut) : mesh(in_mesh), voxelh(vxh) {
		print_time_message("start ske");

		//BS_voxel_coding();
		//BS_local_maxima();
		//SS_voxel_coding();
		//clusters_init();
		//classify_clusters();
		//create_skeleton();

		//generate_force_field();
		thining();
		print_time_message("thining end");
		//connect_skeleton();
		if (cut) cut_skeleton();
		//split_mesh_by_border_voxel();
		//split_mesh_by_SS_code();
		print_time_message("refine ske end");
	}
	SkeletonGenerator::SkeletonGenerator(Mesh* in_mesh, VoxelHandler* vxh) : mesh(in_mesh), voxelh(vxh) {
		print_time_message("start ske");

		//BS_voxel_coding();
		//BS_local_maxima();
		//SS_voxel_coding();
		//clusters_init();
		//classify_clusters();
		//create_skeleton();

		//generate_force_field();
		thining();
		print_time_message("thining end");
		//connect_skeleton();
		cut_skeleton();
		//split_mesh_by_border_voxel();
		//split_mesh_by_SS_code();
		print_time_message("refine ske end");
	}
	// thining
	void SkeletonGenerator::construct_neighbor_array(const Voxel& vx, bool ns[][3][3])
	{
		for (int num = 0; num < 27; num++)
		{
			int temp[3];
			temp[0] = (num % 3) - 1;
			temp[1] = ((int)((float)num / (float)3) % 3) - 1;
			temp[2] = (int)((float)num / (float)9) - 1;
			bool flag = true;
			temp[0] += vx.x; temp[1] += vx.y; temp[2] += vx.z;
			if (temp[0] < 0 || temp[0] >= voxelh->vxlen[0] ||
				temp[1] < 0 || temp[1] >= voxelh->vxlen[1] ||
				temp[2] < 0 || temp[2] >= voxelh->vxlen[2])
				flag = false;
			int index = temp[2] * voxelh->vxlen[1] * voxelh->vxlen[0] + temp[1] * voxelh->vxlen[0] + temp[0];
			temp[0] -= vx.x; temp[1] -= vx.y; temp[2] -= vx.z;
			if (flag)
				flag = voxelh->is_object(index);
			ns[temp[0] + 1][temp[1] + 1][temp[2] + 1] = flag;
		}
	}
	void SkeletonGenerator::transform_neighbor_array(bool ns[][3][3], DirectionType dir)
	{
		bool ons[3][3][3] = { 0 };
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				for (int k = 0; k < 3; k++)
					ons[i][j][k] = ns[i][j][k];
		if (dir == DIR_D)  // reflection
		{
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					bool temp = ns[i][2][j];
					ns[i][2][j] = ns[i][0][j];
					ns[i][0][j] = temp;
				}
			}
		}
		else if (dir == DIR_W)
		{
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					for (int k = 0; k < 3; k++)
						ns[2 - j][i][k] = ons[i][j][k];
		}
		else if (dir == DIR_E)
		{
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					for (int k = 0; k < 3; k++)
						ns[j][2 - i][k] = ons[i][j][k];
		}
		else if (dir == DIR_N)
		{
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					for (int k = 0; k < 3; k++)
						ns[i][k][2 - j] = ons[i][j][k];
		}
		else if (dir == DIR_S)
		{
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					for (int k = 0; k < 3; k++)
						ns[i][2 - k][j] = ons[i][j][k];
		}
	}
	bool SkeletonGenerator::match_mask(const bool ns[][3][3])
	{
		int count = 0;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (!ns[i][2][j])
					count++;
			}
		}
		if (count == 9)  // M1 M5 M6
		{
			if (ns[1][0][1])  // M1
			{
				bool flag = false;
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 2; j++)
					{
						for (int k = 0; k < 3; k++)
						{
							if ((i != 1 || k != 1) && ns[i][j][k])
							{
								flag = true;
								break;
							}
						}
					}
				}
				if (flag)
					return true;
			}
			else
			{
				if (ns[1][0][2])  // M6
				{
					if (ns[0][0][1])
					{
						if (!ns[2][0][0] && !ns[2][0][1] && !ns[1][0][0] &&
							!ns[2][1][0] && !ns[2][1][1] && !ns[1][1][0])
							return true;
					}
					else if (ns[2][0][1])
					{
						if (!ns[0][0][0] && !ns[0][0][1] && !ns[1][0][0] &&
							!ns[0][1][0] && !ns[0][1][1] && !ns[1][1][0])
							return true;
					}
				}
				else if (ns[1][0][0])
				{
					if (ns[0][0][1])
					{
						if (!ns[2][0][2] && !ns[2][0][1] && !ns[1][0][2] &&
							!ns[2][1][2] && !ns[2][1][1] && !ns[1][1][2])
							return true;
					}
					else if (ns[2][0][1])
					{
						if (!ns[0][0][2] && !ns[0][0][1] && !ns[1][0][2] &&
							!ns[0][1][2] && !ns[0][1][1] && !ns[1][1][2])
							return true;
					}
				}
				for (int i = 0; i < 4; i++)  // M5
				{
					int ix = i >= 2 ? 1 : i * 2;
					int iz = i <= 1 ? 1 : (i - 2) * 2;
					if (ns[ix][0][iz])
					{
						if (ix == 1)
						{
							if (!ns[0][0][2 - iz] && !ns[1][0][2 - iz] && !ns[2][0][2 - iz] &&
								!ns[0][1][2 - iz] && !ns[1][1][2 - iz] && !ns[2][1][2 - iz])
								if (ns[0][1][iz] || ns[1][1][iz] || ns[2][1][iz] ||
									ns[0][1][1] || ns[2][1][1] ||
									ns[0][0][iz] || ns[0][0][1] ||
									ns[2][0][iz] || ns[2][0][1])
									return true;
						}
						else if (iz == 1)
						{
							if (!ns[2 - ix][0][0] && !ns[2 - ix][0][1] && !ns[2 - ix][0][2] &&
								!ns[2 - ix][1][0] && !ns[2 - ix][1][1] && !ns[2 - ix][1][2])
								if (ns[ix][1][0] || ns[ix][1][1] || ns[ix][1][2] ||
									ns[1][1][0] || ns[1][1][2] ||
									ns[ix][0][0] || ns[1][0][0] ||
									ns[ix][0][2] || ns[1][0][2])
									return true;
						}
					}
				}
			}
		}
		if (ns[1][0][1])
		{
			if (count == 8)  // M4
			{
				for (int i = 0; i <= 2; i += 2)
					for (int j = 0; j <= 2; j += 2)
						if (ns[i][2][j] && ns[i][1][j])
							return true;
			}
			if (count >= 6)  // M2
			{
				for (int i = 0; i < 4; i++)
				{
					int ix = i >= 2 ? 1 : i * 2;
					int iz = i <= 1 ? 1 : (i - 2) * 2;
					if (ns[ix][1][iz])
					{
						if (ix == 1)
						{
							if (!ns[0][2][2 - iz] && !ns[1][2][2 - iz] && !ns[2][2][2 - iz] &&
								!ns[0][2][1] && !ns[1][2][1] && !ns[2][2][1])
								return true;
						}
						else if (iz == 1)
						{
							if (!ns[2 - ix][2][0] && !ns[2 - ix][2][1] && !ns[2 - ix][2][2] &&
								!ns[1][2][0] && !ns[1][2][1] && !ns[1][2][2])
								return true;
						}
					}
				}
			}
			if (count >= 4 && !ns[1][2][1])  // M3
			{
				if (ns[1][1][2])
				{
					if (ns[0][1][1])
					{
						if (!ns[2][2][0] && !ns[2][2][1] && !ns[1][2][0])
							return true;
					}
					else if (ns[2][1][1])
					{
						if (!ns[0][2][0] && !ns[0][2][1] && !ns[1][2][0])
							return true;
					}
				}
				else if (ns[1][1][0])
				{
					if (ns[0][1][1])
					{
						if (!ns[2][2][2] && !ns[2][2][1] && !ns[1][2][2])
							return true;
					}
					else if (ns[2][1][1])
					{
						if (!ns[0][2][2] && !ns[0][2][1] && !ns[1][2][2])
							return true;
					}
				}
			}
		}
		return false;
	}
	//void SkeletonGenerator::thining()
	//{
	//	// do thining
	//	while (1)
	//	{
	//		int count = 0;
	//		for (int i = 0; i < 6; i++)  // for each direction
	//		{
	//			voxelh->voxels_set_unvisited();
	//			DirectionType dir = DirectionType(i);
	//			for (int idx = 0; idx < voxelh->voxels.size(); idx++)  // search for deletable points
	//			{
	//				if (voxelh->is_object(idx))
	//				{
	//					Voxel& v = voxelh->voxels[idx];
	//					bool ns[3][3][3];
	//					construct_neighbor_array(v, ns);
	//					transform_neighbor_array(ns, dir);
	//					if (match_mask(ns))
	//						v.visited = true;
	//				}
	//			}
	//			for (int idx = 0; idx < voxelh->voxels.size(); idx++)  // delete them simultaneously
	//			{
	//				Voxel& v = voxelh->voxels[idx];
	//				if (v.visited)
	//				{
	//					v.type = Voxel::deleted;
	//					count++;
	//				}
	//			}
	//		}
	//		if (count <= 0)
	//			break;
	//	}
	//	// rest points are skeleton points
	//	for (int i = 0; i < voxelh->voxels.size(); i++)
	//	{
	//		if (voxelh->is_object(i))
	//			voxelh->voxels[i].type = Voxel::skeleton;
	//		else if (voxelh->voxels[i].type == Voxel::deleted)
	//			voxelh->voxels[i].type = Voxel::inside;
	//	}
	//}
	void SkeletonGenerator::thining()
	{
		// do thining
		std::list<int> vxlist;
		for (int idx = 0; idx < voxelh->voxels.size(); idx++)  // search for deletable points
			if (voxelh->is_object(idx))
				vxlist.push_back(idx);
		while (1)
		{
			int count = 0;
			for (int i = 0; i < 6; i++)  // for each direction
			{
				voxelh->voxels_set_unvisited();
				DirectionType dir = DirectionType(i);
				for (auto it = vxlist.begin(); it != vxlist.end(); ++it) {
					int idx = *it;
					if (voxelh->is_object(idx))
					{
						Voxel& v = voxelh->voxels[idx];
						bool ns[3][3][3];
						construct_neighbor_array(v, ns);
						transform_neighbor_array(ns, dir);
						if (match_mask(ns))
							v.visited = true;
					}
				}
				for (auto it = vxlist.begin(); it != vxlist.end();) {
					int idx = *it;
					Voxel& v = voxelh->voxels[idx];
					if (v.visited)
					{
						v.type = Voxel::deleted;
						count++;
						it = vxlist.erase(it);
					}
					else ++it;
				}
			}
			if (count <= 0)
				break;
		}
		// rest points are skeleton points
		for (int i = 0; i < voxelh->voxels.size(); i++)
		{
			if (voxelh->is_object(i))
				voxelh->voxels[i].type = Voxel::skeleton;
			else if (voxelh->voxels[i].type == Voxel::deleted)
				voxelh->voxels[i].type = Voxel::inside;
		}
	}
	// form skeleton
	void SkeletonGenerator::search_and_label_skeleton_points(IntVec& ske, IntVec& end)
	{
		// collect skeleton points and label their type
		voxelh->voxels_reset_label();
		end.clear();
		ske.clear();
		for (int idx = 0; idx < (int)voxelh->voxels.size(); idx++)
		{
			if (voxelh->is_skeleton(idx))
			{
				int count = 0;
				for (int i = 0; i < 27; i++)
				{
					int ni = -1;
					if (voxelh->voxel_neighbor(idx, i, ni))
						if (voxelh->is_skeleton(ni))
							count++;
				}
				ske.push_back(idx);
				voxelh->voxels[idx].label = count;
			}
		}
		// deal with triangle problem
		voxelh->voxels_set_unvisited();
		int count_triangle = 0;
		for (int idx = 0; idx < (int)ske.size(); idx++)
		{
			if (voxelh->voxels[ske[idx]].label > 2)
			{
				IntVec ns;
				IntVec nsv;
				ns.push_back(ske[idx]);
				nsv.push_back(0);
				for (int i = 0; i < 27; i++)
				{
					int ni = -1;
					if (voxelh->voxel_neighbor(ske[idx], i, ni))
					{
						if (voxelh->is_skeleton(ni))
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
						if (voxelh->voxel_neighbor(ns[i], j, ni))
							if (voxelh->is_skeleton(ni))
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
					Vertex vtx; vtx.x = 0; vtx.y = 0; vtx.z = 0;
					for (int i = 0; i < (int)tri.size(); i++)
					{
						vtx += voxelh->voxel_center(tri[i]);
					}
					float trisize = (float)tri.size();
					vtx.x /= trisize; vtx.y /= trisize; vtx.z /= trisize;
					float mindist = DECIMAL_MAX; int index = -1;
					for (int i = 0; i < (int)tri.size(); i++)
					{
						float dist = (voxelh->voxel_center(tri[i]) - vtx).square_length();
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
							if (!voxelh->voxels[tri[i]].visited)
							{
								voxelh->voxels[tri[i]].label -= 1;
								voxelh->voxels[tri[i]].visited = true;
							}
						}
					}

				}
			}
		}
		// collect end points
		for (int idx = 0; idx < (int)ske.size(); idx++)
		{
			if (voxelh->voxels[ske[idx]].label <= 1)
			{
				end.push_back(ske[idx]);
			}
		}
	}
	void SkeletonGenerator::connect_skeleton()
	{
		// collect skeleton points
		IntVec skeleton_points;
		for (int idx = 0; idx < voxelh->voxels.size(); idx++)
			if (voxelh->is_skeleton(idx))
				skeleton_points.push_back(idx);
		// label skeleton points to find disconnected part
		voxelh->voxels_reset_label();
		std::vector<IntVec> ske;
		int count_label = 0;
		for (int idx = 0; idx < skeleton_points.size(); idx++)
		{
			int vxi = skeleton_points[idx];
			if (voxelh->voxels[vxi].label == -1)
			{
				// BFS
				IntVec vec;
				voxelh->voxels_set_unvisited();
				ArrayQueue<int> Q((int)voxelh->voxels.size());
				voxelh->voxels[vxi].visited = true;
				voxelh->voxels[vxi].label = count_label;
				vec.push_back(vxi);
				Q.push(vxi);
				while (!Q.empty())
				{
					int current = Q.front();
					Q.pop();
					for (int i = 0; i < 27; i++)
					{
						int ni = -1;
						if (voxelh->voxel_neighbor(current, i, ni))
						{
							if (voxelh->is_skeleton(ni) && !voxelh->voxels[ni].visited)
							{
								voxelh->voxels[ni].label = count_label;
								voxelh->voxels[ni].visited = true;
								vec.push_back(ni);
								Q.push(ni);
							}
						}
					}
				}
				ske.push_back(vec);
				count_label++;
			}
		}
		// if there are disconnected parts
		if (count_label > 1)
		{
			while (count_label > 1)
			{
				// find the smallest part
				int minskesize = INTEGER_MAX, minskeidx = -1;
				for (int idx = 0; idx < (int)ske.size(); idx++)
				{
					int size = (int)ske[idx].size();
					if (size > 0 && size < minskesize)
					{
						minskesize = size;
						minskeidx = idx;
					}
				}
				// find isolated or end points of the smallest part
				IntVec vec;
				if ((int)ske[minskeidx].size() <= 1)
					vec.push_back(ske[minskeidx][0]);
				else
				{
					for (int idx = 0; idx < (int)ske[minskeidx].size(); idx++)
					{
						int index = ske[minskeidx][idx];
						int count = 0;
						for (int i = 0; i < 27; i++)
						{
							int ni = -1;
							if (voxelh->voxel_neighbor(index, i, ni))
							{
								if (voxelh->is_skeleton(ni))
									count++;
							}
						}
						if (count == 1 || count == 0)
							vec.push_back(index);
					}
				}
				if (vec.size() <= 0)
				{
					std::cout << "error when connecting skeleton, couldn't find end point\n";
					return;
				}
				// find a closest skeleton point to connect the isolated or end point
				float tmindist = DECIMAL_MAX;
				int tminidx = -1, tminpt;
				for (int idx = 0; idx < vec.size(); idx++)
				{
					int current_label = voxelh->voxels[vec[idx]].label;
					float mindist = DECIMAL_MAX;
					int minidx = -1;
					for (int i = 0; i < skeleton_points.size(); i++)
					{
						int index = skeleton_points[i];
						if (voxelh->voxels[index].label != current_label)
						{
							float dist = voxelh->voxel_distance_s(index, vec[idx]);
							if (dist < mindist)
							{
								mindist = dist;
								minidx = index;
							}
						}
					}
					if (mindist < tmindist)
					{
						tmindist = mindist;
						tminidx = minidx;
						tminpt = idx;
					}
				}
				if (tminpt < 0)
				{
					std::cout << "error when connecting skeleton\n";
					return;
				}
				int tgtlabel = voxelh->voxels[tminidx].label;
				connect_skeleton_point_greedy(vec[tminpt], tminidx, ske[tgtlabel]);
				for (int idx = 0; idx < (int)ske[minskeidx].size(); idx++)
				{
					voxelh->voxels[ske[minskeidx][idx]].label = tgtlabel;
					ske[tgtlabel].push_back(ske[minskeidx][idx]);
				}
				ske[minskeidx].clear();
				count_label--;
			}
		}
	}
	void SkeletonGenerator::connect_skeleton_point_greedy(int start, int end, IntVec& vec)
	{
		float err = voxelh->vxsize;
		voxelh->voxels_set_unvisited();
		int current = start;
		voxelh->voxels[current].visited = true;
		int count = 0;
		while (1)
		{
			int next = -1;
			float mindist = DECIMAL_MAX;
			for (int i = 0; i < 27; i++)
			{
				int ni = -1;
				if (voxelh->voxel_neighbor(current, i, ni))
				{
					float dist = voxelh->voxel_distance_s(ni, end);
					if (dist < mindist && !voxelh->voxels[ni].visited)
					{
						next = ni;
						mindist = dist;
					}
				}
			}
			if (mindist <= err)
				break;
			if (next < 0)
			{
				std::cout << "error when connecting two skeleton point using A*\n";
				break;
			}
			voxelh->voxels[next].visited = true;
			voxelh->voxels[next].type = Voxel::skeleton;
			vec.push_back(next);
			current = next;
			count++;
		}
	}
	void SkeletonGenerator::cut_skeleton()
	{
		// collect skeleton points and end points, also label their type
		IntVec end_points;
		IntVec skeleton_points;
		search_and_label_skeleton_points(skeleton_points, end_points);
		// cut the redundant branches
		if ((int)end_points.size() > 5)
		{
			// cut leaves
			while ((int)end_points.size() > 5)
				//for (int ti = 0; ti < 3; ti++)
			{
				// find leaves
				std::vector<IntVec> leaves((int)end_points.size());
				for (int idx = 0; idx < (int)end_points.size(); ++idx)
				{
					voxelh->voxels_set_unvisited();
					int current = end_points[idx];
					while (1)
					{
						voxelh->voxels[current].visited = true;
						bool stop = false;
						Voxel& vx = voxelh->voxels[current];
						if (voxelh->voxels[current].label > 2)
							break;
						leaves[idx].push_back(current);
						int next = -1;
						for (int i = 0; i < 27; i++)
						{
							int ni = -1;
							if (voxelh->voxel_neighbor(current, i, ni))
							{
								if (voxelh->is_skeleton(ni))
								{
									if (voxelh->voxels[ni].label > 2)
										stop = true;
									if (!voxelh->voxels[ni].visited)
										next = ni;
								}
							}
						}
						if (stop)
							break;
						if (next == -1)
						{
							std::cout << "error when finding leaf\n"; break;
						}
						current = next;
					}
				}
				// find shortest
				int minsize = INTEGER_MAX, minidx = -1;
				for (int i = 0; i < (int)leaves.size(); ++i)
				{
					int size = (int)leaves[i].size();
					if (size < minsize && voxelh->is_skeleton(leaves[i][0]))
					{
						minsize = size;
						minidx = i;
					}
				}
				// cut leaf
				for (int i = 0; i < (int)leaves[minidx].size(); ++i)
					voxelh->voxels[leaves[minidx][i]].type = Voxel::inside;
				// collect skeleton points and end points, also label their type
				end_points.clear();
				skeleton_points.clear();
				search_and_label_skeleton_points(skeleton_points, end_points);
			}
		}
	}
	void SkeletonGenerator::smooth_skeleton()
	{
		// branching voxels are candidate joints
		IntVec branch_points;
		IntVec skeleton_points;
		for (int idx = 0; idx < (int)voxelh->voxels.size(); idx++)
		{
			if (voxelh->is_skeleton(idx))
			{
				int count = 0;
				for (int i = 0; i < 27; i++)
				{
					int ni = -1;
					if (voxelh->voxel_neighbor(idx, i, ni))
						if (voxelh->is_skeleton(ni))
							count++;
				}
				if (count > 2 || count <= 1)
					branch_points.push_back(idx);
				skeleton_points.push_back(idx);
			}
		}
		for (int idx = 0; idx < (int)skeleton_points.size(); idx++)
			voxelh->voxels[skeleton_points[idx]].type = Voxel::inside;
		for (int idx = 0; idx < (int)branch_points.size(); idx++)
			voxelh->voxels[branch_points[idx]].type = Voxel::skeleton;
		//// replace
		//IntVec skeleton_points;
		//IntVec new_skeleton_points;
		//for (int idx = 0; idx < (int)voxelh->voxels.size(); idx++)
		//{
		//	if (voxelh->is_skeleton(idx) && voxelh->voxels[idx].label <= 2)
		//	{
		//		float minf = voxelh->voxels[idx].force_mag;
		//		int minidx = idx;
		//		for (int i = 0; i < 27; i++)
		//		{
		//			int ni = -1;
		//			if (voxelh->voxel_neighbor(idx, i, ni))
		//			{
		//				if (voxelh->is_skeleton(ni))
		//				{
		//					if (voxelh->voxels[ni].force_mag < minf)
		//					{
		//						minf = voxelh->voxels[ni].force_mag;
		//						minidx = ni;
		//					}
		//				}
		//			}
		//		}
		//		skeleton_points.push_back(idx);
		//		new_skeleton_points.push_back(minidx);
		//	}
		//}
		//for (int idx = 0; idx < (int)skeleton_points.size(); idx++)
		//	voxelh->voxels[skeleton_points[idx]].type = Voxel::inside;
		//for (int idx = 0; idx < (int)skeleton_points.size(); idx++)
		//	voxelh->voxels[new_skeleton_points[idx]].type = Voxel::skeleton;
	}
	// force field
	void SkeletonGenerator::generate_force_field()
	{
		// initiate rays
		std::vector<Vertex> rays;
		int ceni = DIVIDE_LEVEL*voxelh->vxlen[1] * voxelh->vxlen[0];
		ceni += DIVIDE_LEVEL*voxelh->vxlen[0] + DIVIDE_LEVEL;
		Voxel& v = voxelh->voxels[ceni];
		int hlen = (int)((DIVIDE_LEVEL - 1) / 2);
		Vertex vcen = voxelh->voxel_center(ceni);
		for (int i = v.z - hlen; i <= v.z + hlen; i++)
		{
			for (int j = v.y - hlen; j <= v.y + hlen; j++)
			{
				for (int k = v.x - hlen; k <= v.x + hlen; k++)
				{
					if (abs(i - v.z) == hlen || abs(j - v.y) == hlen || abs(k - v.x) == hlen)
					{
						int index = i*voxelh->vxlen[1] * voxelh->vxlen[0] + j*voxelh->vxlen[0] + k;
						Vector t_ray = voxelh->voxel_center(index) - vcen; t_ray.normalize();
						rays.push_back(t_ray);
					}
				}
			}
		}
		// how many object voxels ?
		int count = 0;
		for (int i = 0; i < voxelh->voxels.size(); i++)
		{
			if (voxelh->is_object(i))
				count++;
		}
		count = 0;
		for (int i = 0; i < voxelh->voxels.size(); i++)
		{
			if (voxelh->is_object(i))
			{
				Vertex tf;
				tf.x = 0; tf.y = 0; tf.z = 0;
				for (int j = 0; j < rays.size(); j++)
					tf += compute_force_fast(voxelh->voxel_center(i), rays[j]);
				voxelh->voxels[i].force_mag = sqrt(tf.x*tf.x + tf.y*tf.y + tf.z*tf.z);
				count++;
				if (count % 1000 == 0)
					std::cout << "finished " << count << std::endl;
			}
		}
	}
	void SkeletonGenerator::generate_skeleton_from_force_field()
	{
		IntVec ske;
		for (int idx = 0; idx < voxelh->voxels.size(); idx++)
		{
			if (voxelh->is_object(idx))
			{
				Voxel& v = voxelh->voxels[idx];
				float comp = v.force_mag;
				bool flag = true;
				for (int i = 0; i < 27; i++)
				{
					int index = -1;
					if (voxelh->voxel_neighbor(idx, i, index))
					{
						Voxel& n = voxelh->voxels[index];
						if (voxelh->is_object(index))
							if (n.force_mag < comp)
								flag = false;
					}
				}
				if (flag)
					ske.push_back(idx);
			}
		}
		//// label the voxels
		//voxelh->voxels_reset_label();
		//for (int idx = 0; idx < (int)voxelh->voxels.size(); idx++)
		//{
		//	if (voxelh->is_skeleton(idx))
		//	{
		//		int count = 0;
		//		for (int i = 0; i < 27; i++)
		//		{
		//			int ni = -1;
		//			if (voxelh->voxel_neighbor(idx, i, ni))
		//				if (voxelh->is_skeleton(ni))
		//					count++;
		//		}
		//		voxelh->voxels[idx].label = count;
		//	}
		//}
		//// points adjacent to branching points cannot be delete
		//for (int idx = 0; idx < (int)voxelh->voxels.size(); idx++)
		//{
		//	if (voxelh->is_skeleton(idx))
		//	{
		//		for (int i = 0; i < 27; i++)
		//		{
		//			int ni = -1;
		//			if (voxelh->voxel_neighbor(idx, i, ni))
		//				if (voxelh->is_skeleton(ni) && voxelh->voxels[ni].label > 2)
		//					voxelh->voxels[idx].label = voxelh->voxels[ni].label;
		//		}
		//	}
		//}
		//// delete adjacent thining points except branching points
		//for (int idx = 0; idx < (int)ske.size(); idx++)
		//{
		//	for (int i = 0; i < 27; i++)
		//	{
		//		int ni = -1;
		//		if (voxelh->voxel_neighbor(ske[idx], i, ni))
		//			if (voxelh->is_skeleton(ni) && voxelh->voxels[ni].label <= 2)
		//				voxelh->voxels[ni].type = Voxel::inside;
		//	}
		//}
		for (int idx = 0; idx < (int)ske.size(); idx++)
			voxelh->voxels[ske[idx]].type = Voxel::skeleton;
	}
	bool SkeletonGenerator::intersect_triangle(const Vertex& orig, const Vertex& dir, const Vertex& v0,
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
	bool SkeletonGenerator::check_boundary(const Voxel& vx, const Vertex& center, const Vertex& ray, Vertex& ret)
	{
		for (int i = 0; i < vx.face_num; i++)
		{
			int fi = voxelh->boundary_faces[vx.face_start + i];
			const Vertex& v0 = mesh->v[mesh->t[fi].v[0]];
			const Vertex& v1 = mesh->v[mesh->t[fi].v[1]];
			const Vertex& v2 = mesh->v[mesh->t[fi].v[2]];
			Vertex intersect; float t, u, v;
			if (intersect_triangle(center, ray, v0, v1, v2, t, u, v, intersect))
			{
				float f = 1 / (intersect - center).square_length();
				ret.x = ray.x*f;
				ret.y = ray.y*f;
				ret.z = ray.z*f;
				return true;
			}
		}
		return false;
	}
	Vertex SkeletonGenerator::compute_force_slow(const Vertex& center, const Vertex& ray)
	{
		Vertex cloest_intersect;
		float min_t = -1;
		for (int i = 0; i < mesh->t.size(); i++)
		{
			const Vertex& v0 = mesh->v[mesh->t[i].v[0]];
			const Vertex& v1 = mesh->v[mesh->t[i].v[1]];
			const Vertex& v2 = mesh->v[mesh->t[i].v[2]];
			Vertex intersect; float t, u, v;
			if (intersect_triangle(center, ray, v0, v1, v2, t, u, v, intersect))
			{
				if (min_t < 0 || t < min_t)
				{
					min_t = t;
					cloest_intersect = intersect;
					break;
				}
			}
		}
		Vertex ret; ret.x = 0; ret.y = 0; ret.z = 0;
		if (min_t < 0)
		{
			std::cout << "error : no intersection found\n";
			return ret;
		}
		float f = 1 / (cloest_intersect - center).square_length();
		ret.x = ray.x*f;
		ret.y = ray.y*f;
		ret.z = ray.z*f;
		return ret;
	}
	Vertex SkeletonGenerator::compute_force_fast(const Vertex& center, const Vertex& ray)
	{
		float step = voxelh->vxsize / 2;
		float t = step;
		int last = -1;
		while (1)
		{
			Vertex temp = ray; temp.x *= t; temp.y *= t; temp.z *= t;
			temp += center;
			int v = voxelh->vertex_in_which_voxel(temp);
			if (v < 0) return compute_force_slow(center, ray);
			if (v != last)
			{
				const Voxel& vx = voxelh->voxels[v];
				if (vx.type == Voxel::outside)
				{
					std::cout << "couldn't find intersection\n";
					return compute_force_slow(center, ray);
				}
				else if (vx.type == Voxel::boundary)
				{
					Vertex vtx;
					if (check_boundary(vx, center, ray, vtx))
						return vtx;
				}
			}
			t += step;
			last = v;
		}
	}
}
