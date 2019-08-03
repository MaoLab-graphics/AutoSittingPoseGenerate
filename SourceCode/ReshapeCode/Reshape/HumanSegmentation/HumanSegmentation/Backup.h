//// Backups
//
//// -------------------- Backups --------------------
//
//namespace ManSeg {
//
//	// -------------------- Class Voxel Code --------------------
//
//	class Voxel_Code : public Segmentation {
//	public:
//		Voxel_Code(Mesh* in_mesh, Voxel_Handler* vxh) : Segmentation(in_mesh, vxh) {}
//	private:
//		struct Cluster {
//			enum ClusterType { undefined, maximum };
//			bool flag;
//			int center;
//			ClusterType type;
//			IntVec voxels;
//			IntVec adj_code;  // adjacent cluster's SS code
//			IntVec adj_num;  // adjacent cluster's number
//			Cluster(bool pf = false, int pc = -1, ClusterType pt = undefined)
//				: flag(pf), center(pc), type(pt) {}
//		};
//		std::vector<std::vector<Cluster>> clusters;
//		int max_SS;
//		int max_BS;
//		// voxel coding
//		void BS_voxel_coding();
//		void SS_voxel_coding(int seed);
//		void do_SS_voxel_coding(ArrayQueue<int>& seed);
//		void do_BS_voxel_coding(ArrayQueue<int>& seed);
//		bool propagate_SS_code(const Voxel& current, Voxel& neighbor);
//		bool propagate_BS_code(const Voxel& current, Voxel& neighbor);
//		void BS_local_maxima();
//		// clustering
//		void clusters_init();
//		void clusters_clear();
//		void search_cluster(int seed);
//		void clusters_set_unlabeled();
//		void classify_clusters();
//		void search_adjacent_clusters(int code, int num);
//		void classify_cluster(int code, int num);
//		void compute_cluster_center(int code, int num);
//		void create_skeleton();
//		void refine_maximum(int code, int num);
//		int refine_skeleton_point(int seed);
//		void shortest_path(int code, int num);
//		void shortest_path(int seed);
//		// split mesh by SS code
//		int search_SS_seed(const IntVec& end, int num);
//		int ray_hits_mesh_inside(const Vertex& center, const Vertex& ray);
//		bool propagate_SS_code_label(const Voxel& current, Voxel& neighbor);
//		int find_cluster_center_and_label_distance(int code);
//		void extract_feature_by_boundary(int num, char pname);
//		void sort_end_points(const IntVec& end_points, int order[]);
//		void extract_feature_by_cluster_sizes();
//		void label_cloest_vertex_for_split(int vxidx);
//		void split_mesh_by_SS_code();
//	};
//
//	// -------------------- Class Classify Vertex --------------------
//
//	class Classify_Vertex : public Segmentation {
//	public:
//		Classify_Vertex(Mesh* in_mesh, Voxel_Handler* vxh) : Segmentation(in_mesh, vxh) {}
//	private:
//		// split mesh by vertex
//		bool intersect_triangle(const Vertex& orig, const Vertex& dir, const Vertex& v0, const Vertex& v1,
//			const Vertex& v2, float& t, float& u, float& v, Vertex& intersect);
//		bool check_boundary(const Voxel& vx, const Vertex& center, const Vertex& ray, Vertex& ret);
//		void split_mesh_by_vertex();
//		void split_head_and_arm_by_vertex(Int3DVec skerings);
//		void split_arm_by_vertex(Int3DVec skerings);
//		void split_leg_by_vertex(Int3DVec skerings);
//
//		bool test_occlusion(const Vertex& head, const Vertex& tail, float dist);
//		// border
//		void split_mesh_by_border_voxel();
//	};
//
//	// -------------------- Class Voxel Code --------------------
//
//	// voxel coding
//	void Voxel_Code::BS_voxel_coding()
//	{
//		int voxel_num = (int)voxelh->voxels.size();
//		ArrayQueue<int> Q(voxel_num);
//		voxelh->voxels_set_unvisited();
//		for (int i = 0; i < voxelh->voxels.size(); i++)
//		{
//			voxelh->voxels[i].BS_code = -1;
//			if (voxelh->voxels[i].type == Voxel::boundary)
//			{
//				voxelh->voxels[i].BS_code = 0;
//				voxelh->voxels[i].visited = true;
//				Q.push(i);
//			}
//		}
//		do_BS_voxel_coding(Q);
//		max_BS = std::numeric_limits<int>::min();
//		for (int i = 0; i < voxelh->voxels.size(); i++)
//		{
//			Voxel& vx = voxelh->voxels[i];
//			if (vx.type == Voxel::boundary || vx.type == Voxel::inside)
//				if (vx.BS_code > max_BS)
//					max_BS = vx.BS_code;
//		}
//	}
//	void Voxel_Code::SS_voxel_coding(int seed)
//	{
//		int voxel_num = (int)voxelh->voxels.size();
//		ArrayQueue<int> Q(voxel_num);
//		voxelh->voxels_set_unvisited();
//		voxelh->voxels_reset_SS_code();
//		voxelh->voxels[seed].SS_code = 0;
//		voxelh->voxels[seed].visited = true;
//		Q.push(seed);
//		do_SS_voxel_coding(Q);
//		max_SS = std::numeric_limits<int>::min();
//		for (int i = 0; i < (int)voxelh->voxels.size(); i++)
//		{
//			Voxel& vx = voxelh->voxels[i];
//			if (voxelh->is_object(i))
//				if (vx.SS_code > max_SS)
//					max_SS = vx.SS_code;
//		}
//	}
//	void Voxel_Code::do_BS_voxel_coding(ArrayQueue<int>& Q)
//	{
//		while (!Q.empty())
//		{
//			int current = Q.front();
//			Q.pop();
//			Voxel& v = voxelh->voxels[current];
//			for (int i = 0; i < 27; i++)
//			{
//				int neighbor = -1;
//				if (voxelh->voxel_neighbor(current, i, neighbor))
//				{
//					Voxel& n = voxelh->voxels[neighbor];
//					if (voxelh->is_object(neighbor))
//					{
//						propagate_BS_code(v, n);
//						if (!n.visited)
//						{
//							n.visited = true;
//							Q.push(neighbor);
//						}
//					}
//				}
//			}
//		}
//	}
//	void Voxel_Code::do_SS_voxel_coding(ArrayQueue<int>& Q)
//	{
//		while (!Q.empty())
//		{
//			int current = Q.front();
//			Q.pop();
//			Voxel& v = voxelh->voxels[current];
//			for (int i = 0; i < 27; i++)
//			{
//				int neighbor = -1;
//				if (voxelh->voxel_neighbor(current, i, neighbor))
//				{
//					Voxel& n = voxelh->voxels[neighbor];
//					if (voxelh->is_object(neighbor))
//					{
//						propagate_SS_code(v, n);
//						if (!n.visited)
//						{
//							n.visited = true;
//							Q.push(neighbor);
//						}
//					}
//				}
//			}
//		}
//	}
//	bool Voxel_Code::propagate_BS_code(const Voxel& current, Voxel& neighbor)
//	{
//		int count = 0;
//		count += abs(current.x - neighbor.x);
//		count += abs(current.y - neighbor.y);
//		count += abs(current.z - neighbor.z);
//		if (count < 0)
//		{
//			std::cout << "error, count < 0\n";
//			return false;
//		}
//		if (count == 0)
//			return false;
//		int ret = 0;
//		if (count >= 3)
//			ret = current.BS_code + ADD_BS_CODE[2];
//		else if (count >= 2)
//			ret = current.BS_code + ADD_BS_CODE[1];
//		else if (count >= 1)
//			ret = current.BS_code + ADD_BS_CODE[0];
//		if (neighbor.BS_code == -1 || ret < neighbor.BS_code)
//			neighbor.BS_code = ret;
//		else
//			return false;
//		return true;
//	}
//	bool Voxel_Code::propagate_SS_code(const Voxel& current, Voxel& neighbor)
//	{
//		int count = 0;
//		count += abs(current.x - neighbor.x);
//		count += abs(current.y - neighbor.y);
//		count += abs(current.z - neighbor.z);
//		if (count < 0)
//		{
//			std::cout << "error, count < 0\n";
//			return false;
//		}
//		if (count == 0)
//			return false;
//		int ret = 0;
//		if (count >= 3)
//			ret = current.SS_code + ADD_SS_CODE[2];
//		else if (count >= 2)
//			ret = current.SS_code + ADD_SS_CODE[1];
//		else if (count >= 1)
//			ret = current.SS_code + ADD_SS_CODE[0];
//		if (neighbor.SS_code == -1 || ret < neighbor.SS_code)
//			neighbor.SS_code = ret;
//		else
//			return false;
//		return true;
//	}
//	void Voxel_Code::BS_local_maxima()
//	{
//		for (int idx = 0; idx < voxelh->voxels.size(); idx++)
//		{
//			Voxel& v = voxelh->voxels[idx];
//			if (v.type == Voxel::inside)
//			{
//				bool flag = false;
//				for (int i = std::max(v.z - 1, 0); i <= std::min(v.z + 1, voxelh->vxlen[2] - 1); i++)
//				{
//					for (int j = std::max(v.y - 1, 0); j <= std::min(v.y + 1, voxelh->vxlen[1] - 1); j++)
//					{
//						for (int k = std::max(v.x - 1, 0); k <= std::min(v.x + 1, voxelh->vxlen[0] - 1); k++)
//						{
//							int index = i*voxelh->vxlen[1] * voxelh->vxlen[0] + j*voxelh->vxlen[0] + k;
//							Voxel& n = voxelh->voxels[index];
//							if (voxelh->is_object(index) || n.type == Voxel::boundary)
//								if (index != idx && n.BS_code > v.BS_code)
//									flag = true;
//						}
//					}
//				}
//				if (!flag)
//					v.type = Voxel::BS_maximum;
//			}
//		}
//	}
//	// clustering
//	void Voxel_Code::clusters_init()
//	{
//		clusters.resize(max_SS + 1);
//		voxelh->voxels_set_unvisited();
//		for (int i = 0; i < (int)voxelh->voxels.size(); i++)
//		{
//			if (voxelh->is_object(i))
//			{
//				Voxel& v = voxelh->voxels[i];
//				if (!v.visited)
//				{
//					v.visited = true;
//					search_cluster(i);
//					//compute_cluster_center(v.SS_code, v.cluster_num);
//				}
//			}
//		}
//	}
//	void Voxel_Code::clusters_clear()
//	{
//		for (int i = 0; i < (int)(clusters.size()); i++)
//		{
//			for (int j = 0; j < (int)clusters[i].size(); j++)
//				IntVec().swap(clusters[i][j].voxels);
//			std::vector<Cluster>().swap(clusters[i]);
//		}
//		std::vector<std::vector<Cluster>>().swap(clusters);
//	}
//	void Voxel_Code::search_cluster(int seed)
//	{
//		int voxel_num = (int)voxelh->voxels.size();
//		ArrayQueue<int> Q(voxel_num);
//		Cluster cluster;
//		int seed_code = voxelh->voxels[seed].SS_code;
//		int cst_num = (int)clusters[seed_code].size();
//		voxelh->voxels[seed].cluster_num = cst_num;
//		cluster.voxels.push_back(seed);
//		Q.push(seed);
//		while (!Q.empty())
//		{
//			int current = Q.front();
//			Q.pop();
//			Voxel& v = voxelh->voxels[current];
//			for (int i = std::max(v.z - 1, 0); i <= std::min(v.z + 1, voxelh->vxlen[2] - 1); i++)
//			{
//				for (int j = std::max(v.y - 1, 0); j <= std::min(v.y + 1, voxelh->vxlen[1] - 1); j++)
//				{
//					for (int k = std::max(v.x - 1, 0); k <= std::min(v.x + 1, voxelh->vxlen[0] - 1); k++)
//					{
//						int index = i*voxelh->vxlen[1] * voxelh->vxlen[0] + j*voxelh->vxlen[0] + k;
//						Voxel& n = voxelh->voxels[index];
//						if (voxelh->is_object(index) && n.SS_code == seed_code && !n.visited)
//						{
//							n.visited = true;
//							n.cluster_num = cst_num;
//							cluster.voxels.push_back(index);
//							Q.push(index);
//						}
//					}
//				}
//			}
//		}
//		clusters[seed_code].push_back(cluster);
//	}
//	void Voxel_Code::clusters_set_unlabeled()
//	{
//		for (int i = 0; i < clusters.size(); i++)
//		{
//			for (int j = 0; j < clusters[i].size(); j++)
//			{
//				clusters[i][j].flag = false;
//			}
//		}
//	}
//	void Voxel_Code::classify_clusters()
//	{
//		for (int i = 0; i < clusters.size(); i++)
//		{
//			for (int j = 0; j < clusters[i].size(); j++)
//			{
//				clusters_set_unlabeled();
//				search_adjacent_clusters(i, j);
//				classify_cluster(i, j);
//			}
//		}
//	}
//	void Voxel_Code::search_adjacent_clusters(int code, int num)
//	{
//		for (int idx = 0; idx < clusters[code][num].voxels.size(); idx++)
//		{
//			Voxel& v = voxelh->voxels[clusters[code][num].voxels[idx]];
//			for (int i = std::max(v.z - 1, 0); i <= std::min(v.z + 1, voxelh->vxlen[2] - 1); i++)
//			{
//				for (int j = std::max(v.y - 1, 0); j <= std::min(v.y + 1, voxelh->vxlen[1] - 1); j++)
//				{
//					for (int k = std::max(v.x - 1, 0); k <= std::min(v.x + 1, voxelh->vxlen[0] - 1); k++)
//					{
//						int index = i*voxelh->vxlen[1] * voxelh->vxlen[0] + j*voxelh->vxlen[0] + k;
//						Voxel& n = voxelh->voxels[index];
//						if (voxelh->is_object(index))
//							if (n.SS_code != code || n.cluster_num != num)
//								clusters[n.SS_code][n.cluster_num].flag = true;
//					}
//				}
//			}
//		}
//		for (int i = 0; i < clusters.size(); i++)
//		{
//			for (int j = 0; j < clusters[i].size(); j++)
//			{
//				if (clusters[i][j].flag)
//				{
//					clusters[code][num].adj_code.push_back(i);
//					clusters[code][num].adj_num.push_back(j);
//				}
//			}
//		}
//	}
//	void Voxel_Code::classify_cluster(int code, int num)
//	{
//		bool local_maximum = true;
//		for (int i = 0; i < clusters[code][num].adj_code.size(); i++)
//		{
//			int n_code = clusters[code][num].adj_code[i];
//			int n_num = clusters[code][num].adj_num[i];
//			if (n_code >= code)
//				local_maximum = false;
//		}
//		if (local_maximum)
//		{
//			clusters[code][num].type = Cluster::maximum;
//			for (int idx = 0; idx < clusters[code][num].voxels.size(); idx++)
//			{
//				int index = clusters[code][num].voxels[idx];
//				Voxel& v = voxelh->voxels[index];
//				v.type = Voxel::skeleton;
//			}
//		}
//	}
//	void Voxel_Code::compute_cluster_center(int code, int num)
//	{
//		Vertex v0; v0.x = 0; v0.y = 0; v0.z = 0;
//		std::vector<IntVec> vec;
//		vec.resize(max_BS + 1);
//		for (int idx = 0; idx < clusters[code][num].voxels.size(); idx++)
//		{
//			int index = clusters[code][num].voxels[idx];
//			v0.x += (float)voxelh->voxels[index].x*voxelh->vxsize;
//			v0.y += (float)voxelh->voxels[index].y*voxelh->vxsize;
//			v0.z += (float)voxelh->voxels[index].z*voxelh->vxsize;
//			vec[voxelh->voxels[index].BS_code].push_back(index);
//		}
//		float size = (float)clusters[code][num].voxels.size();
//		v0.x /= size; v0.y /= size; v0.z /= size;
//		int maxi = -1;
//		for (int i = (int)vec.size() - 1; i >= 0; i--)
//			if (vec[i].size() > 0) { maxi = i; break; }
//		float min_dist = std::numeric_limits<float>::max();
//		int max_idx = vec[maxi][0];
//		if (vec[maxi].size() > 1)
//		{
//			for (int i = 0; i < vec[maxi].size(); i++)
//			{
//				Vertex v1;
//				v1.x = (float)voxelh->voxels[vec[maxi][i]].x*voxelh->vxsize;
//				v1.y = (float)voxelh->voxels[vec[maxi][i]].y*voxelh->vxsize;
//				v1.z = (float)voxelh->voxels[vec[maxi][i]].z*voxelh->vxsize;
//				float comp = (v0 - v1).length_s();
//				if (comp < min_dist)
//				{
//					min_dist = comp;
//					max_idx = vec[maxi][i];
//				}
//			}
//		}
//		clusters[code][num].center = max_idx;
//	}
//	void Voxel_Code::create_skeleton()
//	{
//		for (int i = 0; i < clusters.size(); i++)
//		{
//			for (int j = 0; j < clusters[i].size(); j++)
//			{
//				if (clusters[i][j].type == Cluster::maximum)
//				{
//					refine_maximum(i, j);
//				}
//			}
//		}
//		for (int i = 0; i < clusters.size(); i++)
//		{
//			for (int j = 0; j < clusters[i].size(); j++)
//			{
//				if (clusters[i][j].type == Cluster::maximum)
//				{
//					//shortest_path(i, j);
//					shortest_path(clusters[i][j].center);
//				}
//			}
//		}
//	}
//	void Voxel_Code::refine_maximum(int code, int num)
//	{
//		int center = clusters[code][num].center;
//		for (int i = 0; i < clusters.size(); i++)
//		{
//			for (int j = 0; j < clusters[i].size(); j++)
//			{
//				if (i != code || j != num)
//				{
//					if (clusters[i][j].type == Cluster::maximum)
//					{
//						if (voxelh->voxel_distance_s(center, clusters[i][j].center) <= 90000)
//						{
//							if (code >= i)
//								clusters[i][j].type = Cluster::undefined;
//							else
//							{
//								clusters[code][num].type = Cluster::undefined;
//								return;
//							}
//						}
//					}
//				}
//			}
//		}
//	}
//	void Voxel_Code::shortest_path(int code, int num)
//	{
//		voxelh->voxels[clusters[code][num].center].type = Voxel::skeleton;
//		int cen = clusters[code][num].center;
//		while (1)
//		{
//			int size = (int)clusters[code][num].adj_code.size();
//			if (size <= 0)
//			{
//				std::cout << "error : no adjacent cluster\n";
//				break;
//			}
//			int ncode = clusters[code][num].adj_code[0];
//			int nnum = clusters[code][num].adj_num[0];
//			int ncen = clusters[ncode][nnum].center;
//			float dist = voxelh->voxel_distance_s(cen, ncen);
//			if (dist >= 40000)
//			{
//				std::cout << "dist > 200\n";
//				for (int idx = 0; idx < clusters[ncode][nnum].voxels.size(); idx++)
//					voxelh->voxels[clusters[ncode][nnum].voxels[idx]].type = Voxel::skeleton;
//				for (int idx = 0; idx < clusters[code][num].voxels.size(); idx++)
//					voxelh->voxels[clusters[code][num].voxels[idx]].type = Voxel::skeleton;
//			}
//			voxelh->voxels[clusters[ncode][nnum].center].type = Voxel::skeleton;
//			if (ncode <= 0)
//				break;
//			code = ncode;
//			num = nnum;
//			cen = ncen;
//		}
//	}
//	void Voxel_Code::shortest_path(int seed)
//	{
//		IntVec ske;
//		int zero = clusters[0][0].center;
//		//int current = seed;
//		int current = refine_skeleton_point(seed);
//		voxelh->voxels[current].type = Voxel::skeleton;
//		int last_min = std::numeric_limits<int>::max();
//		while (1)
//		{
//			Voxel& v = voxelh->voxels[current];
//			int next = -1, minc = std::numeric_limits<int>::max();
//			for (int i = std::max(v.z - 1, 0); i <= std::min(v.z + 1, voxelh->vxlen[2] - 1); i++)
//			{
//				for (int j = std::max(v.y - 1, 0); j <= std::min(v.y + 1, voxelh->vxlen[1] - 1); j++)
//				{
//					for (int k = std::max(v.x - 1, 0); k <= std::min(v.x + 1, voxelh->vxlen[0] - 1); k++)
//					{
//						int index = i*voxelh->vxlen[1] * voxelh->vxlen[0] + j*voxelh->vxlen[0] + k;
//						Voxel& n = voxelh->voxels[index];
//						if (n.type == Voxel::inside /*|| n.type == Voxel::boundary*/)
//						{
//							if (n.SS_code < minc)
//							{
//								next = index;
//								minc = n.SS_code;
//							}
//							else if (n.SS_code == minc)
//							{
//								/*if (voxelh->voxel_distance_s(zero, index) < voxelh->voxel_distance_s(zero, next))
//								next = index;*/
//								if (voxelh->voxels[index].BS_code > voxelh->voxels[index].BS_code)
//									next = index;
//							}
//						}
//					}
//				}
//			}
//			if (next < 0)
//			{
//				std::cout << "error : no adjacent voxel\n";
//				break;
//			}
//			//ske.push_back(refine_skeleton_point(next));
//			//next = refine_skeleton_point(next);
//			if (last_min < minc)
//			{
//				std::cout << "error : reverse\n";
//				break;
//			}
//			voxelh->voxels[next].type = Voxel::skeleton;
//			//voxelh->voxels[clusters[voxelh->voxels[next].SS_code][voxelh->voxels[next].cluster_num].center].type = Voxel::skeleton;
//			if (minc <= 0)
//				break;
//			current = next;
//			last_min = minc;
//		}
//		/*for (int i = 0; i < ske.size(); i++)
//		{
//		voxelh->voxels[ske[i]].type = Voxel::skeleton;
//		}*/
//	}
//	int Voxel_Code::refine_skeleton_point(int seed)
//	{
//		int count = 0;
//		int current = seed;
//		int maxc = voxelh->voxels[current].BS_code;
//		while (count < 1000)
//		{
//			Voxel& v = voxelh->voxels[current];
//			int next = -1;
//			for (int i = std::max(v.z - 1, 0); i <= std::min(v.z + 1, voxelh->vxlen[2] - 1); i++)
//			{
//				for (int j = std::max(v.y - 1, 0); j <= std::min(v.y + 1, voxelh->vxlen[1] - 1); j++)
//				{
//					for (int k = std::max(v.x - 1, 0); k <= std::min(v.x + 1, voxelh->vxlen[0] - 1); k++)
//					{
//						int index = i*voxelh->vxlen[1] * voxelh->vxlen[0] + j*voxelh->vxlen[0] + k;
//						Voxel& n = voxelh->voxels[index];
//						if (n.type == Voxel::inside /*|| n.type == Voxel::boundary*/)
//						{
//							if (n.BS_code > maxc)
//							{
//								next = index;
//								maxc = n.BS_code;
//							}
//						}
//					}
//				}
//			}
//			if (next < 0)
//				return current;
//			current = next;
//			count++;
//		}
//		return current;
//	}
//	// split mesh by SS code
//	int Voxel_Code::search_SS_seed(const IntVec& end, int num)
//	{
//		voxelh->voxels_set_unvisited();
//		int current = end[num];
//		int joint = -1;
//		while (1)
//		{
//			voxelh->voxels[current].visited = true;
//			bool stop = false;
//			Voxel& vx = voxelh->voxels[current];
//			if (voxelh->voxels[current].label > 2)
//			{
//				joint = current;
//				break;
//			}
//			int next = -1;
//			for (int i = 0; i < 27; i++)
//			{
//				int ni = -1;
//				if (voxelh->voxel_neighbor(current, i, ni))
//				{
//					if (voxelh->is_skeleton(ni))
//					{
//						if (voxelh->voxels[ni].label > 2)
//						{
//							joint = ni;
//							stop = true;
//						}
//						if (!voxelh->voxels[ni].visited)
//							next = ni;
//					}
//				}
//			}
//			if (stop)
//				break;
//			if (next == -1)
//			{
//				std::cout << "error when finding leaf\n"; break;
//			}
//			current = next;
//		}
//		Vertex ray = voxelh->voxel_center(end[num]) - voxelh->voxel_center(joint);
//		ray.normalize();
//		return ray_hits_mesh_inside(voxelh->voxel_center(end[num]), ray);
//	}
//	int Voxel_Code::ray_hits_mesh_inside(const Vertex& center, const Vertex& ray)
//	{
//		float step = voxelh->vxsize / 2;
//		float t = step;
//		int last = -1;
//		while (1)
//		{
//			Vertex temp = ray; temp.x *= t; temp.y *= t; temp.z *= t;
//			temp += center;
//			int v = voxelh->vertex_in_which_voxel(temp);
//			if (v < 0) return -1;
//			if (v != last)
//			{
//				const Voxel::VoxelType& vxt = voxelh->voxels[v].type;
//				if (vxt == Voxel::outside)
//				{
//					std::cout << "couldn't find intersection\n";
//					return -1;
//				}
//				else if (vxt == Voxel::boundary)
//				{
//					return last;
//				}
//			}
//			t += step;
//			last = v;
//		}
//	}
//	void Voxel_Code::extract_feature_by_cluster_sizes()
//	{
//		const int interval = 5;
//		const int start_point = 10;
//		// sizes
//		IntVec sizes(clusters.size());
//		int max_size = -1;
//		for (int i = 0; i < (int)clusters.size(); i++)
//		{
//			int size = 0;
//			for (int j = 0; j < (int)clusters[i].size(); j++)
//				size += (int)clusters[i][j].voxels.size();
//			if (size > max_size)
//				max_size = size;
//			sizes[i] = size;
//		}
//		// show result
//		float max_cosine = -1;
//		int max_idx = -1;
//		std::fstream fs("e:\\a.txt", std::ios::out);
//		float ratio = (float)clusters.size() / (float)max_size;
//		const float adder = (float)(interval*interval);
//		for (int i = 0; i < (int)(clusters.size() / 1); i += 1)
//		{
//			fs << i;
//			fs << " " << sizes[i] << " " << sizes[i] - sizes[std::max(0, i - 1)];
//			if (i >= start_point && i < (int)(clusters.size() - interval - 1))
//			{
//				float y0_y1 = (float)(sizes[i - interval] - sizes[i]);
//				y0_y1 *= ratio;
//				float y2_y1 = (float)(sizes[i + interval] - sizes[i]);
//				y2_y1 *= ratio;
//				float len0 = adder + y0_y1*y0_y1; len0 = sqrt(len0);
//				float len1 = adder + y2_y1*y2_y1; len1 = sqrt(len1);
//				float cosine = (y0_y1*y2_y1 - adder) / (len0*len1);
//				if (cosine > max_cosine)
//				{
//					max_cosine = cosine;
//					max_idx = i;
//				}
//				fs << " " << cosine;
//			}
//			fs << std::endl;
//		}
//		fs.close();
//		//// do not display skeleton points
//		//for (int i = 0; i < (int)skeleton_points.size(); i++)
//		//{
//		//	voxelh->voxels[skeleton_points[i]].type = Voxel::inside;
//		//}
//		//for (int i = 0; i < (int)(clusters.size() / 2); i += 6)
//		//{
//		//	for (int j = 0; j < (int)clusters[i].size(); j++)
//		//	{
//		//		for (int k = 0; k < clusters[i][j].voxels.size(); k++)
//		//		{
//		//			int index = clusters[i][j].voxels[k];
//		//			voxelh->voxels[index].type = Voxel::skeleton;
//		//		}
//		//	}
//		//}
//		if (max_idx >= 0)
//		{
//			std::cout << "max_idx is " << max_idx << std::endl;
//			for (int j = 0; j < (int)clusters[max_idx].size(); j++)
//			{
//				for (int k = 0; k < clusters[max_idx][j].voxels.size(); k++)
//				{
//					int index = clusters[max_idx][j].voxels[k];
//					voxelh->voxels[index].type = Voxel::skeleton;
//				}
//			}
//		}
//		else
//			std::cout << "split_mesh_SS_code, max_idx < 0\n";
//	}
//	bool Voxel_Code::propagate_SS_code_label(const Voxel& current, Voxel& neighbor)
//	{
//		int count = 0;
//		count += abs(current.x - neighbor.x);
//		count += abs(current.y - neighbor.y);
//		count += abs(current.z - neighbor.z);
//		if (count < 0)
//		{
//			std::cout << "error, count < 0\n";
//			return false;
//		}
//		if (count == 0)
//			return false;
//		int ret = 0;
//		if (count >= 3)
//			ret = current.label + ADD_SS_CODE[2];
//		else if (count >= 2)
//			ret = current.label + ADD_SS_CODE[1];
//		else if (count >= 1)
//			ret = current.label + ADD_SS_CODE[0];
//		if (neighbor.label == -1 || ret < neighbor.label)
//			neighbor.label = ret;
//		else
//			return false;
//		return true;
//	}
//	int Voxel_Code::find_cluster_center_and_label_distance(int code)
//	{
//		int cen = -1;
//		for (int j = 0; j < (int)clusters[code].size(); j++)
//		{
//			for (int k = 0; k < clusters[code][j].voxels.size(); k++)
//			{
//				int index = clusters[code][j].voxels[k];
//				Voxel& v = voxelh->voxels[index];
//				if (voxelh->is_skeleton(index))
//				{
//					cen = index;
//					break;
//				}
//			}
//			if (cen >= 0)
//				break;
//		}
//		if (cen < 0)
//			return cen;
//		voxelh->voxels_reset_label();
//		voxelh->voxels_set_unvisited();
//		int voxel_num = (int)voxelh->voxels.size();
//		ArrayQueue<int> Q(voxel_num);
//		voxelh->voxels[cen].visited = true;
//		voxelh->voxels[cen].label = 0;
//		Q.push(cen);
//		while (!Q.empty())
//		{
//			int current = Q.front();
//			Q.pop();
//			Voxel& v = voxelh->voxels[current];
//			for (int i = 0; i < 27; i++)
//			{
//				int neighbor = -1;
//				if (voxelh->voxel_neighbor(current, i, neighbor))
//				{
//					Voxel& n = voxelh->voxels[neighbor];
//					if (n.SS_code == code)
//					{
//						propagate_SS_code_label(v, n);
//						if (!n.visited)
//						{
//							n.visited = true;
//							Q.push(neighbor);
//						}
//					}
//				}
//			}
//		}
//		return cen;
//	}
//	void Voxel_Code::extract_feature_by_boundary(int num, char pname)
//	{
//		const int interval = 10;
//		const int start_point = 20;
//		const float ratio[3] = { (float)0.3, (float)0.36, (float)0.5 };
//		// compute scalars
//		std::vector<float> max_r(clusters.size());
//		std::vector<float> min_r(clusters.size());
//		std::vector<float> avg_r(clusters.size());
//		std::vector<float> ecc(clusters.size());
//		std::vector<float> cosines(clusters.size());
//		for (int i = 0; i < (int)(clusters.size()); i++)
//		{
//			IntVec border_points;
//			for (int j = 0; j < (int)clusters[i].size(); j++)
//			{
//				for (int k = 0; k < clusters[i][j].voxels.size(); k++)
//				{
//					int index = clusters[i][j].voxels[k];
//					for (int m = 0; m < 27; m++)
//					{
//						int ni = -1;
//						if (voxelh->voxel_neighbor(index, m, ni))
//						{
//							if (voxelh->is_boundary(ni))
//							{
//								border_points.push_back(index);
//								break;
//							}
//						}
//					}
//				}
//			}
//			ecc[i] = max_r[i] = min_r[i] = avg_r[i] = 0;
//			if (border_points.size() > 0)
//			{
//				//// center
//				//Vertex cen; cen.x = 0; cen.y = 0; cen.z = 0;
//				//for (int idx = 0; idx < (int)border_points.size(); idx++)
//				//	cen += voxelh->voxel_center(border_points[idx]);
//				//float sizef = (float)border_points.size();
//				//cen.x /= sizef; cen.y /= sizef; cen.z /= sizef;
//				int cen = find_cluster_center_and_label_distance(i);
//				if (i == 98)
//					int x = 0;
//				if (cen >= 0)
//				{
//
//					// radius
//					avg_r[i] = 0; max_r[i] = -1; min_r[i] = std::numeric_limits<float>::max();
//					for (int idx = 0; idx < (int)border_points.size(); idx++)
//					{
//						//float radius = sqrt(vertex_distance_s(cen, voxelh->voxel_center(border_points[idx])));
//						//float radius = sqrt(voxelh->voxel_distance_s(cen, border_points[idx]));
//						float radius = (float)voxelh->voxels[border_points[idx]].label;
//						if (radius < min_r[i]) min_r[i] = radius;
//						if (radius > max_r[i]) max_r[i] = radius;
//						avg_r[i] += radius;
//					}
//					avg_r[i] /= (float)border_points.size();
//					ecc[i] = sqrt(max_r[i] * max_r[i] - min_r[i] * min_r[i]) / max_r[i];
//				}
//			}
//		}
//		// refine
//		int left = -1, right = -1;
//		for (int i = 0; i < (int)(clusters.size()); i++)
//		{
//			if (max_r[i] < (float)0.0001)
//			{
//				if (left >= 0) right = i;
//				else left = right = i;
//			}
//			else
//			{
//				if (left >= 0)
//				{
//					int tleft = std::max(0, left - 1);
//					int tright = std::min((int)(clusters.size() - 1), right + 1);
//					float tavg = (avg_r[tleft] + avg_r[tright]) / (float)2;
//					float tecc = (ecc[tleft] + ecc[tright]) / (float)2;
//					float tmax = (max_r[tleft] + max_r[tright]) / (float)2;
//					float tmin = (min_r[tleft] + min_r[tright]) / (float)2;
//					for (int idx = left; idx <= right; idx++)
//					{
//						avg_r[idx] = tavg; ecc[idx] = tecc; max_r[idx] = tmax; min_r[idx] = tmin;
//					}
//					left = right = -1;
//				}
//			}
//		}
//		// extract feature
//		float max_cosine = -1;
//		int max_idx = -1;
//		const float adder = (float)(interval*interval);
//		for (int i = 0; i < (int)((float)clusters.size()*ratio[num]); i++)
//		{
//			cosines[i] = -1;
//			if (i >= start_point && i <= (int)(clusters.size() - interval - 1))
//			{
//				float y0_y1 = (float)(max_r[i - interval] - max_r[i]);
//				if (y0_y1 > 0)
//					y0_y1 = 0;
//				float y2_y1 = (float)(max_r[i + interval] - max_r[i]);
//				float len0 = adder + y0_y1*y0_y1; len0 = sqrt(len0);
//				float len1 = adder + y2_y1*y2_y1; len1 = sqrt(len1);
//				float cosine = (y0_y1*y2_y1 - adder) / (len0*len1);
//				if (y2_y1 < 0)
//					cosine = -1;
//				cosines[i] = cosine;
//				if (cosine > max_cosine)
//				{
//					max_cosine = cosine;
//					max_idx = i;
//				}
//			}
//		}
//		// show result
//		//if (pname == '1')
//		//max_idx = 24;
//		if (max_idx >= 0)
//		{
//			std::cout << "max_idx is " << max_idx << std::endl;
//			/*IntVec display_border_points;
//			for (int j = 0; j < (int)clusters[max_idx].size(); j++)
//			{
//			for (int k = 0; k < clusters[max_idx][j].voxels.size(); k++)
//			{
//			int index = clusters[max_idx][j].voxels[k];
//			for (int m = 0; m < 27; m++)
//			{
//			int ni = -1;
//			if (voxelh->voxel_neighbor(index, m, ni))
//			{
//			if (voxelh->is_boundary(ni))
//			{
//			display_border_points.push_back(index);
//			break;
//			}
//			}
//			}
//			}
//			}
//			for (int j = 0; j < (int)display_border_points.size(); j++)
//			{
//			int index = display_border_points[j];
//			voxelh->voxels[index].type = Voxel::skeleton;
//			}*/
//			for (int j = 0; j < (int)clusters[max_idx].size(); j++)
//			{
//				for (int k = 0; k < clusters[max_idx][j].voxels.size(); k++)
//				{
//					int index = clusters[max_idx][j].voxels[k];
//					//voxelh->voxels[index].type = Voxel::maximum;
//					label_cloest_vertex_for_split(index);
//				}
//			}
//		}
//		else
//			std::cout << "split_mesh_SS_code, max_idx < 0\n";
//		// write scalars to txt
//		std::string fname = "e:\\a"; fname += pname; fname += ".txt";
//		std::fstream fs(fname.c_str(), std::ios::out);
//		for (int i = 0; i < (int)((float)clusters.size()*(float)1); i += 1)
//			fs << i << "\t" << cosines[i] << "\t" << ecc[i] << "\t" << avg_r[i]
//			<< "\t" << min_r[i] << "\t" << max_r[i] << std::endl;
//		fs.close();
//	}
//	void Voxel_Code::sort_end_points(const IntVec& end_points, int order[])
//	{
//		// find leaves
//		int sizes[5] = { 0 };
//		for (int idx = 0; idx < (int)end_points.size(); idx++)
//		{
//			sizes[idx] = 0;
//			voxelh->voxels_set_unvisited();
//			int current = end_points[idx];
//			while (1)
//			{
//				voxelh->voxels[current].visited = true;
//				bool stop = false;
//				Voxel& vx = voxelh->voxels[current];
//				if (voxelh->voxels[current].label > 2)
//					break;
//				sizes[idx]++;
//				int next = -1;
//				for (int i = 0; i < 27; i++)
//				{
//					int ni = -1;
//					if (voxelh->voxel_neighbor(current, i, ni))
//					{
//						if (voxelh->is_skeleton(ni))
//						{
//							if (voxelh->voxels[ni].label > 2)
//								stop = true;
//							if (!voxelh->voxels[ni].visited)
//								next = ni;
//						}
//					}
//				}
//				if (stop)
//					break;
//				if (next == -1)
//				{
//					std::cout << "error when finding leaf\n"; break;
//				}
//				current = next;
//			}
//		}
//		int sort_size[5] = { 0 };
//		for (int idx = 0; idx < (int)end_points.size(); idx++)
//			sort_size[idx] = sizes[idx];
//		std::sort(sort_size, sort_size + 5);
//		const int temp[5] = { 0, 1, 1, 2, 2 };
//		for (int i = 0; i < (int)end_points.size(); i++)
//		{
//			for (int j = 0; j < (int)end_points.size(); j++)
//			{
//				if (sizes[i] == sort_size[j])
//				{
//					order[i] = temp[j];
//					break;
//				}
//			}
//		}
//	}
//	void Voxel_Code::label_cloest_vertex_for_split(int vxidx)
//	{
//		//// label
//		//Vertex v = voxelh->voxel_center(vxidx);
//		//int minidx = 0; float mindist = std::numeric_limits<float>::max();
//		//for (int i = 0; i < mesh->v.size(); i++) {
//		//	float dist = vertex_distance_s(mesh->v[i], v);
//		//	if (dist < mindist) {
//		//		mindist = dist;
//		//		minidx = i;
//		//	}
//		//}
//		//mesh->v[minidx].code = 1;
//		// display
//		for (int i = 0; i < 27; i++) {
//			int n = -1;
//			if (voxelh->voxel_neighbor(vxidx, i, n) && voxelh->voxels[n].type == Voxel::boundary) {
//				for (int j = 0; j < voxelh->voxels[n].face_num; j++) {
//					int f = voxelh->boundary_faces[voxelh->voxels[n].face_start + j];
//					mesh->v[mesh->t[f].v[0]].code = 1;
//					mesh->v[mesh->t[f].v[1]].code = 1;
//					mesh->v[mesh->t[f].v[2]].code = 1;
//				}
//			}
//		}
//	}
//	void Voxel_Code::split_mesh_by_SS_code()
//	{
//		// find end points
//		IntVec end_points;
//		IntVec skeleton_points;
//		search_and_label_skeleton_points(skeleton_points, end_points);
//		if ((int)end_points.size() != 5)
//		{
//			std::cout << "split_mesh_SS_code, end_points.size() != 5\n";
//			return;
//		}
//		int sort[5] = { 0 };
//		sort_end_points(end_points, sort);
//		// search seed
//		int seed[5] = { 0 };
//		for (int idx = 0; idx < (int)end_points.size(); idx++) {
//			seed[idx] = search_SS_seed(end_points, idx);
//			//seed[idx] = end_points[idx];
//		}
//		// do segmentation for each part
//		//int idx = 1;
//		for (int idx = 0; idx < 2/*(int)end_points.size()*/; idx++)
//		{
//			if (seed[idx] != -1)
//			{
//				// SS coding
//				SS_voxel_coding(seed[idx]);
//				clusters_init();
//				/*for (int i = 0; i < (int)(clusters.size() / 3); i += 6) {
//				IntVec border_points;
//				for (int j = 0; j < (int)clusters[i].size(); j++) {
//				for (int k = 0; k < clusters[i][j].voxels.size(); k++) {
//				int index = clusters[i][j].voxels[k];
//				voxelh->voxels[index].type = Voxel::skeleton;
//				}
//				}
//				}*/
//				//extract_feature_by_cluster_sizes();
//				extract_feature_by_boundary(sort[idx], '0' + idx);
//				clusters_clear();
//			}
//			else
//				std::cout << "split_mesh_SS_code, seed == -1\n";
//		}
//	}
//
//	// -------------------- Class Classify Vertex --------------------
//
//	// split mesh by vertex
//	bool Classify_Vertex::intersect_triangle(const Vertex& orig, const Vertex& dir, const Vertex& v0,
//		const Vertex& v1, const Vertex& v2, float& t, float& u, float& v, Vertex& intersect)
//	{
//		// E1
//		Vertex E1 = v1 - v0;
//		// E2
//		Vertex E2 = v2 - v0;
//		// P
//		Vertex P = dir.cross(E2);
//		// determinant
//		float det = E1.dot(P);
//		// keep det > 0, modify T accordingly
//		Vertex T;
//		if (det > 0)
//			T = orig - v0;
//		else
//		{
//			T = v0 - orig;
//			det = -det;
//		}
//		// If determinant is near zero, ray lies in plane of triangle
//		if (det < 0.0001f)
//			return false;
//		// Calculate u and make sure u <= 1
//		u = T.dot(P);
//		if (u < 0.0f || u > det)
//			return false;
//		// Q
//		Vertex Q = T.cross(E1);
//		// Calculate v and make sure u + v <= 1
//		v = dir.dot(Q);
//		if (v < 0.0f || u + v > det)
//			return false;
//		// Calculate t, scale parameters, ray intersects triangle
//		t = E2.dot(Q);
//		float fInvDet = 1.0f / det;
//		t *= fInvDet;
//		u *= fInvDet;
//		v *= fInvDet;
//		if (t < 0.0001f)
//			return false;
//		intersect.x = t*dir.x;
//		intersect.y = t*dir.y;
//		intersect.z = t*dir.z;
//		intersect += orig;
//		return true;
//	}
//	bool Classify_Vertex::check_boundary(const Voxel& vx, const Vertex& center, const Vertex& ray, Vertex& ret)
//	{
//		for (int i = 0; i < vx.face_num; i++)
//		{
//			int fi = voxelh->boundary_faces[vx.face_start + i];
//			const Vertex& v0 = mesh->v[mesh->t[fi].v[0]];
//			const Vertex& v1 = mesh->v[mesh->t[fi].v[1]];
//			const Vertex& v2 = mesh->v[mesh->t[fi].v[2]];
//			Vertex intersect; float t, u, v;
//			if (intersect_triangle(center, ray, v0, v1, v2, t, u, v, intersect))
//			{
//				float f = 1 / (intersect - center).length_s();
//				ret.x = ray.x*f;
//				ret.y = ray.y*f;
//				ret.z = ray.z*f;
//				return true;
//			}
//		}
//		return false;
//	}
//	bool Classify_Vertex::test_occlusion(const Vertex& head, const Vertex& tail, float dist)
//	{
//		Vector ray = head - tail; ray.normalize();
//		Vertex center = tail;
//		float step = voxelh->vxsize / 2;
//		float t = step;
//		int last = -1;
//		while (1)
//		{
//			if (t*t > dist)
//				return true;
//			Vertex temp = ray; temp.x *= t; temp.y *= t; temp.z *= t;
//			temp += center;
//			int v = voxelh->vertex_in_which_voxel(temp);
//			if (v < 0) return false;
//			if (v != last)
//			{
//				const Voxel::VoxelType& vxt = voxelh->voxels[v].type;
//				if (vxt == Voxel::boundary)
//				{
//					Vertex vtx;
//					if (check_boundary(v, center, ray, vtx))
//						return false;
//				}
//				//else if (v.type == Voxel::outside)
//				//{
//				//	return true;
//				//}
//			}
//			t += step;
//			last = v;
//		}
//		return true;
//	}
//	void Classify_Vertex::split_mesh_by_vertex()
//	{
//		// init
//		bool find_body = true;
//		for (int idx = 0; idx < (int)mesh->v.size(); idx++) {
//			mesh->color_vertex(idx, preset_color[6]);
//		}
//		// find leaves
//		Int2DVec leaves;
//		find_skeleton_leaves(find_body, leaves);
//		Vtx2DVec skepts(leaves.size());
//		Int3DVec skerings(leaves.size());
//		for (int i = 0; i < (int)leaves.size(); i++) {
//			skerings[i].resize((int)leaves[i].size());
//			for (int j = 0; j < (int)leaves[i].size(); j++) {
//				skepts[i].push_back(voxelh->voxel_center(leaves[i][j]));
//			}
//		}
//		// classify vertex
//		for (int idx = 0; idx < (int)mesh->v.size(); idx++) {
//			float mindist = std::numeric_limits<float>::max(); int minidx = -1, minidx2 = -1;
//			for (int i = 0; i < (int)skepts.size(); i++) {
//				for (int j = 0; j < (int)skepts[i].size(); j++) {
//					float dist = (mesh->v[idx] - skepts[i][j]).length_s();
//					if (dist < mindist) {
//						mindist = dist; minidx = i; minidx2 = j;
//					}
//				}
//			}
//			if (!test_occlusion(skepts[minidx][minidx2], mesh->v[idx], mindist)) {
//				mindist = std::numeric_limits<float>::max(); minidx = -1; minidx2 = -1;
//				for (int i = 0; i < (int)skepts.size(); i++) {
//					for (int j = 0; j < (int)skepts[i].size(); j++) {
//						float dist = (mesh->v[idx] - skepts[i][j]).length_s();
//						if (dist < mindist && test_occlusion(skepts[i][j], mesh->v[idx], dist)) {
//							mindist = dist; minidx = i; minidx2 = j;
//						}
//					}
//				}
//			}
//			if (minidx >= 0 && minidx2 >= 0) {
//				mesh->v[idx].label = minidx;
//				skerings[minidx][minidx2].push_back(idx);
//			}
//		}
//		//// refine
//		//const float limit = (float)0.69;
//		//for (int idx = 0; idx < (int)mesh->v.size(); idx++) {
//		//	int count = 0; int self = mesh->v[idx].label;
//		//	int change = 0;
//		//	for (int i = 0; i < mesh->v[idx].nn; i++) {
//		//		int n = mesh->vv[mesh->v[idx].ns + i];
//		//		if (mesh->v[n].label != self) {
//		//			change = mesh->v[n].label;
//		//			count++;
//		//		}
//		//	}
//		//	if ((float)count / (float)mesh->v[idx].nn >= limit)
//		//		mesh->v[idx].label = change;
//		//}
//		//// display
//		//for (int idx = 0; idx < (int)mesh->v.size(); idx++) {
//		//	mesh->color_vertex(idx, color[mesh->v[idx].label]);
//		//}
//		// compute scalars
//		int gs = 1, sw = 15, pi = 1;
//		int vec_size = (int)((int)leaves[pi].size() / gs);
//		std::vector<float> max_r(vec_size);
//		std::vector<float> min_r(vec_size);
//		std::vector<float> avg_r(vec_size);
//		std::vector<float> ecc(vec_size);
//		std::vector<float> cosines(vec_size);
//		for (int i = 0; i < (int)((int)leaves[pi].size() / gs); i++) {
//			//// center
//			//Vertex cen; cen.x = 0; cen.y = 0; cen.z = 0;
//			//for (int idx = 0; idx < (int)border_points.size(); idx++)
//			//	cen += voxelh->voxel_center(border_points[idx]);
//			//float sizef = (float)border_points.size();
//			//cen.x /= sizef; cen.y /= sizef; cen.z /= sizef;
//			// radius
//			avg_r[i] = 0; max_r[i] = -1; min_r[i] = std::numeric_limits<float>::max();
//			int count_size = 0;
//			for (int j = 0; j < gs; j++) {
//				Vertex cen = skepts[pi][i*gs + j];
//				for (int idx = 0; idx < (int)skerings[pi][i*gs + j].size(); idx++)
//				{
//					float radius = (cen - mesh->v[skerings[pi][i*gs + j][idx]]).length();
//					//float radius = sqrt(voxelh->voxel_distance_s(cen, border_points[idx]));
//					if (radius < min_r[i]) min_r[i] = radius;
//					if (radius > max_r[i]) max_r[i] = radius;
//					avg_r[i] += radius;
//					count_size++;
//				}
//			}
//			if (count_size <= 0) {
//				avg_r[i] = max_r[i] = min_r[i] = 0; ecc[i] = 1;
//			}
//			else {
//				avg_r[i] /= (float)count_size;
//				//avg_r[i] = (min_r[i] + max_r[i]) / (float)2;
//				float df = max_r[i] * max_r[i] - min_r[i] * min_r[i];
//				if (df < (float)0.0001)
//					ecc[i] = 1;
//				else
//					ecc[i] = sqrt(df) / max_r[i];
//			}
//		}
//		// write scalars to txt
//		std::string fname = "e:\\aa"; fname += ('0' + pi); fname += ".txt";
//		std::fstream fs(fname.c_str(), std::ios::out);
//		for (int i = 0; i < (int)((int)leaves[pi].size() / gs); i++)
//			fs << i << "\t" << cosines[i] << "\t" << ecc[i] << "\t" << avg_r[i]
//			<< "\t" << min_r[i] << "\t" << max_r[i] << std::endl;
//		fs.close();
//		//// display
//		//for (int i = 0; i < sw; i++) {
//		//	for (int j = 0; j < gs; j++) {
//		//		for (int k = 0; k < (int)skerings[pi][gs*i + j].size(); k++) {
//		//			mesh->color_vertex(skerings[pi][gs*i + j][k], preset_color[i % 6]);
//		//		}
//		//	}
//		//}
//		for (int k = 0; k < (int)skerings[pi][43].size(); k++) {
//			mesh->color_vertex(skerings[pi][43][k], preset_color[0]);
//		}
//		//split_arm_by_vertex(skerings);
//		//split_head_and_arm_by_vertex(skerings);
//		//split_leg_by_vertex(skerings);
//	}
//	void Classify_Vertex::split_head_and_arm_by_vertex(Int3DVec skerings)
//	{
//		const int skip = 30;
//		// split
//		std::vector<bool> labeled((int)mesh->v.size(), false);
//		int disp[5] = { 0 };
//		for (int idx = 0; idx <= 2; idx++) {
//			bool flag = false; int opp = 0;
//			for (int i = 0; i < (int)skerings[idx].size(); i++) {
//				for (int j = 0; j < (int)skerings[idx][i].size(); j++) {
//					int vidx = skerings[idx][i][j];
//					for (int k = 0; k < mesh->vvn[vidx]; k++) {
//						int n = mesh->vv(vidx, k);
//						if (idx != 0 && mesh->v[n].label == opp && i > skip) {
//							flag = true;
//							disp[idx] = i;
//							break;
//						}
//						if (idx == 0 && mesh->v[n].label == 2) {
//							flag = true;
//							disp[idx] = i;
//							break;
//						}
//					}
//					if (flag)
//						break;
//				}
//				if (flag)
//					break;
//			}
//			// display
//			for (int i = 0; i < disp[idx] + 1; i++) {
//				for (int j = 0; j < (int)skerings[idx][disp[idx] - i].size(); j++) {
//					labeled[skerings[idx][disp[idx] - i][j]] = true;
//					mesh->color_vertex(skerings[idx][disp[idx] - i][j], preset_color[idx]);
//				}
//			}
//			//voxelh->voxels[leaves[corr[idx]][disp]].type = Voxel::maximum;
//		}
//	}
//	void Classify_Vertex::split_arm_by_vertex(Int3DVec skerings)
//	{
//		for (int idx = 0; idx < (int)mesh->v.size(); idx++) {
//			if (mesh->v[idx].code == 1)
//				mesh->color_vertex(idx, preset_color[mesh->v[idx].label]);
//		}
//		for (int idx = 1; idx <= 2; idx++) {
//			int face_num = (int)mesh->t.size();
//			ArrayQueue<int> Q(face_num);
//			mesh->v[skerings[idx][0][0]].code = 1;
//			mesh->color_vertex(skerings[idx][0][0], preset_color[idx]);
//			Q.push(skerings[idx][0][0]);
//			while (!Q.empty()) {
//				int curr = Q.front();
//				Q.pop();
//				for (int i = 0; i < mesh->vvn[curr]; i++) {
//					int n = mesh->vv(curr, i);
//					if (mesh->v[n].code == 0) {
//						mesh->v[n].code = 1;
//						mesh->color_vertex(n, preset_color[idx]);
//						Q.push(n);
//					}
//				}
//			}
//		}
//		//const int skip = 30;
//		//// split
//		//int disp[5] = { 0 };
//		//for (int idx = 1; idx <= 2; idx++) {
//		//	bool flag = false; int opp = 0;
//		//	for (int i = 0; i < (int)skerings[idx].size(); i++) {
//		//		bool inflag = false;
//		//		for (int j = 0; j < (int)skerings[idx][i].size(); j++) {
//		//			int vidx = skerings[idx][i][j];
//		//			if (mesh->m_V[vidx].code == 1) {
//		//				inflag = flag = true;
//		//				//flag = true;
//		//				//disp[idx] = i;
//		//				break;
//		//			}
//		//		}
//		//		if (flag && !inflag) {
//		//			disp[idx] = i;
//		//			break;
//		//		}
//		//	}
//		//	// display
//		//	for (int i = 0; i < disp[idx] + 1; i++) {
//		//		for (int j = 0; j < (int)skerings[idx][disp[idx] - i].size(); j++) {
//		//			mesh->color_vertex(skerings[idx][disp[idx] - i][j], color[idx]);
//		//		}
//		//	}
//		//	//voxelh->voxels[leaves[corr[idx]][disp]].type = Voxel::maximum;
//		//}
//	}
//	void Classify_Vertex::split_leg_by_vertex(Int3DVec skerings)
//	{
//		const int skip = 30;
//		// split
//		std::vector<bool> labeled((int)mesh->v.size(), false);
//		int foot_start = 3;
//		int disp[5] = { 0 };
//		for (int idx = 3; idx < 5; idx++) {
//			bool flag = false;
//			int opp = idx < foot_start ? 0 : 7 - idx;
//			for (int i = 0; i < (int)skerings[idx].size(); i++) {
//				for (int j = 0; j < (int)skerings[idx][i].size(); j++) {
//					int vidx = skerings[idx][i][j];
//					for (int k = 0; k < mesh->vvn[vidx]; k++) {
//						int n = mesh->vv(vidx, k);
//						if (mesh->v[n].label == opp && i > skip) {
//							flag = true;
//							disp[idx] = i;
//							break;
//						}
//					}
//					if (flag)
//						break;
//				}
//				if (flag)
//					break;
//			}
//			// display
//			for (int i = 0; i < disp[idx] + 1; i++) {
//				for (int j = 0; j < (int)skerings[idx][disp[idx] - i].size(); j++) {
//					labeled[skerings[idx][disp[idx] - i][j]] = true;
//					mesh->color_vertex(skerings[idx][disp[idx] - i][j], preset_color[idx]);
//				}
//			}
//			//voxelh->voxels[leaves[corr[idx]][disp]].type = Voxel::maximum;
//		}
//		// A* tracing
//		for (int idx = 3; idx < 5; idx++) {
//			int opp = 7 - idx;
//			for (int it = 0; it < disp[idx] + 1; it++) {
//				int skeidx = disp[idx] - it;
//				for (int i = 0; i < (int)skerings[idx][skeidx].size(); i++) {
//					int vi = skerings[idx][skeidx][i];
//					while (1) {
//						float mindist = std::numeric_limits<float>::max(); int minidx = 0;
//						for (int j = 0; j < mesh->vvn[vi]; j++) {
//							int ni = mesh->vv(vi, j);
//							for (int k = 0; k < disp[opp] + 1; k++) {
//								int oppske = disp[opp] - k;
//								for (int m = 0; m < (int)skerings[opp][oppske].size(); m++) {
//									int vopp = skerings[opp][oppske][m];
//									float dist = (mesh->v[ni] - mesh->v[vopp]).length_s();
//									if (dist < mindist) {
//										minidx = ni;
//										mindist = dist;
//									}
//								}
//							}
//						}
//						if (labeled[minidx] || mesh->v[minidx].label != idx)
//							break;
//						labeled[minidx] = true;
//						mesh->color_vertex(minidx, preset_color[idx]);
//						vi = minidx;
//					}
//				}
//			}
//		}
//		// find holes
//		std::vector<IntVec> holes;
//		std::vector<bool> visited((int)mesh->v.size(), false);
//		for (int idx = 0; idx < (int)mesh->v.size(); idx++) {
//			if (!labeled[idx] && !visited[idx]) {
//				IntVec hole; hole.push_back(idx);
//				std::stack<int> S; visited[idx] = true; S.push(idx);
//				while (!S.empty()) {
//					int curr = S.top(); S.pop();
//					for (int i = 0; i < mesh->vvn[curr]; i++) {
//						int ni = mesh->vv(curr, i);
//						if (!labeled[ni] && !visited[ni]) {
//							S.push(ni); hole.push_back(ni);
//							visited[ni] = true;
//						}
//					}
//				}
//				holes.push_back(hole);
//			}
//		}
//		// fix holes
//		int max_hole = -1, max_size = std::numeric_limits<int>::min();
//		for (int i = 0; i < (int)holes.size(); i++) {
//			if ((int)holes[i].size() > max_size) {
//				max_size = (int)holes[i].size();
//				max_hole = i;
//			}
//		}
//		if (max_hole >= 0) {
//			for (int i = 0; i < (int)holes.size(); i++) {
//				if (i != max_hole) {
//					int seed = holes[i][0]; labeled[seed] = true;
//					mesh->color_vertex(seed, preset_color[mesh->v[seed].label]);
//					std::stack<int> S; S.push(seed);
//					while (!S.empty()) {
//						int curr = S.top(); S.pop();
//						for (int j = 0; j < mesh->vvn[curr]; j++) {
//							int ni = mesh->vv(curr, j);
//							if (!labeled[ni]) {
//								S.push(ni); labeled[ni] = true;
//								mesh->color_vertex(ni, preset_color[mesh->v[ni].label]);
//							}
//						}
//					}
//				}
//			}
//		}
//	}
//	// split mesh by border voxel
//	void Classify_Vertex::split_mesh_by_border_voxel()
//	{
//		bool find_body = false;
//		// collect skeleton points and end points, also label their type
//		IntVec end_points;
//		IntVec skeleton_points;
//		search_and_label_skeleton_points(skeleton_points, end_points);
//		// find leaves
//		int end_num = (int)end_points.size();
//		if (find_body)
//			end_num = (int)end_points.size() + 1;
//		std::vector<IntVec> leaves(end_num);
//		for (int idx = 0; idx < (int)end_points.size(); idx++)
//		{
//			voxelh->voxels_set_unvisited();
//			int current = end_points[idx];
//			while (1)
//			{
//				voxelh->voxels[current].visited = true;
//				bool stop = false;
//				Voxel& vx = voxelh->voxels[current];
//				if (voxelh->voxels[current].label > 2)
//					break;
//				leaves[idx].push_back(current);
//				int next = -1;
//				for (int i = 0; i < 27; i++)
//				{
//					int ni = -1;
//					if (voxelh->voxel_neighbor(current, i, ni))
//					{
//						if (voxelh->is_skeleton(ni))
//						{
//							if (voxelh->voxels[ni].label > 2)
//								stop = true;
//							if (!voxelh->voxels[ni].visited)
//								next = ni;
//						}
//					}
//				}
//				if (stop)
//					break;
//				if (next == -1)
//				{
//					std::cout << "error when finding leaf\n"; break;
//				}
//				current = next;
//			}
//		}
//		// find body
//		if (find_body) {
//			voxelh->voxels_set_unvisited();
//			for (int i = 0; i < (int)leaves.size(); i++) {
//				for (int j = 0; j < (int)leaves[i].size(); j++) {
//					voxelh->voxels[leaves[i][j]].visited = true;
//				}
//			}
//			int bodyidx = (int)end_points.size();
//			for (int idx = 0; idx < (int)voxelh->voxels.size(); idx++) {
//				if (voxelh->is_skeleton(idx) && !voxelh->voxels[idx].visited && voxelh->voxels[idx].label <= 2)
//					leaves[bodyidx].push_back(idx);
//			}
//		}
//		std::vector<std::vector<Vertex>> skepts(end_num);
//		for (int i = 0; i < (int)leaves.size(); i++) {
//			for (int j = 0; j < (int)leaves[i].size(); j++) {
//				skepts[i].push_back(voxelh->voxel_center(leaves[i][j]));
//			}
//		}
//		for (int idx = 0; idx < (int)mesh->v.size(); idx++)
//		{
//			float mindist = std::numeric_limits<float>::max(); int minidx = -1;
//			for (int i = 0; i < (int)skepts.size(); i++) {
//				for (int j = 0; j < (int)skepts[i].size(); j++) {
//					float dist = (mesh->v[idx] - skepts[i][j]).length_s();
//					if (dist < mindist)
//					{
//						if (test_occlusion(skepts[i][j], mesh->v[idx], dist)) {
//							mindist = dist;
//							minidx = i;
//						}
//					}
//				}
//			}
//			mesh->color_vertex(idx, preset_color[minidx]);
//		}
//	}
//}