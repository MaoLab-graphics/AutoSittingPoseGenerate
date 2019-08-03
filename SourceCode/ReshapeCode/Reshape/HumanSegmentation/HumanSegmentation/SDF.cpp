// SDF

#include "SDF.h"
#include "Mesh.h"
#include "Voxel.h"
#include <algorithm>
#include <fstream>

namespace ManSeg {

	// -------------------- Global Function --------------------

	void calculate_sdf_for_mesh(float angle, string in_file, string out_file) {
		print_time_message("start sdf");
		Mesh mesh;
		mesh.read_from_file(in_file);
		VoxelHandler voxel_handler(&mesh, 10);
		Shape_Diameter_Function sdf(&mesh, &voxel_handler);
		sdf.compute_sdf(30, true, angle, out_file);
		//mesh.write_to_file(out_file);
		print_time_message("end sdf");
	}

	// -------------------- Cone Ray --------------------

	void Cone_Ray::sampling(bool uniform, int size) {
		weighted_samples.resize(size);
		// sample in a disk (radius = 1)
		if (uniform) {
			for (int i = 0; i < size; ++i) {
				float Q = i * GOLDEN_RATIO * PI;
				float R = std::sqrt((float)i / (float)size);
				float weight = exp((float)-0.5 * (std::pow(R / ANGLE_ST_DEV_DIVIDER, 2)));
				weighted_samples[i].w = weight; weighted_samples[i].x = R * cos(Q); weighted_samples[i].y = R * sin(Q);
			}
		}
		else {
			const float custom_power = (float)1.0;
			// it determines how much sampling is biased to center
			// for uniform result one can use 0.5 (i.e. sqrt)
			for (int i = 0; i < size; ++i) {
				float Q = i * GOLDEN_RATIO * PI;
				float R = std::pow((float)i / (float)size, custom_power);
				// use uniform weigths, since we already give importance to locations that are close to center.
				weighted_samples[i].w = 1; weighted_samples[i].x = R * cos(Q); weighted_samples[i].y = R * sin(Q);
			}
		}
	}
	Vector Cone_Ray::gen_ray(const Vector& origin, Vector base[2], int index) const {
		Vector ret = origin;
		ret += (base[0] * weighted_samples[index].x + base[1] * weighted_samples[index].y);
		ret.normalize();
		return ret;
	}
	float Cone_Ray::compute_sdf_value_from_samples(FltVec& ray_lens) const {
		// special cases
		int accepted_ray_count = (int)ray_lens.size();
		if (accepted_ray_count == 0) return (float)0.0;
		else if (accepted_ray_count == 1) return ray_lens[0];
		// calculate median SDF
		int half_ray_count = accepted_ray_count / 2;
		std::nth_element(ray_lens.begin(), ray_lens.begin() + half_ray_count, ray_lens.end());
		float median_sdf = ray_lens[half_ray_count];
		if (accepted_ray_count % 2 == 0) {
			median_sdf += *std::max_element(ray_lens.begin(), ray_lens.begin() + half_ray_count);
			median_sdf /= (float)2.0;
		}
		// calculate median absolute deviation
		FltVec absolute_deviation;
		absolute_deviation.reserve(accepted_ray_count);
		for (auto it = ray_lens.begin(); it != ray_lens.end(); ++it) {
			absolute_deviation.push_back(std::abs(*it - median_sdf));
		}
		std::nth_element(absolute_deviation.begin(),
			absolute_deviation.begin() + half_ray_count, absolute_deviation.end());
		float median_deviation = absolute_deviation[half_ray_count];
		if (accepted_ray_count % 2 == 0) {
			median_deviation += *std::max_element(absolute_deviation.begin(), absolute_deviation.begin() + half_ray_count);
			median_deviation /= (float)2.0;
		}
		// calculate sdf, accept rays if ray_dist - median < (median_deviation * Factor)
		float total_weights = (float)0.0, total_distance = (float)0.0;
		int count = 0;
		for (auto it = ray_lens.begin(); it != ray_lens.end(); ++it) {
			if (std::abs(*it - median_sdf) <= (median_deviation * OUTLIER_RATIO)) {
				float weight = weighted_samples[count].w;
				total_distance += *it * weight;
				total_weights += weight;
			}
			count++;
		}
		if (total_distance == (float)0.0) {
			std::cout << "unexpected issue : no ray is accepted, return median\n";
			return median_sdf;  // no ray is accepted, return median.
		}
		else return total_distance / total_weights;
	}

	// -------------------- Shape Diameter Function --------------------

	Shape_Diameter_Function::Shape_Diameter_Function(const Mesh* in_mesh, VoxelHandler* vxh)
		: mesh(in_mesh), voxelh(vxh) {}
	// sdf
	void Shape_Diameter_Function::log_normalize_sdf_values() {
		int face_num = (int)mesh->t.size();
		FltVec normalized_sdf_values;
		normalized_sdf_values.reserve(face_num);
		for (int i = 0; i < face_num; ++i) {
			float log_normalized = std::log(sdf_values[i] * NORMALIZATION_ALPHA + 1);
			log_normalized /= std::log(NORMALIZATION_ALPHA + 1);
			normalized_sdf_values.push_back(log_normalized);
		}
		std::memcpy(&sdf_values[0], &normalized_sdf_values[0], face_num*sizeof(float));
	}
	void Shape_Diameter_Function::compute_sdf(int ray_num, bool uniform, float angle, std::string out_file) {
		int face_num = (int)mesh->t.size();
		float rad = (angle * PI) / (float)180; // default : 60
		float tan_rad = tan(rad);
		float face_normal_len = (float)1 / (float)tan_rad;
		face_normals.resize(face_num);
		face_centers.resize(face_num);
		face_normal_lens.resize(face_num);
		sdf_values.resize(face_num);
		for (int i = 0; i < face_num; ++i) {
			make_normal(i, face_normals[i]); face_normals[i] *= face_normal_len;
			face_normal_lens[i] = face_normals[i].length();
			face_centers[i] = triangle_center(i);
		}
		Vector base[2]; float ret_ray_len;
		Cone_Ray cone_ray(uniform, ray_num);
		for (int i = 0; i < face_num; ++i) {
			if (i % 1000 == 0) std::cout << i << std::endl;
			FltVec lens; lens.reserve(ray_num);
			make_plane_base(i, base);
			for (int j = 0; j < ray_num; ++j) {
				if (compute_ray_length(i, cone_ray.gen_ray(face_normals[i], base, j), ret_ray_len))
					lens.push_back(ret_ray_len);
			}
			sdf_values[i] = cone_ray.compute_sdf_value_from_samples(lens);
		}
		// write
		std::fstream fs("./text.txt", std::ios::out);
		for (int i = 0; i < face_num; ++i) {
			fs << i << "   " << sdf_values[i] << '\n';
		}
		fs.close();
		// write binary
		std::string out_path = "./"; out_path += out_file;
		std::FILE* fp = std::fopen(out_path.c_str(), "wb");
		std::fwrite(&sdf_values[0], sizeof(float)*face_num, 1, fp);
		std::fclose(fp);
		log_normalize_sdf_values();
		out_path = "./n"; out_path += out_file;
		fp = std::fopen(out_path.c_str(), "wb");
		std::fwrite(&sdf_values[0], sizeof(float)*face_num, 1, fp);
		std::fclose(fp);
	}
	bool Shape_Diameter_Function::in_same_direction(int face1, int face2) const {
		float cosine = face_normals[face1].dot(face_normals[face2]);
		cosine /= (face_normal_lens[face1] * face_normal_lens[face2]);
		if (cosine > 0) return true;
		else return false;
	}
	bool Shape_Diameter_Function::check_boundary(const Voxel& vx, int face, const Vertex& ray, float& ret) const {
		const Vertex& center = face_centers[face];
		for (int i = 0; i < vx.face_num; i++)
		{
			int fi = voxelh->boundary_faces[vx.face_start + i];
			if (intersect_triangle(center, ray, fi, ret))
			{
				if (ret < 0 || in_same_direction(face, fi))
					return false;
				return true;
			}
		}
		return false;
	}
	bool Shape_Diameter_Function::intersect_triangle(const Vertex& orig, const Vertex& dir, int face, float& t) const {
		// vertices of triangle
		const Vertex& v0 = mesh->v[mesh->t[face].v[0]];
		const Vertex& v1 = mesh->v[mesh->t[face].v[1]];
		const Vertex& v2 = mesh->v[mesh->t[face].v[2]];
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
	bool Shape_Diameter_Function::compute_ray_length(int face, const Vector& ray, float& ret) const {
		const Vertex& center = face_centers[face];
		//// using slow
		//float mt = DECIMAL_MAX;
		//for (int i = 0; i < (int)mesh->t.size(); ++i) {
		//	const Vertex& v0 = mesh->v[mesh->t[i].v[0]];
		//	const Vertex& v1 = mesh->v[mesh->t[i].v[1]];
		//	const Vertex& v2 = mesh->v[mesh->t[i].v[2]];
		//	float t;
		//	if (intersect_triangle(center, ray, i, t))
		//	{
		//		if (!in_same_direction(face, i)) {
		//			if (t < mt) mt = t;
		//		}
		//	}
		//}
		// using fast
		float step = voxelh->vxsize / 2;
		float t = step;
		int last = -1;
		while (1)
		{
			Vertex temp = ray; temp *= t; temp += center;
			int v = voxelh->vertex_in_which_voxel(temp);
			if (v < 0) return false;
			if (v != last)
			{
				const Voxel& vx = voxelh->voxels[v];
				if (vx.type == Voxel::outside)
				{
					//std::cout << "couldn't find intersection\n";
					//return compute_force_slow(center, ray);
					return false;
				}
				else if (vx.type == Voxel::boundary)
				{
					if (check_boundary(vx, face, ray, ret))
						return true;
				}
			}
			t += step;
			last = v;
		}
	}
	void Shape_Diameter_Function::make_plane_base(int face, Vector output[2]) const {
		const Vertex& p1 = mesh->v[mesh->t[face].v[0]];
		const Vertex& p2 = mesh->v[mesh->t[face].v[1]];
		const Vertex& p3 = mesh->v[mesh->t[face].v[2]];
		Vector v1 = p2 - p1;
		Vector v2 = p3 - p1;
		output[0] = p3 - p2; output[0].normalize();
		float t = -(v2.x*output[0].x + v2.y*output[0].y + v2.z*output[0].z);
		t /= (v1.x*output[0].x + v1.y*output[0].y + v1.z*output[0].z);
		v2 *= t;
		output[1] = v1 + v2; output[1].normalize();
	}
	void Shape_Diameter_Function::make_normal(int face, Vector& normal) const {
		const Vertex& p1 = mesh->v[mesh->t[face].v[0]];
		const Vertex& p2 = mesh->v[mesh->t[face].v[1]];
		const Vertex& p3 = mesh->v[mesh->t[face].v[2]];
		Vector na = p3 - p1;
		Vector nb = p2 - p1;
		normal.x = na.y * nb.z - na.z * nb.y;
		normal.y = na.z * nb.x - na.x * nb.z;
		normal.z = na.x * nb.y - na.y * nb.x;
		normal.normalize();
	}
	Vertex Shape_Diameter_Function::triangle_center(int face) const {
		Vertex ret; ret.x = 0; ret.y = 0; ret.z = 0;
		for (int i = 0; i < 3; i++)
			ret += mesh->v[mesh->t[face].v[i]];
		ret.x /= (float)3; ret.y /= (float)3; ret.z /= (float)3;
		return ret;
	}
}