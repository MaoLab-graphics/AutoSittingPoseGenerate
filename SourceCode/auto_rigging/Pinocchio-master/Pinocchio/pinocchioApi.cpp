/*  This file is part of the Pinocchio automatic rigging library.
    Copyright (C) 2007 Ilya Baran (ibaran@mit.edu)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "pinocchioApi.h"
#include "debugging.h"
#include <fstream>

//ostream *Debugging::outStream = new ofstream();
ostream *Debugging::outStream = &std::cout;

#pragma pack(1)

void write_ske(const PinocchioOutput& out, string ply_path, const Mesh& mesh) {
	struct PlyPart {
		double x; double y; double z; unsigned char r; unsigned char g; unsigned char b; unsigned char a;
	};
	int size = (int)out.embedding.size();
	char t[256];
	sprintf(t, "%d", size);
	string size_str = t;
	FILE* fp = fopen(ply_path.c_str(), "wb");
	string header = "ply\nformat binary_little_endian 1.0\ncomment VCGLIB generated\nelement vertex ";
	header += size_str;
	header += "\nproperty double x\nproperty double y\nproperty double z\n";
	header += "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";
	header += "element face 0\nproperty list uchar int vertex_indices\nend_header\n";
	int len = (int)strlen(header.c_str());
	fwrite(header.c_str(), len, 1, fp);
	PlyPart* ply = new PlyPart[size];
	for (int i = 0; i < size; ++i) {
		auto pt = out.embedding[i];
		ply[i].x = (double)(pt[0] - mesh.toAdd[0]) / mesh.scale;
		ply[i].y = (double)(pt[1] - mesh.toAdd[1]) / mesh.scale;
		ply[i].z = (double)(pt[2] - mesh.toAdd[2]) / mesh.scale;
		ply[i].r = 0;
		ply[i].g = 0;
		ply[i].b = 0;
		ply[i].a = 255;
		std::cout << ply[i].x << '\t' << ply[i].y << '\t' << ply[i].z << std::endl;
	}
	fwrite(ply, sizeof(PlyPart)*size, 1, fp);
	delete[] ply;
	fclose(fp);
}
void write_mesh(const PinocchioOutput& out, string out_path, string in_path, const Mesh& mesh) {
	struct Vertex { float x, y, z; unsigned char r, g, b, a; };
	struct Face { unsigned char n; int v[3]; };
	vector<Vertex> v;
	vector<Face> f;
	int vn, fn, en;
	std::fstream fs(in_path, std::ios::in);
	string format;
	fs >> format >> vn >> fn >> en;
	v.resize(vn);
	f.resize(fn);
	for (int i = 0; i < vn; ++i) {
		fs >> v[i].x >> v[i].y >> v[i].z;
	}
	for (int i = 0; i < fn; ++i) {
		int t;
		fs >> t >> f[i].v[0] >> f[i].v[1] >> f[i].v[2];
		f[i].n = t;
	}

	char t[256]; sprintf(t, "%d", vn); string vn_str = t;
	FILE* fp = fopen(out_path.c_str(), "wb");
	string header = "ply\nformat binary_little_endian 1.0\ncomment VCGLIB generated\nelement vertex ";
	header += vn_str;
	header += "\nproperty float x\nproperty float y\nproperty float z\n";
	header += "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";
	char tt[256]; sprintf(tt, "%d", fn); string fn_str = tt;
	header += "element face ";
	header += fn_str;
	header += "\nproperty list uchar int vertex_indices\nend_header\n";
	int len = (int)strlen(header.c_str());
	fwrite(header.c_str(), len, 1, fp);
	for (int i = 0; i < vn; ++i) {
		auto pt = mesh.vertices[i].pos;
		v[i].x = (double)(pt[0] - mesh.toAdd[0]) / mesh.scale;
		v[i].y = (double)(pt[1] - mesh.toAdd[1]) / mesh.scale;
		v[i].z = (double)(pt[2] - mesh.toAdd[2]) / mesh.scale;
		v[i].r = 0;
		v[i].g = 0;
		v[i].b = 0;
		v[i].a = 255;
	}
	fwrite(&v[0], sizeof(Vertex)*vn, 1, fp);
	fwrite(&f[0], sizeof(Face)*fn, 1, fp);
	fclose(fp);
}
void write_weight(const PinocchioOutput& out, string out_path, string in_path, int part) {
	if (part <= 0 || part > 17) return;
	char s[256]; sprintf(s, "%d", part); string part_str = s;
	out_path = part_str + out_path;
	struct Vertex { float x, y, z; unsigned char r, g, b, a; };
	struct Face { unsigned char n; int v[3]; };
	vector<Vertex> v;
	vector<Face> f;
	int vn, fn, en;
	std::fstream fs(in_path, std::ios::in);
	string format;
	fs >> format >> vn >> fn >> en;
	v.resize(vn);
	f.resize(fn);
	for (int i = 0; i < vn; ++i) {
		fs >> v[i].x >> v[i].y >> v[i].z;
	}
	for (int i = 0; i < fn; ++i) {
		int t;
		fs >> t >> f[i].v[0] >> f[i].v[1] >> f[i].v[2];
		f[i].n = t;
	}

	char t[256]; sprintf(t, "%d", vn); string vn_str = t;
	FILE* fp = fopen(out_path.c_str(), "wb");
	string header = "ply\nformat binary_little_endian 1.0\ncomment VCGLIB generated\nelement vertex ";
	header += vn_str;
	header += "\nproperty float x\nproperty float y\nproperty float z\n";
	header += "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";
	char tt[256]; sprintf(tt, "%d", fn); string fn_str = tt;
	header += "element face ";
	header += fn_str;
	header += "\nproperty list uchar int vertex_indices\nend_header\n";
	int len = (int)strlen(header.c_str());
	fwrite(header.c_str(), len, 1, fp);
	for (int i = 0; i < vn; ++i) {
		v[i].r = (int)(out.attachment->getWeights(i)[part - 1] * (double)255);
		v[i].g = 0;
		v[i].b = 0;
		v[i].a = 255;
	}
	fwrite(&v[0], sizeof(Vertex)*vn, 1, fp);
	fwrite(&f[0], sizeof(Face)*fn, 1, fp);
	fclose(fp);
}
void write_ske_data(const PinocchioOutput& out, string ply_path, const Mesh& mesh) {
	int ske_size = 18;
	int size = mesh.vertices.size();
	FILE* fp = fopen(ply_path.c_str(), "wb");
	fwrite(&ske_size, sizeof(int), 1, fp);
	fwrite(&size, sizeof(int), 1, fp);
	struct Vertex { float x, y, z; };
	vector<Vertex> ske(ske_size);
	for (int i = 0; i < ske_size; ++i) {
		auto pt = out.embedding[i];
		ske[i].x = (double)(pt[0] - mesh.toAdd[0]) / mesh.scale;
		ske[i].y = (double)(pt[1] - mesh.toAdd[1]) / mesh.scale;
		ske[i].z = (double)(pt[2] - mesh.toAdd[2]) / mesh.scale;
	}
	fwrite(&ske[0], sizeof(Vertex) * ske_size, 1, fp);
	vector<double> w(size * (ske_size - 1));
	for (int i = 0; i < size; ++i) {
		auto ws = out.attachment->getWeights(i);
		for (int j = 0; j < ske_size - 1; ++j) {
			w[i * (ske_size - 1) + j] = ws[j];
		}
	}
	fwrite(&w[0], sizeof(double) * (ske_size - 1) * size, 1, fp);
	fclose(fp);
}

PinocchioOutput autorig(const Skeleton &given, const Mesh &m, string path)
{
    int i;
    PinocchioOutput out;

    Mesh newMesh = prepareMesh(m);

    if(newMesh.vertices.size() == 0)
        return out;

    TreeType *distanceField = constructDistanceField(newMesh);

    //discretization
    vector<Sphere> medialSurface = sampleMedialSurface(distanceField);

    vector<Sphere> spheres = packSpheres(medialSurface);

    PtGraph graph = connectSamples(distanceField, spheres);

    //discrete embedding
    vector<vector<int> > possibilities = computePossibilities(graph, spheres, given);

    //constraints can be set by respecifying possibilities for skeleton joints:
    //to constrain joint i to sphere j, use: possiblities[i] = vector<int>(1, j);

    vector<int> embeddingIndices = discreteEmbed(graph, spheres, given, possibilities);

    if(embeddingIndices.size() == 0) { //failure
        delete distanceField;
        return out;
    }

    vector<Vector3> discreteEmbedding = splitPaths(embeddingIndices, graph, given);

    //continuous refinement
    vector<Vector3> medialCenters(medialSurface.size());
    for(i = 0; i < (int)medialSurface.size(); ++i)
        medialCenters[i] = medialSurface[i].center;

    out.embedding = refineEmbedding(distanceField, medialCenters, discreteEmbedding, given);

    //attachment
    VisTester<TreeType> *tester = new VisTester<TreeType>(distanceField);
    out.attachment = new Attachment(newMesh, given, out.embedding, tester);

	// output
	vector<Transform<>> trans(17);
	auto rot = Quaternion<>(Vector3(0, 1, 0), 3.14 / 4);
	auto parent = out.embedding[0];
	auto move = parent - rot * parent;
	trans[12 - 1] = Transform<>(rot, 1., move);
	//newMesh = out.attachment->deform(newMesh, trans);
	write_ske(out, "ske.ply", newMesh);
	//write_mesh(out, "mesh.ply", IN_MESH_PATH, newMesh);
	for (int i = 1; i < 18; ++i) write_weight(out, "mesh.ply", path, i);
	write_ske_data(out, "skedata.bin", newMesh);

    //cleanup
    delete tester;
    delete distanceField;

    return out;
}

