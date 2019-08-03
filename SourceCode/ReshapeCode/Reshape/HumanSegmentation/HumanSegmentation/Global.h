// Global.h

#pragma once

#ifndef GLOBAL_H
#define GLOBAL_H

#include <iostream>
#include <limits>

namespace ManSeg {

	// -------------------- Types --------------------

	using std::string;

	// -------------------- Classes --------------------

	struct Vector;
	typedef Vector Vertex;
	struct Quaternion;
	struct Plane;
	struct OpenMeshTriangleMesh;
	typedef OpenMeshTriangleMesh Mesh;
	struct Voxel;
	class VoxelHandler;
	
	// -------------------- Constants --------------------

	const static int NONE = -1;
	const static float PI = (float)3.14159265;
	const static int INTEGER_MAX = std::numeric_limits<int>::max();
	const static int INTEGER_MIN = std::numeric_limits<int>::min();
	const static float DECIMAL_SMALLEST = std::numeric_limits<float>::min();
	const static float DECIMAL_MAX = std::numeric_limits<float>::max();
	const static float DECIMAL_MIN = -std::numeric_limits<float>::max();

	// -------------------- Function --------------------

	string get_current_system_time();
	void print_time_message(const string& mes);

	// -------------------- Class --------------------

	template<class T>
	struct ArrayQueue {
		T* data; int s, e, m, size;
		ArrayQueue(int max_size) { data = new T[max_size]; size = s = 0; e = -1; m = max_size - 1; }
		~ArrayQueue() { delete[] data; }
		void push(const T& t) { if ((++e) >= m) e = 0; data[e] = t; ++size; }
		void pop() { if ((++s) >= m) s = 0; --size; }
		T& front() { return data[s]; }
		bool empty() { return size <= 0; }
		void clear() { size = s = 0; e = -1; }
	};
	template<class T>
	struct ArrayStack {
		T* data; int size, m;
		ArrayStack(int max_size) { m = max_size; data = new T[max_size]; size = 0; }
		~ArrayStack() { delete[] data; }
		void push(const T& t) { data[size] = t; ++size; if (size > m) std::cout << "reach max size !"; }
		void pop() { --size; }
		T& top() { return data[size - 1]; }
		bool empty() { return size <= 0; }
		void clear() { size = 0; }
	};
}

#endif