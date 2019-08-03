// Functions

#pragma once

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <string>

namespace ManSeg {

	// -------------------- Constants --------------------



	// -------------------- Function --------------------

	void split_mesh(float voxel_size, const std::string& filename);
	void transfer_coordinate(const std::string& in_file, const std::string& out_file);
	void extract_skeleton(float voxel_size, const std::string& in_file);
	void hks_feature_points(const std::string& in_file, const std::string& out_file);
	void compute_landmark_total_error();
	void deform_to_sitting_pose(int personNum, int chairnum);
}

#endif