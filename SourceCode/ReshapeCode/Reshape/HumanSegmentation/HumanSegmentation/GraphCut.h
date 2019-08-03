// Copyright (c) 2014  GeometryFactory Sarl (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
// You can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

#pragma once

#ifndef GRAPHCUT_H
#define GRAPHCUT_H

#include "Graph.h"
#include <vector>

namespace ManSeg
{
	/**
	* @brief Implements alpha-expansion graph cut algorithm.
	*
	* For underlying max-flow algorithm, it uses the MAXFLOW software implemented by Boykov & Kolmogorov.
	*  Also no pre-allocation is made.
	*/
	class Alpha_expansion_graph_cut_boykov_kolmogorov
	{
	public:
		/**
		* Applies alpha-expansion graph-cut for energy minimization.
		* @param edges contains incident vertex-id pairs for each edge (vertex-ids should be between [0, number of vertices -1])
		* @param edge_weights contains weights for each edge in @a edges (correspondence according to order)
		* @param probability_matrix contains responsibility of the center on the vertex probability[center][vertex]
		* @param[in, out] labels as input it contains initial labeling of vertices (i.e. a center-id between [0, number of centers -1]),
		* and as output it returns final labeling of vertices
		* @return result of energy function
		*/
		double operator()(const std::vector<std::pair<std::size_t, std::size_t> >&
			edges,
			const std::vector<double>& edge_weights,
			const std::vector<std::vector<double> >& probability_matrix,
			std::vector<std::size_t>& labels) const {
			const double tolerance = 1e-10;

			double min_cut = (std::numeric_limits<double>::max)();

#ifdef CGAL_SEGMENTATION_BENCH_GRAPHCUT
			double vertex_creation_time, edge_creation_time, cut_time;
			vertex_creation_time = edge_creation_time = cut_time = 0.0;
#endif

			std::vector<Graph::node_id> inserted_vertices;
			inserted_vertices.resize(labels.size());
			bool success;
			do {
				success = false;
				std::size_t alpha = 0;
				for (std::vector<std::vector<double> >::const_iterator it =
					probability_matrix.begin();
					it != probability_matrix.end(); ++it, ++alpha) {
					Graph graph;
#ifdef CGAL_SEGMENTATION_BENCH_GRAPHCUT
					Timer timer;
					timer.start();
#endif
					// For E-Data
					// add every facet as a vertex to graph, put edges to source-sink vertices
					for (std::size_t vertex_i = 0; vertex_i < labels.size(); ++vertex_i) {
						Graph::node_id new_vertex = graph.add_node();
						inserted_vertices[vertex_i] = new_vertex;

						double source_weight = probability_matrix[alpha][vertex_i];
						// since it is expansion move, current alpha labeled vertices will be assigned to alpha again,
						// making sink_weight 'infinity' guarantee this.
						double sink_weight = (labels[vertex_i] == alpha) ?
							(std::numeric_limits<double>::max)()
							: probability_matrix[labels[vertex_i]][vertex_i];
						graph.add_tweights(new_vertex, source_weight, sink_weight);
					}
#ifdef CGAL_SEGMENTATION_BENCH_GRAPHCUT
					vertex_creation_time += timer.time();
					timer.reset();
#endif
					// For E-Smooth
					// add edge between every vertex,
					std::vector<double>::const_iterator weight_it = edge_weights.begin();
					for (std::vector<std::pair<std::size_t, std::size_t> >::const_iterator edge_it =
						edges.begin(); edge_it != edges.end();
						++edge_it, ++weight_it) {
						Graph::node_id v1 = inserted_vertices[edge_it->first];
						Graph::node_id v2 = inserted_vertices[edge_it->second];
						std::size_t label_1 = labels[edge_it->first], label_2 = labels[edge_it->second];
						if (label_1 == label_2) {
							if (label_1 != alpha) {
								graph.add_edge(v1, v2, *weight_it, *weight_it);
							}
						}
						else {
							Graph::node_id inbetween = graph.add_node();

							double w1 = (label_1 == alpha) ? 0 : *weight_it;
							double w2 = (label_2 == alpha) ? 0 : *weight_it;
							graph.add_edge(inbetween, v1, w1, w1);
							graph.add_edge(inbetween, v2, w2, w2);

							graph.add_tweights(inbetween, 0.0, *weight_it);
						}
					}
#ifdef CGAL_SEGMENTATION_BENCH_GRAPHCUT
					edge_creation_time += timer.time();
					timer.reset();
#endif

					double flow = graph.maxflow();
#ifdef CGAL_SEGMENTATION_BENCH_GRAPHCUT
					cut_time += timer.time();
#endif

					if (min_cut - flow < flow * tolerance) {
						continue;
					}

					min_cut = flow;
					success = true;
					//update labeling
					for (std::size_t vertex_i = 0; vertex_i < labels.size(); ++vertex_i) {
						if (labels[vertex_i] != alpha
							&& graph.what_segment(inserted_vertices[vertex_i]) == Graph::SINK) {
							labels[vertex_i] = alpha;
						}
					}
				}
			} while (success);

#ifdef CGAL_SEGMENTATION_BENCH_GRAPHCUT
			CGAL_TRACE_STREAM << "vertex creation time: " << vertex_creation_time <<
				std::endl;
			CGAL_TRACE_STREAM << "edge creation time: " << edge_creation_time << std::endl;
			CGAL_TRACE_STREAM << "max flow algorithm time: " << cut_time << std::endl;
#endif
			return min_cut;
		}
	};
}

#endif