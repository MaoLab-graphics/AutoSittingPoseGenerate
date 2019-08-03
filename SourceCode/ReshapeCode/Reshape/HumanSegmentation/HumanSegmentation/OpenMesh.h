// OpenMesh library

#pragma once

#ifndef OPENMESH_H
#define OPENMESH_H

// -------------------- OpenMesh --------------------

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

// -------------------- Types --------------------

typedef OpenMesh::TriMesh_ArrayKernelT<> OMTriangleMesh;
typedef OpenMesh::PolyMesh_ArrayKernelT<> OMPolygonMesh;
typedef OpenMesh::IO::Options Options;
typedef OMTriangleMesh::Color Color;

#endif