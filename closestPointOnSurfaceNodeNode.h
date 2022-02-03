#ifndef _closestPointOnSurfaceNodeNode
#define _closestPointOnSurfaceNodeNode
//
// Copyright (C) 
// 
// File: closestPointOnSurfaceNodeNode.h
//
// Dependency Graph Node: closestPointOnSurfaceNode
//
// Author: Maya Plug-in Wizard 2.0
//

#include <maya/MPxNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MTypeId.h>
#include <maya/MFnMesh.h>
#include <maya/MUIntArray.h>
#include <maya/MFloatPointArray.h>
#include <vector>
#include <unordered_map>


struct AABB
{
	int id;
	MUintArray ids;
	MUintArray children;
	int parent;
	int octant;
	MVector centre() { return pA + (pB - pA) / 2.0; }
	MVector pA; // bottom left
	MVector pB; // top right
};


class Octree
{
public:
	Octree() {};
	Octree(MFloatPointArray& vertsPoints, bool precomp = false);
	Octree(MFloatPointArray& triPoints, MFloatPointArray& vertsPoints, bool precomp = false);
	~Octree() {};
	int numCells() { return cells.size(); };
	void getClosestCellIds(MVector& src, int startId, AABB& test_box, MUintArray &out_arr);
	void getClosestCellIds(MVector& src, int startId, float max_dist, MUintArray &out_arr);
	int getClosestChild(MVector &src, AABB *curr_cell);
	bool intersectBBox(AABB* curr_cell, AABB* test_box);
	bool isCellInBox(AABB *curr_cell, AABB *test_box);
	void subdivCell(int cell_id, bool recursive = false);
	bool intersectSphere(MVector &src, AABB* curr_cell, float max_dist);
	MUintArray getClosestCells(MVector &src, float max_dist);
	MVector getMostDistantCorner(MVector &src, AABB *curr_cell);
	std::unordered_map<int, MUintArray> depth_map;
	int maxElems;

	std::vector<AABB> cells;

private:
	float bestLength;
	MFloatPointArray *ptr_points;
};

 
class closestPointOnSurfaceNode : public MPxNode
{
public:
						closestPointOnSurfaceNode();
	virtual				~closestPointOnSurfaceNode(); 

	virtual MStatus		compute( const MPlug& plug, MDataBlock& data );

	static  void*		creator();
	static  MStatus		initialize();
	MVector computePointTriangleIntersect(MVector& src, MVector& vtxA, MVector& vtxB, MVector& vtxC, MVector& tNorm);
	void precompTriCentres(MFloatPointArray& vertList, MIntArray& triVerts);

public:

	// There needs to be a MObject handle declared for each attribute that
	// the node will have.  These handles are needed for getting and setting
	// the values later.
	//
	static  MObject		inputGeo;
	static  MObject		inputPoint;
	static  MObject		inputPointX;
	static  MObject		inputPointY;
	static  MObject		inputPointZ;
	static  MObject		output;	
	static  MObject		outputX;
	static  MObject		outputY;
	static  MObject		outputZ;
	static  MObject		output2;
	static  MObject		output2X;
	static  MObject		output2Y;
	static  MObject		output2Z;
	static  MObject		output3;
	static  MObject		output3X;
	static  MObject		output3Y;
	static  MObject		output3Z;

	MFloatPointArray	triCentres;
	MFloatPointArray	verts;

	Octree otree_tris;
	Octree otree_verts;
	bool initTree = true;


	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId		id;
};

#endif
