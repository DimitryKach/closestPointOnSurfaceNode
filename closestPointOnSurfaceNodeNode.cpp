//
// Copyright (C) 
// 
// File: closestPointOnSurfaceNodeNode.cpp
//
// Dependency Graph Node: closestPointOnSurfaceNode
//
// Description: this node take as input a 3D point in world space, and an input mesh, and outputs the closest point on the mesh surface.
//
// Author: Dimitry Kachkovski
//

#include "closestPointOnSurfaceNodeNode.h"

#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>

#include <maya/MGlobal.h>
#include <limits>

#include <chrono>
using namespace std::chrono;

// You MUST change this to a unique value!!!  The id is a 32bit value used
// to identify this type of node in the binary file format.  
//
MTypeId     closestPointOnSurfaceNode::id(0x5ffff);

// Maya attributes
MObject     closestPointOnSurfaceNode::inputGeo;
MObject     closestPointOnSurfaceNode::inputPoint;
MObject     closestPointOnSurfaceNode::inputPointX;
MObject     closestPointOnSurfaceNode::inputPointY;
MObject     closestPointOnSurfaceNode::inputPointZ;
MObject     closestPointOnSurfaceNode::output;
MObject     closestPointOnSurfaceNode::outputX;
MObject     closestPointOnSurfaceNode::outputY;
MObject     closestPointOnSurfaceNode::outputZ;
MObject     closestPointOnSurfaceNode::output2;
MObject     closestPointOnSurfaceNode::output2X;
MObject     closestPointOnSurfaceNode::output2Y;
MObject     closestPointOnSurfaceNode::output2Z;
MObject     closestPointOnSurfaceNode::output3;
MObject     closestPointOnSurfaceNode::output3X;
MObject     closestPointOnSurfaceNode::output3Y;
MObject     closestPointOnSurfaceNode::output3Z;

Octree::Octree(MFloatPointArray& triPoints, MFloatPointArray& vertsPoints, bool precomp)
{
	// Initializes one main top cell that will be dynamically refined per-query.
	ptr_points = &triPoints;
	AABB topCell;
	float minX = std::numeric_limits<double>::infinity();
	float minY = std::numeric_limits<double>::infinity();
	float minZ = std::numeric_limits<double>::infinity();
	float maxX = -std::numeric_limits<double>::infinity();
	float maxY = -std::numeric_limits<double>::infinity();
	float maxZ = -std::numeric_limits<double>::infinity();
	
	// use vertex bounds to define the AABB cell
	for (int id = 0; id < vertsPoints.length(); id++)
	{
		if (vertsPoints[id][0] < minX) minX = vertsPoints[id][0];
		if (vertsPoints[id][0] > maxX) maxX = vertsPoints[id][0];
		if (vertsPoints[id][1] < minY) minY = vertsPoints[id][1];
		if (vertsPoints[id][1] > maxY) maxY = vertsPoints[id][1];
		if (vertsPoints[id][2] < minZ) minZ = vertsPoints[id][2];
		if (vertsPoints[id][2] > maxZ) maxZ = vertsPoints[id][2];
	}
	// Append all the triangle centres to the initial cell
	for (int id = 0; id < triPoints.length(); id++)
	{
		topCell.ids.append(id);
	}

	// just for fun :) Find the longest side (oh boy is this ugly...)
	float dim = ((maxX - minX) > (maxY - minY)) ? ((maxX - minX) > (maxZ - minZ)) ? maxX - minX : maxZ - minZ : ((maxY - minY) > (maxZ - minZ)) ? maxY - minY : maxZ - minZ;

	topCell.pA = MVector(minX-0.0001, minY-0.0001, minZ-0.0001);
	topCell.pB = topCell.pA + MVector(dim, dim, dim);
	topCell.id = 0;

	depth_map[0] = MUintArray();
	depth_map[0].append(0);

	cells.push_back(topCell);
	if (precomp)
		subdivCell(0, true);
}

Octree::Octree(MFloatPointArray& vertsPoints, bool precomp)
{
	// Initializes one main top cell that will be dynamically refined per-query.
	ptr_points = &vertsPoints;
	AABB topCell;
	float minX = std::numeric_limits<double>::infinity();
	float minY = std::numeric_limits<double>::infinity();
	float minZ = std::numeric_limits<double>::infinity();
	float maxX = -std::numeric_limits<double>::infinity();
	float maxY = -std::numeric_limits<double>::infinity();
	float maxZ = -std::numeric_limits<double>::infinity();

	// use vertex bounds to define the AABB cell
	for (int id = 0; id < vertsPoints.length(); id++)
	{
		topCell.ids.append(id);
		if (vertsPoints[id][0] < minX) minX = vertsPoints[id][0];
		if (vertsPoints[id][0] > maxX) maxX = vertsPoints[id][0];
		if (vertsPoints[id][1] < minY) minY = vertsPoints[id][1];
		if (vertsPoints[id][1] > maxY) maxY = vertsPoints[id][1];
		if (vertsPoints[id][2] < minZ) minZ = vertsPoints[id][2];
		if (vertsPoints[id][2] > maxZ) maxZ = vertsPoints[id][2];
	}
	// Append all the triangle centres to the initial cell

	topCell.pA = MVector(minX - 0.0001, minY - 0.0001, minZ - 0.0001);
	topCell.pB = MVector(maxX + 0.0001, maxY + 0.0001, maxZ + 0.0001);
	topCell.id = 0;
	depth_map[0] = MUintArray();
	depth_map[0].append(0);

	cells.push_back(topCell);
	if (precomp)
		subdivCell(0, true);
}

closestPointOnSurfaceNode::closestPointOnSurfaceNode() {}
closestPointOnSurfaceNode::~closestPointOnSurfaceNode() {}

MStatus closestPointOnSurfaceNode::compute( const MPlug& plug, MDataBlock& data )
{
	MStatus returnStatus;

	if( plug == output )
	{

		MDataHandle inMeshHandle = data.inputValue(inputGeo, &returnStatus);
		if (returnStatus != MS::kSuccess)
			MGlobal::displayError("Unable to get handle to tgt mesh");
		MObject inMeshObj = inMeshHandle.asMesh();
		MFnMesh tgtMesh(inMeshObj, &returnStatus);
		if (returnStatus != MS::kSuccess)
			MGlobal::displayError("Could not create MFnMesh for the input geom.");
		
		MDataHandle inPositionXDataHandle = data.inputValue(inputPointX);
		double inPositionX = inPositionXDataHandle.asDouble();
		MDataHandle inPositionYDataHandle = data.inputValue(inputPointY);
		double inPositionY = inPositionYDataHandle.asDouble();
		MDataHandle inPositionZDataHandle = data.inputValue(inputPointZ);
		double inPositionZ = inPositionZDataHandle.asDouble();

		MVector inPosition(inPositionX, inPositionY, inPositionZ);

		if( returnStatus != MS::kSuccess )
			MGlobal::displayError( "Node closestPointOnSurfaceNode cannot get value\n" );
		else
		{
			
			MFloatPointArray vertexList;
			tgtMesh.getPoints(vertexList, MSpace::kWorld);
			
			MIntArray triangleCounts;
			MIntArray triangleVertices;
			tgtMesh.getTriangles(triangleCounts, triangleVertices);

			// Octree is initialized on request.
			// TODO: Automatically set the initTree flag to true when mesh input changes.
			if (initTree)
			{
				// We precompute triangle centres to then have them in an octree
				precompTriCentres(vertexList, triangleVertices);
				verts = vertexList;
				bool precomp = true;
				otree_tris = Octree(triCentres, vertexList, precomp);
				otree_tris.maxElems = 1;
				otree_verts = Octree(verts, precomp);
				otree_verts.maxElems = 10;
				initTree = false;
			}

			int n_queries = 0;

			// Brute force

			float bestLength = std::numeric_limits<double>::infinity();
			auto start = high_resolution_clock::now();
			MVector bestPoint(0, 0, 0);

			for (int i = 0; i < (triangleVertices.length() / 3); i++)
			{
				n_queries++;
				int idA = triangleVertices[i * 3];
				int idB = triangleVertices[i * 3 + 1];
				int idC = triangleVertices[i * 3 + 2];
				MVector vtxA(vertexList[idA][0], vertexList[idA][1], vertexList[idA][2]);
				MVector vtxB(vertexList[idB][0], vertexList[idB][1], vertexList[idB][2]);
				MVector vtxC(vertexList[idC][0], vertexList[idC][1], vertexList[idC][2]);

				MVector normal = ((vtxB - vtxA) ^ (vtxC - vtxA));
				normal.normalize();

				if ((inPosition - vtxA)*normal < 0)
					continue;

				MVector closestPoint = computePointTriangleIntersect(inPosition, vtxA, vtxB, vtxC, normal);
				float currLen = (inPosition - closestPoint).length();

				if (currLen < bestLength)
				{
					bestLength = currLen;
					bestPoint = closestPoint;
				}
			}
			auto stop = high_resolution_clock::now();
			auto duration = duration_cast<microseconds>(stop - start);
			std::cout << "Brute force required " << n_queries << " queries." << std::endl;
			std::cout << "Brute force took " << duration.count() << " microseconds" << std::endl;

			// Octree method
			// Closest vert
			start = high_resolution_clock::now();

			MVector bestPoint2(0, 0, 0);
			float max_dist = std::numeric_limits<double>::infinity();

			MUintArray cells_to_test = otree_verts.getClosestCells(inPosition, max_dist);

			float best_dist = std::numeric_limits<double>::infinity();
			for (int i = 0; i < (cells_to_test.length()); i++)
			{
				AABB *cell = &otree_verts.cells[cells_to_test[i]];
				for (int elem_i = 0; elem_i < cell->ids.length(); elem_i++)
				{
					MVector vert_p(vertexList[cell->ids[elem_i]][0], vertexList[cell->ids[elem_i]][1], vertexList[cell->ids[elem_i]][2]);
					float elem_dist = (vert_p - inPosition) * (vert_p - inPosition);
					if (elem_dist <= best_dist)
					{
						best_dist = elem_dist;
						bestPoint2 = vert_p;
					}
				}
			}
			

			/*int closest_vertCell_id = otree_verts.getClosestCellId(inPosition, 0);
			if (closest_vertCell_id == -1)
				return MS::kSuccess;
			int vert_id = otree_verts.cells[closest_vertCell_id].ids[0];
			MVector vert_p(vertexList[vert_id][0], vertexList[vert_id][1], vertexList[vert_id][2]);*/

			// Closest triangle
			n_queries = 0;
			float dim = (inPosition - bestPoint2).length()*2;

			AABB bbox;
			bbox.pA = inPosition - MVector(dim, dim, dim);
			bbox.pB = inPosition + MVector(dim, dim, dim);
			//std::cout << "Intersect Box centre: " << bbox.centre() << std::endl;

			MUintArray intersect_ids;
			otree_tris.getClosestCellIds(inPosition, 0, bbox, intersect_ids);
			//std::cout << intersect_ids << std::endl;
			MVector bestPoint3 = MVector(0, 0, 0);
			bestLength = std::numeric_limits<double>::infinity();
			MUintArray vertsTested;
			for (int i = 0; i < (intersect_ids.length()); i++)
			{
				//std::cout << "Cell id and pos: " << intersect_ids[i] << " " << otree_tris.cells[intersect_ids[i]].centre() << std::endl;
				for (int j = 0; j < otree_tris.cells[intersect_ids[i]].ids.length(); j++)
				{
					n_queries++;
					int tri_id = otree_tris.cells[intersect_ids[i]].ids[j];
					int idA = triangleVertices[tri_id * 3];
					int idB = triangleVertices[tri_id * 3 + 1];
					int idC = triangleVertices[tri_id * 3 + 2];
					vertsTested.append(idA);
					vertsTested.append(idB);
					vertsTested.append(idC);
					MVector vtxA(vertexList[idA][0], vertexList[idA][1], vertexList[idA][2]);
					MVector vtxB(vertexList[idB][0], vertexList[idB][1], vertexList[idB][2]);
					MVector vtxC(vertexList[idC][0], vertexList[idC][1], vertexList[idC][2]);

					//std::cout << "Triangle vert ids: " << idA << " " << idB << " " << idC << std::endl;
					//std::cout << "IdA pos: " << vtxA << std::endl;
					//std::cout << "IdB pos: " << vtxB << std::endl;
					//std::cout << "IdC pos: " << vtxC << std::endl;

					MVector normal = ((vtxB - vtxA) ^ (vtxC - vtxA));
					normal.normalize();

					if ((inPosition - vtxA)*normal < 0)
					{
						//std::cout << "Ignored based on normal..." << std::endl;
						continue;
					}

					MVector closestPoint = computePointTriangleIntersect(inPosition, vtxA, vtxB, vtxC, normal);
					//std::cout << "Closest point: " << closestPoint << std::endl;
					float currLen = (inPosition - closestPoint) * (inPosition - closestPoint);
					//std::cout << "Distance: " << currLen << std::endl;

					if (currLen < bestLength)
					{
						//std::cout << "Setting new best point." << std::endl;
						bestLength = currLen;
						bestPoint3 = closestPoint;
					}
				}
			}
			stop = high_resolution_clock::now();
			duration = duration_cast<microseconds>(stop - start);
			std::cout << "Octree method took " << duration.count() << " microseconds" << std::endl;
			std::cout << "Octree method required " << n_queries << " queries." << std::endl;
			std::cout << "Tri tree has " << otree_tris.numCells() << " cells." << std::endl;
			std::cout << "Vert tree has " << otree_verts.numCells() << " cells." << std::endl;
			//std::cout << "Verts testd: " << vertsTested << std::endl;
			// Set the two outputs to compare
			MDataHandle outputHandle = data.outputValue( closestPointOnSurfaceNode::output );
			outputHandle.set(bestPoint3[0], bestPoint3[1], bestPoint3[2]);
			MDataHandle outputHandle2 = data.outputValue(closestPointOnSurfaceNode::output2);
			outputHandle2.set(bestPoint[0], bestPoint[1], bestPoint[2]);
			MDataHandle outputHandle3 = data.outputValue(closestPointOnSurfaceNode::output3);
			outputHandle3.set(bestPoint2[0], bestPoint2[1], bestPoint2[2] );

			data.setClean(plug);
		}
	} else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

void closestPointOnSurfaceNode::precompTriCentres(MFloatPointArray& vertList, MIntArray& triVerts)
{
	triCentres.clear();
	for (int i = 0; i < (triVerts.length() / 3); i++)
	{
		int idA = triVerts[i * 3];
		int idB = triVerts[i * 3 + 1];
		int idC = triVerts[i * 3 + 2];
		triCentres.append((vertList[idA][0] + vertList[idB][0] + vertList[idC][0]) / 3, (vertList[idA][1] + vertList[idB][1] + vertList[idC][1]) / 3, (vertList[idA][2] + vertList[idB][2] + vertList[idC][2]) / 3);
	}
}

MVector closestPointOnSurfaceNode::computePointTriangleIntersect(MVector& src, MVector& vtxA, MVector& vtxB, MVector& vtxC, MVector& tNorm)
{	
	MVector result(0, 0, 0);

	// One of the verts plus the normal define the plane. Compute closest point on plane
	MVector pToVtx = vtxA - src;
	result = src + tNorm * (tNorm * pToVtx);

	//return result;

	// Determine if point is inside the triangle
	bool inside = false;

	MVector alpha = (vtxB - vtxA) ^ (result - vtxA);
	MVector beta = (vtxC - vtxB) ^ (result - vtxB);
	MVector gamma = (vtxA - vtxC) ^ (result - vtxC);

	inside = ((alpha*tNorm) > 0 && (beta*tNorm) > 0 && (gamma*tNorm) > 0);

	if (inside)
		return result;

	// determine outside of which edge the point is. All but the needed edge will face same dir as the norm.
	// if there is a case of a corner, and 2 candidates emerge, it will still result in the vertex being picked.
	MVector pA(0, 0, 0);
	MVector pB(0, 0, 0);
	if ((alpha*tNorm) < 0)
	{
		pA = vtxA;
		pB = vtxB;
	}
	else if ((beta*tNorm) < 0)
	{
		pA = vtxB;
		pB = vtxC;
	}
	else if ((gamma*tNorm) < 0)
	{
		pA = vtxC;
		pB = vtxA;
	}

	MVector edgeA = pB - pA;
	MVector edgeB = result - pA;
	float edgeALen = edgeA.length();
	MVector edgeANorm = edgeA/edgeALen;

	float projLen = edgeB * edgeANorm;

	if (projLen < 0)
		return pA;
	if (projLen > edgeALen)
		return pB;

	return (pA + edgeANorm * projLen);
}

void* closestPointOnSurfaceNode::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new closestPointOnSurfaceNode();
}

MStatus closestPointOnSurfaceNode::initialize()		
{
	MFnNumericAttribute nAttr;
	MStatus				stat;
	MFnTypedAttribute   tAttr;

	MStatus status;
	
	inputGeo = tAttr.create("inputGeom", "ing", MFnData::kMesh, MObject::kNullObj, &status);
	tAttr.setWritable(true);
	tAttr.setStorable(true);
	addAttribute(inputGeo);

	inputPointX = nAttr.create("inputX", "ix", MFnNumericData::kDouble, 0.0);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	addAttribute(inputPointX);

	inputPointY = nAttr.create("inputY", "iy", MFnNumericData::kDouble, 0.0);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	addAttribute(inputPointY);

	inputPointZ = nAttr.create("inputZ", "iz", MFnNumericData::kDouble, 0.0);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	addAttribute(inputPointZ);

	inputPoint = nAttr.create("inputPoint", "ip", inputPointX, inputPointY, inputPointZ);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	addAttribute(inputPoint);

	outputX = nAttr.create("outputX", "ox", MFnNumericData::kDouble, 0.0);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(outputX);

	outputY = nAttr.create("outputY", "oy", MFnNumericData::kDouble, 0.0);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(outputY);

	outputZ = nAttr.create("outputZ", "oz", MFnNumericData::kDouble, 0.0);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(outputZ);

	output = nAttr.create("output", "out", outputX, outputY, outputZ);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(output);

	output2X = nAttr.create("output2X", "ox2", MFnNumericData::kDouble, 0.0);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(output2X);

	output2Y = nAttr.create("output2Y", "oy2", MFnNumericData::kDouble, 0.0);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(output2Y);

	output2Z = nAttr.create("output2Z", "oz2", MFnNumericData::kDouble, 0.0);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(output2Z);

	output2 = nAttr.create("output2", "out2", output2X, output2Y, output2Z);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(output2);

	output3X = nAttr.create("output3X", "ox3", MFnNumericData::kDouble, 0.0);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(output3X);

	output3Y = nAttr.create("output3Y", "oy3", MFnNumericData::kDouble, 0.0);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(output3Y);

	output3Z = nAttr.create("output3Z", "oz3", MFnNumericData::kDouble, 0.0);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(output3Z);

	output3 = nAttr.create("output3", "out3", output3X, output3Y, output3Z);
	nAttr.setStorable(false);
	nAttr.setKeyable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(output3);

	status = attributeAffects(inputGeo, output );
		if (!status) { status.perror("attributeAffects"); return status;}
	status = attributeAffects(inputPoint, output);
		if (!status) { status.perror("attributeAffects"); return status; }
	status = attributeAffects(inputGeo, output2);
		if (!status) { status.perror("attributeAffects"); return status; }
	status = attributeAffects(inputPoint, output2);
		if (!status) { status.perror("attributeAffects"); return status; }
	status = attributeAffects(inputGeo, output3);
		if (!status) { status.perror("attributeAffects"); return status; }
	status = attributeAffects(inputPoint, output3);
		if (!status) { status.perror("attributeAffects"); return status; }

	return MS::kSuccess;

}

bool Octree::isCellInBox(AABB *curr_cell, AABB *test_box)
{
	bool is_inside = (*curr_cell).pA[0] >= (*test_box).pA[0] &&
					 (*curr_cell).pA[1] >= (*test_box).pA[1] &&
					 (*curr_cell).pA[2] >= (*test_box).pA[2] &&
					 (*curr_cell).pB[0] <= (*test_box).pB[0] &&
					 (*curr_cell).pB[1] <= (*test_box).pB[1] &&
					 (*curr_cell).pB[2] <= (*test_box).pB[2];

	return is_inside;
}

void Octree::getClosestCellIds(MVector& src, int start_id, AABB& test_box, MUintArray &out_arr)
{
	// Dynamical octree query.
	// If there is only 1 triangle in the cell we are querying, that means we already picked the closest one - DONE.
	AABB* curr_cell = &cells[start_id];

	if (curr_cell->ids.length() == 0 || !intersectBBox(curr_cell, &test_box))
		return;

	if (curr_cell->ids.length() == maxElems || isCellInBox(curr_cell, &test_box))
	{
		out_arr.append(start_id);
		return;
	}

	if (curr_cell->ids.length() > maxElems)
	{
		if (curr_cell->children.length() == 0)
		{
			subdivCell(curr_cell->id);
			curr_cell = &cells[start_id];
		}
		if (curr_cell->children.length() > 0)
		{
			// get closest child
			float best_len = std::numeric_limits<double>::infinity();
			int best_cell_id;
			for (int child_id = 0; child_id < curr_cell->children.length(); child_id++)
			{
				// Ignore empty sub-cells
				if (cells[curr_cell->children[child_id]].ids.length() == 0)
					continue;
				getClosestCellIds(src, curr_cell->children[child_id], test_box, out_arr);
				curr_cell = &cells[start_id];
			}
		}

	}

	return;
}

void Octree::getClosestCellIds(MVector& src, int start_id, float max_dist, MUintArray &out_arr)
{
	// Dynamical octree query.
	// If there is only 1 triangle in the cell we are querying, that means we already picked the closest one - DONE.
	AABB* curr_cell = &cells[start_id];

	if (curr_cell->ids.length() == 0 || !intersectSphere(src, curr_cell, max_dist))
		return;

	if (curr_cell->ids.length() == maxElems)
	{
		out_arr.append(start_id);
		return;
	}
	
	if (curr_cell->ids.length() > maxElems)
	{
		if (curr_cell->children.length() == 0)
		{
			subdivCell(curr_cell->id);
			curr_cell = &cells[start_id];
		}
		if (curr_cell->children.length() > 0)
		{
			// get closest child
			float best_len = std::numeric_limits<double>::infinity();
			int best_cell_id;
			for (int child_id = 0; child_id < curr_cell->children.length(); child_id++)
			{
				// Ignore empty sub-cells
				if (cells[curr_cell->children[child_id]].ids.length() == 0)
					continue;
				getClosestCellIds(src, curr_cell->children[child_id], max_dist, out_arr);
			}
		}

	}

	return;
}


int Octree::getClosestChild(MVector &src, AABB *curr_cell)
{
	float best_len = std::numeric_limits<double>::infinity();
	int best_cell_id = -1;
	for (int child_id = 0; child_id < curr_cell->children.length(); child_id++)
	{
		// Ignore empty sub-cells
		if (cells[curr_cell->children[child_id]].ids.length() == 0)
			continue;
		float child_len = (src - cells[curr_cell->children[child_id]].centre()) * (src - cells[curr_cell->children[child_id]].centre());
		if (child_len < best_len)
		{
			best_len = child_len;
			best_cell_id = curr_cell->children[child_id];
		}
	}
	return best_cell_id;
}

MVector Octree::getMostDistantCorner(MVector &src, AABB *curr_cell)
{
	MVector result;
	float best_dist = 0.0;
	
	MVector dir_vec = (curr_cell->pB - curr_cell->pA);
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				float X = curr_cell->pA[0] + i * dir_vec[0];
				float Y = curr_cell->pA[1] + j * dir_vec[1];
				float Z = curr_cell->pA[2] + k * dir_vec[2];

				MVector point(X, Y, Z);
				float curr_dist = ((point - src) * (point - src));
				if (curr_dist > best_dist)
				{
					best_dist = curr_dist;
					result = point;
				}
			}
		}
	}

	return result;
}


MUintArray Octree::getClosestCells(MVector &src, float max_dist)
{
	MUintArray result;

	AABB *top = &cells[0];

	if (intersectSphere(src, top, max_dist))
	{
		MUintArray valid_cells;

		if (top->children.length() == 0)
		{
			subdivCell(0);
			// if we subdiv, cells vector is updated, so we need to re-assign pointer
			top = &cells[0];
		}

		valid_cells = top->children;
		
		// get nearest cell and update the max_dist
		while (true)
		{
			float best_len = std::numeric_limits<double>::infinity();
			int best_cell_id;
			for (int v_id = 0; v_id < valid_cells.length(); v_id++)
			{
				AABB *test_cell = &cells[valid_cells[v_id]];
				// Ignore empty cells
				if (test_cell->ids.length() == 0)
					continue;
				float test_len = (src - test_cell->centre()) * (src - test_cell->centre());
				if (test_len < best_len)
				{
					best_len = test_len;
					best_cell_id = test_cell->id;
				}
			}
			MVector dist_corner = getMostDistantCorner(src, &cells[best_cell_id]);
			float n_len = (src - dist_corner) * (src - dist_corner);
			// if the new distance is longer or same as the last one, then whatever valid cells we currently have are the one we return.
			if (n_len >= max_dist)
			{
				result = valid_cells;
				break;
			}
			// otherwise, we set the new distance, and check which of the old cells are still valid, and if any of them have more then one element and no children yet, we subdivide it
			// then append all of it's children
			max_dist = n_len;
			MUintArray n_valid_cells;
			for (int v_id = 0; v_id < valid_cells.length(); v_id++)
			{
				AABB *test_cell = &cells[valid_cells[v_id]];
				if (intersectSphere(src, test_cell, max_dist))
				{
					n_valid_cells.append(test_cell->id);
					if (test_cell->ids.length() > maxElems && test_cell->children.length() == 0)
					{
						subdivCell(test_cell->id);
						test_cell = &cells[valid_cells[v_id]];
					}
					if (test_cell->children.length() > 0)
					{
						for (int child_id = 0; child_id < test_cell->children.length(); child_id++)
						{
							n_valid_cells.append(test_cell->children[child_id]);
						}
					}
				}
			}
			valid_cells = n_valid_cells;
		}
	}

	return result;
}

void Octree::subdivCell(int cell_id, bool recursive)
{
	AABB *curr_cell = &cells[cell_id];
	if (recursive)
	{
		if (curr_cell->children.length() > 0 || curr_cell->ids.length() <= maxElems)
			return;
	}

	MVector dir_vec = (curr_cell->pB - curr_cell->pA) / 2.0;
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				float minX = curr_cell->pA[0] + i * dir_vec[0];
				float minY = curr_cell->pA[1] + j * dir_vec[1];
				float minZ = curr_cell->pA[2] + k * dir_vec[2];

				float maxX = curr_cell->pA[0] + (i + 1) * dir_vec[0];
				float maxY = curr_cell->pA[1] + (j + 1) * dir_vec[1];
				float maxZ = curr_cell->pA[2] + (k + 1) * dir_vec[2];

				AABB n_cell;
				n_cell.pA = MVector(minX, minY, minZ);
				n_cell.pB = MVector(maxX, maxY, maxZ);

				n_cell.id = numCells();
				n_cell.parent = cell_id;
				cells.push_back(n_cell);
				curr_cell = &cells[cell_id];
				curr_cell->children.append(n_cell.id);
			}
		}
	}
	// sort the current cell's triangles into sub-cells.
	for (int p_id = 0; p_id < curr_cell->ids.length(); p_id++)
	{
		int tri_id = curr_cell->ids[p_id];
		float p_x = (*ptr_points)[tri_id][0];
		float p_y = (*ptr_points)[tri_id][1];
		float p_z = (*ptr_points)[tri_id][2];
		for (int child_id = 0; child_id < curr_cell->children.length(); child_id++)
		{
			int c_id = curr_cell->children[child_id];
			if (p_x >= cells[c_id].pA[0] && p_x <= cells[c_id].pB[0] &&
				p_y >= cells[c_id].pA[1] && p_y <= cells[c_id].pB[1] &&
				p_z >= cells[c_id].pA[2] && p_z <= cells[c_id].pB[2])
			{
				// append triangle to cell
				cells[c_id].ids.append(tri_id);
			}
		}
	}
	if (recursive)
	{
		for (int child_id = 0; child_id < curr_cell->children.length(); child_id++)
		{
			subdivCell(curr_cell->children[child_id], recursive);
			curr_cell = &cells[cell_id];
		}
	}
}


bool Octree::intersectBBox(AABB* curr_cell, AABB *test_box)
{
	bool intersected =  ((*curr_cell).pA[0] <= (*test_box).pB[0] && (*curr_cell).pB[0] >= (*test_box).pA[0]) &&
						((*curr_cell).pA[1] <= (*test_box).pB[1] && (*curr_cell).pB[1] >= (*test_box).pA[1]) &&
						((*curr_cell).pA[2] <= (*test_box).pB[2] && (*curr_cell).pB[2] >= (*test_box).pA[2]);

	return intersected;
}

bool Octree::intersectSphere(MVector &src, AABB* curr_cell, float max_dist)
{		
	MVector dir_vec = (curr_cell->pB - curr_cell->pA) / 2.0;
	float bot_l_back   = (curr_cell->pA - src) * (curr_cell->pA - src); // srqd distance
	float bot_l_front  = ((curr_cell->pA + MVector(0, 0, dir_vec[2])) - src) * ((curr_cell->pA + MVector(0, 0, dir_vec[2])) - src);
	float top_l_back   = ((curr_cell->pA + MVector(0, dir_vec[1], 0)) - src) * ((curr_cell->pA + MVector(0, dir_vec[1], 0)) - src);
	float top_l_front  = ((curr_cell->pA + MVector(0, dir_vec[1], dir_vec[2])) - src) * ((curr_cell->pA + MVector(0, dir_vec[1], dir_vec[2])) - src);
	float bot_r_back   = ((curr_cell->pA + MVector(dir_vec[0], 0, 0)) - src) * ((curr_cell->pA + MVector(dir_vec[0], 0, 0)) - src);
	float bot_r_front  = ((curr_cell->pA + MVector(dir_vec[0], 0, dir_vec[2])) - src) * ((curr_cell->pA + MVector(dir_vec[0], 0, dir_vec[2])) - src);
	float top_r_back   = ((curr_cell->pA + MVector(dir_vec[0], dir_vec[1], 0)) - src) * ((curr_cell->pA + MVector(dir_vec[0], dir_vec[1], 0)) - src);
	float top_r_front  = (curr_cell->pB - src) * (curr_cell->pB - src);

	bool intersected = (bot_l_back < max_dist || bot_l_front < max_dist || top_l_back < max_dist || top_l_front < max_dist ||
						bot_r_back < max_dist || bot_r_front < max_dist || top_r_back < max_dist || top_r_front < max_dist);

	return intersected;
}
