//
// Copyright (C) 
// 
// File: pluginMain.cpp
//
// Author: Maya Plug-in Wizard 2.0
//

#include "closestPointOnSurfaceNodeNode.h"

#include <maya/MFnPlugin.h>
#include <maya/MStreamUtils.h>

MStatus initializePlugin( MObject obj )
//
//	Description:
//		this method is called when the plug-in is loaded into Maya.  It 
//		registers all of the services that this plug-in provides with 
//		Maya.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{ 
	std::cout.set_rdbuf(MStreamUtils::stdOutStream().rdbuf());
	std::cerr.set_rdbuf(MStreamUtils::stdErrorStream().rdbuf());
	MStatus   status;
	MFnPlugin plugin( obj, "", "2018", "Any");

	status = plugin.registerNode( "closestPointOnSurfaceNode", closestPointOnSurfaceNode::id, closestPointOnSurfaceNode::creator,
								  closestPointOnSurfaceNode::initialize );
	if (!status) {
		status.perror("registerNode");
		return status;
	}

	return status;
}

MStatus uninitializePlugin( MObject obj)
//
//	Description:
//		this method is called when the plug-in is unloaded from Maya. It 
//		deregisters all of the services that it was providing.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{
	MStatus   status;
	MFnPlugin plugin( obj );

	status = plugin.deregisterNode( closestPointOnSurfaceNode::id );
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
