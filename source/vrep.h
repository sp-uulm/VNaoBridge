#ifndef INCLUDE_VREP_H
#define INCLUDE_VREP_H


#include "util.h"


std::map<simxInt, std::string> vrepGetAllObjectNames(simxInt vRepClientID) {
	std::map<simxInt, std::string> result;

	simxInt VREP_DATATYPE_NAME = 0;
	simxInt   handlesCount;
	simxInt*  handles;
	simxInt   stringDataCount;
	simxChar* stringData;
	simxInt ret = simxGetObjectGroupData(vRepClientID, sim_appobj_object_type, VREP_DATATYPE_NAME, &handlesCount, &handles, NULL, NULL, NULL, NULL, &stringDataCount, &stringData, simx_opmode_blocking);
	if(ret == simx_return_ok && handlesCount == stringDataCount) {
		for(simxInt i = 0; i < handlesCount; i++) {
			std::string name(stringData);
			result[handles[i]] = name;
			stringData += name.length() + 1;
		}
	}

	return result;
}


std::map<simxInt, std::vector<simxInt> > vrepGetAllChildHandles(simxInt vRepClientID) {
	std::map<simxInt, std::vector<simxInt> > result;

	simxInt VREP_DATATYPE_PARENT = 2;
	simxInt  handlesCount;
	simxInt* handles;
	simxInt  intDataCount;
	simxInt* intData;
	simxInt ret = simxGetObjectGroupData(vRepClientID, sim_appobj_object_type, VREP_DATATYPE_PARENT, &handlesCount, &handles, &intDataCount, &intData, NULL, NULL, NULL, NULL, simx_opmode_blocking);
	if(ret == simx_return_ok && handlesCount == intDataCount) {
		for(simxInt i = 0; i < handlesCount; i++)
			result[intData[i]].push_back(handles[i]);
	}

	return result;
}


std::string vrepStripSuffix(const std::string & objectName) {
	size_t pos = objectName.find('#');
	if(pos == std::string::npos)
		return objectName;
	else
		return objectName.substr(0, pos);
}


std::string vrepGetObjectName(simxInt vRepClientID, simxInt objectHandle) {
	// retrieve all V-Rep object names
	std::map<simxInt, std::string> vrepAllObjectNames = vrepGetAllObjectNames(vRepClientID);
	auto iter = vrepAllObjectNames.find(objectHandle);
	if(iter != vrepAllObjectNames.end())
		return iter->second;
	else
		return "";
}


std::map<std::string, simxInt> vrepGetDescendantHandles(simxInt vRepClientID, simxInt rootObjectHandle) {
	std::map<std::string, simxInt> result;

	// retrieve all V-Rep object names
	std::map<simxInt, std::string> vrepAllObjectNames = vrepGetAllObjectNames(vRepClientID);

	// based on the parent-child relations, find all transitive children of rootObjectHandle
	std::vector<simxInt> vrepChildHandles = getDescendants(rootObjectHandle, vrepGetAllChildHandles(vRepClientID));
	for(size_t i = 0; i < vrepChildHandles.size(); i++) {
		simxInt vrepChildHandle = vrepChildHandles[i];
		auto iter = vrepAllObjectNames.find(vrepChildHandle);
		if(iter != vrepAllObjectNames.end()) {
			std::string name = vrepStripSuffix(iter->second);
			result[name] = vrepChildHandle;
		}
	}

	return result;
}


#endif
