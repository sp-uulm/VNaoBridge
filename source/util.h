#ifndef INCLUDE_UTIL_H
#define INCLUDE_UTIL_H


std::string intToString(int value) {
	#ifdef _WIN32
		// we can't use std::to_string in MSVC 2010...
		char buffer[64];
		sprintf_s(buffer, sizeof(buffer), "%d", value);
		return std::string(buffer);
	#else
		return std::to_string(value);
	#endif
}


// computes a list of all descendants in a hierarchy given a root element and a parent-child mapping
template<typename T> std::vector<T> getDescendants(T root, const std::map<T, std::vector<T> > & children) {
	std::list<T> remainingParents;

	// start with the root
	remainingParents.push_back(root);

	std::vector<T> descendants;
	while(!remainingParents.empty()) {
		// fetch the next parent
		T parent = remainingParents.front();
		remainingParents.pop_front();

		// look up the children of this parent
		auto iter = children.find(parent);
		if(iter != children.end()) {
			for(auto i = iter->second.begin(); i != iter->second.end(); i++) {
				T child = *i;

				// add the child to the result, and to the list of possible parents
				descendants.push_back(child);
				remainingParents.push_back(child);
			}
		}
	}

	return descendants;
}


#endif
