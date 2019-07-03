#ifndef INCLUDE_OS_H
#define INCLUDE_OS_H


#ifndef _WIN32
std::string getCurrentWorkingDirectory() {
	char cwd[1024];
	getcwd(cwd, sizeof(cwd));
	return std::string(cwd);
}
#endif


std::string getCurrentExecutableDirectory(std::string argv0) {
	#ifdef _WIN32
		char path[MAX_PATH];
		GetModuleFileName(NULL, path, MAX_PATH);
		PathRemoveFileSpec(path);
		return std::string(path);
	#else
		size_t pos = argv0.find_last_of("/");
		if(pos != std::string::npos) {
			if(argv0[0] == '/')
				return argv0.substr(0, pos);
			else
				return getCurrentWorkingDirectory() + "/" + argv0.substr(0, pos);
		}
		else
			return getCurrentWorkingDirectory();
	#endif
}


#endif
