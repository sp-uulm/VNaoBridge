#ifndef INCLUDE_OS_H
#define INCLUDE_OS_H


#include <boost/filesystem.hpp>


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


#ifdef _WIN32
	typedef HANDLE PROCESS_ID;
	#define INVALID_PROCESS_ID 0

	HANDLE spawnProcess(const char* argv, ...) {
		std::wstring cmdline;

		{
			va_list ap;
			va_start(ap, argv);
			for(const char* arg = argv; arg != NULL; arg = va_arg(ap, const char*)) {
				cmdline += boost::filesystem::path(arg).wstring();
				cmdline += L' ';
			}
			va_end(ap);
		}

		STARTUPINFOW startupInfo;
		ZeroMemory(&startupInfo, sizeof(startupInfo));
		startupInfo.cb = sizeof(startupInfo);

		PROCESS_INFORMATION processInfo;
		ZeroMemory(&processInfo, sizeof(processInfo));

		const BOOL spawned = CreateProcessW(NULL, (LPWSTR)cmdline.c_str(), NULL, NULL, false, NULL, NULL, NULL, &startupInfo, &processInfo);
		if(!spawned)
			return INVALID_PROCESS_ID;
		return processInfo.hProcess;
	}

	void killProcess(HANDLE handle) {
		if(!TerminateProcess(handle, SIGINT))
			return;

		WaitForSingleObject(handle, INFINITE);
	}

#else
	typedef int PROCESS_ID;
	#define INVALID_PROCESS_ID 0
	#define spawnProcess qi::os::spawnlp

	void killProcess(PROCESS_ID id) {
		qi::os::kill(id, SIGINT);
		int status;
		qi::os::waitpid(id, &status);
	}
#endif


#endif
