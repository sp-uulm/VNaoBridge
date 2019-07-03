#ifndef INCLUDE_THREADS_H
#define INCLUDE_THREADS_H


#ifdef _WIN32
	#include <Windows.h>
	#include <process.h>
	#ifndef QT_COMPIL
		#pragma message("Adding library: Ws2_32.lib")
		#pragma comment(lib,"Ws2_32.lib")
	#endif
	#define MUTEX_HANDLE HANDLE
	#define MUTEX_HANDLE_X MUTEX_HANDLE
	#define THREAD_ID DWORD
	#define THREAD_RET_TYPE void
#elif defined (__linux) || defined (__APPLE__)
	#include <pthread.h>
	#define MUTEX_HANDLE pthread_mutex_t
	#define MUTEX_HANDLE_X MUTEX_HANDLE*
	#define THREAD_ID pthread_t
	#define DWORD unsigned long
	#define THREAD_RET_TYPE void*
#endif


unsigned char launchThread(THREAD_RET_TYPE (*startAddress)(void*), void* arg) {
	#ifdef _WIN32
		return (_beginthread(startAddress, 0, arg) != 0);
	#elif defined (__linux) || defined (__APPLE__)
		pthread_t th;
		return (pthread_create(&th, NULL, startAddress, arg) == 0);
	#endif
}


typedef boost::function<void ()> Callback;


THREAD_RET_TYPE callCallback(void* callback) {
	(*(Callback*)callback)();

	#if defined (__linux) || defined (__APPLE__)
		return NULL;
	#endif
}


void async(Callback callback) {
	Callback* copy = new Callback;
	*copy = callback;
	launchThread(&callCallback, copy);
}


class Mutex {
private:
	MUTEX_HANDLE mutex;

public:
	Mutex() {
		#ifdef _WIN32
			mutex = CreateMutex(0, FALSE, 0);
		#elif defined (__linux) || defined (__APPLE__)
			pthread_mutex_init(&mutex, 0);
		#endif
	}

	~Mutex() {
		#ifdef _WIN32
			CloseHandle(mutex);
		#elif defined (__linux) || defined (__APPLE__)
			pthread_mutex_destroy(&mutex);
		#endif
	}

	void lock() {
		#ifdef _WIN32
			WaitForSingleObject(mutex, INFINITE);
		#elif defined (__linux) || defined (__APPLE__)
			pthread_mutex_lock(&mutex);
		#endif
	}

	void unlock() {
		#ifdef _WIN32
			ReleaseMutex(mutex);
		#elif defined (__linux) || defined (__APPLE__)
			pthread_mutex_unlock(&mutex);
		#endif
	}
};


#endif
