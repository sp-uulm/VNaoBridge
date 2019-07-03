#ifndef INCLUDE_CHTTPD_H
#define INCLUDE_CHTTPD_H


#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <vector>


#ifdef _WIN32
	/* See http://stackoverflow.com/questions/12765743/getaddrinfo-on-win32 */
	#ifndef _WIN32_WINNT
		#define _WIN32_WINNT 0x0502 // Windows XP SP2
	#endif
	#include <winsock2.h>
	#include <Ws2tcpip.h>
	// #pragma message("Adding library: Winmm.lib")
	// #pragma comment(lib,"Winmm.lib")
	#pragma message("Adding library: Ws2_32.lib")
	#pragma comment(lib,"Ws2_32.lib")
	typedef int _socklen;
#else
	// POSIX
	#include <sys/socket.h>
	#include <arpa/inet.h>
	#include <netdb.h>
	#include <unistd.h>
	#define SOCKET int
	#define INVALID_SOCKET (-1)
	typedef socklen_t _socklen;
#endif


enum HttpState {
	HTTP_REQUESTING,
	HTTP_RESPONDING
};


struct HttpConnection {
	SOCKET      socket;
	std::string ip;
	HttpState   state;
	std::string requestBuffer;
	std::string responseBuffer;
};


class HttpServer {
public:
	SOCKET                       serverSocket;
	std::vector<HttpConnection*> connections;
	bool                         debug;

	HttpServer(u_short port) : debug(false) {
		// initialize Winsock2
		#ifdef _WIN32
			WSADATA wsa_data;
			WSAStartup(MAKEWORD(1,1), &wsa_data);
		#endif

		// initialize the server connection
		struct sockaddr_in serverAddr;
		memset(&serverAddr, 0, sizeof(struct sockaddr_in));
		serverAddr.sin_family      = AF_INET;
		serverAddr.sin_addr.s_addr = INADDR_ANY;
		serverAddr.sin_port        = htons((u_short)port);

		serverSocket = socket(AF_INET, SOCK_STREAM, 0);
		if(serverSocket == INVALID_SOCKET)
			throw std::runtime_error("could not create server socket");
		int enableReuse = 1;
		if(setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, (const char *)&enableReuse, sizeof(int)) < 0)
			throw std::runtime_error("could not set SO_REUSEADDR on server socket");
		if(bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) != 0)
			throw std::runtime_error("could not bind server socket");
		if(listen(serverSocket, 10) != 0)
			throw std::runtime_error("could not listen on server socket");
	}

	virtual ~HttpServer() {
		// close all open sockets
		for(size_t i = 0; i < connections.size(); i++)
			closeSocket(connections[i]->socket);
		closeSocket(serverSocket);

		// clean up Winsock2
		#ifdef _WIN32
			WSACleanup();
		#endif
	}

	// blocking
	// false means stop
	bool step() {
		fd_set readFdSet;
		FD_ZERO(&readFdSet);
		fd_set writeFdSet;
		FD_ZERO(&writeFdSet);
		u_short maxFd = 0;

		// add connection sockets to fd set
		for(auto iter = connections.begin(); iter != connections.end(); iter++) {
			HttpConnection* connection = *iter;
			if(connection->state == HTTP_REQUESTING) {
				FD_SET((*iter)->socket, &readFdSet);
				if((*iter)->socket + 1 > maxFd)
					maxFd = (*iter)->socket + 1;
			}
			else if(connection->state == HTTP_RESPONDING) {
				FD_SET((*iter)->socket, &writeFdSet);
				if((*iter)->socket + 1 > maxFd)
					maxFd = (*iter)->socket + 1;
			}
		}

		// add server socket to fd set
		if(serverSocket + 1 > maxFd)
			maxFd = serverSocket + 1;
		FD_SET(serverSocket, &readFdSet);

		// wait until there is work to do
		int selectResult = select(maxFd, &readFdSet, &writeFdSet, NULL, NULL); // blocking, without timeout
		// TODO do we need to check for signals here?
		if(selectResult <= 0) {
			std::cerr << "something bad happened during select()" << std::endl;
			return false;
		}

		// handle connections
		for(size_t index = 0; index < connections.size(); ) {
			HttpConnection* connection = connections[index];
			if(connection->state == HTTP_REQUESTING && FD_ISSET(connection->socket, &readFdSet)) {
				char buffer[2048];
				int n = recv(connection->socket, buffer, sizeof(buffer), 0);
				if(n < 1) {
					std::cerr << "[" << connection->ip << "] error while receiving, closing socket" << std::endl;
					closeSocket(connection->socket);
					delete connection;
					connections[index] = connections.back();
					connections.pop_back();
					continue;
				}
				else {
					std::string data(buffer, n);
					if(debug) {
						std::cout << "[" << connection->ip << "] data received" << std::endl;
						std::cout << data << std::endl;
					}
					connection->requestBuffer += data;

					size_t pos = connection->requestBuffer.find("\r\n\r\n");
					if(pos != std::string::npos) {
						std::string requestHeader = (pos + 4 == connection->requestBuffer.size() ? connection->requestBuffer : connection->requestBuffer.substr(0, pos));

						std::vector<std::string> requestHeaderLines = splitString(requestHeader, "\r\n");
						std::string firstHeaderLine = requestHeaderLines[0];

						if(firstHeaderLine.length() < 4 + 8 || firstHeaderLine.substr(0, 4) != "GET " || firstHeaderLine.substr(firstHeaderLine.length() - 9, 6) != " HTTP/")
							connection->responseBuffer = generate404Response();
						else {
							std::string requestPath = firstHeaderLine.substr(4, firstHeaderLine.length() - 9 - 4);
							connection->responseBuffer = handleGet(requestPath);
						}

						connection->state = HTTP_RESPONDING;
						if(debug) {
							std::cout << "[" << connection->ip << "] reponding with" << std::endl;
							std::cout << connection->responseBuffer << std::endl;
						}
					}
				}
			}
			else if(connection->state == HTTP_RESPONDING && FD_ISSET(connection->socket, &writeFdSet)) {
				int n = send(connection->socket, connection->responseBuffer.data(), connection->responseBuffer.length(), 0);
				if(n < 1) {
					std::cerr << "[" << connection->ip << "] error while sending, closing socket" << std::endl;
					closeSocket(connection->socket);
					delete connection;
					connections[index] = connections.back();
					connections.pop_back();
					continue;
				}
				else if(n == connection->responseBuffer.length()) {
					// we're done sending
					closeSocket(connection->socket);
					delete connection;
					connections[index] = connections.back();
					connections.pop_back();
				}
				else
					connection->responseBuffer = connection->responseBuffer.substr(n);
			}
			index++;
		}

		// handle server socket last, in case the server adds a new connection
		if(FD_ISSET(serverSocket, &readFdSet)) {
			struct sockaddr_in clientAddr;
			int clientAddrSize = sizeof(clientAddr);

			SOCKET clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, (_socklen *)&clientAddrSize);
			std::string clientIp = inet_ntoa(clientAddr.sin_addr);
			if(debug)
				std::cout << "[" << clientIp << "] incoming connection" << std::endl;

			HttpConnection* connection = new HttpConnection;
			connection->socket = clientSocket;
			connection->ip     = clientIp;
			connection->state  = HTTP_REQUESTING;
			connections.push_back(connection);
		}

		return true;
	}

	// blocking
	void run() {
		while(step()) {
		}
	}

	virtual std::string handleGet(const std::string & requestPath) {
		return generate404Response();
	}

	static std::string generate404Response() {
		std::stringstream response;
		response << "HTTP/1.0 404 OK\r\n";
		response << "Connection: close\r\n";
		response << "\r\n";
		return response.str();
	}

	static std::string generate200Response(const std::string & data, const std::string & contentType) {
		std::stringstream response;
		response << "HTTP/1.0 200 OK\r\n";
		response << "Content-Type: " << contentType << "\r\n";
		response << "Content-Length: " << data.size() << "\r\n";
		response << "Connection: close\r\n";
		response << "\r\n";
		response << data;
		return response.str();
	}

private:
	static void closeSocket(SOCKET socket) {
		#ifdef _WIN32
			shutdown(socket, SD_BOTH);
			closesocket(socket);
		#elif defined (__linux) || defined (__APPLE__)
			if(socket != -1)
				close(socket);
		#endif
	}

	static std::vector<std::string> splitString(const std::string & str, const std::string & delimiter) {
		size_t begin = 0;
		size_t delim_length = delimiter.length();

		std::vector<std::string> parts;
		size_t end;
		while((end = str.find(delimiter, begin)) != std::string::npos) {
			parts.push_back(str.substr(begin, end - begin));
			begin = end + delim_length;
		}
		parts.push_back(str.substr(begin));
		return parts;
	}
};


template <typename Arg>
class CallbackHttpServer : public HttpServer {
private:
	std::string (*callback)(const std::string &, Arg* arg);
	Arg* arg;

public:
	CallbackHttpServer(u_short port, std::string (*callback)(const std::string &, Arg* arg), Arg* arg) : HttpServer(port), callback(callback), arg(arg) {
	}

	std::string handleGet(const std::string & requestPath) {
		return (*callback)(requestPath, arg);
	}
};


template <typename Arg>
void runHttpServer(u_short port, std::string (*callback)(const std::string &, Arg* arg), Arg* arg) {
	CallbackHttpServer<Arg>* server = new CallbackHttpServer<Arg>(port, callback, arg);
	server->run();
	delete server;
}


#endif
