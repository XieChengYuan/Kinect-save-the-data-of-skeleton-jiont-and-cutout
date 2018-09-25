#include "server.h"
#include <iostream>
#include <WS2tcpip.h>
#include <process.h>
#include <cctype>
#pragma comment(lib, "ws2_32.lib")  

using std::cerr;
using std::cout;
using std::endl;
using std::string;


#define SERVER_PORT 4999
#define MSG_BUF_SIZE 1024

int ret_val;
char sendStart[MSG_BUF_SIZE];
char sendSave[MSG_BUF_SIZE];
char sendEnd[MSG_BUF_SIZE];
char sendAct[MSG_BUF_SIZE];


Server::Server()
{
	cout << "Initializing server...\n";
	//
	winsock_ver = MAKEWORD(2, 2);
	addr_len = sizeof(SOCKADDR_IN);
	addr_svr.sin_family = AF_INET;
	addr_svr.sin_port = ::htons(SERVER_PORT);
	addr_svr.sin_addr.S_un.S_addr = ADDR_ANY;
	memset(buf_ip, 0, IP_BUF_SIZE);
	//
	ret_val = ::WSAStartup(winsock_ver, &wsa_data);
	if (ret_val != 0)
	{
		cerr << "WSA failed to start up!Error code: " << ::WSAGetLastError() << "\n";
		system("pause");
		exit(1);
	}
	cout << "WSA started up successfully...\n";
	//
	sock_svr = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock_svr == INVALID_SOCKET)
	{
		cerr << "Failed to create server socket!Error code: " << ::WSAGetLastError() << "\n";
		::WSACleanup();
		system("pause");
		exit(1);
	}
	cout << "Server socket created successfully...\n";
	//
	ret_val = ::bind(sock_svr, (SOCKADDR*)&addr_svr, addr_len);
	if (ret_val != 0)
	{
		cerr << "Failed to bind server socket!Error code: " << ::WSAGetLastError() << "\n";
		::WSACleanup();
		system("pause");
		exit(1);
	}
	cout << "Server socket bound successfully...\n";
	//
	ret_val = ::listen(sock_svr, SOMAXCONN);
	if (ret_val == SOCKET_ERROR)
	{
		cerr << "Server socket failed to listen!Error code: " << ::WSAGetLastError() << "\n";
		::WSACleanup();
		system("pause");
		exit(1);
	}
	cout << "Server socket started to listen...\n";
	//
	cout << "Server started successfully..." << endl;
}

Server::~Server()
{
	::closesocket(sock_svr);
	::closesocket(sock_clt);
	::WSACleanup();
	cout << "Socket closed..." << endl;
}

unsigned __stdcall CreateClientThread(LPVOID lpParameter);

void Server::WaitForClient()
{

	cout << "***********************************" << endl;
	cout << "*     抠图后彩+深+骨骼数据获取    *" << endl;
	cout << "***********************************" << endl;
	cout << "输入开始命令【任意键】:";
	
	std::cin >> sendStart;
	
	while (true)
	{
		sock_clt = ::accept(sock_svr, (SOCKADDR*)&addr_clt, &addr_len);
		if (sock_clt == INVALID_SOCKET)
		{
			cerr << "Failed to accept client!Error code: " << ::WSAGetLastError() << "\n";
			::WSACleanup();
			system("pause");
			exit(1);
		}
		::InetNtop(addr_clt.sin_family, &addr_clt, (PWSTR)buf_ip, IP_BUF_SIZE);
		cout << "A new client connected...IP address: " << buf_ip << ", port number: " << ::ntohs(addr_clt.sin_port) << endl;
		h_thread = (HANDLE)_beginthreadex(nullptr, 0, CreateClientThread, (LPVOID)sock_clt, 0, nullptr);
		if (h_thread == NULL)
		{
			cerr << "Failed to create a new thread!Error code: " << ::WSAGetLastError() << "\n";
			::WSACleanup();
			system("pause");
			exit(1);
		}
		::CloseHandle(h_thread);

	}
}

unsigned __stdcall CreateClientThread(LPVOID lpParameter)
{

	SOCKET sock_clt = (SOCKET)lpParameter;
	send(sock_clt, (char*)&sendStart, sizeof(sendStart), 0);

	cout << "输入录制命令【2】：" << endl;
	std::cin >> sendAct;
	send(sock_clt, (char*)&sendAct, sizeof(sendAct), 0);


//	cout << "输入录制命令【2】：" << endl;
//	std::cin >> sendSave;
//	send(sock_clt, (char*)&sendEnd, sizeof(sendEnd), 0);

	cout << "输入结束命令【3】：" << endl;
	std::cin >> sendEnd;
	send(sock_clt, (char*)&sendEnd, sizeof(sendEnd), 0);

	int ret_val = ::shutdown(sock_clt, SD_SEND);
	if (ret_val == SOCKET_ERROR)
	{
		cerr << "Failed to shutdown the client socket!Error code: " << ::GetLastError() << "\n";
		::closesocket(sock_clt);
		system("pause");
		return 1;
	}
	return 0;
}

