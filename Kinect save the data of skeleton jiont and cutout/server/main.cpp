#include "server.h"

int main()
{
	system("title �����");//����cmd���ڱ���
	system("color 0B");
	Server svr;
	svr.WaitForClient();
	system("pause");
	return 0;
}