#include <stdio.h>
#include <string.h>
#include <iostream>
#include "asiosys.h"
#include "asio.h"
#include "asiodrivers.h"

int main()
{
	AsioDrivers asioDrivers;

	char buffer[10][32] = {};
	char* buf[10];

	for (int i = 0; i < 10; i++)
    {
		buf[i] = buffer[i];
	}

	asioDrivers.getDriverNames(buf, 10);

	for (int i = 0; i < 10; i++)
    {
		std::cout << buf[i] << std::endl;
	}
}