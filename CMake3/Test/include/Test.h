#ifndef _CPP_TEST_H_
#define _CPP_TEST_H_

#include "export.h"

class CLASSEXPORT Test
{
public:
	Test();
	virtual ~Test();
	
	int Sum(int a, int b);
};

#endif