/*
		2011 Takahiro Harada
*/
#ifndef SYNC_OBJ_H
#define SYNC_OBJ_H

#include <Windows.h>

template<typename T>
T atomAdd(const T* ptr, int value)
{
	return (T)InterlockedExchangeAdd((LONG*)ptr, value);
}

template<typename T>
T atomCmpxhg(const T* ptr, int cmp, int value)
{
	return (T)InterlockedCompareExchange((LONG*)ptr, value, cmp);
}

#endif

