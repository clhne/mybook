/*
		2011 Takahiro Harada
*/
#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <windows.h>
#include <time.h>

class Stopwatch
{
	public:
		__inline
		Stopwatch();

		__inline
		void start();
		__inline
		void split();
		__inline
		void stop();
		__inline
		float getMs();
		__inline
		void getMs( float* times, int capacity );
		__inline
		int getNIntervals() const { return m_idx-1; }

	private:
		enum
		{
			CAPACITY = 64,
		};

		LARGE_INTEGER m_frequency;
		LARGE_INTEGER m_t[CAPACITY];
	public:
		int m_idx;
};

__inline
Stopwatch::Stopwatch()
{
	QueryPerformanceFrequency( &m_frequency );
}

__inline
void Stopwatch::start()
{
	m_idx = 0;
	QueryPerformanceCounter(&m_t[m_idx++]);
}

__inline
void Stopwatch::split()
{
	QueryPerformanceCounter(&m_t[m_idx++]);
}

__inline
void Stopwatch::stop()
{
	split();
}

__inline
float Stopwatch::getMs()
{
	return (float)(1000*(m_t[1].QuadPart - m_t[0].QuadPart))/m_frequency.QuadPart;
}

__inline
void Stopwatch::getMs(float* times, int capacity)
{
	for(int i=0; i<capacity; i++) times[i] = 0.f;

	for(int i=0; i<min2(capacity, m_idx-1); i++)
	{
		times[i] = (float)(1000*(m_t[i+1].QuadPart - m_t[i].QuadPart))/m_frequency.QuadPart;
	}
}



#endif

