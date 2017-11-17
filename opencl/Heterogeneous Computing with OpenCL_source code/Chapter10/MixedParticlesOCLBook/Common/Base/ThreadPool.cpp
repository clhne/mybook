/*
		2011 Takahiro Harada
*/
#include <Common/Base/ThreadPool.h>

ThreadPool::ThreadPool(int nThreads)
: m_nThreads( nThreads ), m_deleteSignal( false ), m_taskHead( 0 ), m_taskTail( 0 )
{
	InitializeCriticalSection(&m_cs);

	m_threads = new Thread[nThreads];
	for(int i=0; i<nThreads; i++)
		m_threads[i].init(i, this);

	resetThreadTimer();
}

ThreadPool::~ThreadPool()
{
	m_deleteSignal = true;
	start();
	wait();

	DeleteCriticalSection(&m_cs);

	delete [] m_threads;

	m_threads = 0;
	m_nThreads = 0;
	m_taskHead = 0;
	m_taskTail = 0;
}

void ThreadPool::resetThreadTimer()
{
	QueryPerformanceFrequency( &m_frequency );
	QueryPerformanceCounter( &m_startTime );

	for(int i=0; i<m_nThreads; i++)
	{
		m_threads[i].timestampReset();
	}
}

void ThreadPool::start(bool resetTimestamp)
{
	if( resetTimestamp )
		resetThreadTimer();

	for(int i=0; i<m_nThreads; i++)
	{
		m_threads[i].start();
	}
}

void ThreadPool::wait()
{
	HANDLE* finSignals = new HANDLE[m_nThreads];
	for(int i=0; i<m_nThreads; i++) finSignals[i] = m_threads[i].m_finSignal;

	WaitForMultipleObjects( m_nThreads, finSignals, true, INFINITE );

	for(int i=0; i<m_nThreads; i++)
	{
		ResetEvent( m_threads[i].m_finSignal );
	}

	delete [] finSignals;

}

void ThreadPool::pushBack(Task* task)
{
	EnterCriticalSection(&m_cs);
	CLASSERT( m_taskHead != ((m_taskTail+1)&TASK_MASK) ); // full
	m_tasks[m_taskTail] = task;
	m_taskTail = (m_taskTail+1) & TASK_MASK;
	LeaveCriticalSection(&m_cs);
}

ThreadPool::Task* ThreadPool::pop()
{
	Task* task = NULL;
	EnterCriticalSection(&m_cs);

	if( m_taskHead != m_taskTail )
	{
		task = m_tasks[ m_taskHead ];
		m_taskHead = (m_taskHead+1)&TASK_MASK;
	}

	LeaveCriticalSection(&m_cs);
	return task;
}

ThreadPool::Thread::Thread()
{

}

void ThreadPool::Thread::init(int idx, ThreadPool* threadPool)
{
	m_args.m_threadPool = threadPool;
	m_args.m_idx = idx;

	_beginthreadex(NULL, 0, run, &m_args, 0, &m_threadIdx);

	m_runSignal = CreateEvent(NULL, TRUE, FALSE, NULL);
	m_finSignal = CreateEvent(NULL, TRUE, FALSE, NULL);
}

ThreadPool::Thread::~Thread()
{

}

void ThreadPool::Thread::start()
{
	SetEvent( m_runSignal );
}

void ThreadPool::Thread::timestampReset()
{
	m_nTimestamps = 0;
}

u32 __stdcall ThreadPool::Thread::run(void* args)
{
	ThreadArgs* tArgs = (ThreadArgs*)args;
	ThreadPool* threadPool = tArgs->m_threadPool;
	int idx = tArgs->m_idx;
	Thread* th = &threadPool->m_threads[idx];

	volatile bool& deleteSignal = threadPool->m_deleteSignal;
	while(!deleteSignal)
	{
		WaitForSingleObject(th->m_runSignal, INFINITE);
		ResetEvent( th->m_runSignal );

		if( deleteSignal )
		{
			break;
		}

		{
			Task* task = threadPool->pop();
			while( task )
			{
				LARGE_INTEGER s,e;
				QueryPerformanceCounter( &s );

				task->run( idx );

				QueryPerformanceCounter( &e );

				float start, end;
				start = (float)(1000*(s.QuadPart - threadPool->m_startTime.QuadPart))/threadPool->m_frequency.QuadPart;
				end = (float)(1000*(e.QuadPart - threadPool->m_startTime.QuadPart))/threadPool->m_frequency.QuadPart;
				th->pushBackTimeStamp( task->getType(), start, end );

				delete task;

				task = threadPool->pop();
			}
		}

		SetEvent( th->m_finSignal );
	}

	SetEvent( th->m_finSignal );
	_endthreadex(0);
	return 0;
}

void ThreadPool::Thread::pushBackTimeStamp(u16 type, float s, float e)
{
	if( m_nTimestamps < MAX_TIMESTAMPS-1 )
	{
		m_timestamps[ m_nTimestamps ].m_type = type;
		m_timestamps[ m_nTimestamps ].m_start = s;
		m_timestamps[ m_nTimestamps ].m_end = e;
		m_nTimestamps++;
	}
}

