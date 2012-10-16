//#include <StdAfx.h>		// delete this line if you are NOT using Microsoft MFC in your project
						// for example, delete this line if you are creating a Console project
						// but if you are creating a GUI or window type application, leave it in

#include "CThread.h"

CThread::CThread(UINT __stdcall Function( void *), // name/pointer to function that is to be the new thread
				 BOOL bCreateState,		// A flag indicating if the thread should commence SUSPENDED (TRUE) or ACTIVE (FALSE)
				 void *ThreadArgs		// a generic pointer (can point to anything) to any data the calling thread
										// wishes to pass to the child thread
					 )
{
	UINT		ThreadControlFlags = 0 ;

	if(bCreateState == SUSPENDED)		// if caller wants thread initially suspended
		ThreadControlFlags = CREATE_SUSPENDED ;

	ThreadHandle = (HANDLE)(_beginthreadex(NULL, 0, Function, ThreadArgs, ThreadControlFlags, &ThreadID)) ;
	PERR( ThreadHandle != 0, string("Unable to Create Thread")) ;	// check for error and print message if appropriate
}


/*
**	The following two functions work together to assist in the design of CThread derived classes with a main() to simulate the main of the thread
**	All active class objects have their root thread within this function
*/

UINT __stdcall __GlobalThreadMain__(void *theThreadPtr) 	// receives a pointer to the thread object 
{
	ExitThread(((ActiveClass *)(theThreadPtr))->main()) ;	// run the activeclass virtual main function it should be overridden in derived class
	return 0 ;
}


//##ModelId=3DE6123A0178
CThread::CThread(BOOL bCreateState)			// A flag indicating if the thread should commence SUSPENDED (TRUE) or ACTIVE (FALSE)
{
	UINT		ThreadControlFlags = 0 ;

	if(bCreateState == SUSPENDED)		// if caller wants thread initially suspended
		ThreadControlFlags = CREATE_SUSPENDED ;

	ThreadHandle = (HANDLE)(_beginthreadex(NULL, 
							0, 
							__GlobalThreadMain__,	
							this,				//pointer to cthread derived object passed to __GlobalThreadMain__() above
							ThreadControlFlags,	// thread is created in suspended state 
							&ThreadID)) ;

	PERR( ThreadHandle != 0, string("Unable to Create Thread")) ;	// check for error and print message if appropriate
}


//##ModelId=3DE6123A0223
ActiveClass::ActiveClass()
: TerminateFlag(FALSE) ,
CThread()		// Call base class constructor to create the SUSPENDED thread, i.e. main in derived class does not yet run
				// All objects derived from active class are thus initially created in the suspended state and have to
				// be RESUMED before they become active. This is the safest way, as it allows control over the
				// order in which objects run and avoids potential race hazards.
{}				// Note that active class objects like all derived classes are constructed from the base upwards 
				// and thus we have to make sure that base/derived class constructors are run to completion
				// before the thread for the class (which is created in the base class) is allowed to resume
				// otherwise the thread could run BEFORE the active class object had completed its construction
				// this was the main reason for creating all ActiveClass objects in the Suspended state so don't change it
				// I learn't the hard way and it was painful to debug !!!


// this is important as we want to allow the derived 
//classes constructor i.e. the one
// written by the user, to complete its constructor 
//before the class main() is run
// otherwise the class might run with unitialised 
//variables/state
// This means that the user has to call Resume() for the 
//class object to allow it to run.
//##ModelId=3DE6123A022C
ActiveClass::~ActiveClass()
{}



//
//	The following can be used to terminate a thread at any point in the thread execution
//	If the thread reaches the end of its 'thread function' then executing return 0 does the same trick.
//

//##ModelId=3DE6123A01D4
void CThread::Exit(UINT	ExitCode) const
{
	ExitThread(ExitCode) ;
}
	
/////////////////////////////////////////////////////////////////////////////////////////////
//	These two functions can be called to suspend and resume a threads activity. Once suspended
//	the thread will no longer run and consume CPU time
//
//	Note that if you suspend a thread more than once, then it will require that many number
//	of resumes to allow it to continue
//
//	Both functions require the thread HANDLE, which can be obtained when the thread is created
///////////////////////////////////////////////////////////////////////////////////////////////

//##ModelId=3DE6123A01AA
BOOL CThread::Suspend() const
{
	UINT	Result = SuspendThread(ThreadHandle) ;
	PERR( Result != 0xffffffff, string("Cannot Suspend Thread\n")) ;	// check for error and print message if appropriate

	if(Result != 0xffffffff)
		return TRUE ;
	else
		return FALSE ;
}

//##ModelId=3DE6123A01B4
BOOL CThread::Resume() const
{
	UINT	Result = ResumeThread(ThreadHandle) ;
	PERR( Result != 0xffffffff, string("Cannot Resume Thread\n")) ;	// check for error and print message if appropriate

	if(Result != 0xffffffff)
		return TRUE ;
	else
		return FALSE ;
}

////////////////////////////////////////////////////////////////////////////////////////////
//	The following function can be used to change the priority of a thread once it has been
//	created (irrespective of whether it is active or suspended). Threads with higher priority
//	will block/stop threads with lower priority from executing so be careful.
//
//	All you need is the handle for the thread obtained from CREATE_THREAD() (see above)
//	Windows NT has a limited number of thread priorities and the following symbolic
//	constants can be used as a valid value for the parameter 'Priority' in the function
//	Normally a thread will inherit a 'base' priority from its parent thread, or, if it is
//	the primary thread, from its parent process (See FORK_PROCESS() above
//	The constant below simply allow you to make small adjustments to the thread priority
/////////////////////////////////////////////////////////////////////////////////////////////
//
//	THREAD_PRIORITY_ABOVE_NORMAL  = Indicates 1 point above normal priority for the priority class. 
//	THREAD_PRIORITY_BELOW_NORMAL  = Indicates 1 point below normal priority for the priority class. 
//	THREAD_PRIORITY_HIGHEST = Indicates 2 points above normal priority for the priority class. 
//	THREAD_PRIORITY_IDLE = Indicates a base priority level of 1 for IDLE_PRIORITY_CLASS, NORMAL_PRIORITY_CLASS, or HIGH_PRIORITY_CLASS processes, and a base priority level of 16 for REALTIME_PRIORITY_CLASS processes. 
//	THREAD_PRIORITY_LOWEST = Indicates 2 points below normal priority for the priority class. 
//	THREAD_PRIORITY_NORMAL = Indicates normal priority for the priority class. 
//	THREAD_PRIORITY_TIME_CRITICAL = Indicates a base priority level of 15 for IDLE_PRIORITY_CLASS, NORMAL_PRIORITY_CLASS, or HIGH_PRIORITY_CLASS processes, and a base priority level of 31 for REALTIME_PRIORITY_CLASS processes. 
/////////////////////////////////////////////////////////////////////////////////////////////

//##ModelId=3DE6123A01BF
BOOL CThread::SetPriority(UINT Priority) const
{	
	BOOL Success;

	// check for error in priority value and print message if appropriate

	PERR(((Priority == THREAD_PRIORITY_ABOVE_NORMAL) ||
			(Priority == THREAD_PRIORITY_BELOW_NORMAL) ||
			(Priority == THREAD_PRIORITY_HIGHEST) ||
			(Priority == THREAD_PRIORITY_IDLE) ||
			(Priority == THREAD_PRIORITY_LOWEST) ||
			(Priority == THREAD_PRIORITY_NORMAL) ||
			(Priority == THREAD_PRIORITY_TIME_CRITICAL)) ,
			string("Illegal Priority value specified for Thread in call to CThread::SetPriority()")) ;

	Success = SetThreadPriority(ThreadHandle, Priority) ;	// set priority

	PERR( Success == TRUE, string("Cannot Set Thread Priority\n")) ;	// check for error and print message if appropriate
	
	return Success ;
}


//
//	This function will wait for the child thread specified by ThreadHandle to terminate
//	main or parent threads can use this to wait for their child thread to terminate
//
//	A time period can be specified indicating the maximum time to attempt a wait
//
//	This function returns WAIT_FAILED if there was an error, WAIT_TIMEOUT if the wait operation timed out
//	of WAIT_OBJECT_0 if the operation did in fact wait and then returned
//

//##ModelId=3DE6123A01B6
UINT CThread::WaitForThread(DWORD Time) const		
{
	UINT Result = WaitForSingleObject(ThreadHandle, Time) ;	// return WAIT_FAILED on error
	PERR( Result != WAIT_FAILED, string("Cannot Wait For Thread")) ;	// check for error and print error message as appropriate

	return Result ;
}

//	The following function sends a message to a thread
//	If the thread is currently suspended, waiting for a message, then posting it a message
//	will wake it up and allow it to continue
//
//	Message must bve in the range 0 - 32000
//	In order to use messages, the thread receiving the message must have created a message queue.
//	(See CMessageQueue later )
//

//##ModelId=3DE6123A01CA
BOOL CThread::Post(UINT Message) const	// message value and ID of thread
{
	BOOL Result ;

	if(Message > 32000)	{
		PERR( Message <= 32000, string("Could not Post User Message: Message > 32000")) ;	// check for error and print error message as appropriate
		return FALSE ;
	}
	else {
		Result = PostThreadMessage(ThreadID, WM_USER + Message, 0, 0L) ;
		PERR( Result != 0, string("Could not Post User Message. The Thread might have died")) ;	// check for error and print error message as appropriate
		return Result ;
	}
}

//
//	This function suspends the current thread until the specified time has elapsed
//	This allows other processes with the same priority to run. Co-operative processing
//	can be implemented with the help of this function,as processes voluntarily relinquish
//	control of the CPU. If the delay is 0, the processes simply gives up the remainder of
//	its CPU time. If the value is 'INFINITE', then the process waits forever. 
//	If the delay is somewhere inbetween, the process is suspended for the duration specified
//	sending it a message will not wake it up prematurely.
//

void	SLEEP(UINT	Time) 
{
	Sleep(Time) ;
}

//
//	This function tests the keyboard to see if a key has been pressed. If so, the value TRUE is
//	returned and the thread can read the character without getting suspended using a function
//	such as _getch(), _getche() etc.
//

BOOL	TEST_FOR_KEYBOARD()
{
	return kbhit() ;
}


//
//	These functions returns handles to a console standard input/output and error device
//	i.e. keyboard, screen and screen respectively. NULL is returned on error
//	For more example of console functions read the Vis C++ Help/Win32 functions by category/console functions
//

HANDLE	GET_STDIN()
{
	return GetStdHandle(STD_INPUT_HANDLE) ;
}

HANDLE	GET_STDOUT()
{
	return GetStdHandle(STD_OUTPUT_HANDLE) ;
}

HANDLE	GET_STDERR()
{
	return GetStdHandle(STD_ERROR_HANDLE) ;
}

//
//	This function pauses the thread/process until user input is entered at the keyboard
//
//	This function returns WAIT_FAILED if there was an error, WAIT_TIMEOUT if the wait operation timed out
//	of WAIT_OBJECT_0 if the operation did in fact wait and then returned
//

UINT WAIT_FOR_CONSOLE_INPUT(HANDLE hEvent, DWORD Time)
{
	UINT	Status = WaitForSingleObject(hEvent, Time) ;
	PERR( Status != WAIT_FAILED, string("Cannot Wait for Console Input")) ;	// check for error and print message if appropriate
	return Status ;
}

//	moves the cursor to the x,y coord on the screen. [0,0] is top left
//	all calls to printf cause output to occur at the current cursor position
//	obviously, the cursor moves with text output operations

void MOVE_CURSOR(int x, int y)
{
	COORD	c = {(short)x, (short)y}  ;
	SetConsoleCursorPosition(GET_STDOUT(), c) ;
}

//
//	These two function turns off/on the cursor so that it is not visible
//	but it still operates and output text still occurs at the current curso
//	position
//

void CURSOR_OFF()
{
	CONSOLE_CURSOR_INFO	cci = {1, FALSE} ;
	SetConsoleCursorInfo(GET_STDOUT(), &cci) ;
}

void CURSOR_ON()
{
	CONSOLE_CURSOR_INFO	cci = {1, TRUE} ;
	SetConsoleCursorInfo(GET_STDOUT(), &cci) ;
}

//
//	These two functions turns on/off reverse video, so that text comes black on white background
//	For more details set the SetConsoleTextAttribute() function in on-line
//	help
//

void REVERSE_ON()
{
	SetConsoleTextAttribute(GET_STDOUT(), 
		BACKGROUND_RED | BACKGROUND_GREEN | BACKGROUND_BLUE) ;
}

void REVERSE_OFF()
{
	SetConsoleTextAttribute(GET_STDOUT(), 
		FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE) ;
}

void CLEAR_SCREEN()
{
	for(int i = 0; i < 50; i ++)
		putchar('\n') ;
}

/*Contains a function to change the text colour.

  To Use:

  This function takes 2 arguments, foreground and background.
  foreground and background select one of 15 colours numbered as follows:

  0 - Black
  1 - Dark Blue
  2 - Dark Green
  3 - Dark Cyan
  4 - Dark Red
  5 - Dark Magenta
  6 - Dark Yellow
  7 - Grey
  8 - Black (again)
  9 - Blue
  10 - Green
  11 - Cyan
  12 - Red
  13 - Magenta
  14 - Yellow
  15 - White

  Note... background and foreground cannot be the same value, and an
  error (-1) will be returned.

  If colours are valid, and operation was successful, 0 will be returned.

  Use TEXT_COLOUR with no arguements to return values to default.

  A foreground colour only can be specified, eg TEXT_COLOUR(4) will make text dark red .*/  

int TEXT_COLOUR(unsigned char foreground, unsigned char background)
{
	if ((foreground>15)||(foreground<0)||(background>15)||(background<0)||(background==foreground))
	{
		return -1;
	}
	int colour=0;
	background=background<<4;
	colour=colour|background|foreground;
	SetConsoleTextAttribute(GET_STDOUT(), colour);
	return 0;
}


void flush(istream &is)		// can be used to flush an input stream, useful for removing operator entered rubbish
{
	is.clear() ;
	streambuf  *ptr = is.rdbuf() ;		// get pointer to stream buffer
	int avail = ptr->in_avail() ;			// get number of chars waiting to be read
	is.ignore(avail) ;						// ignore this many characters
	is.clear() ;
}

void PERR(bool bSuccess, string ErrorMessageString)
{		
	UINT LastError = GetLastError() ;

	if(!(bSuccess)) {
		char buff[512] ;
		Beep(500, 100);
		MOVE_CURSOR(0,0) ;
		REVERSE_ON() ;
		FormatMessage( FORMAT_MESSAGE_FROM_SYSTEM, NULL, LastError,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), buff, 1024, NULL );
		printf(" Error %d in Process %s: line %d:\n", LastError, __FILE__, __LINE__);
		printf(" Translation: %s Error: %s", buff, ErrorMessageString.c_str()) ;
		REVERSE_OFF() ;
		printf("\n\nPress Return to Continue...") ;
		getch();
	}
}