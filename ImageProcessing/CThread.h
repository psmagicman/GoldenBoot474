#ifndef __CTHREAD__
#define __CTHREAD__

#include <process.h>	// for spawnl and createthread
#include <windows.h>	// for perror and sleep
#include <stdio.h>		// for printf
#include <limits.h>		// for UINT_MAX
#include <conio.h>		// for _kbhit(), getch() and getche()
#include <iostream>
#include <string>

using namespace std ;

#define PerThreadStorage  __declspec(thread)
#define _CRT_SECURE_NO_WARNINGS	

#define SUSPENDED			101000		// for threads
#define ACTIVE				101001		// for thread
#define OWN_WINDOW			101002		// for FORK_PROCESS
#define PARENT_WINDOW		101003		// ditto
#define MAXMESSAGEVALUE		32000		// maximum value of a user generated message

#define MANUAL				101004		// for events
#define AUTORESET			101005		// for events

#define SINGLE_RELEASE		101100		// for events
#define MULTIPLE_RELEASE	101101		// for events

#define SIGNALLED			101006		// for events
#define NOTSIGNALLED		101007		// for events

#define OWNED				101008		// for mutex's
#define NOTOWNED			101009		// for mutex's

#define ECHO_ON()		/* no definition for OS9 compatibility */
#define ECHO_OFF()		/* no definition for OS9 compatibility */

// data structure related to datapools

typedef struct {
	HANDLE DataPoolHandle ;		// handle to datapool
	void *DataPoolPointer ;		// pointer that can be used to access datapool
} DATAPOOLINFO ;



//	Miscellaneous functions
void	SLEEP(UINT	Time);			// suspend current thread for 'Time' mSec
BOOL	TEST_FOR_KEYBOARD();		// tests a keyboard for a key press returns true if key pressed
HANDLE	GET_STDIN();				// get handle to standard input device (keyboard)
HANDLE	GET_STDOUT();				// ditto output device
HANDLE	GET_STDERR();				// ditto erro device
UINT	WAIT_FOR_CONSOLE_INPUT(HANDLE hEvent, DWORD Time = INFINITE);	//wait for console input to happen


void	MOVE_CURSOR(int x, int y) ;	// move console cursor to x,y coord
void	CURSOR_ON() ;				// turn flashing cursor on (the default)
void	CURSOR_OFF() ;				// turn flashing cursor off
void	REVERSE_ON() ;				// turn on inverse video
void	REVERSE_OFF() ;				// turn off inverse video
void	CLEAR_SCREEN() ;			// clears the screen

void PERR(bool bSuccess, string ErrorMessageString) ;


UINT	WAIT_FOR_MULTIPLE_OBJECTS(	UINT nCount,             // number of handles in the handle array
									CONST HANDLE *lpHandles , // pointer to the object-handle array
									DWORD Time = INFINITE
								);

class CThread {
private:
	//##ModelId=3DE6123A0164
	HANDLE	ThreadHandle ;	// handle to thread
	//##ModelId=3DE6123A016E
	UINT	ThreadID ;		// id of thread

protected:
	//##ModelId=3DE6123A0178
	CThread(BOOL bCreateState=SUSPENDED) ;		// default constructor ONLY suitable when CThread is a base class 
												// for Active classes.

public:
	//##ModelId=3DE6123A0182
	CThread(UINT __stdcall Function( void *),	// name/pointer to function that is to be the new thread
			BOOL bCreateState=ACTIVE ,			// use SUSPENDED thread created in suspended state
												// use ACTIVE to make thread active
			void *ThreadArgs=NULL) ;			// any args to the thread


	//##ModelId=3DE6123A018C
	inline virtual ~CThread() { ::TerminateThread(ThreadHandle, 0); } 


	//##ModelId=3DE6123A018E
	inline HANDLE	GetThreadHandle() const { return ThreadHandle ; }
	//##ModelId=3DE6123A0197
	inline UINT	GetThreadID() const { return ThreadID ; }

	//##ModelId=3DE6123A01A0
	inline operator HANDLE() const { return ThreadHandle ; } 
	//##ModelId=3DE6123A01A2
	inline operator UINT()	const { return ThreadID ; }

	//##ModelId=3DE6123A01AA
	BOOL Suspend() const ;					// can be used to suspend the thread
	//##ModelId=3DE6123A01B4
	BOOL Resume() const ;					// can be used to activate a suspended thread
	//##ModelId=3DE6123A01B6
	UINT WaitForThread(DWORD Time=INFINITE) const ;			// caller waits for the thread to terminate
	//##ModelId=3DE6123A01BF
	BOOL SetPriority(UINT Priority) const ;	// caller sets thread priority, see SET_THREAD_PRORITY() in rt.cpp
	//##ModelId=3DE6123A01CA
	BOOL Post(UINT Message) const ;		// caller sends a signal/message to the thread

private:
	//##ModelId=3DE6123A01D4
	void Exit(UINT ExitCode=0) const ;		// called by thread to terminate itself

	inline BOOL TerminateThread(DWORD ExitStatus = 0) { return ::TerminateThread(ThreadHandle, ExitStatus) ; }
} ;
  
// this is a base class for deriving classes that will eventually give rise to objects with their own thread
//	of parallel processing. Just override the virtual function main() and put the code/statemachine for your active object in here just like a C program
//
//	If necessary, create a constructor for your derived class and call the base class constructor to control the active/suspended state of your thread
//	then control the threads activity via the CThread member functions

//##ModelId=3DE6123A0218
class ActiveClass : public CThread  
{	// see Thread related functions in rt.cpp for more details
private:
	BOOL	TerminateFlag ;		// can be polled by class thread to see if parent wants it to terminate

public:
	void	RequestTerminate() { TerminateFlag = TRUE ;}		// set a flag requesting the active class to terminate
	BOOL	TerminateStatus() const { return TerminateFlag ; }	// can be called by active class to see if it should terminate

	//##ModelId=3DE6123A0223
	ActiveClass() ;						// default constructor creates class in suspended state, i.e. not running.
	// this is important as we want to allow the derived 
	//classes constructor i.e. the one
	// written by the user, to complete its constructor 
	//before the class main() is run
	// otherwise the class might run with unitialised 
	//variables/state
	// This means that the user has to call Resume() for the 
	//class object to allow it to run.
	//##ModelId=3DE6123A022C
	virtual ~ActiveClass() ;

	//##ModelId=3DE6123A022E
	virtual int main() = 0;  			// pure virtual, makes this an abstract class therefore it must be overridden in your derived class
} ;
	
//
//	Template class for threads running within a class
//	by default threads created with this code run in either ACTIVE or SUSPENDED state depending up the argument bState
//	The default is to alow them to run, since the containing class will have been constructed and initialised and therefore it is
//	not unsafe. However you can control this if req'd
//

//##ModelId=3DE6123A027C
template <class ClassName>
class ClassThread : 
public ActiveClass  
{
private:
	//##ModelId=3DE6123A0287
	void *ThreadData ;
	
	//##ModelId=3DE6123A0290
	ClassName *PointerToClassInstanceOwningThread ;							// a pointer to the object which is creating the thread
	//##ModelId=3DE6123A029A
	int (ClassName::*PointerToClassFunctionActingAsTheThread)(void *ThreadData) ;		// a pointer to the function acting as the thread. Must belong to the same class as the object pointer above

public:
	// constructor

	//##ModelId=3DE6123A02A4
	ClassThread(ClassName *_PointerToClassInstanceOwningThread,		// pointer to the threads enacpsulating object, usually 'this'
				int (ClassName::*_PointerToClassFunctionActingAsTheThread)(void *ThreadData),	// pointer to 'int function(void *)' function in class acting as the threafd
				int bState,				// run state
				void *_ThreadData		// pointer to thread data
	
	)
		:	PointerToClassInstanceOwningThread(_PointerToClassInstanceOwningThread), 
			PointerToClassFunctionActingAsTheThread(_PointerToClassFunctionActingAsTheThread), 
			ThreadData(_ThreadData),
			ActiveClass()					// state of class thread will be suspened by deault
	{
		if(bState == ACTIVE)
			Resume() ;			// allow class threads to resume if the programmer asked for it, otherwise they control it within main program rather than constructor
	}	

	//##ModelId=3DE6123A02AF
	virtual int main()		// override from ActiveClass and CThread, thus will run when the Resume() function in constructor above is run or later in users program
	{
		return (PointerToClassInstanceOwningThread->*PointerToClassFunctionActingAsTheThread)(ThreadData) ;	// invoke function in dispatcher as an active thread
	}
} ;

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

int TEXT_COLOUR(unsigned char foreground=7, unsigned char background=0);





#endif
