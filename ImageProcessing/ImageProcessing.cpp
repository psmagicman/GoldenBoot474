#include "GLOBALS.h"
#include "rt.h"

int main()
{
	CProcess webcam1(
		"Webcam1.exe",
		NORMAL_PRIORITY_CLASS,
		OWN_WINDOW,
		ACTIVE
		);
	webcam1.Resume();

	Sleep(1000);
	/*
	CProcess webcam2(
		"Webcam2.exe",
		NORMAL_PRIORITY_CLASS,
		OWN_WINDOW,
		ACTIVE
		);

	webcam2.Resume();
	*/

	webcam1.WaitForProcess();
	//webcam2.WaitForProcess();

	return 0;
}