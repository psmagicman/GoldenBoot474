#ifndef __QUICKSORT__
#define __QUICKSORT__

class QuickSort
{
public:
	QuickSort();

	void sort(int* array, int startIndex, int endIndex);

private:
	int SplitArray(int* array, int pivotValue, int startIndex, int endIndex);
	void swap(int &a, int &b);
};

#endif