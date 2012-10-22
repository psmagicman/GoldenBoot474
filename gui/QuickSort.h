#ifndef QUICKSORT
#define QUICKSORT

class QuickSort
{
public:
	void QuickSort(int* array, int startIndex, int endIndex);
	int SplitArray(int* array, int pivotValue, int startIndex, int endIndex);
	void swap(int &a, int &b);
};

#endif