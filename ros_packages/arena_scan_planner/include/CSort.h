/*
 * CSort.h
 *
 *  Created on: 25. 10. 2015
 *      Author: Robert Penicka
 */

#ifndef SRC_COMMON_CSORT_H_
#define SRC_COMMON_CSORT_H_
#include <vector>
//#define SORT_GET_VALUE(element) element->getValue()

template<typename T>
struct SortCompareVar {
	int (*greaterThan)(T, T, struct SortCompareVar<T>);
	void * data;
};

template<class T>
class CSort {

public:
	CSort();
	virtual ~CSort();
	static void quicksort(std::vector<T> * array, int left_begin, int right_begin);
	static void quicksort(std::vector<T> * array, int left_begin, int right_begin, struct SortCompareVar<T> sortCompare);
	static void quicksort(std::vector<T*> * array, int left_begin, int right_begin);
	static int binarySearchInOrdered(std::vector<T> * array, T toFind);
	static void insertInOrdered(std::vector<T> * array, T toInsert);
	static void insertInOrdered(std::vector<T> * array, T toInsert, struct SortCompareVar<T> sortCompare);
	//static void CSort::quicksort(std::vector array[], int left_begin, int right_begin);
};

template<class T>
CSort<T>::CSort() {

}

template<class T>
CSort<T>::~CSort() {

}

template<class T>
void CSort<T>::quicksort(std::vector<T> * array, int left_begin, int right_begin) {
	//WINFO("b quicksort "<<left_begin <<" "<<right_begin);
	T pivot = (*array)[(left_begin + right_begin) / 2];
	//WINFO("pivot val "<< SORT_GET_VALUE(pivot));
	int left_index = left_begin;
	int right_index = right_begin;
	T temp;
	do {
		//WINFO("looop");
		while ((*array)[left_index] < pivot && left_index < right_begin)
			left_index++;
		while ((*array)[right_index] > pivot && right_index > left_begin)
			right_index--;

		//WVARIABLE(left_index);
		//WVARIABLE(right_index);
		if (left_index <= right_index) {
			//	WINFO("switch "<<left_index<<" "<<right_index);
			temp = (*array)[left_index];
			(*array)[left_index++] = (*array)[right_index];
			(*array)[right_index--] = temp;
		}
	} while (left_index < right_index);

	if (right_index > left_begin)
		quicksort(array, left_begin, right_index);
	if (left_index < right_begin)
		quicksort(array, left_index, right_begin);
}

template<class T>
void CSort<T>::quicksort(std::vector<T> * array, int left_begin, int right_begin, struct SortCompareVar<T> sortCompare) {
	if (right_begin < 0)
		return;

	//INFO("b quicksort "<<left_begin <<" "<<right_begin);
	T pivot = (*array)[(left_begin + right_begin) / 2];
	//INFO("right_begin "<< right_begin);
	//INFO("left_beginl "<< left_begin);
	int left_index = left_begin;
	int right_index = right_begin;
	T temp;
	//int (*f)(T, T,void * data) =;
	do {
		//WINFO("looop");
		while ((sortCompare.greaterThan)((*array)[left_index], pivot, sortCompare) == -1 && left_index < right_begin)
			left_index++;
		while ((sortCompare.greaterThan)((*array)[right_index], pivot, sortCompare) == 1 && right_index > left_begin)
			right_index--;

		//WVARIABLE(left_index);
		//WVARIABLE(right_index);
		if (left_index <= right_index) {
			//	WINFO("switch "<<left_index<<" "<<right_index);
			temp = (*array)[left_index];
			(*array)[left_index++] = (*array)[right_index];
			(*array)[right_index--] = temp;
		}
	} while (left_index < right_index);

	if (right_index > left_begin)
		quicksort(array, left_begin, right_index, sortCompare);
	if (left_index < right_begin)
		quicksort(array, left_index, right_begin, sortCompare);
}

template<class T>
void CSort<T>::quicksort(std::vector<T*> * array, int left_begin, int right_begin) {
	//WINFO("b quicksort "<<left_begin <<" "<<right_begin);
	T * pivot = (*array)[(left_begin + right_begin) / 2];
	//WINFO("pivot val "<< SORT_GET_VALUE(pivot));
	int left_index = left_begin;
	int right_index = right_begin;
	T * temp;
	do {
		//WINFO("looop");
		while ((*(*array)[left_index]) < (*pivot) && left_index < right_begin)
			left_index++;
		while ((*(*array)[right_index]) > (*pivot) && right_index > left_begin)
			right_index--;

		//WVARIABLE(left_index);
		//WVARIABLE(right_index);
		if (left_index <= right_index) {
			//	WINFO("switch "<<left_index<<" "<<right_index);
			temp = (*array)[left_index];
			(*array)[left_index++] = (*array)[right_index];
			(*array)[right_index--] = temp;
		}
	} while (left_index < right_index);

	if (right_index > left_begin)
		quicksort(array, left_begin, right_index);
	if (left_index < right_begin)
		quicksort(array, left_index, right_begin);
}

template<class T>
int CSort<T>::binarySearchInOrdered(std::vector<T> * array, T toFind) {
	int Mid;
	int Lbound = 0;
	int Ubound = array->size() - 1;
	//WINFO("search "<<toFind);
	int positionToInsert = 0;
	while (Lbound <= Ubound) {

		Mid = (Lbound + Ubound) / 2;
		//WVARIABLE(Mid);
		if (toFind > (*array)[Mid]) {
			Lbound = Mid + 1;
			//WVARIABLE(Lbound);
		} else if (toFind < (*array)[Mid]) {
			Ubound = Mid - 1;
			//WVARIABLE(Ubound);
		} else {
			return Mid;
		}
	}
	return -1;
}

template<class T>
void CSort<T>::insertInOrdered(std::vector<T> * array, T toInsert) {

	int Mid;
	int Lbound = 0;
	int Ubound = array->size() - 1;
	int positionToInsert = 0;
	while (Lbound <= Ubound) {
		Mid = (Lbound + Ubound) / 2;
		if (toInsert > (*array)[Mid]) {
			Lbound = Mid + 1;
			positionToInsert = Lbound;
			//WVARIABLE(Lbound);
		} else if (toInsert < (*array)[Mid]) {
			Ubound = Mid - 1;
			positionToInsert = Ubound;
			//WVARIABLE(Ubound);
		} else {
			//WINFO("mid");
			positionToInsert = Mid;
			break;
		}
	}

	if (Lbound > Ubound) {
		positionToInsert = Lbound;
	}

	if (positionToInsert < 0) {
		positionToInsert = 0;
	}

	array->insert(array->begin() + positionToInsert, toInsert);
}

template<class T>
void CSort<T>::insertInOrdered(std::vector<T> * array, T toInsert, struct SortCompareVar<T> sortCompare) {
	/*
	 WINFO("insertInOrdered " << toInsert);
	 for (int var2 = 0; var2 < array->size(); ++var2) {
	 WINFO(var2 << " " << (*array)[var2]);
	 }
	 */
	int Mid;
	int Lbound = 0;
	int Ubound = array->size() - 1;
	int positionToInsert = 0;
	while (Lbound <= Ubound) {
		Mid = (Lbound + Ubound) / 2;
		if ((sortCompare.greaterThan)(toInsert, (*array)[Mid], sortCompare) == 1) {
			Lbound = Mid + 1;
			positionToInsert = Lbound;
			//WVARIABLE(Lbound);
		} else if ((sortCompare.greaterThan)(toInsert, (*array)[Mid], sortCompare) == -1) {
			Ubound = Mid - 1;
			positionToInsert = Ubound;
			//WVARIABLE(Ubound);
		} else {
			//WINFO("mid");
			positionToInsert = Mid;
			break;
		}
	}

	if (Lbound > Ubound) {
		positionToInsert = Lbound;
	}

	if (positionToInsert < 0) {
		positionToInsert = 0;
	}

	//WINFO("inserting " << toInsert << " to " << positionToInsert);
	array->insert(array->begin() + positionToInsert, toInsert);

	for (int var = 0; var < array->size() - 1; ++var) {
		if ((*array)[var] < (*array)[var + 1]) {
			std::cout << "mega chyba razeni kurva" << std::endl;
			for (int var2 = 0; var2 < array->size(); ++var2) {
				std::cout << var2 << " " << (*array)[var2] << std::endl;
			}
			exit(0);
		}
	}

}

#endif /* SRC_COMMON_CSORT_H_ */
