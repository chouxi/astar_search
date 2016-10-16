#ifndef _MIN_HEAP_H_
#define _MIN_HEAP_H_

#pragma once
#include <vector>
#include <functional>
#define STARTIND 1
using namespace std;
template <class ElemType, class Find = equal_to<ElemType>, class Compare = less<ElemType>>
class min_heap
{
private:
	vector<ElemType> heap_vec;//heap vector
	Compare comp;// compare function
	Find find; //finding function
	void percolate_up(int hole, ElemType ele);
	void percolate_down(int hole, ElemType ele);
public:
	min_heap();
	~min_heap();
	void push_heap(ElemType ele);
	ElemType pop_heap();
	//get min_key but not pop it.
	ElemType top();
	int find_heap(ElemType ele);
	void remove_heap(ElemType ele);
	void print_heap();
	bool is_empty();
};

template <class ElemType, class Find, class Compare >
min_heap<ElemType, Find, Compare>::min_heap() {
	//start from 1, easy to calculate
	heap_vec.push_back(ElemType{});
}

template <class ElemType, class Find, class Compare>
min_heap<ElemType, Find, Compare>::~min_heap() {}

template <class ElemType, class Find, class Compare>
void min_heap<ElemType, Find, Compare>::push_heap(ElemType ele) {
	heap_vec.push_back(ele);
	percolate_up(heap_vec.size() - 1, ele);
}

template <class ElemType, class Find, class Compare>
ElemType min_heap<ElemType, Find, Compare>::pop_heap() {
	if (heap_vec.size() <= 1)
		return ElemType{};
	ElemType min = heap_vec[STARTIND];
	heap_vec[STARTIND] = heap_vec.back();
	//need to minus one from size
	heap_vec.pop_back();
	if (heap_vec.size() >= STARTIND + 1)
		percolate_down(STARTIND, heap_vec[STARTIND]);
	return min;
}

template<class ElemType, class Find, class Compare>
inline ElemType min_heap<ElemType, Find, Compare>::top()
{
	if (heap_vec.size() <= 1)
		return ElemType{};
	return heap_vec[STARTIND];
}

template <class ElemType, class Find, class Compare>
int min_heap<ElemType, Find, Compare>::find_heap(ElemType ele) {
	for (int i = STARTIND; i < heap_vec.size(); i++) {
		if (find(ele, heap_vec[i]))
			return i;
	}
	return -1;
}

template <class ElemType, class Find, class Compare>
void min_heap<ElemType, Find, Compare>::remove_heap(ElemType ele) {
	int index = find_heap(ele);
	//No such element
	if (index < 1 || heap_vec.size() < 2) return;
	heap_vec[index] = heap_vec.back();
	//need to minus one from size
	heap_vec.pop_back();
	if(index < heap_vec.size() - 1)
		percolate_down(index, heap_vec[index]);
}

//when insert
template <class ElemType, class Find, class Compare>
void min_heap<ElemType, Find, Compare>::percolate_up(int hole, ElemType ele) {
	for (; hole > STARTIND && comp(ele, heap_vec[hole / 2]); hole /= 2)
		heap_vec[hole] = heap_vec[hole / 2];
	heap_vec[hole] = ele;
}

//When remove
template <class ElemType, class Find, class Compare>
void min_heap<ElemType, Find, Compare>::percolate_down(int hole, ElemType ele) {
	if (heap_vec.size() <= STARTIND + 1 || hole >= heap_vec.size())
		return;
	int child_index;
	for (; hole * 2 <= heap_vec.size() - 1; hole = child_index) {
		child_index = hole * 2;
		//compare two child
		if (child_index < heap_vec.size() - 2 && comp(heap_vec[child_index + 1], heap_vec[child_index])) {
			//to the one smaller
			child_index++;
		}
		//compare child and element
		if (comp(heap_vec[child_index], ele)) {
			//swap if child small
			heap_vec[hole] = heap_vec[child_index];
		}
		else
			break;
	}
	heap_vec[hole] = ele;
}
template <class ElemType, class Find, class Compare>
void min_heap<ElemType, Find, Compare>::print_heap() {
	for (int i = STARTIND; i < heap_vec.size(); i++) {
		cout << heap_vec[i] << ",";
	}
	cout << endl;
}

template <class ElemType, class Find, class Compare>
bool min_heap<ElemType, Find, Compare>::is_empty() {
	return heap_vec.size() < 2;
}
#endif // !_MIN_HEAP_H_
