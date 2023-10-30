#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <cmath>
#include <bitset>
#include <memory.h>
#include <limits.h>
#include <ctime>
#include <unordered_map>
#include <math.h>
#include <fstream>
#include <time.h>
#include <cstdlib>
#include <windows.h>
#include <functional>

#include "head/LSketch_NoW.h"
#include "head/BitMap.hpp"

using namespace std;

// #define lu_d 2 // bloom filter的哈希函数个数
// #define bitmapsize 600 // phone
#define bitmapsize 610 // enron

struct node
{
	// uint32_t key;
	int key;
	string label;

public:
	node(uint32_t k, string l)
	{
		key = k;
		label = l;
	}
	node() {} // 不然会有编译错误?
	// bool operator < (const node& t)
	// {
	// 	return key < t.key;
	// }
	friend bool operator<(const node &t1, const node &t2)
	{
		return t1.key < t2.key;
	}
};
// //哈希函数
// struct node_hash
// {
// 	size_t operator()(const node& r1) const
// 	{
// 		return hash<uint32_t>()(r1.key) ^ hash<string>()(r1.label);
// 	}
// };
// //equal相当于重载operator==
// struct node_equal
// {
// 	bool operator()(const node& rc1, const node& rc2) const noexcept
// 	{
// 		return rc1.key == rc2.key && rc1.label == rc2.label;
// 	}

// };

class LSketch
{
private:
	HGSS_NoW *sketch;
	// heap structure
	struct heap_node
	{
		uint32_t key;
		// string label;
		uint32_t value;
	};
	// struct heap_nei
	// {
	// 	uint32_t key;
	// 	uint32_t value;
	// };

	heap_node *heap;
	heap_node *heapN;
	// chh
	int k;
	int l;
	BitMap *bitmap;
	// neighbot list
	// unordered_map<uint32_t, set<node>> neighbors;
	unordered_map<uint32_t, set<uint32_t>> neighbors;

	void Clear()
	{
		for (uint32_t i = 0; i < k; i++)
		{
			heap[i].key = 0;
			heap[i].value = 0;
		}
	}
	void Update_heap();
	void MINHEAPIFY(uint32_t i)
	{
		uint32_t left = 2 * i + 1;
		uint32_t right = 2 * i + 2;
		uint32_t smallest = i;
		if (left < k && heap[left].value < heap[i].value)
		{
			smallest = left;
		}
		if (right < k && heap[right].value < heap[smallest].value)
		{
			smallest = right;
		}
		if (smallest != i)
		{ // swap
			SWAP(i, smallest);
			MINHEAPIFY(smallest);
		}
	}
	void SWAP(int i, int j)
	{

		heap_node temp = heap[i];
		heap[i] = heap[j];
		heap[j] = temp;
	}
	void MINHEAPIFYN(uint32_t i)
	{
		uint32_t left = 2 * i + 1;
		uint32_t right = 2 * i + 2;
		uint32_t smallest = i;
		if (left < l && heapN[left].value < heapN[i].value)
		{
			smallest = left;
		}
		if (right < l && heapN[right].value < heapN[smallest].value)
		{
			smallest = right;
		}
		if (smallest != i)
		{ // swap
			SWAPN(i, smallest);
			MINHEAPIFYN(smallest);
		}
	}
	void SWAPN(int i, int j)
	{
		heap_node temp = heapN[i];
		heapN[i] = heapN[j];
		heapN[j] = temp;
	}

public:
	LSketch(int width, int range, int p_num, int size, int f_num, vector<pair<string, pair<double, double>>> elw, int tablesize, int hh, int hn);
	~LSketch()
	{
		delete[] sketch;
		delete[] heap;
		delete[] heapN;
		delete bitmap;
	}
	void update(string s1, string s2, int weight, string l_e);

	map<uint32_t, int> query_topk();
	map<uint32_t, int> query_neighbors(uint32_t key);
};

LSketch::LSketch(int width, int range, int p_num, int size, int f_num, vector<pair<string, pair<double, double>>> elw, int tablesize, int hh, int hn)
{
	sketch = new HGSS_NoW(width, range, p_num, size, f_num, true, elw, tablesize);
	k = hh;
	l = hn;

	heap = new heap_node[k];
	heapN = new heap_node[l];
	bitmap = new BitMap(bitmapsize);
	bitmap->init();
	Clear();
}

void LSketch::update(string s1, string s2, int weight, string l_e)
{
	clock_t start1 = clock();
	sketch->insert(s1, s2, weight, l_e);
	int in_weight = sketch->nodeValueQuery(s2, 1);
	int neighbor_weight = sketch->edgeQuery(s1, s2);
	clock_t end1 = clock();

	uint32_t key = stoi(s2);
	// 维护neighbor list
	if (neighbors.count(key) == 0)
	{
		set<uint32_t> nei;
		nei.insert(stoi(s1));
		neighbors[key] = nei;
	}
	else
	{
		set<uint32_t> nei = neighbors[key];
		nei.insert(stoi(s1));
		neighbors[key] = nei;
	}

	// part1 bitmap判断s2是否是候选topk
	// flag == 1 means that the packet stored in heap, so don't need other operations.
	uint32_t flag = bitmap->find(key);
	if (flag == 0)
	{
		if (heap[0].value < in_weight)
		{
			// 这里是有一次拯救，heap的权重可能不是实时的，更新一次
			Update_heap(); // update HEAP, and flag in CM
			for (int t = floor(k / 2); t >= 0; t--)
			{
				MINHEAPIFY(t);
			}
			flag = bitmap->find(key);
			if (flag == 0 && in_weight > heap[0].value) // heap确实需要更新，淘汰最少的
			{
				bitmap->remove(heap[0].key); // 删除原来的heap[0]
				bitmap->add(key);
				heap[0].key = key;
				heap[0].value = in_weight;
				MINHEAPIFY(0);
			}
		}
	}
}

void LSketch::Update_heap()
{
	uint32_t tmp_value;
	uint32_t tmp_index;
	for (uint32_t i = 0; i < k; i++)
	{
		bitmap->add(heap[i].key); // set flag
		int weight = sketch->nodeValueQuery(to_string(heap[i].key), 1);
		heap[i].value = weight; // update weight
	}
}

// map<uint32_t, int> LSketch::query_topk_label()
// {
// 	map<uint32_t, int> result;
// 	cout << "result:" << endl;
// 	// update
// 	Update_heap_label();
// 	for (int t = floor(k / 2); t >= 0; t--)
// 		MINHEAPIFY(t);
// 	// output result
// 	uint32_t key;
// 	for (uint32_t i = 0; i < k; i++)
// 	{
// 		// cout << "key: " << key << "\t";
// 		// cout << "value: " << heap[0].value << endl;
// 		result.insert(make_pair(heap[0].key, heap[0].value));
// 		heap[0].key = 0;
// 		heap[0].value = 1 << 29;
// 		MINHEAPIFY(0);
// 	}
// 	return result;
// }

map<uint32_t, int> LSketch::query_topk()
{
	map<uint32_t, int> result;
	Update_heap();
	for (int t = floor(k / 2); t >= 0; t--)
		MINHEAPIFY(t);
	// output result
	uint32_t key;
	for (uint32_t i = 0; i < k; i++)
	{
		// cout << "key: " << key << "\t";
		// cout << "value: " << heap[0].value << endl;
		result.insert(make_pair(heap[0].key, heap[0].value));
		heap[0].key = 0;
		heap[0].value = 1 << 29;
		MINHEAPIFY(0);
	}
	return result;
}

map<uint32_t, int> LSketch::query_neighbors(uint32_t node)
{
	map<uint32_t, int> result;
	// cout << "result:";
	// 构建neighbor的heap
	// heap_nei *heapN = new heap_nei[l];
	// 初始化
	for (uint32_t i = 0; i < l; i++)
	{
		heapN[i].key = 0;
		heapN[i].value = 0;
	}
	set<uint32_t> tmp = neighbors[node];

	for (set<uint32_t>::iterator it = tmp.begin(); it != tmp.end(); ++it)
	{
		int neighbor_weight = sketch->edgeQuery(to_string((*it)), to_string((node)));
		if (heapN[0].value < neighbor_weight)
		{
			heapN[0].key = (*it);
			heapN[0].value = neighbor_weight;
			MINHEAPIFYN(0);
		}
	}
	// output result
	uint32_t key;
	uint32_t value;
	for (uint32_t i = 0; i < l; i++)
	{
		if (heapN[0].key != 0)
			result.insert(make_pair(heapN[0].key, heapN[0].value));
		// 删掉
		heapN[0].key = 0;
		heapN[0].value = 1 << 29;
		MINHEAPIFYN(0);
	}
	return result;
}
