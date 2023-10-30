#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <cmath>
#include <stdlib.h>
#include <bitset>
#include <memory.h>
#include <limits.h>
#include <ctime>

#include <boost/bimap/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp >
#include <boost/bimap/multiset_of.hpp >
#include <boost/optional.hpp >
#include <boost/none.hpp >
#include <boost/foreach.hpp >
#include <boost/assign/list_inserter.hpp >

#include "head/LSketch_NoW.h"

using namespace boost::bimaps;
using namespace boost;
using namespace std;


class LSketch
{
private:
	HGSS_NoW *sketch;
	// chh
	int k;
	int l;		

public:
	typedef bimap<

        unordered_set_of<string>,
        // multiset_of< long, std::greater<long> >
        multiset_of<int>

    > bm_type;
	typedef bm_type::value_type node;

	// heavy hitter
	// key:node hash value, value: weight
	bm_type hh;

	//heavy neighnors of each node
	// node name, hn
	map<string, bm_type> hns;
	// map<string, map<string, int>> hns;

	LSketch(int width, int range, int p_num, int size, int f_num, vector<pair<string, pair<double,double>>> elw, int tablesize, int hh, int hn);
	~LSketch()
	{
		delete[] sketch;
	}
	void update(string s1, string s2, int weight, string l_e);
    void update(string s1, string s2, int weight, string l_A, string l_B, string l_e);

	double time1 = 0;
	double time2 = 0;

};

LSketch::LSketch(int width, int range, int p_num, int size, int f_num, vector<pair<string, pair<double,double>>> elw, int tablesize, int hh, int hn)
{
	sketch = new HGSS_NoW(width, range, p_num, size, f_num, true, elw, tablesize);
	k = hh;
	l = hn;
	hns.clear();
}

void LSketch::update(string s1, string s2, int weight, string l_e)
{
	clock_t start1 = clock();
	sketch->insert(s1, s2, weight, l_e);
	int in_weight = sketch->nodeValueQuery(s2, 1);
	int neighbor_weight = sketch->edgeQuery(s1, s2);
	clock_t end1 = clock();
	time1 += (double)(end1- start1);

 	clock_t start2 = clock();
	if (hh.left.find(s2) != hh.left.end()) //bm_type::left_const_iterator iter 
	{		
		// update weight of s2
		hh.left.erase(s2);
		hh.insert(node(s2, in_weight));

		bm_type temp = hns[s2]; //neighbors of s2
		if (temp.left.find(s1) != temp.left.end())
		{
			temp.left.erase(s1);
			temp.insert(node(s1, neighbor_weight));
		}
		else
		{
			int min_weight = temp.right.begin()->first;
			string min_node = temp.right.begin()->second;
			if(temp.size() == l and min_weight <= neighbor_weight){
				temp.left.erase(min_node);
			}
			if(temp.size() < l){
				temp.insert(node(s1, neighbor_weight));
			}
			
		}
		hns[s2] = temp; // 保证修改在hns
	}
	else
	{
		if (hh.size() == k)
		{
			int min_weight = hh.right.begin()->first;
			string min_node = hh.right.begin()->second;
			if(in_weight >= min_weight)  // >=, 虽然权重一样，但是这个靠近现在的时间点
			{
				hh.left.erase(min_node);
				// hns.erase(min_node);  // 现在不删掉hns了，一直维护着
			}
		}
		if (hh.size() < k) // 如果一个节点之前是hh，后来被挤出去了，那之前的hns可能会遗失（如果后续不出现的话）
		{
			if(hns.count(s2) !=0 ){ // hns已经存在
				bm_type temp = hns[s2]; //neighbors of s2
				if (temp.left.find(s1) != temp.left.end())
				{
					temp.left.erase(s1);
					temp.insert(node(s1, neighbor_weight));
				}
				else
				{
					int min_weight = temp.right.begin()->first;
					string min_node = temp.right.begin()->second;
					if(temp.size() == l and min_weight <= neighbor_weight){
						temp.left.erase(min_node);
					}
					if(temp.size() < l){
						temp.insert(node(s1, neighbor_weight));
					}
				}
				hns[s2] = temp; // 保证修改在hns
			}
			else {
				bm_type temp;  // 之前没有hns
				temp.insert(node(s1,neighbor_weight));
				hns[s2] = temp; // 保证修改在hns
			}
			
			
			hh.insert(node(s2, in_weight));
		}
	}
	clock_t end2 = clock();
	time2 += (double)(end2- start2);
}

void LSketch::update(string s1, string s2, int weight, string l_A, string l_B, string l_e)
{
	clock_t start1 = clock();
	sketch->insert(s1, s2, weight, l_A, l_B, l_e);
	int in_weight = sketch->nodeValueQuery(s2, 1, l_B);
	int neighbor_weight = sketch->edgeQuery(s1, l_A, s2, l_B);
	clock_t end1 = clock();
	time1 += (double)(end1- start1);

 	clock_t start2 = clock();
	if (hh.left.find(s2) != hh.left.end()) //bm_type::left_const_iterator iter 
	{		
		// update weight of s2
		hh.left.erase(s2);
		hh.insert(node(s2, in_weight));

		bm_type temp = hns[s2]; //neighbors of s2
		if (temp.left.find(s1) != temp.left.end())
		{
			temp.left.erase(s1);
			temp.insert(node(s1, neighbor_weight));
		}
		else
		{
			int min_weight = temp.right.begin()->first;
			string min_node = temp.right.begin()->second;
			if(temp.size() == l and min_weight <= neighbor_weight){
				temp.left.erase(min_node);
			}
			if(temp.size() < l){
				temp.insert(node(s1, neighbor_weight));
			}
			
		}
		hns[s2] = temp; // 保证修改在hns
	}
	else
	{
		if (hh.size() == k)
		{
			int min_weight = hh.right.begin()->first;
			string min_node = hh.right.begin()->second;
			if(in_weight >= min_weight)  // >=, 虽然权重一样，但是这个靠近现在的时间点
			{
				hh.left.erase(min_node);
				// hns.erase(min_node);  // 现在不删掉hns了，一直维护着
			}
		}
		if (hh.size() < k) // 如果一个节点之前是hh，后来被挤出去了，那之前的hns可能会遗失（如果后续不出现的话）
		{
			if(hns.count(s2) !=0 ){ // hns已经存在
				bm_type temp = hns[s2]; //neighbors of s2
				if (temp.left.find(s1) != temp.left.end())
				{
					temp.left.erase(s1);
					temp.insert(node(s1, neighbor_weight));
				}
				else
				{
					int min_weight = temp.right.begin()->first;
					string min_node = temp.right.begin()->second;
					if(temp.size() == l and min_weight <= neighbor_weight){
						temp.left.erase(min_node);
					}
					if(temp.size() < l){
						temp.insert(node(s1, neighbor_weight));
					}
				}
				hns[s2] = temp; // 保证修改在hns
			}
			else {
				bm_type temp;  // 之前没有hns
				temp.insert(node(s1,neighbor_weight));
				hns[s2] = temp; // 保证修改在hns
			}
			
			
			hh.insert(node(s2, in_weight));
		}
	}
	clock_t end2 = clock();
	time2 += (double)(end2- start2);
}

