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
#include "head/TCM.h"
using namespace std;

struct cmpfortcm
{
	bool operator()(const pair<int, string> &a, const pair<int, string> &b)
	{
		return a.first > b.first; //小顶堆
	}
};
// bool cmp(const pair<int, string> &a, const pair<int, string> &b){
// 	return a.first > b.first; //小顶堆
// }

class TCMLARGE
{
private:
	TCM *sketch;
	// chh
	int k;
	int l;

public:
	// heavy hitter
	priority_queue<pair<int, string>, vector<pair<int, string>>, cmpfortcm> hh;
	set<string> nodes;

	// heavy neighnors of each node
	map<string, priority_queue<pair<int, string>, vector<pair<int, string>>, cmpfortcm>> hns;
	map<string, set<string>> neighbors;

	TCMLARGE(int width, int v, int tablesize, int hh, int hn); // v是副本数量
	~TCMLARGE()
	{
		delete[] sketch;
	}
	void update(string s1, string s2, int weight);

	double time1 = 0;
	double time2 = 0;
};

TCMLARGE::TCMLARGE(int width, int v, int tablesize, int hh, int hn)
{
	sketch = new TCM(width, width, v, true, tablesize);
	k = hh;
	l = hn;
}

void TCMLARGE::update(string s1, string s2, int weight)
{
	clock_t start1 = clock();
	// 类型转换
	const unsigned char *node1 = (unsigned char *)(s1.c_str()); 
	const unsigned char *node2 = (unsigned char *)(s2.c_str());
	int len1 = s1.length();
	int len2 = s2.length();

	sketch->insert(node1, node2, weight, len1, len2);
	int in_weight = sketch->nodequery(node2, len2, 1);
	int neighbor_weight = sketch->query(node1, node2, len1, len2);
	clock_t end1 = clock();
	time1 += (double)(end1 - start1);

	clock_t start2 = clock();
	if (nodes.count(s2) != 0)
	{
		// update weight of hh
		set<pair<int, string>> store1;
		while (true)
		{
			pair<int, string> p = hh.top();
			hh.pop();
			if (p.second == s2)
			{
				store1.insert(pair<int, string>(in_weight, s2));
				break;
			}
			else
				store1.insert(p);
		}
		for (set<pair<int, string>>::iterator it = store1.begin(); it != store1.end(); it++)
		{
			hh.push(*it);
		}
		// update hns
		if (neighbors[s2].count(s1) != 0) 
		{
			set<pair<int, string>> store;
			while (true)
			{
				pair<int, string> p = hns[s2].top();
				hns[s2].pop();
				if (p.second == s1)
				{
					store.insert(pair<int, string>(neighbor_weight, s1));
					break;
				}
				else
					store.insert(p);
			}
			for (set<pair<int, string>>::iterator it = store.begin(); it != store.end(); it++)
			{
				hns[s2].push(*it);
			}
		}
		else // update neighbors of s2
		{
			if (hns[s2].size() == l and neighbor_weight > hns[s2].top().first) // 更新
			{
				neighbors[s2].erase(hns[s2].top().second);
				hns[s2].pop();
			}
			if (hns[s2].size() < l)
			{
				hns[s2].push(pair<int, string>(neighbor_weight, s1));
				neighbors[s2].insert(s1);
			}
		}
	}
	else
	{
		if (hh.size() == k && hh.top().first < in_weight)
		{
			nodes.erase(hh.top().second);
			// delete related hn
			// hns.erase(hh.top().second);   // hns不删
			hh.pop();
		}
		if (hh.size() < k) // 如果一个节点之前是hh，后来被挤出去了，那之前的hns可能会遗失（如果后续不出现的话）
		{
			// hns之前已经存在
			if (hns.count(s2) != 0)
			{
				// update hns
				if (neighbors[s2].count(s1) != 0) 
				{
					set<pair<int, string>> store;
					while (true)
					{
						pair<int, string> p = hns[s2].top();
						hns[s2].pop();
						if (p.second == s1)
						{
							store.insert(pair<int, string>(neighbor_weight, s1));
							break;
						}
						else
							store.insert(p);
					}
					for (set<pair<int, string>>::iterator it = store.begin(); it != store.end(); it++)
					{
						hns[s2].push(*it);
					}
				}
				else // update neighbors of s2
				{
					if (hns[s2].size() == l and neighbor_weight > hns[s2].top().first) // 更新
					{
						neighbors[s2].erase(hns[s2].top().second);
						hns[s2].pop();
					}
					if (hns[s2].size() < l)
					{
						hns[s2].push(pair<int, string>(neighbor_weight, s1));
						neighbors[s2].insert(s1);
					}
				}
			} else {
				// 之前没有hns
				priority_queue<pair<int, string>, vector<pair<int, string>>, cmpfortcm> temp;
				temp.push(pair<int, string>(neighbor_weight, s1));
				hns[s2] = temp;

				// update the set as well
				set<string> newneigh;
				newneigh.insert(s1);
				neighbors[s2] = newneigh;
			}
			
			hh.push(pair<int, string>(in_weight, s2));
			nodes.insert(s2);
		}
	}
	clock_t end2 = clock();
	time2 += (double)(end2 - start2);
}
