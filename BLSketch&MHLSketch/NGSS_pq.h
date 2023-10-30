#include<iostream>
#include<string>
#include<vector>
#include<queue>
#include<set>
#include<map>
#include<cmath>
#include<stdlib.h>
#include<bitset>
#include<memory.h>
#include<limits.h>
#include "head/GSS.h"
using namespace std;


// struct cmpfortcm
// {
//     bool operator() (const pair<int, string> &a, const pair<int, string> &b) 
//     {
//        return a.first > b.first; //小顶堆
//     }
// };
struct cmpforgss
{
    bool operator() (const pair<int, string> &a, const pair<int, string> &b) 
    {
       return a.first > b.first; //小顶堆
    }
};

class NGSS
{
private:
	GSS *sketch;
	// chh
	int k;
	int l;
public:
	// heavy hitter
	priority_queue<pair<int, string>, vector<pair<int, string>>, cmpforgss> hh;
	set<string> nodes;

	//heavy neighnors of each node
	map<string, priority_queue<pair<int, string>, vector<pair<int, string>>, cmpforgss>> hns;
	map<string, set<string>>  neighbors;

	NGSS(int width, int range, int p_num, int size,int f_num, int tablesize, int hh, int hn);
	~NGSS()
		{
			delete[] sketch;
		}
	void update(string s1, string s2,int weight);
};

NGSS::NGSS(int width, int range, int p_num, int size,int f_num, int tablesize, int hh, int hn){
	sketch = new GSS(width, range, p_num, size, f_num, true, tablesize);
	k = hh;
	l = hn;
}

void NGSS::update(string s1, string s2,int weight){
	sketch->insert(s1,s2,weight);
	int in_weight = sketch->nodeValueQuery(s2, 1);
	int neighbor_weight = sketch->edgeQuery(s1, s2); 

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
				priority_queue<pair<int, string>,vector<pair<int, string>>, cmpforgss> temp;
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
	
}

