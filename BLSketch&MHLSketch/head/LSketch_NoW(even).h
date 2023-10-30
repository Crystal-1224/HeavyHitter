#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <deque>
#include <set>
#include <map>
#include <unordered_map>
#include <cmath>
#include <stdlib.h>
#include <bitset>
#include <memory.h>
#include <time.h>
#include <ctime>
#ifndef HASHTABLE_H
#define HASHTABLE_H
#include "GraphUndirected.h"
#endif // HASHTABLE_H

#define prime 739
#define bigger_p 1048576
#define timer 5
#define M 80000

#define Roomnum 2   // This is the parameter to controll the maximum number of rooms in a bucket.

using namespace std;
struct cell
{
    unsigned short src[Roomnum];
    unsigned short dst[Roomnum];
    int  C[Roomnum];  
    // short  P[Roomnum];
    vector<int> P[Roomnum];   //根据large不断分割
    
    unsigned int idx;

    // 存h
    unsigned short h1[Roomnum];
    unsigned short h2[Roomnum];
};
struct mnode
{
    unsigned int h;
    unsigned short g;
};
struct link_node
{
    unsigned int key;
    int  C;  
    // int  P;
    vector<int> P; //根据large不断分割
    link_node *next;
};

class HGSS_NoW
{
private:
    hashTable<string> mapTable;
    int w;
    int b; // block
    int r;
    int p;
    int s;
    int f;
    bool useT;
    int tablesize;

    cell *value;
    // map<int, string> k2label; //存k值对应的label，知道每个节点的类别

public:
    int large=0x3fffff;	//predefined large number threshold  0011 1111 1111 1111 1111 1111
    int prime_num = 10; //control #of prime numbers we use
    int Prime[168] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29,
                      31, 37, 41, 43, 47, 53, 59, 61, 67, 71,
                      73, 79, 83, 89, 97, 101, 103, 107, 109, 113,
                      127, 131, 137, 139, 149, 151, 157, 163, 167, 173,
                      179, 181, 191, 193, 197, 199, 211, 223, 227, 229,
                      233, 239, 241, 251, 257, 263, 269, 271, 277, 281,
                      283, 293, 307, 311, 313, 317, 331, 337, 347, 349,
                      353, 359, 367, 373, 379, 383, 389, 397, 401, 409,
                      419, 421, 431, 433, 439, 443, 449, 457, 461, 463,
                      467, 479, 487, 491, 499, 503, 509, 521, 523, 541,
                      547, 557, 563, 569, 571, 577, 587, 593, 599, 601,
                      607, 613, 617, 619, 631, 641, 643, 647, 653, 659,
                      661, 673, 677, 683, 691, 701, 709, 719, 727, 733,
                      739, 743, 751, 757, 761, 769, 773, 787, 797, 809,
                      811, 821, 823, 827, 829, 839, 853, 857, 859, 863,
                      877, 881, 883, 887, 907, 911, 919, 929, 937, 941,
                      947, 953, 967, 971, 977, 983, 991, 997};
    map<string, int> random_index; // 对应边标签的序号

    vector<link_node *> buffer;
    map<unsigned int, int> index;
    int buffer_index;
    
    int edge_num; // count the number of edges in the buffer to assist buffer size analysis. Self loop edge is not included as it does not use additional memory.
    HGSS_NoW(int width, int block, int range, int p_num, int size, int f_num, bool usetable, int tablesize = 0);
    ~HGSS_NoW()
    {
        delete[] value;
        cleanupBuffer();
    }
    void cleanupBuffer();
    // insert
    void insert(string s1, string s2, int weight, string l_A, string l_B, string l_e); // node_label1 node_label2 edge_label

    int nodeValueQuery(string s1, int type, string l_A); //src_type = 0 dst_type = 1
    int edgeQuery(string s1, string l_A, string s2, string l_B); // s1 is the ID of the source node, s2 is the ID of the destination node, return the weight of the edge
};

HGSS_NoW::HGSS_NoW(int width, int block, int range, int p_num, int size, int f_num, bool usehashtable, int TableSize) //the side length of matrix, the length of hash addtress list, the number of candidate bucekt
// the number of rooms, whether to use hash table, and the size of the table.
// Hash table which stores the original nodes can be omitted if not needed. For nodequery,
//  reachability, edgequery not needed. But needed for triangel counting, degree query, and successor / precursor queries.
{
    w = width;
    b = block;
    r = range;        /* r x r mapped baskets */
    p = p_num;        /*candidate buckets*/
    s = size;         /*multiple rooms*/
    f = f_num;        /*finger print lenth*/
    buffer_index = 0;
    edge_num = 0;
    value = new cell[w * w];
    useT = usehashtable;
    tablesize = TableSize;
    memset(value, 0, sizeof(cell) * w * w);
    if (usehashtable)
        mapTable.init(tablesize);
}

void HGSS_NoW::cleanupBuffer()
{
    vector<link_node *>::iterator IT = buffer.begin();
    link_node *e, *tmp;
    for (; IT != buffer.end(); ++IT)
    {
        e = *IT;
        while (e != NULL)
        {
            tmp = e->next;
            delete e;
            e = tmp;
        }
    }
}

void HGSS_NoW::insert(string s1, string s2, int weight, string l_A, string l_B, string l_e) // 增加倾斜标签的处理
{
    unsigned int hash1 = (*hfunc[0])((unsigned char *)(s1.c_str()), s1.length());
    unsigned int hash2 = (*hfunc[0])((unsigned char *)(s2.c_str()), s2.length());
    unsigned int tmp = pow(2, f) - 1;
    unsigned short g1 = hash1 & tmp;
    if (g1 == 0)
        g1 += 1;
    unsigned int h1 = (hash1 >> f) % w;
    unsigned short g2 = hash2 & tmp;
    if (g2 == 0)
        g2 += 1;
    unsigned int h2 = (hash2 >> f) % w;

    unsigned int k1 = (h1 << f) + g1;
    unsigned int k2 = (h2 << f) + g2;

    if (useT)
    {
        mapTable.insert(k1, s1);
        mapTable.insert(k2, s2);
    }

    
    // 引入矩阵块,m n 分别对应选中的矩阵块数
    unsigned int label1 = (*hfunc[0])((unsigned char *)(l_A.c_str()), l_A.length());
    unsigned int label2 = (*hfunc[0])((unsigned char *)(l_B.c_str()), l_B.length());
    unsigned int m = (label1 % (w / b));
    unsigned int n = (label2 % (w / b));

    // edge label
    unsigned int edge_index = (*hfunc[0])((unsigned char *)(l_e.c_str()), l_e.length());
    int edge_label = Prime[edge_index%prime_num];

    int *tmp1 = new int[r];
    int *tmp2 = new int[r];
    tmp1[0] = g1;
    tmp2[0] = g2;
    for (int i = 1; i < r; i++)
    {
        tmp1[i] = (tmp1[i - 1] * timer + prime) % bigger_p;
        tmp2[i] = (tmp2[i - 1] * timer + prime) % bigger_p;
    }
    bool inserted = false;
    long key = g1 + g2;

    for (int i = 0; i < p; i++)
    {
        key = (key * timer + prime) % bigger_p;
        int index = key % (r * r);
        int index1 = index / r;
        int index2 = index % r;
        int p1 = (h1 + tmp1[index1]) % b; //在block中的位置
        int p2 = (h2 + tmp2[index2]) % b;

        int pos = (m + p1) * w + (n + p2); 
        for (int j = 0; j < s; j++)
        {
            if ((((value[pos].idx >> (j << 3)) & ((1 << 8) - 1)) == (index1 | (index2 << 4))) && (value[pos].src[j] == g1) && (value[pos].dst[j] == g2))
            {
                value[pos].C[j] += weight;
                vector<int> tmp=value[pos].P[j];
                int cur = tmp.at(tmp.size()-1);
                if(cur<large){
                    cur*=edge_label;
                    tmp[tmp.size()-1]=cur;
                }
                else
                    tmp.push_back(edge_label);    
                value[pos].P[j]=tmp;

                inserted = true;
                break;
            }
            if (value[pos].src[j] == 0)
            {
                value[pos].idx |= ((index1 | (index2 << 4)) << (j << 3));
                value[pos].src[j] = g1;
                value[pos].dst[j] = g2;
                // h
                value[pos].h1[j] = h1;
                value[pos].h2[j] = h2;

                value[pos].C[j] = weight;
                value[pos].P[j].push_back(edge_label);
                inserted = true;
                break;
            }
        }
        if (inserted)
            break;
    }
    if (!inserted)
    {
        map<unsigned int, int>::iterator it = index.find(k1);
        if (it != index.end())
        {
            int tag = it->second;
            link_node *node = buffer[tag];
            while (true)
            {
                if (node->key == k2)  
                {
                    node->C += weight;
                    int cur = node->P.at(node->P.size()-1);
                    if(cur<large){
                        cur*=edge_label;
                        node->P[node->P.size()-1]=cur;
                    }
                    else
                        node->P.push_back(edge_label);
					break;
                }
                if (node->next == NULL)
                {
                    link_node *ins = new link_node;
                    ins->key = k2;                   
                    ins->C = weight;
                    ins->P.push_back(edge_label);

                    ins->next = NULL;
                    node->next = ins;
                    edge_num++;
                    break;
                }
                node = node->next;
            }
        }
        else
        {
            index[k1] = buffer_index;
            buffer_index++;
            link_node *node = new link_node;
            node->key = k1;
            node->C = 0;  //初始化自连边
            node->P.push_back(1);  //初始化自连边
            if (k1 != k2) //k1==k2 means loop
            {
                link_node *ins = new link_node;
                ins->key = k2;
                ins->C = weight;  
                // ins->P = edge_label;
                ins->P.push_back(edge_label);

                ins->next = NULL;
                node->next = ins;
                edge_num++;
            }
            else
            {
                node->C += weight;
                int cur = node->P.at(node->P.size()-1);
                if(cur<large){
                    cur*=edge_label;
                    node->P[node->P.size()-1]=cur;
                }
                else
                    node->P.push_back(edge_label);
                    
                node->next = NULL;
            }
            buffer.push_back(node);
        }
    }
    
    delete[] tmp1;
    delete[] tmp2;
    return;
}


/*type 0 is for successor query, type 1 is for precusor query*/
int HGSS_NoW::nodeValueQuery(string s1, int type, string l_A)
{
    int weight = 0;
    unsigned int hash1 = (*hfunc[0])((unsigned char *)(s1.c_str()), s1.length());
    int tmp = pow(2, f) - 1;
    unsigned short g1 = hash1 & tmp;
    if (g1 == 0)
        g1 += 1;
    unsigned int h1 = (hash1 >> f) % w;
    int *tmp1 = new int[r];
    tmp1[0] = g1;
    for (int i = 1; i < r; i++)
    {
        tmp1[i] = (tmp1[i - 1] * timer + prime) % bigger_p;
    }
    // 矩阵块
    unsigned int label1 = (*hfunc[0])((unsigned char *)(l_A.c_str()), l_A.length());
    unsigned int m = (label1 % (w / b));

    // int p1 = (h1 + tmp1[i]) % b; //block
    // int pos = (m * b + p1) * w + k;
    for (int i = 0; i < r; i++)
    {
        int p1 = (h1 + tmp1[i]) % b; //block
        for (int k = 0; k < w; k++)
        {
            if (type == 0) /*successor query*/
            {
                int pos = (m * b + p1) * w + k;
                for (int j = 0; j < s; ++j)
                {
                    if (type == 0 && (((value[pos].idx >> ((j << 3))) & ((1 << 4) - 1)) == i) && (value[pos].src[j] == g1))
                    {
                        weight += value[pos].C[j];
                    }
                }
            }
            else if (type == 1) /*precursor query*/
            {
                int pos = k * w + (m * b + p1);
                for (int j = 0; j < s; ++j)
                {
                    if (type == 1 && (((value[pos].idx >> ((j << 3) + 4)) & ((1 << 4) - 1)) == i) && (value[pos].dst[j] == g1))
                    {
                        weight += value[pos].C[j];
                    }
                }
            }
        }
    }
    if (type == 0)
    {
        unsigned int k1 = (h1 << f) + g1;
        map<unsigned int, int>::iterator it = index.find(k1);
        if (it != index.end())
        {
            int tag = it->second;
            link_node *node = buffer[tag];
            weight += node->C;
            node = node->next;
            while (node != NULL)
            {
                weight += node->C;
                node = node->next;
            }
        }
    }
    else if (type == 1)
    {
        unsigned int k1 = (h1 << f) + g1;
        // 计算自连边的入度贡献
        map<unsigned int, int>::iterator it = index.find(k1);
        if (it != index.end())
            weight += buffer[it->second]->C;
        
        // 其他
        for (map<unsigned int, int>::iterator it = index.begin(); it != index.end(); ++it)
        {
            int tag = it->second;
            link_node *node = buffer[tag];
            node = node->next;
            while (node != NULL)
            {
                if(node->key == k1)
					weight += node->C;
                node = node->next;
            }
        }
    }
    delete[] tmp1;
    return weight;
}

int HGSS_NoW::edgeQuery(string s1, string l_A, string s2, string l_B) // s1 is the ID of the source node, s2 is the ID of the destination node, return the weight of the edge
{
    unsigned int hash1 = (*hfunc[0])((unsigned char *)(s1.c_str()), s1.length());
    unsigned int hash2 = (*hfunc[0])((unsigned char *)(s2.c_str()), s2.length());
    int tmp = pow(2, f) - 1;
    unsigned short g1 = hash1 & tmp;
    if (g1 == 0)
        g1 += 1;
    unsigned int h1 = (hash1 >> f) % w;
    unsigned short g2 = hash2 & tmp;
    if (g2 == 0)
        g2 += 1;
    unsigned int h2 = (hash2 >> f) % w;
    int *tmp1 = new int[r];
    int *tmp2 = new int[r];
    tmp1[0] = g1;
    tmp2[0] = g2;
    for (int i = 1; i < r; i++)
    {
        tmp1[i] = (tmp1[i - 1] * timer + prime) % bigger_p;
        tmp2[i] = (tmp2[i - 1] * timer + prime) % bigger_p;
    }
    long key = g1 + g2;

    // 矩阵块
    unsigned int label1 = (*hfunc[0])((unsigned char *)(l_A.c_str()), l_A.length());
    unsigned int label2 = (*hfunc[0])((unsigned char *)(l_B.c_str()), l_B.length());
    unsigned int m = (label1 % (w / b));
    unsigned int n = (label2 % (w / b));

    for (int i = 0; i < p; i++)
    {
        key = (key * timer + prime) % bigger_p;
        int index = key % (r * r);
        int index1 = index / r;
        int index2 = index % r;
        int p1 = (h1 + tmp1[index1]) % b; // 在block中的位置
        int p2 = (h2 + tmp2[index2]) % b;
        int pos = (p1 + m * b) * w + (p2 + n * b);
        for (int j = 0; j < s; j++)
        {

            if ((((value[pos].idx >> (j << 3)) & ((1 << 8) - 1)) == (index1 | (index2 << 4))) && (value[pos].src[j] == g1) && (value[pos].dst[j] == g2))
            {
                delete[] tmp1;
                delete[] tmp2;
                return value[pos].C[j];
            }
        }
    }
    unsigned int k1 = (h1 << f) + g1;
    unsigned int k2 = (h2 << f) + g2;
    map<unsigned int, int>::iterator it = index.find(k1);
    if (it != index.end())
    {
        int tag = it->second;
        link_node *node = buffer[tag];
        while (node != NULL)
        {
            if (node->key == k2)
            {
                delete[] tmp1;
                delete[] tmp2;
                return node->C;
            }
            node = node->next;
        }
    }
    delete[] tmp1;
    delete[] tmp2;
    return 0;
}

