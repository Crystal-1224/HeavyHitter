#include<iostream>
#include<string>
#include<memory.h>
#include "Cell_LGS_NoW.h"
using namespace std;

class LGS_NoW {
    public:
	int w,d;	//control matrix size
	int c=168;	//control #of prime numbers we use

	int v;	//we need to use v sketches to reduce collision caused by hash functions
	int* P=new int[168]{2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 
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
			947, 953, 967, 971, 977, 983, 991, 997};	//168 prime numbers less than 1000
	// Cell_LGS *S;
    Cell_LGS_NoW **S;
	bool *visited;	//used for path query

	LGS_NoW(){}; //默认构造函数
    LGS_NoW(int a, int b, int p, int i)	//matrix control, #prime number, sketch No.
	{
		w=a;
		d=b;
		c=p;
		v=i; //当前的副本编号
     
		S=new Cell_LGS_NoW*[w*d]; 
        for(int i=0;i<w*d;i++)
            S[i] = new Cell_LGS_NoW[w*d];  
		for(int j=0;j<w*d;j++)
			for(int q=0;q<w*d;q++)
				S[j][q]=Cell_LGS_NoW();
       
	}

	int BKDRHash(string key, int primes) 	//hash function
	{  
        int hash = 0;  
        int n = key.length();  
        for (int i = 0; i < n; ++i)  
            hash = primes * hash + key[i];  
        return (hash & 0x7FFFFFFF);  
    } 
	
    void Insert(string A, string B, string L_e)	//insertion with two end nodes' identifiers, edge label and timestamp
	{  
		int m,n,p_e;
		m=BKDRHash(A,P[v])%(w*d);
		n=BKDRHash(B,P[v])%(w*d);
		p_e=P[BKDRHash(L_e,P[v])%c];
		S[m][n].insert(p_e);
	}

	void Insert(string A, string B, string L_A, string L_B, string L_e)	//insertion with two end nodes' identifiers, node labels, edge label and timestamp
	{
		int m,n,p_e;
		m=d*(BKDRHash(L_A,P[v])%w)+BKDRHash(A,P[v])%d;
		n=d*(BKDRHash(L_B,P[v])%w)+BKDRHash(B,P[v])%d;
		p_e=P[BKDRHash(L_e,P[v])%c];
		S[m][n].insert(p_e);
	}

	int getNodes_in(string A)	//aggregated node income queries,without edge label constraint
	{
		int i=0;
		int m;
		m=BKDRHash(A,P[v])%(w*d);
		for(int j=0;j<d*w;j++)
		{
			i+=S[j][m].getC();
		}
		return i;
	}
	
	int getNodes_in(string A, string L_A)	//aggregated node income queries,without edge label constraint
	{
		int i=0;
		int m;
		m=d*(BKDRHash(L_A,P[v])%w)+BKDRHash(A,P[v])%d;
		for(int j=0;j<d*w;j++)
		{
			i+=S[j][m].getC();
		}
		return i;
	}
	
	int getNodes_in_withE(string A, string L_A, string L_e)	//aggregated node income queries,with edge label constraint
	{
		int i=0;
		int m;
		m=d*(BKDRHash(L_A,P[v])%w)+BKDRHash(A,P[v])%d;
		int p_e;
		p_e=P[BKDRHash(L_e,P[v])%c];
		for(int j=0;j<d*w;j++)
		{
			i+=S[j][m].gete(p_e);
		}
		return i;
	}

	int getNodes_out(string A)	//aggregated node out queries, without edge label constraint
	{
		int i=0;
		int m;
		m=BKDRHash(A,P[v])%(w*d);
		for(int j=0;j<d*w;j++)
		{
			i+=S[m][j].getC();
		}
		return i;
	}
	
	int getNodes_out(string A, string L_A)	//aggregated node out queries, without edge label constraint
	{
		int i=0;
		int m;
		m=d*(BKDRHash(L_A,P[v])%w)+BKDRHash(A,P[v])%d;
		for(int j=0;j<d*w;j++)
		{
			i+=S[m][j].getC();
		}
		return i;
	}

	int getNodes_out_withE(string A, string L_e)	//aggregated node out queries, with edge label constraint
	{
		int i=0;
		int m;
		m=BKDRHash(A,P[v])%(w*d);
		int p_e;
		p_e=P[BKDRHash(L_e,P[v])%c];
		for(int j=0;j<d*w;j++)
		{
			i+=S[m][j].gete(p_e);
		}
		return i;
	}
	
	int getNodes_out_withE(string A, string L_A, string L_e)	//aggregated node out queries, with edge label constraint
	{
		int i=0;
		int m;
		m=d*(BKDRHash(L_A,P[v])%w)+BKDRHash(A,P[v])%d;
		int p_e;
		p_e=P[BKDRHash(L_e,P[v])%c];
		for(int j=0;j<d*w;j++)
		{
			i+=S[m][j].gete(p_e);
		}
		return i;
	}
	
	int getEdge(string A, string B)	//aggregated edge queries, without edge label constraint
	{
		int i=0;
		int m,n;
		m=BKDRHash(A,P[v])%(w*d);
		n=BKDRHash(B,P[v])%(w*d);
		i=S[m][n].getC();
		return i;
	}

	int getEdge(string A, string L_A, string B, string L_B)	//aggregated edge queries, without edge label constraint
	{
		int i=0;
		int m,n;
		m=d*(BKDRHash(L_A,P[v])%w)+BKDRHash(A,P[v])%d;
		n=d*(BKDRHash(L_B,P[v])%w)+BKDRHash(B,P[v])%d;
		i=S[m][n].getC();
		return i;
	}

	int getEdge_withE(string A, string B, string L_e)	//aggregated edge queries, with edge label constraint
	{
		int i=0,m,n,p_e;
		m=BKDRHash(A,P[v])%(w*d);
		n=BKDRHash(B,P[v])%(w*d);
		p_e=P[BKDRHash(L_e,P[v])%c];
		i=S[m][n].gete(p_e);
		return i;
	}

	int getEdge_withE(string A, string L_A, string B, string L_B, string L_e)	//aggregated edge queries, with edge label constraint
	{
		int i=0,m,n,p_e;
		m=d*(BKDRHash(L_A,P[v])%w)+BKDRHash(A,P[v])%d;
		n=d*(BKDRHash(L_B,P[v])%w)+BKDRHash(B,P[v])%d;
		p_e=P[BKDRHash(L_e,P[v])%c];
		i=S[m][n].gete(p_e);
		return i;
	}
	
	bool path(string A, string B)	//path query, whether there's a path from A to B
	{
		int s=BKDRHash(A,P[v])%(w*d);  //start point 
		int t=BKDRHash(B,P[v])%(w*d);  //end point
		if(reach(s,t))
			return true;
		visited=new bool[d*w];
		for(int i=0;i<d*w;i++)
			visited[i]=false;
		return DFS(s,t);
//		return false;
	}

	bool path(string A, string L_A, string B, string L_B)	//path query, whether there's a path from A to B
	{
		int s=d*(BKDRHash(L_A,P[v])%w)+BKDRHash(A,P[v])%d;	//start point 
		int t=d*(BKDRHash(L_B,P[v])%w)+BKDRHash(B,P[v])%d;	//end point
		if(reach(s,t))
			return true;
		visited=new bool[d*w];
		for(int i=0;i<d*w;i++)
			visited[i]=false;
		return DFS(s,t);
//		return false;
	}
	
	bool reach(int s, int t)	//path query auxiliary function
	{
		if(S[s][t].getC()>0)
			return true;
		return false;
	}
	
	bool DFS(int s, int t)
	{		
		visited[s]=true;
		if(reach(s,t))	//check whether destination t can be reached from this point, if not, continue DFS search
			return true;
		for(int i=0;i<d*w;i++)
		{
			if(S[s][i].getC()>0 && (!visited[i]))
				if(DFS(i,t))
					return true;
		}
		return false;
	}

	bool path_withE(string A, string B, string L_e)	//whether there's a path from A to B all with edge type L_e
	{
		int s=BKDRHash(A,P[v])%(w*d);  //start point 
		int t=BKDRHash(B,P[v])%(w*d);  //end point
		int p_e=P[BKDRHash(L_e,P[v])%c];
		if(reach_withE(s,t,p_e))
			return true;
		visited=new bool[d*w];
		for(int i=0;i<d*w;i++)
			visited[i]=false;
		return DFS_withE(s,t,p_e);
	}
	
	bool path_withE(string A, string L_A, string B, string L_B, string L_e)	//whether there's a path from A to B all with edge type L_e
	{
		int s=d*(BKDRHash(L_A,P[v])%w)+BKDRHash(A,P[v])%d;	//start point 
		int t=d*(BKDRHash(L_B,P[v])%w)+BKDRHash(B,P[v])%d;	//end point
		int p_e=P[BKDRHash(L_e,P[v])%c];
		if(reach_withE(s,t,p_e))
			return true;
		visited=new bool[d*w];
		for(int i=0;i<d*w;i++)
			visited[i]=false;
		return DFS_withE(s,t,p_e);
	}
	
	bool reach_withE(int s, int t,int p_e)	//path query auxiliary function, with edge type
	{
		if(S[s][t].gete(p_e)>0)
			return true;
		return false;
	}
	
	bool DFS_withE(int s, int t,int p_e)
	{		
		visited[s]=true;
		if(reach_withE(s,t,p_e))	//check whether destination t can be reached from this point, if not, continue DFS search
			return true;
		for(int i=0;i<d*w;i++)
		{
			if(S[s][i].gete(p_e)>0 && (!visited[i]))
				if(DFS_withE(i,t,p_e))
					return true;
		}
		return false;
	}
	
	int getEdge_withE_NE(string L_A, string L_B, string L_e)	//get aggreagate edge query between two node types
	{
		int count=0,m,n,p_e;
		p_e=P[BKDRHash(L_e,P[v])%c];
		for(int i=0;i<d;i++)
			for(int j=0;j<d;j++)
			{
				m=d*(BKDRHash(L_A,P[v])%w)+i;
				n=d*(BKDRHash(L_B,P[v])%w)+j;
				count+=S[m][n].gete(p_e);
			}
		return count;
	}
	
// 	public static void main(String args[])
// 	{
// /*		LGS newLGS=new LGS(1000,100,1);	//window, sub_window size, # of sketches
// 		for(int i=0;i<newLGS.P.length;i++)
// 			System.out.println(newLGS.P[i]);
// 		System.out.println(newLGS.P.length);*/
// 	}
};