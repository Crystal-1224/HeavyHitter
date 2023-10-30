#include<vector>
using namespace std;

class Cell_LGS_NoW
{
private:
	int C;
	int large=0x3fffff;	//predefined large number threshold  0011 1111 1111 1111 1111 1111
	vector<int> e;
	
public:
    Cell_LGS_NoW(){  
		C=0;		
    }
    ~Cell_LGS_NoW(){}

    int getC()	//get edge counts in this cell regardless of edge label
	{
		return C;
	}

    int gete(int p_e)	//get edge counts in this cell with edge label L1 that L1's corresponding prime number is p_e
	{
		int i=0;
		for(int t=0;t<e.size();t++)
		{
			int currentE=e[t];
			while(currentE>0 and currentE%p_e==0)
			{
				i++;
				currentE=currentE/p_e;
			}
        }
		return i;
	}

    void insert(int p_e)	//edge insertion, update C and e, expiration handles in this step. p_e:prime number representation for current edge label, t: edge timestamp
	{
        C++;
        if(e.empty()){
            int newE=1;
            newE*=p_e;
            e.push_back(newE);
        }
        else{
            int currentE = e[e.size()-1];
            if(currentE<large)
			{
				currentE*=p_e;
				e[e.size()-1]=currentE;
			}
			else	//use a new cell for large number
			{
				int newE=1;
				newE*=p_e;
				e.push_back(newE);
			}
        }
	}

    
};

