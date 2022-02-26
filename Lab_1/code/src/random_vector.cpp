#include "random_vector.h"
#include<vector>
#define N 999999
// TODO: add any include you might require

RandomVector::RandomVector(int size, double max_val) { 
	double random[size];
	for(int i = 0; i<size; i++)
	{
	random[i] = rand()%(N+1)/float(N+1);
	vect.push_back(random[i]);
	}
}

void RandomVector::print()
{
	for (std::vector<double>::iterator it = vect.begin(); it != vect.end(); it++) 
	{
		std::cout << *it << " ";
	}
	std::cout << std::endl;
}

double RandomVector::mean(){
	double sum = 0;
	for (std::vector<double>::iterator it = vect.begin(); it != vect.end(); it++)
	{
		sum += *it;
	}
	return sum/vect.size();
}

double RandomVector::max(){
	double max = 0;
	for (std::vector<double>::iterator it = vect.begin(); it != vect.end(); it++)
	{
	if(max<*it)
	{
		max = *it;
	}
	}
	return max;
}

double RandomVector::min(){
	double min = 1;
	for (std::vector<double>::iterator it = vect.begin(); it != vect.end(); it++)
	{
	if(min>*it)
	{
		min = *it;
	}
	}
	return min;
}

void RandomVector::printHistogram(int bins){
	int num[bins];
	for(int i=0;i<bins;i++)
	{
		num[i]=0;
	}
	double width = (RandomVector::max() - RandomVector::min())/bins;
	for(int i =0;i<bins;i++)
	{	
		double left = RandomVector::min() + width*i;
		double right = RandomVector::min() + width*(i+1);
		for (std::vector<double>::iterator it = vect.begin(); it != vect.end(); it++)
		{
			if(*it >= left && *it <= right)
			{
				num[i]++;
			}
		}
	}
	int max = 0;
	for(int i = 0; i<bins;i++)
	{
		if(num[i]>max)
		{
			max = num[i];
		}
	}
	for(int i = max; i>0; i--)
	{
		for (int j = 0; j<bins; j++)
		{
			if(num[j]>=i)
			{
				std::cout<<"***\t";
			}
			else
			{
				std::cout<<"   \t";
			}
		}
		std::cout<<std::endl;
	}	
}