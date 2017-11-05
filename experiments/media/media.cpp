#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;

int main(int argc, char **argv)
{
	ifstream arq;	

	arq.open(argv[1]);	

	float media=0,desv_p=0;

	int cont=0;

	char numeros[100];

	vector <float> n;

	

	
	
	while(arq.getline(numeros,99) )
	{

		media+=atof(numeros);	
		
		n.push_back(atof(numeros ) );	
				
		cont++;

	}

	std::sort (n.begin(), n.end()); 

	

	media/=cont;

	
	for(int i=0;i<n.size();i++)
	{
		desv_p+= pow( (n.at(i)-media), 2 );

	}

	desv_p/=n.size();
	
	desv_p=sqrt(desv_p);
	

	cout<<media<<" +/- "<<desv_p<<endl;

	cout<<"menor "<<n.at(0)<<endl;

	cout<<"maior "<<n.back()<<endl;


	arq.close();
	

	return 0;
}
