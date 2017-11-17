#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <stdlib.h>
#include <sstream>
using namespace std;

int main(int argc, char * argv[])
{

	cout<<"\tPRIMEIRO:  crie uma pasta 'resultados/icp/klt/' no diretorio\n";
	cout<<"\tPasse o nome do dataset como argumento e o caminho dele\n";
	cout<<"\tSua saída será os arquivos referente a estimação de movimento  numa pasta com nome de acordo com o dataset\n";
	cout<<"\tOK?";
	
	string concorda;

	cin.ignore();
	
	while(!concorda.empty()){
		cin>>concorda;

	}	

	
	system("clear");

	ofstream arq("icp_experimento.sh");
	for(int i=1;i<argc;i++){
	
	string dataset= argv[i];
	
	string aux="datasets";

	int n=50;	

	
	int posicao_dataset = dataset.find("datasets")+aux.length()+1;
	
	dataset = dataset.substr (posicao_dataset,1000);
	
	posicao_dataset = dataset.find("/");

	dataset = dataset.substr (0,posicao_dataset);

	cout<<dataset<<endl;
	

		//roda o motion estimation	
		arq<<"./motion_estimator_test "<<argv[i]<<"/index.txt"<<endl;
		/*	cria pasta 
		*	roda medidor de tempo
		*	move arquivo de tempo
		*/
		arq<<"mkdir resultados/icp/klttacw/"<<dataset<<"\n";
		arq<<"./media tempo_frame.txt > media.txt\n";
		arq<<"mv media.txt resultados/icp/klttacw/"<<dataset<<"\n\n";


		/*
			analisa estimação de movimento com script
		
		*/
		arq<<"./evaluate_rpe.py --verbose --fixed_delta "<<argv[i]<<"/groundtruth.txt"<<" pos_relativa.txt > resultados/icp/klttacw/"<<dataset<<"/vo_analisys.txt";
		
		arq<<"\n\n";		
		
	}	

	

	cout<<"ok";
	
	arq.close();




	return 0;
}


