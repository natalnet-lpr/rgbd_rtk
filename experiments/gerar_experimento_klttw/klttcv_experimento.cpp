#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <stdlib.h>

using namespace std;

int main(int argc, char * argv[])
{

	cout<<"\tinstrução de uso:\n";
	cout<<"\tPRIMEIRO:  crie uma pasta 'resultados/klt/klttcv' no diretorio do klttcv\n";
	cout<<"\tPasse o nome do dataset como argumento e o caminho dele\n";
	cout<<"\tSua saída será os arquivos referente ao heatmap e aos outros dados numa pasta com nome de acordo com o dataset\n";
	cout<<"\tNo seguinte formato : dataset_raiomin_raiomax.\n rgbd_dataset_freiburg1_xyz_5_100\n";
	cout<<"\tOK?";

	string concorda;

	cin.ignore();
	
	while(!concorda.empty()){
		cin>>concorda;

	}	
		
	
	system("clear");

	ofstream arq("klttcv_experimento.sh");
	
	string dataset= argv[1];
	
	string aux="datasets";
	int n=50;	
				
	//cout<<dataset.find("datasets")+aux.length()<<endl<<dataset<<endl;
	
	int posicao_dataset = dataset.find("datasets")+aux.length()+1;
	
	dataset = dataset.substr (posicao_dataset,1000);
	
	posicao_dataset = dataset.find("/");

	dataset = dataset.substr (0,posicao_dataset);

	cout<<dataset<<endl;
	
	
	for(int i=5;i<n;i+=3)
	{
		for(int j=i+1;j<=n;j+=3)
		{	
			arq<<"./klt_tracker_test "<<argv[1]<<" 1 "<<i<<endl;
			arq<<"mkdir resultados/klt/klttcv/"<<dataset<<"_"<<i<<"_"<<j<<"\n";
			arq<<"python plot_heatmap.py dados_calor 32 "<<dataset<<".pdf"<<endl;
			arq<<"mv "<<dataset<<".pdf"<<" resultados/klt/klttcv/"<<dataset<<"_"<<i<<"_"<<j<<"\n";
			arq<<"mv dados_calor resultados/klt/klttcv/"<<dataset<<"_"<<i<<"_"<<j<<"\n";
			arq<<"mv dados_tempo resultados/klt/klttcv/"<<dataset<<"_"<<i<<"_"<<j<<"\n";
			arq<<"mv dados_rastreador resultados/klt/klttcv/"<<dataset<<"_"<<i<<"_"<<j<<"\n";
			
			arq<<"\n\n";		
		}
		

	}

	cout<<"ok";
	
	return 0;
}
	
