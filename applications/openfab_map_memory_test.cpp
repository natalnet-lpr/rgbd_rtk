#include <iostream>
#include "fabMap/fabmap_memory.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>
#include <rgbd_loader.h>
#include <string>

using namespace std;
class openFabMapArgParser{
	public:
		static void usage(){
			cout<<";/openfab_map_memory -mode <mode> <filename>\n";
			cout<<"modes:\n";
			cout<<"\tgenVocab\n";
			cout<<"\tshowFeatures\n";
			cout<<"\ttrainVocab\n";
			exit(0);
		}
		vector<string> valid_args;
		//input vocab or output
		char * vocab_filename;
		char * index_filename;
		string mode;
		openFabMapArgParser(){

			valid_args.push_back("genVocab");
			valid_args.push_back("showFeatures");
			valid_args.push_back("trainVocab");
			vocab_filename = NULL;
			index_filename = NULL;

		}
		~openFabMapArgParser(){
			
		}
		bool is_valid_arg(const char * arg){

			for(int i = 0 ;i<valid_args.size();i++){
				
				if(valid_args[i].compare(arg) == 0)
					return true;

			}
			return false;

		}


		void arg_parser(int argc , char **argv){
			bool valid_args = false;
			for (int i =0;i<argc;i++){
				string str = argv[i];
				
				if(str== "-mode" || str == "-index"){
					if(argc>i+1){
						i=i+1;
						if(is_valid_arg(argv[i])){
							mode = string(argv[i]);

							i = i+1;
							if(str == "-mode"){
								vocab_filename = argv[i];
							}
							valid_args = true;
						}
						else if(str == "-index"){
								index_filename = argv[i];
						}
						else
						{
							valid_args = false;

						}
						
					}
					else{
						cout<<"entrou\n";
						usage();
					}
				}


			}
			if(index_filename == NULL || vocab_filename == NULL){
				usage();
			}

		}

};

int main(int argc, char **argv){

    
	RGBDLoader loader;

	Mat frame, depth;

    FABMapMemory memory;
	openFabMapArgParser parser;


	parser.arg_parser(argc,argv);


	loader.processFile(parser.index_filename);
	for(int i = 0; i < loader.num_images_; i++)
	{
		loader.getNextImage(frame, depth);
        if(parser.mode == "genVocab"){
			memory.addTrainDataToVocab(frame,true);
			if(i==loader.num_images_+1)
			{
				memory.generateVocabTrainDataFile(parser.vocab_filename);
			}
		}
        //imshow("img", frame);
		char key = waitKey(15);
			
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
            destroyAllWindows();
			break;
		}
	}

	return 0;

}