#include <iostream>
#include "fabMap/fabmap_memory.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>
#include <rgbd_loader.h>
#include <string>

using namespace std;
class openFabMapArgParser{
	public:
		std::map<string,string> input_map;
		openFabMapArgParser(){
			input_map["mode"] = "None";
			input_map["index_file"] = "None";
			input_map["untrainedVocab"] = "None";
			input_map["trainedVocab"] = "None";

		}
		
		void arg_parser(int argc, char ** argv){

			for(int i = 0;i<argc;i++){
				if(i+1<argc)
				{
					string str = argv[i];

					if(str=="-index")
					{	
						input_map["index_file"] = argv[i+1];
					}
					if(str == "-mode")
					{
						input_map["mode"] = argv[i+1];
					}
					if(str == "-untrainedVocab")
					{
						input_map["untrainedVocab"] = argv[i+1];
					}
					if(str == "-trainedVocab")
					{
						input_map["trainedVocab"] = argv[i+1];

					}
				}
			}

		}
		string get_second(char * map_first){
			return input_map[map_first];
		}

};

int main(int argc, char **argv){

    
	RGBDLoader loader;

	Mat frame, depth;

    FABMapMemory memory;
	openFabMapArgParser parser;

	parser.arg_parser(argc,argv);
	loader.processFile(parser.get_second("index_file"));
	cout<<parser.get_second("untrainedVocab")<<endl;
	for(int i = 0; i < loader.num_images_; i++)
	{
		loader.getNextImage(frame, depth);
        if(parser.get_second("mode") == "genVocab"){
			memory.addTrainDataToVocab(frame,true);
			
			if(i+1==loader.num_images_)
			{
				memory.generateVocabTrainDataFile(parser.get_second("untrainedVocab"));
			}
		}
		else if(parser.get_second("mode") == "trainVocab"){
			memory.trainVocabulary(parser.get_second("trainedVocab"),parser.get_second("untrainedVocab"),0.5);
			break;
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
	destroyAllWindows();


	return 0;

}
