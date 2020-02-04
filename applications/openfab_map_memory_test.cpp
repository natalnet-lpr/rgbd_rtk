#include <iostream>
#include "fabMap/fabmap_memory.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>
#include <rgbd_loader.h>


using namespace std;


int main(int argc, char **argv){

    
	string index_file_name;
	RGBDLoader loader;

	Mat frame, depth;

    FABMapMemory memory;

	if(argc != 2)
	{
		fprintf(stderr, "Usage: %s <index file>\n", argv[0]);
		exit(0);
	}

	index_file_name = argv[1];
	loader.processFile(index_file_name);

	for(int i = 0; i < loader.num_images_; i++)
	{
		loader.getNextImage(frame, depth);
        memory.addTrainDataToVocab(frame,true);
        
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