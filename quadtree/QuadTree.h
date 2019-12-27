#include <vector>
#include <opencv2/core/core.hpp>

using namespace cv;

using namespace std;

//this project makes use of QuadTree datastructure;

class QuadTree
{


public:
    //region than will be marked and segmented	
    Rect Boundary;
    //maximum capacity of a node of quadtree
    int capacity;
    //flag used to check if the current node is subdived
    bool divided = false;
    //flag used to check if the current node is empty or not, in the other words, if have a point there
    bool alocated = false;
    //set of points added into a node 
    vector<Point2f> pts;
    /*
  
    These pointers represents four regions presents in a 2-D plain
    */
    QuadTree * TopLeft;
    QuadTree * TopRight;
    QuadTree * BotLeft ;
    QuadTree * BotRight;
   //this variable helps in task of mark the mask
   //if some region have the max_density equal or bigger than this value
   //this region is automatically marked like a black region, does mean, no point need to be detected here
    float max_density;
	
	
    QuadTree()
    {
       //defaul constructor
        capacity =4;


    }
  
   QuadTree(Rect Boundary,int capacity,float max_density=0)
    {
	
        this->Boundary = Boundary;
        this->capacity = capacity;
        TopLeft = new QuadTree();
        TopRight = new QuadTree();
        BotLeft =new QuadTree();
        BotRight = new QuadTree();
	this->max_density = max_density;

    }

    //subdivide the region if the region is not divided and the region is not out of the capacity
    void Subdivide();
    //insert a point in region
    virtual void Insert(Point2f new_point);

    void DrawTree(Mat &img);
    //mark the mask, black region are regions than will be dispensed in point detection
    virtual void MarkMask(Mat &Mask,vector<Point2f> pts ,bool initialized);



};
