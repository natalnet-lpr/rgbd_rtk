#include <opencv2/core/core.hpp>
#include <vector>
using namespace cv;
struct Node
{
    Point2f pos;
    //int data;

    Node(Point _pos)
    {
        pos = _pos;
       // data = _data;

    }
    Node()
    {
       // data = 0;

    }
};
 
// The main quadtree class
class Quad
{
    // Hold details of the boundary of this node
    Point2f topLeft;
    Point2f botRight;

    // Contains details of node
    Node *n;
 
    // Children of this tree
    Quad *topLeftTree;
    Quad *topRightTree;
    Quad *botLeftTree;
    Quad *botRightTree;

    float SizeMin;
 
public:
    Mat *Mask;
    Mat img;
    Quad()
    {
        topLeft = Point2f(0, 0);
        botRight = Point2f(0, 0);
        n = NULL;
        topLeftTree  = NULL;
        topRightTree = NULL;
        botLeftTree  = NULL;
        botRightTree = NULL;
	SizeMin=1;

    }
    Quad(Point2f topL, Point2f botR,float SizeMin)
    {
        n = NULL;
        topLeftTree  = NULL;
        topRightTree = NULL;
        botLeftTree  = NULL;
        botRightTree = NULL;
        topLeft = topL;
        botRight = botR;
	this->SizeMin = SizeMin;

    }
     Quad(Mat img,float SizeMin)
    {
        n = NULL;
        topLeftTree  = NULL;
        topRightTree = NULL;
        botLeftTree  = NULL;
        botRightTree = NULL;
        topLeft = Point2f(0,0);
	botRight = Point2f(img.rows-1,img.cols-1);
	//std::cout<<botRight<<std::endl;
	Mask = new Mat(img.rows,img.cols,CV_8UC1);
	this-> img = img;
	Mask[0].setTo(Scalar(255));

	this->SizeMin = SizeMin;
        

    }
    void insert(Node*);
    void insert(Node *node,Mat &Mask,Mat &img);
    Node* search(Point2f);
    bool inBoundary(Point2f);
    void MarkMask(Mat &Mask,Point2f topLeft, Point2f botRight);
    void DrawTree(Mat & img, Point2f TL, Point2f BR );
	
};
bool insert_KPs(Quad tree,vector <Point2f> keyPoints);
