#include <vector>
#include <opencv2/core/core.hpp>
using namespace cv;
using namespace std;
class QuadTree
{
public:

    Rect Boundary;
    int capacity;
    bool divided = false;
    bool alocated = false;
    vector<Point2f> pts;
    QuadTree * TopLeft;
    QuadTree * TopRight;
    QuadTree * BotLeft ;
    QuadTree * BotRight;
	
	
    QuadTree()
    {

        capacity =4;


    }
    QuadTree(Rect Boundary,int capacity)
    {

        this->Boundary = Boundary;
        this->capacity = capacity;
        TopLeft = new QuadTree();
        TopRight = new QuadTree();
        BotLeft =new QuadTree();
        BotRight = new QuadTree();


    }

    virtual void Subdivide();
    virtual void Insert(Point2f new_point);
    virtual void Insert(Point2f new_point,Mat &img, float size_min);
    virtual void DrawTree(Mat &img);
    virtual void DrawTree(Point2f pt,Mat &img);
    virtual void MarkMask(Mat &Mask,vector<Point2f> pts ,bool initialized);

    virtual void UnMarkMask(Mat &Mask,Point2f topLeft, Point2f botRight);

};
