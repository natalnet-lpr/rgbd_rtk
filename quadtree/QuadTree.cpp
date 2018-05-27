#include <iostream>
#include <cmath>
#include "QuadTree.h"
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;

// The objects that we want stored in the quadtree

 
// Insert a node into the quadtree
void Quad::insert(Node *node)
{


    if (node == NULL)
        return;
 
    // Current quad cannot contain it
    if (!inBoundary(node->pos))
        return;
 
    // We are at a quad of unit area
    // We cannot subdivide this quad further

    	
    if (abs(topLeft.x - botRight.x) <= SizeMin &&
        abs(topLeft.y - botRight.y) <= SizeMin)
    {


	if(Mask!=NULL){	

		//MarkMask(Mask[0],topLeft,botRight);
	}
        if (n == NULL)
            n = node;
        return;
    }

    if ((topLeft.x + botRight.x) / 2 >= node->pos.x)
    {
        // Indicates topLeftTree
        if ((topLeft.y + botRight.y) / 2 >= node->pos.y)
        {
            if (topLeftTree == NULL)
                topLeftTree = new Quad(
                    Point(topLeft.x, topLeft.y),
                    Point((topLeft.x + botRight.x) / 2,
                        (topLeft.y + botRight.y) / 2),SizeMin);
            topLeftTree->insert(node);
        }
 
        // Indicates botLeftTree
        else
        {
            if (botLeftTree == NULL)
                botLeftTree = new Quad(
                    Point(topLeft.x,
                        (topLeft.y + botRight.y) / 2),
                    Point((topLeft.x + botRight.x) / 2,
                        botRight.y),SizeMin);
            botLeftTree->insert(node);
        }
    }
    else
    {
        // Indicates topRightTree
        if ((topLeft.y + botRight.y) / 2 >= node->pos.y)
        {
            if (topRightTree == NULL)
                topRightTree = new Quad(
                    Point((topLeft.x + botRight.x) / 2,
                        topLeft.y),
                    Point(botRight.x,
                        (topLeft.y + botRight.y) / 2),SizeMin);
            topRightTree->insert(node);
        }
 
        // Indicates botRightTree
        else
        {
            if (botRightTree == NULL)
                botRightTree = new Quad(
                    Point((topLeft.x + botRight.x) / 2,
                        (topLeft.y + botRight.y) / 2),
                    Point(botRight.x, botRight.y),SizeMin);
            botRightTree->insert(node);
        }
    }
}
void Quad::insert(Node *node,Mat &Mask,Mat &img)
{
	

    if (node == NULL)
        return;
 
    // Current quad cannot contain it
    if (!inBoundary(node->pos))
        return;
 
    // We are at a quad of unit area
    // We cannot subdivide this quad further
    if (abs(topLeft.x - botRight.x) <= SizeMin &&
        abs(topLeft.y - botRight.y) <= SizeMin)
    {

	if(this->Mask!=NULL){	

		MarkMask(Mask,topLeft,botRight);
	}
        if (n == NULL)
            n = node;
        return;
    }

    if ((topLeft.x + botRight.x) / 2 >= node->pos.x)
    {
	DrawTree(img,topLeft,botRight);

        // Indicates topLeftTree
        if ((topLeft.y + botRight.y) / 2 >= node->pos.y)
        {
            if (topLeftTree == NULL)
                topLeftTree = new Quad(
                    Point(topLeft.x, topLeft.y),
                    Point((topLeft.x + botRight.x) / 2,
                        (topLeft.y + botRight.y) / 2),SizeMin);
            topLeftTree->insert(node,Mask,img);


        }
 
        // Indicates botLeftTree
        else
        {
            if (botLeftTree == NULL)
                botLeftTree = new Quad(
                    Point(topLeft.x,
                        (topLeft.y + botRight.y) / 2),
                    Point((topLeft.x + botRight.x) / 2,
                        botRight.y),SizeMin);
            botLeftTree->insert(node,Mask,img);
        }
    }
    else
    {
        // Indicates topRightTree
        if ((topLeft.y + botRight.y) / 2 >= node->pos.y)
        {
            if (topRightTree == NULL)
                topRightTree = new Quad(
                    Point((topLeft.x + botRight.x) / 2,
                        topLeft.y),
                    Point(botRight.x,
                        (topLeft.y + botRight.y) / 2),SizeMin);
            topRightTree->insert(node,Mask,img);
        }
 
        // Indicates botRightTree
        else
        {
            if (botRightTree == NULL)
                botRightTree = new Quad(
                    Point((topLeft.x + botRight.x) / 2,
                        (topLeft.y + botRight.y) / 2),
                    Point(botRight.x, botRight.y),SizeMin);
            botRightTree->insert(node,Mask,img);
        }
    }
}
 
// Find a node in a quadtree
Node* Quad::search(Point2f p)
{
    // Current quad cannot contain it
    if (!inBoundary(p))
        return NULL;
 
    // We are at a quad of unit length
    // We cannot subdivide this quad further
    if (n != NULL)
        return n;


 
    if ((topLeft.x + botRight.x) / 2 >= p.x)
    {	

	    /*if(Mask!=NULL){
		if(abs(topLeft.x-botRight.x)<SizeMin)
			MarkMask(Mask[0],topLeft,botRight);
	    }*/
 	
	
        // Indicates topLeftTree
        if ((topLeft.y + botRight.y) / 2 >= p.y)
        {
            if (topLeftTree == NULL)
                return NULL;
            return topLeftTree->search(p);
        }
 
        // Indicates botLeftTree
        else
        {
            if (botLeftTree == NULL)
                return NULL;
            return botLeftTree->search(p);
        }
    }
    else
    {
        // Indicates topRightTree
        if ((topLeft.y + botRight.y) / 2 >= p.y)
        {
            if (topRightTree == NULL)
                return NULL;
            return topRightTree->search(p);
        }
 
        // Indicates botRightTree
        else
        {
            if (botRightTree == NULL)
                return NULL;
            return botRightTree->search(p);
        }
    }
};
 
// Check if current quadtree contains the point
bool Quad::inBoundary(Point2f p)
{
    return (p.x >= topLeft.x &&
        p.x <= botRight.x &&
        p.y >= topLeft.y &&
        p.y <= botRight.y);
}
void Quad::MarkMask(Mat &Mask,Point2f TL, Point2f BR){

	/*Point2f TL_real(TL.y,TL.x);
	Point2f BR_real(BR.y,BR.x);
*/
	for(int i=TL.x;i<BR.x;i++){
	
		for(int j=TL.y;j<BR.y;j++){

			Mask.at<uchar>(i,j) = 0;
			
		}

	}


}
void Quad::DrawTree(Mat & img, Point2f TL, Point2f BR ){
	Point2f pt1(TL.y,TL.x);
	Point2f pt2(BR.y,TL.x);
	Point2f pt3(TL.y,BR.x);
	Point2f pt4(BR.y,BR.x);
	

	line(img, pt1, pt2, CV_RGB(0,255,0));
	line(img, pt2, pt4, CV_RGB(255,255,0));
	line(img, pt4, pt3, CV_RGB(0,255,255));
	line(img, pt3, pt1, CV_RGB(0,0,255));
}


bool insert_KPs(Quad tree,vector <Point2f> KP){
		
	if(tree.Mask==NULL)
	{
		cout<<"você inicializou a arvore errada\n utilize :      Quad(Mat img,float SizeMin)\n";
		return 0;
	}	
	for(int i=0;i<KP.size();i++){
		Point2f P(KP[i].y,KP[i].x);
		Mat a(1,1,CV_32F);
			
		
		tree.insert(new Node(P),tree.Mask[0],tree.img);
		

		
		
	}
	/*if(KP.size()>=3){
	Point2f P(KP[1].y,KP[2].x);
	tree.insert(new Node(P),tree.Mask[0],tree.img);
	P.x=KP[2].y;
	P.y=KP[2].x;
	tree.insert(new Node(P),tree.Mask[0],tree.img);
	P.x=KP[3].y;
	P.y=KP[3].x;
	tree.insert(new Node(P),tree.Mask[0],tree.img);
	}*/
	return 1;

}


