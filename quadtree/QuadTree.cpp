#include <iostream>
#include <cmath>
#include "QuadTree.h"
#include <opencv2/imgproc/imgproc.hpp>


void QuadTree::DrawTree(Mat & img){
	int x = Boundary.x;
        int y= Boundary.y;
        int h= Boundary.height;
        int w = Boundary.width;	
	

		Rect TR_Boundary (x,y,w,h);

	if(alocated){
		rectangle(img,TR_Boundary,Scalar(255,255,0),1);

		TopRight->DrawTree(img);



		TopLeft->DrawTree(img);

		BotLeft->DrawTree(img);

		BotRight->DrawTree(img);
	}
	

	
}

void QuadTree::DrawTree(Point2f pt,Mat & img){
	int x = pt.x;
        int y= pt.y;
        int h= Boundary.height;
        int w = Boundary.width;	
	if(divided){
		Rect TR_Boundary (x,y,w,h);

		rectangle(img,TR_Boundary,Scalar(255,255,0),3);
		TopRight->DrawTree(img);
		TopLeft->DrawTree(img);
		BotLeft->DrawTree(img);
		BotRight->DrawTree(img);
	}
	else return;
	
}

void QuadTree::MarkMask(Mat &Mask,vector<Point2f> pts ,bool initialized ){

	int x = Boundary.x;
        int y= Boundary.y;
        int h= Boundary.height;
        int w = Boundary.width;	
	float area = Mask.cols*Mask.rows;
	float density_total = pts.size()/area;

	Rect TR_Boundary (x,y,w,h);

	if(alocated){
		int n_points=0;
		for(int i=0;i<pts.size();i++){
			if(Boundary.contains(pts[i])){
				n_points++;

			}


		}
		float sub_density = n_points/float(h*w);

		if(sub_density*2>=density_total){

			if(initialized)
				rectangle(Mask,TR_Boundary,Scalar(0),-1);
		}
		else
			rectangle(Mask,TR_Boundary,Scalar(255),-1);
		
		TopRight->MarkMask(Mask,pts, true);



		TopLeft->MarkMask(Mask,pts ,true);

		BotLeft->MarkMask(Mask,pts,true);

		BotRight->MarkMask(Mask, pts,true);
	}


}

void QuadTree::UnMarkMask(Mat &Mask,Point2f TL, Point2f BR){

	/*Point2f TL_real(TL.y,TL.x);
	Point2f BR_real(BR.y,BR.x);
*/
	for(int i=TL.x;i<BR.x;i++){
	
		for(int j=TL.y;j<BR.y;j++){

			Mask.at<uchar>(i,j) = 255;
			
		}

	}


}
 void QuadTree::Subdivide()
 {
        int x = Boundary.x;
        int y= Boundary.y;
        int h= Boundary.height;
        int w = Boundary.width;

        Rect TL_Boundary (x,y,h/2,w/2);

	

        TopLeft = new QuadTree(TL_Boundary,capacity);

        Rect TR_Boundary (x+w/2,y,h/2,w/2);


        TopRight = new QuadTree(TR_Boundary,capacity);

        Rect BR_Boundary (x+w/2,y+h/2,h/2,w/2);


        BotRight = new QuadTree(BR_Boundary,capacity);


        Rect BL_Boundary (x,y+h/2,h/2,w/2);

        BotLeft = new QuadTree(BL_Boundary,capacity);

        divided=true;

    }

 void QuadTree::Insert(Point2f new_point)
    {
        if(!Boundary.contains(new_point))
            return;
        if( ( pts.size()<capacity))
        {

            pts.push_back(new_point);

        }
        else
        {
            if(!divided)
            {

                Subdivide();

            }


            TopRight->Insert(new_point);
            TopLeft->Insert(new_point);
            BotRight->Insert(new_point);
            BotLeft->Insert(new_point);

        }

    }
void QuadTree::Insert(Point2f new_point,Mat &img, float size_min)
{
	this->alocated = true;
	//circle(img, new_point, 1, Scalar(255), 1);

        if(!Boundary.contains(new_point))
            return;
        if( ( pts.size()<capacity))
        {

            pts.push_back(new_point);

        }
        else
        {

            if(!divided )
            {

                Subdivide();
		divided = true;

            }

		
		
            	TopRight->Insert(new_point,img,size_min);

           	TopLeft->Insert(new_point,img,size_min);
		
           	BotRight->Insert(new_point,img,size_min);
            	BotLeft->Insert(new_point,img,size_min);
	   

        }

 }

