#include <vector>
#include <utility> //std::pair

using namespace cv;
using namespace std;
/*
 * Author: Luiz Felipe Maciel Correia
 * y9luizufrn@gmail.com
 */

class FeatureSurf{
	public:
		
		std::pair <KeyPoint,Mat> feature_;
		
		FeatureSurf(KeyPoint kp){
					feature_.first = kp;
					
				
		}
		FeatureSurf(KeyPoint kp,Mat descriptor){
					feature_ = make_pair(kp,descriptor);
				
		}

};
