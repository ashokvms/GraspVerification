#ifndef GRASP_VERIFICATION
#define GRASP_VERIFICATION

#include <opencv2/video/background_segm.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

using namespace cv;

class GraspVerification
{
public:
	GraspVerification();
	bool verify_grasp(Mat &image);

private:

	Ptr<BackgroundSubtractor> pMOG;

};

#endif
