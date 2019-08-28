#ifndef __VISUALIZER_H__ 
#define __VISUALIZER_H__

#include "incl.h"
#include "model.h"
#include <cv.h>
#include <highgui.h> 
#include <ml.h>

using namespace cv;

class Visualizer
{

private:

    Mat image;
    int width; 
    int length;
    bool m_traj;
    bool m_snake;
    bool m_obstacle;
    Point2f m_object;
    RotatedRect m_obsRec;

public:

    Visualizer();

    bool checkIntersection(RotatedRect rec, Model car);

    void drawRectangle(RotatedRect rec, Mat& image, Scalar col);
    void drawTraj(Model car, vector<float> est);
    void drawVeh(Model car, vector<float> est);
    void drawVeh(Model car);
    void drawObstacle(Model car, Mat& _image);
    
    bool show(Model& car, vector<float> estimation);
    bool showSnake(Model& car);
};

#endif