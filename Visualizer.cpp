#include "Visualizer.h"

Visualizer::Visualizer()
{
    width = 600; 
    length = 600;
    image = Mat::zeros(cv::Size(length, width), CV_8UC3);
    m_traj = false;
    m_obstacle = false;
    m_snake = false;
    m_obsRec = RotatedRect(Point2f(0,0), Size2f(0,0), 0);
}

bool Visualizer::checkIntersection(RotatedRect rec, Model car)
{
    Point2i p1 = Point2i(static_cast<int>(car.getPos()[0]) % width, static_cast<int>(car.getPos()[1]) % length);
    RotatedRect vehRect = RotatedRect(p1,
                                      Size2f(CAR_SIZE_LENGTH * 6, 6 *CAR_SIZE_WIDTH),
                                      180 * car.getAngle() / M_PI);
    Point2f vr[4];
    Point2f vc[4];
    rec.points(vr);
    vehRect.points(vc);
    return checkRecIntersect(vr, vc);
}

void Visualizer::drawRectangle(RotatedRect rec, Mat& _image, Scalar col)
{
    Point2f vertices[4]; 
    rec.points(vertices);
    for(int i = 0; i < 4; i++){
        if(i == 2) line(_image, vertices[i], vertices[i+1], Scalar(255,255,255));
        else line(_image, vertices[i], vertices[(i+1) % 4], col);
    }
}

void Visualizer::drawVeh(Model car, vector<float> est)
{
    Point2i p1 = Point2i(static_cast<int>(car.getPos()[0]) % width, static_cast<int>(car.getPos()[1]) % length);
    Point2i p2 = Point2i(static_cast<int>(est[0]) % width, static_cast<int>(est[1]) % length);
    RotatedRect vehRect = RotatedRect(p1,
                                      Size2f(CAR_SIZE_LENGTH * 6, 6 *CAR_SIZE_WIDTH),
                                      180 * car.getAngle() / M_PI);
    RotatedRect estRect = RotatedRect(p2,
                                      Size2f(CAR_SIZE_LENGTH * 6, 6 *CAR_SIZE_WIDTH),
                                      180 * car.getAngle() / M_PI);

    if(!m_traj) image = Mat::zeros(cv::Size(length, width), CV_8UC3);
    drawRectangle(vehRect, image, Scalar(255,0,0));
    drawRectangle(estRect, image, Scalar(0,0,255));
}   

void Visualizer::drawVeh(Model car)
{
    Point2i p1 = Point2i(static_cast<int>(car.getPos()[0]) % width, static_cast<int>(car.getPos()[1]) % length);
    RotatedRect vehRect = RotatedRect(p1,
                                      Size2f(CAR_SIZE_LENGTH * 6, 6 *CAR_SIZE_WIDTH),
                                      180 * car.getAngle() / M_PI);
    if(!m_traj) image = Mat::zeros(cv::Size(length, width), CV_8UC3);
    drawRectangle(vehRect, image, Scalar(255,0,0));
}

void Visualizer::drawTraj(Model car, vector<float> est)
{
    circle(image,
            cv::Point2f(est[0],est[1]),
            1,
            cv::Scalar(0,0,255),
            -1,
            CV_AA);
    circle(image,
            cv::Point2f(car.getPos()[0], car.getPos()[1]),
            1,
            cv::Scalar(255,0,0),
            -1,
            CV_AA);
}

void Visualizer::drawObstacle(Model car, Mat& _image)
{
    if(!m_obstacle){
        default_random_engine gen(rand()); 
        uniform_real_distribution<float> distribution(100,600);
        m_object.x = distribution(gen);
        m_object.y = distribution(gen);
        m_obsRec = RotatedRect(m_object, Size2f(10,10), 0);
    }
    m_obstacle = true;
    if(!checkIntersection(m_obsRec, car)){
        drawRectangle(m_obsRec, _image, Scalar(0,255,0));
    }
    else{
        m_obstacle = false; 
        cout << "collision" << endl;
    } 
}

bool Visualizer::show(Model& car, vector<float> estimation)
{
    drawVeh(car, estimation);
    drawTraj(car, estimation); 

    imshow("image", image);

    car.acc = 0;
    // car.steeringAngle = 0;
    char key = waitKey(50);

    if(key == 'j' || key == 'J') car.steeringAngle -= M_PI/180;
    if(key == 'l' || key == 'L') car.steeringAngle += M_PI/180;
    if(key == 'o' || key == 'O') drawObstacle(car, image);
    if(key == 't' || key == 'T') m_traj = !m_traj;
    if(key == 'k' || key == 'K') car.acc -= 5;
    if(key == 'i' || key == 'I') car.acc += 5;

    return (key == 'a' || key == 'A')? false : true;
}

bool Visualizer::showSnake(Model& car)
{
    drawVeh(car);
    char key = waitKey(50);

    car.acc = 0;
    car.steeringAngle = 0;
    
    if(key == 'o' || key == 'O'){
        m_snake = !m_snake;
    }
    if(m_snake){
        drawObstacle(car, image);
    
    }
    if(key == 'j' || key == 'J') car.steeringAngle -= M_PI/36;
    if(key == 'l' || key == 'L') car.steeringAngle += M_PI/36;
    if(key == 'i' || key == 'I') car.acc += 5;
    if(key == 'k' || key == 'K') car.acc -= 5;

    imshow("image", image);

    return (key == 'a' || key == 'A')? false : true;
}