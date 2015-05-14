/* 
 * File:   XimeaCap.h
 * Author: leandro
 *
 * Created on 28 de Janeiro de 2014, 11:30
 */

#include <xiApi.h>
#include "opencv2/opencv.hpp"
#include "opencv/highgui.h"

#ifndef XIMEACAP_H
#define	XIMEACAP_H

class XimeaCap {
public:
    XimeaCap(size_t x,size_t y,size_t cx,size_t cy,bool cor);
    XimeaCap(const XimeaCap& orig);
    virtual ~XimeaCap();
    IplImage* getIplImage();
    cv::Mat getMat();
    uchar* getImageChar();
    void SetExposicao(int ex);
    void StopXimea();
private:
    HANDLE xiH;
    XI_IMG image;
    IplImage* imageCV;
    uchar*    imageChar;

};

#endif	/* XIMEACAP_H */

