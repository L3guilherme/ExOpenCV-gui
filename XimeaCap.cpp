/*
 * File:   XimeaCap.cpp
 * Author: leandro
 *
 * Created on 28 de Janeiro de 2014, 11:30
 */

#include <m3api/xiApi.h>
#include "XimeaCap.h"

int n = 1;
XimeaCap::XimeaCap(size_t x,size_t y,size_t cx,size_t cy, bool cor) {

      XI_RETURN stat = XI_OK;


    DWORD dwNumberOfDevices = 0;
    stat = xiGetNumberDevices(&dwNumberOfDevices);
    //HandleResult(stat,"xiGetNumberDevices (no camera found)");

        if (dwNumberOfDevices)
    {
            // Retrieving a handle to the camera device
            stat = xiOpenDevice(0, &xiH);
            //Setting parameters for camera
            stat = xiSetParamInt(xiH, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);

            stat = xiSetParamInt(xiH, XI_PRM_GPO_SELECTOR, 2);

            stat = xiSetParamInt(xiH, XI_PRM_GPO_MODE,  XI_GPO_FRAME_ACTIVE_NEG);

            // Setting "exposure" parameter

             stat = xiSetParamInt(xiH, XI_PRM_EXPOSURE, 9000);

             stat = xiSetParamInt(xiH, XI_PRM_GAIN, 2);

            if(cor){
                // default image format RGB24
                stat = xiSetParamInt( xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_RGB24);
                stat = xiSetParamInt( xiH, XI_PRM_AUTO_WB, 1);
                n = 3;
                imageCV =  cvCreateImage(cvSize(x,y),IPL_DEPTH_8U,n);
            }
            else{
                xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT,  XI_RAW8);
                stat = xiSetParamInt( xiH, XI_PRM_AUTO_WB, 1);
                n = 1;
                imageCV =  cvCreateImage(cvSize(x,y),IPL_DEPTH_8U,n);
            }

            //stat = xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING_TYPE, XI_SKIPPING);
            //stat = xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, 4);

            xiSetParamInt(xiH,XI_PRM_WIDTH,x);
            xiSetParamInt(xiH,XI_PRM_HEIGHT,y);
            xiSetParamInt(xiH, XI_PRM_OFFSET_X, cx);
            xiSetParamInt(xiH, XI_PRM_OFFSET_Y, cy);

            image.size = sizeof(XI_IMG);

            imageChar = (uchar*)malloc(x*y*n);

            if((xiStartAcquisition(xiH) != XI_OK)||stat != XI_OK) goto exit;



    }else{

         exit: int i = 0;printf("ERROR!\n");

        }
}

uchar* XimeaCap::getImageChar(){


    xiSetParamInt(xiH, XI_PRM_TRG_SOFTWARE, 0);
    if(xiGetImage(xiH, 5000, &image) == XI_OK){

        imageChar = (uchar*)image.bp;

        return imageChar;

    }else{
        return NULL;
    }

}

IplImage* XimeaCap::getIplImage(){

    xiSetParamInt(xiH, XI_PRM_TRG_SOFTWARE, 0);
    if(xiGetImage(xiH, 5000, &image) == XI_OK){

        memcpy( imageCV->imageData, image.bp, image.width*image.height*n);

        return imageCV;

    }else{
        return NULL;
    }

}

cv::Mat XimeaCap::getMat(){
    cv::Mat cap;


    xiSetParamInt(xiH, XI_PRM_TRG_SOFTWARE, 0);
    if(xiGetImage(xiH, 5000, &image) == XI_OK){

        if(n == 3){
            cap.create(image.height,image.width,CV_8UC3);
            cap.data = (uchar*)image.bp;
        }else{
            cap.create(image.height,image.width,CV_8UC1);
            cap.data = (uchar*)image.bp;
        }

        return cap;

    }else{
        return cv::Mat();
    }

}


void XimeaCap::StopXimea(){

    xiStopAcquisition(xiH);

}

void XimeaCap::SetExposicao(int ex){

    xiSetParamInt(xiH, XI_PRM_EXPOSURE, ex);

}

XimeaCap::XimeaCap(const XimeaCap& orig) {
}

XimeaCap::~XimeaCap() {
}

