#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QTime>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/stitching/stitcher.hpp"
#include "opencv/cv.h"
#include "opencv2/ocl/ocl.hpp"
#include <opencv2/ml/ml.hpp>
#include <CL/cl.hpp>
#include <CL/cl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <zbar.h>
#include <thread>
#include <mutex>
#include <QtGui>
#include <QGLWidget>
#include <openglp.h>


bool parar = false;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool Calibrar(std::vector<cv::Mat> imgCamera,cv::Size tamTab = cv::Size(4,4)){

    std::vector<std::vector<cv::Point2f> > imagePoints;
    cv::Size imageSize(imgCamera[0].cols,imgCamera[0].rows);
    cv::Mat distCoeffs;
    float squareSize = 14.85f;
    cv::Size boardSize = tamTab;

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Point3f> corners;
    corners.resize(0);

    std::vector<cv::Mat> rvecs; std::vector<cv::Mat> tvecs;

    std::vector<std::vector<cv::Point3f> > objectPoints;

    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            corners.push_back(cv::Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));

    for(uint i = 0;i<imgCamera.size();i++)
    {
        cv::Mat view, viewGray;

        imgCamera[i].copyTo(view);

        imageSize = view.size();

        std::vector<cv::Point2f> pointbuf;

        if(view.channels() > 1){

            cv::cvtColor(view, viewGray, CV_BGR2GRAY);
        }else{
            view.copyTo(viewGray);
        }



        bool found = findChessboardCorners( viewGray, boardSize,
                                            pointbuf,CV_CALIB_CB_ADAPTIVE_THRESH| CV_CALIB_CB_FAST_CHECK |
                                            CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_NORMALIZE_IMAGE);

        if(found){
            cv::cornerSubPix( viewGray, pointbuf, cv::Size(11,11),cv::Size(-1,-1),
                              cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.01 ));
            imagePoints.push_back(pointbuf);
            objectPoints.push_back(corners);
        }else{
            std::cout<<"Processo abortado! Tabuleiro não encontrado na imagem: "<<i<<std::endl;
            return false;
            break;
        }

    }



    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                     distCoeffs, rvecs, tvecs, CV_CALIB_FIX_ASPECT_RATIO|CV_CALIB_FIX_INTRINSIC|CV_CALIB_FIX_K3|
                                     CV_CALIB_FIX_K4|CV_CALIB_FIX_K5|CV_CALIB_FIX_K5);

    std::cout<<"rms= "<<rms<<std::endl;

    cv::VideoCapture cap (0);

    cv::Mat img,imgU,imgC;

    while(!parar){

        cap >> img;

        cv::undistort(img, imgU, cameraMatrix, distCoeffs);

        cv::imshow("Uimg",imgU);

        std::vector<cv::Point2f> pointbuf;

        if(img.channels() > 1){

            cv::cvtColor(img, imgC, CV_BGR2GRAY);
        }else{
            img.copyTo(imgC);
        }


        bool found = findChessboardCorners( imgC, boardSize,
                                            pointbuf,CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);

        if(found){

            cv::Mat rVecObj,tVecObj;

            cv::solvePnP(corners, pointbuf, cameraMatrix, distCoeffs, rVecObj, tVecObj, false);

            float dist = sqrt(tVecObj.at<double>(0,0)*tVecObj.at<double>(0,0)+tVecObj.at<double>(1,0)*tVecObj.at<double>(1,0)
                              +tVecObj.at<double>(2,0)*tVecObj.at<double>(2,0));
            std::cout<<"Dinst= "<<dist<<std::endl;
        }

        char tecla = cv::waitKey(5);
        if(tecla == 's'){

            cv::FileStorage fs( "calibracao.yml", cv::FileStorage::WRITE );

            time_t tt;
            time( &tt );
            struct tm *t2 = localtime( &tt );
            char buf[1024];
            strftime( buf, sizeof(buf)-1, "%c", t2 );

            fs << "calibration_time" << buf;

            fs << "image_width" << img.cols;
            fs << "image_height" << img.rows;
            fs << "board_width" << boardSize.width;
            fs << "board_height" << boardSize.height;
            fs << "square_size" << squareSize;

            fs << "camera_matrix" << cameraMatrix;
            fs << "distortion_coefficients" << distCoeffs;

            //fs << "avg_reprojection_error" << totalAvgErr;

            if( !rvecs.empty() && !tvecs.empty() )
            {
                CV_Assert(rvecs[0].type() == tvecs[0].type());
                cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
                for( int i = 0; i < (int)rvecs.size(); i++ )
                {
                    cv::Mat r = bigmat(cv::Range(i, i+1), cv::Range(0,3));
                    cv::Mat t = bigmat(cv::Range(i, i+1), cv::Range(3,6));

                    CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                    CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                    //*.t() is MatExpr (not Mat) so we can use assignment operator
                    r = rvecs[i].t();
                    t = tvecs[i].t();
                }
                cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
                fs << "extrinsic_parameters" << bigmat;
            }

            if( !imagePoints.empty() )
            {
                cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
                for( int i = 0; i < (int)imagePoints.size(); i++ )
                {
                    cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
                    cv::Mat imgpti(imagePoints[i]);
                    imgpti.copyTo(r);
                }
                fs << "image_points" << imagePtMat;
            }

        }
        if(tecla == 'c')break;
    }

    return true;
}

void MainWindow::on_btnIniciar_clicked()
{
    parar = false;
    cv::Mat img;
    cv::VideoCapture capP;
    cv::Mat ref = cv::imread(ui->edtTxtEndRef->text().toStdString());
    //XimeaCap capXimea(1024,1024,0,0,true);

    cv::Mat imgRef;
    cv::Mat imgCop;

    cv::Mat objDescriptors;

    cv::Mat objData;

    cv::DescriptorExtractor *extrator = new cv::SURF();
    cv::FeatureDetector *detector =new cv::SurfFeatureDetector();

    std::vector<cv::KeyPoint> objKeypoints;

    std::vector<cv::Mat> imgCal;
    std::vector<cv::Point2f> pointbuf;
    bool found;
    clock_t prevTimestamp = 0;
    int delay = 1000;


    switch (ui->tabWidget->currentIndex()){
    case 0:

        if(ui->radioBtnDsipP->isChecked()){
            cv::VideoCapture cap (ui->spinBoxNCam->value());
            capP = cap;
        }
        while(!parar){
            if(ui->radioBtnDsipP->isChecked()){
                capP >> img;
            }


            cv::Mat dst, imgCinza;

            cv::cvtColor(img,imgCinza,CV_BGR2GRAY);

            ui->lblsNivelTH->setText(QString::number(ui->barrraHTH->value()));

            if(ui->radioBtnTHSimp->isChecked())cv::threshold(imgCinza,dst,(double)(ui->barrraHTH->value()),255,cv::THRESH_BINARY);
            if(ui->radioBtnOTSU->isChecked())cv::threshold(imgCinza,dst,(double)(ui->barrraHTH->value()),255,cv::THRESH_OTSU);



            cv::resize(dst,dst,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));
            cv::cvtColor(dst,dst,CV_GRAY2RGB);

            QImage image = QImage((uint8_t*) dst.data,dst.cols,dst.rows,dst.step,QImage::Format_RGB888);

            QPixmap pixma = QPixmap::fromImage(image);

            ui->lblIMG->setPixmap(pixma);

            ui->lblIMG->setFixedSize(pixma.size());


            cv::resize(img,img,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));
            cv::cvtColor(img,img,CV_BGR2RGB);
            image = QImage((uint8_t*) img.data,img.cols,img.rows,img.step,QImage::Format_RGB888);

            pixma = QPixmap::fromImage(image);

            ui->lblIMGOrg->setPixmap(pixma);

            ui->lblIMGOrg->setFixedSize(pixma.size());

            cv::waitKey(10);

        }

        break;

    case 1:

        if(ui->radioBtnDsipP->isChecked()){
            cv::VideoCapture cap (ui->spinBoxNCam->value());
            capP = cap;
        }


        while(!parar){
            if(ui->radioBtnDsipP->isChecked()){
                capP >> img;
            }


            cv::Mat dst, imgCinza;

            cv::cvtColor(img,imgCinza,CV_BGR2GRAY);


            if(ui->radioBtnSobel->isChecked()){
                //GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

                int scale = 1;
                int delta = 0;
                int ddepth = CV_16S;

                /// Generate grad_x and grad_y
                cv::Mat grad_x, grad_y;
                cv::Mat abs_grad_x, abs_grad_y;

                /// Gradient X
                //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
                cv::Sobel( imgCinza, grad_x, ddepth, 1, 0, 1, scale, delta, cv::BORDER_DEFAULT );
                cv::convertScaleAbs( grad_x, abs_grad_x );

                /// Gradient Y
                //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
                cv::Sobel( imgCinza, grad_y, ddepth, 0, 1, 1, scale, delta, cv::BORDER_DEFAULT );
                cv::convertScaleAbs( grad_y, abs_grad_y );

                /// Total Gradient (approximate)
                cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst );

            }
            if(ui->radioBtnCanny->isChecked())
                cv::Canny(imgCinza,dst,(double)(ui->barraCanInf->value()),
                          (double)(ui->barraCanSup->value()));

            cv::resize(dst,dst,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));

            cv::cvtColor(dst,dst,CV_GRAY2RGB);

            QImage image = QImage((uint8_t*) dst.data,dst.cols,dst.rows,dst.step,QImage::Format_RGB888);

            QPixmap pixma = QPixmap::fromImage(image);

            ui->lblIMG->setPixmap(pixma);

            ui->lblIMG->setFixedSize(pixma.size());

            cv::resize(img,img,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));
            cv::cvtColor(img,img,CV_BGR2RGB);
            image = QImage((uint8_t*) img.data,img.cols,img.rows,img.step,QImage::Format_RGB888);

            pixma = QPixmap::fromImage(image);

            ui->lblIMGOrg->setPixmap(pixma);

            ui->lblIMGOrg->setFixedSize(pixma.size());

            cv::waitKey(10);

        }
        break;

    case 2:

        if(ui->radioBtnDsipP->isChecked()){
            cv::VideoCapture cap (ui->spinBoxNCam->value());
            capP = cap;
        }

        while(!parar){
            if(ui->radioBtnDsipP->isChecked()){
                capP >> img;
            }


            cv::Mat dst, matHSV;

            cv::cvtColor(img,matHSV,CV_BGR2HSV);

            cv::Mat matTh;

            cv::inRange(matHSV,cv::Scalar(ui->barraHHinf->value(),ui->barraHSinf->value(),ui->barraHVinf->value()),
                        cv::Scalar(ui->barraHHsup->value(),ui->barraHSsup->value(),ui->barraHVsup->value()),
                        matTh);

            if(ui->spinBoxInter->value() >0){
                if(ui->radioBtnDilate->isChecked()) cv::dilate(matTh,matTh,cv::Mat(),cv::Point(-1,-1),ui->spinBoxInter->value());
                if(ui->radioBtnErude->isChecked()) cv::erode(matTh,matTh,cv::Mat(),cv::Point(-1,-1),ui->spinBoxInter->value());
            }

            dst = matTh;
            cv::resize(dst,dst,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));
            cv::cvtColor(dst,dst,CV_GRAY2RGB);
            QImage image = QImage((uint8_t*) dst.data,dst.cols,dst.rows,dst.step,QImage::Format_RGB888);

            QPixmap pixma = QPixmap::fromImage(image);

            ui->lblIMG->setPixmap(pixma);

            ui->lblIMG->setFixedSize(pixma.size());

            cv::resize(img,img,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));
            cv::cvtColor(img,img,CV_BGR2RGB);
            image = QImage((uint8_t*) img.data,img.cols,img.rows,img.step,QImage::Format_RGB888);

            pixma = QPixmap::fromImage(image);

            ui->lblIMGOrg->setPixmap(pixma);

            ui->lblIMGOrg->setFixedSize(pixma.size());

            cv::waitKey(10);

        }

        break;

    case 3:
        if(ui->radioBtnDsipP->isChecked()){
            cv::VideoCapture cap (ui->spinBoxNCam->value());
            capP = cap;
        }


        while(!parar){

            cv::Mat imgCinza;
            cv::Mat imgCanny;

            if(ui->radioBtnDsipP->isChecked()){
                capP >> img;
            }


            cv::cvtColor(img,imgCinza,CV_BGR2GRAY);

            cv::Canny(imgCinza,imgCanny,(double)(ui->barraCanInf->value()),
                      (double)(ui->barraCanSup->value()));

            if(!ui->checkBoxCircH->isChecked()){
                std::vector<cv::Vec4i> lines;
                //cv::HoughLinesP(imgCanny, lines, 1.3, CV_PI/180, 150, 50, 50 );
                cv::HoughLinesP(imgCanny, lines,ui->SpinBoxDRho->value(), ui->SpinBoxDTheta->value()*CV_PI/180,
                                ui->spinBoxNVotos->value(), (double)ui->spinBoxTamReta->value(),
                                (double)ui->spinBoxTamReta->value() );

                for( size_t j = 0; j < lines.size(); j++ )
                    for( size_t i = j; i < lines.size(); i++ )
                    {
                        cv::Vec4i l = lines[i];
                        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
                    }
            }else{

                std::vector<cv::Vec3f> circles;

                cv::HoughCircles( imgCanny, circles, CV_HOUGH_GRADIENT, 1, img.rows/8, 200, 100, 0, 0 );

                for( size_t i = 0; i < circles.size(); i++ )
                {
                    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                    int radius = cvRound(circles[i][2]);
                    // circle center
                    cv::circle( img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
                    // circle outline
                    cv::circle( img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
                }

            }
            cv::resize(imgCanny,imgCanny,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));
            cv::cvtColor(imgCanny,imgCanny,CV_GRAY2RGB);
            QImage image = QImage((uint8_t*) imgCanny.data,imgCanny.cols,imgCanny.rows,imgCanny.step,QImage::Format_RGB888);

            QPixmap pixma = QPixmap::fromImage(image);

            ui->lblIMG->setPixmap(pixma);

            ui->lblIMG->setFixedSize(pixma.size());

            cv::cvtColor(img,img,CV_BGR2RGB);
            image = QImage((uint8_t*) img.data,img.cols,img.rows,img.step,QImage::Format_RGB888);

            pixma = QPixmap::fromImage(image);

            ui->lblIMGOrg->setPixmap(pixma);

            ui->lblIMGOrg->setFixedSize(pixma.size());

            cv::waitKey(10);

        }

        break;

    case 4 :
        if(ui->radioBtnDsipP->isChecked()){
            cv::VideoCapture cap (ui->spinBoxNCam->value());
            capP = cap;
        }

        while(!parar){

            if(ui->radioBtnDsipP->isChecked()){
                capP >> img;
            }

            cv::Mat imgCinza;
            cv::Mat dst = cv::Mat::zeros( img.size(), CV_32FC1 );

            cv::cvtColor(img,imgCinza,CV_BGR2GRAY);

            cv::cornerHarris( imgCinza, dst,ui->spinBoxHarrisTJ->value(),
                              ui->spinBoxHarrisSobel->value(),
                              ui->SpinBoxDHarrisk->value(), cv::BORDER_DEFAULT );

            cv::Mat dst_norm, dst_norm_scaled;
            cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
            cv::convertScaleAbs( dst_norm, dst_norm_scaled );

            for( int j = 0; j < dst_norm.rows ; j++ )
            { for( int i = 0; i < dst_norm.cols; i++ )
                {
                    if( (int) dst_norm.at<float>(j,i) > ui->brrrasTHHarris->value() )
                    {
                        cv::circle( img, cv::Point( i, j ), 5,  cv::Scalar(0), 2, 8, 0 );
                    }
                }
            }

            cv::resize(dst_norm_scaled,dst_norm_scaled,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));
            dst_norm_scaled.convertTo(dst_norm_scaled,CV_8UC1);
            cv::cvtColor(dst_norm_scaled,dst_norm_scaled,CV_GRAY2RGB);
            QImage image = QImage((uint8_t*) dst_norm_scaled.data,dst_norm_scaled.cols,dst_norm_scaled.rows,
                                  dst_norm_scaled.step,QImage::Format_RGB888);

            QPixmap pixma = QPixmap::fromImage(image);

            ui->lblIMG->setPixmap(pixma);

            ui->lblIMG->setFixedSize(pixma.size());

            cv::resize(img,img,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));
            cv::cvtColor(img,img,CV_BGR2RGB);
            image = QImage((uint8_t*) img.data,img.cols,img.rows,img.step,QImage::Format_RGB888);

            pixma = QPixmap::fromImage(image);

            ui->lblIMGOrg->setPixmap(pixma);

            ui->lblIMGOrg->setFixedSize(pixma.size());

            cv::waitKey(10);

        }
        break;

    case 5:

        if(ui->radioBtnDsipP->isChecked()){
            cv::VideoCapture cap (ui->spinBoxNCam->value());
            capP = cap;
        }

        ref = cv::imread(ui->edtTxtEndRef->text().toStdString());

        cv::cvtColor(ref,imgRef,CV_BGR2GRAY);


        detector->detect(imgRef, objKeypoints);
        extrator->compute(imgRef, objKeypoints, objDescriptors);

        if(objDescriptors.type()!=CV_32F) {
            objDescriptors.convertTo(objData, CV_32F);
        }
        else {
            objData = objDescriptors;
        }

        while(!parar){
            if(ui->radioBtnDsipP->isChecked()){
                capP >> img;
            }


            cv::Mat imgCinza;
            cv::Mat dst;

            cv::cvtColor(img,imgCinza,CV_BGR2GRAY);

            if(ui->radioBtnCasPad->isChecked()){

                cv::matchTemplate( imgCinza, imgRef, dst, CV_TM_SQDIFF );
                cv::normalize( dst, dst, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

                double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;

                cv::minMaxLoc( dst, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
                cv:: Point matchLoc = minLoc;

                cv::rectangle( img, matchLoc, cv::Point( matchLoc.x + ref.cols , matchLoc.y + ref.rows ),
                               cv::Scalar::all(0), 2, 8, 0 );
                cv::rectangle( dst, matchLoc, cv::Point( matchLoc.x + ref.cols , matchLoc.y + ref.rows ),
                               cv::Scalar::all(0), 2, 8, 0 );
            }

            if(ui->radioBtnSURF->isChecked()){

                cv::Mat imgDescriptors;

                std::vector<cv::KeyPoint> imgKeypoints;
                std::vector< cv::DMatch > good_matches;
                good_matches.clear();

                detector->detect(imgCinza, imgKeypoints);
                extrator->compute(imgCinza, imgKeypoints, imgDescriptors);

                cv::Mat imgData;

                if(imgDescriptors.type()!=CV_32F) {
                    imgDescriptors.convertTo(imgData, CV_32F);
                }
                else {
                    imgData = imgDescriptors;
                }

                cv::FlannBasedMatcher matcherF;
                std::vector< std::vector<cv::DMatch> >  matchesF;


                matcherF.knnMatch(objDescriptors, imgDescriptors, matchesF, 2);


                for(int i = 0; i < cv::min(imgDescriptors.rows-1,(int) matchesF.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
                {
                    if((matchesF[i][0].distance < 0.7999*(matchesF[i][1].distance)) && ((int) matchesF[i].size()<=2 && (int) matchesF[i].size()>0))
                    {
                        good_matches.push_back(matchesF[i][0]);
                    }
                }

                std::vector<cv::Point2f> objPts;
                std::vector<cv::Point2f> imgPts;

                for( int i = 0; i < (int)good_matches.size(); i++ )
                {
                    objPts.push_back( objKeypoints[ good_matches.back().queryIdx ].pt );
                    imgPts.push_back( imgKeypoints[ good_matches.back().trainIdx ].pt );
                }

                std::vector<cv::Point2f> img_corners(4);
                std::vector<cv::Point2f> obj_corners(4);
                //Get the corners from the object
                obj_corners[0] = cvPoint(0,0);
                obj_corners[1] = cvPoint( imgRef.cols, 0 );
                obj_corners[2] = cvPoint( imgRef.cols, imgRef.rows );
                obj_corners[3] = cvPoint( 0, imgRef.rows );

                try{

                    cv::Mat H = cv::findHomography( objPts, imgPts, CV_RANSAC );

                    cv::perspectiveTransform( obj_corners, img_corners, H);

                    cv::line( img, img_corners[0] , img_corners[1] , cv::Scalar(0, 255, 0), 4 );
                    cv::line( img, img_corners[1] , img_corners[2] , cv::Scalar( 0, 255, 0), 4 );
                    cv::line( img, img_corners[2] , img_corners[3] , cv::Scalar( 0, 255, 0), 4 );
                    cv::line( img, img_corners[3] , img_corners[0] , cv::Scalar( 0, 255, 0), 4 );
                }catch(int e){}

                cv::drawMatches( imgRef, objKeypoints, img, imgKeypoints, good_matches, imgCop, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            }
            if(!imgCop.empty()){

                cv::resize(imgCop,imgCop,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));

                cv::cvtColor(imgCop,imgCop,CV_BGR2RGB);
                QImage image = QImage((uint8_t*) imgCop.data,imgCop.cols,imgCop.rows,
                                      imgCop.step,QImage::Format_RGB888);

                QPixmap pixma = QPixmap::fromImage(image);

                ui->lblIMG->setPixmap(pixma);

                ui->lblIMG->setFixedSize(pixma.size());
            }


            cv::resize(img,img,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));
            cv::cvtColor(img,img,CV_BGR2RGB);
            QImage image = QImage((uint8_t*) img.data,img.cols,img.rows,img.step,QImage::Format_RGB888);

            QPixmap pixma = QPixmap::fromImage(image);

            ui->lblIMGOrg->setPixmap(pixma);

            ui->lblIMGOrg->setFixedSize(pixma.size());

            cv::waitKey(10);


        }
        break;

    case 6:

        if(ui->radioBtnDsipP->isChecked()){
            cv::VideoCapture cap (ui->spinBoxNCam->value());
            capP = cap;
        }


        while(!parar){

            if(ui->radioBtnDsipP->isChecked()){
                capP >> img;
            }


            cv::Mat img2;
            img.copyTo(img2);

            cv::cvtColor(img, img, CV_BGR2GRAY);


            found = findChessboardCorners( img,cv::Size(ui->spinBoxXtab->text().toInt(),ui->spinBoxYTab->text().toInt()),
                                           pointbuf,
                                           CV_CALIB_CB_ADAPTIVE_THRESH| CV_CALIB_CB_FAST_CHECK |
                                           CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_NORMALIZE_IMAGE);

            int deltaX = img2.cols/3;
            int deltaY = img2.rows/3;
            cv::Point pi(deltaX,0);
            cv::Point pf(deltaX,3*deltaY);

            for(int i = 0;i<2;i++){
                cv::line(img2,pi,pf,cv::Scalar(0,0,255));
                pi.x +=deltaX;
                pf.x +=deltaX;
            }

            pi = cv::Point(0,deltaY);
            pf = cv::Point(3*deltaX,deltaY);

            for(int j = 0;j<2;j++){
                cv::line(img2,pi,pf,cv::Scalar(0,0,255));
                pi.y +=deltaY;
                pf.y +=deltaY;
            }

            cv::imshow("img",img2);
            char tecla = cv::waitKey(100);

            if(tecla == 'c') {break;}

            if(found && ((clock() - prevTimestamp) > (delay*1e-3*CLOCKS_PER_SEC))){
                //std::cout<<"Aceitar img? (Sim ou Não)"<<std::endl;


                //if(tecla == 's'){
                imgCal.push_back(img2.clone());
                prevTimestamp = clock();
                cv::drawChessboardCorners( img, cv::Size(ui->spinBoxXtab->text().toInt(),ui->spinBoxYTab->text().toInt()), cv::Mat(pointbuf), found );
                cv::Mat imgB;
                cv::bitwise_not(img2, imgB);
                cv::imshow("img",imgB);
                //}

                //td::cout<<"Calibrar? (Calibrar ou Pular )"<<std::endl;



            }

            cv::waitKey(5);
        }

        capP.release();

        if(imgCal.size() > 0){ Calibrar(imgCal,cv::Size(ui->spinBoxXtab->text().toInt(),ui->spinBoxYTab->text().toInt()));}


        break;

    case 7:
        break;

    case 8:
        if(ui->radioBtnDsipP->isChecked()){
            cv::VideoCapture cap (ui->spinBoxNCam->value());
            capP = cap;
        }

        while(!parar){

            if(ui->radioBtnDsipP->isChecked()){
                capP >> img;
            }


            zbar::ImageScanner scanner;

            scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

            cv::Mat grey;
            cv::Mat qr = img;

            //cv::imshow("QRimgPre",qr);
            //cv::waitKey();

            cv::cvtColor(qr,grey,CV_BGR2GRAY);
            cv::threshold(grey,grey,(double)(ui->barrraHTH->value()),255,cv::THRESH_OTSU);
            int width = qr.cols;
            int height = qr.rows;
            uchar *raw = (uchar *)grey.data;
            // wrap image data
            zbar::Image image(width, height, "Y800", raw, width * height);
            // scan the image for barcodes
            int n = scanner.scan(image);

            int contQR = -1;

            // extract results
            if (n != 0)
            {
                for(zbar::Image::SymbolIterator symbol = image.symbol_begin();
                    symbol != image.symbol_end();
                    ++symbol)
                {
                    contQR++;
                    std::vector<cv::Point2f> vp;
                    // do something useful with results
                    std::cout << "decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' <<" "<< std::endl;

                    std::string strQR(symbol->get_data());

                    int posCor = strQR.find("Marcador:");

                    char num[1];
                    num[0] = strQR.data()[posCor+10];
                    int index = atoi (num);

                    std::cout<<"Cor Pos: "<<posCor<<" Index "<<index<<std::endl;

                    //this->cor = (int)( strQR[posCor+10]);

                    int n = symbol->get_location_size();
                    for(int i=0;i<n;i++)
                    {
                        vp.push_back(cv::Point2f(symbol->get_location_x(i),symbol->get_location_y(i)));
                    }
                    cv::RotatedRect r = cv::minAreaRect(vp);
                    cv::Point2f pts[4];
                    r.points(pts);

                    //CvPoint2D32f *centroQR = CentroTab(vp);
                    for(int i=0;i<4;i++){
                        //cv::line(roi,pts[i],pts[(i+1)%4],cv::Scalar(255,0,0),3);

                        if(!img.empty()) {
                            cv::line(img,pts[i],pts[(i+1)%4],cv::Scalar(255,0,0),3);
                            cv::circle(img,r.center,5,cv::Scalar(0,255,255),-1);
                        }
                    }
                    std::cout<<"Angle: "<<r.angle<<std::endl;

                    //cv::putText( roi,symbol->get_data(), cv::Point(symbol->get_location_x(2),symbol->get_location_y(2))+cv::Point(-400,-10), 1,1,cv::Scalar(0,255,255),2);
                    if(!img.empty())cv::putText( img,symbol->get_data(), cv::Point(symbol->get_location_x(2),symbol->get_location_y(2))+cv::Point(-400,-10), 1,1,cv::Scalar(0,255,255),2);



                }
            }else{

                std::cout<<"QR NÃO ENCONTRADO!"<<std::endl;
            }

            cv::resize(img,img,cv::Size(img.cols/ui->spinBoxRaz->value(),img.rows/ui->spinBoxRaz->value()));
            cv::cvtColor(img,img,CV_BGR2RGB);
            QImage imageQt = QImage((uint8_t*) img.data,img.cols,img.rows,img.step,QImage::Format_RGB888);

            QPixmap pixma = QPixmap::fromImage(imageQt);

            ui->lblIMGOrg->setPixmap(pixma);

            ui->lblIMGOrg->setFixedSize(pixma.size());

            cv::waitKey(10);

        }

        break;

    }






}

void MainWindow::on_btnParar_clicked()
{
    parar = true;
}

typedef struct pontoM {
cv::Point pt;
bool marcado;
} pontoM;

void contornosCor(cv::Mat imgF,cv::Mat img){
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<pontoM> pts;

    cv::findContours(imgF,contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    std::cout<<"Contornos: "<<contours.size()<<std::endl;
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    for(uint i = 0;i<contours.size();i++){
        if(contours[i].size()>10){
            cv::drawContours( img, contours, i, cv::Scalar(255,0,0), 1, 8, hierarchy, 0, cv::Point() );

            cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i],3.f, true );
            if(contours_poly[i].size()<10){
                cv::polylines(img,contours_poly[i],true,cv::Scalar(0,255,0));
                float radius;
                cv::Point2f center;
                cv::minEnclosingCircle( contours_poly[i], center, radius );
                cv::circle( img, center, 2, cv::Scalar(255,0,255), 2, 8, 0 );
                pontoM p;
                p.marcado = false;
                p.pt = center;
                pts.push_back(p);
            }
        }
    }

    std::vector<std::vector<cv::Point>> cones;
    int conCones = -1;

    for(size_t j = 0;j<pts.size();j++){
        pts[j].marcado = true;
        std::vector<cv::Point>pontoC;
        for(size_t k = j+1;k<pts.size()-1;k++){

            if(!pts[k].marcado){
                pontoC.push_back(pts[k].pt);
                conCones++;
                if(abs(pts[k].pt.x-pts[k+1].pt.x)<=25){
                    if(abs(pts[k].pt.y-pts[k+1].pt.y)<=25){
                     pontoC.push_back(pts[k].pt);
                     pts[k].marcado = true;
                    }
                }
            }
        }
        if(pontoC.size()>0) cones.push_back(pontoC);
    }
}


void MainWindow::on_btIV_1_clicked()
{

   /*Cone
    *  cv::Mat img = cv::imread("/home/lele/Projetos/Robos/Compticao/14725674.JPG");

    cv::Mat imgHSV,imgB,imgL;

    cv::cvtColor(img,imgHSV,CV_BGR2HSV);

    cv::inRange(imgHSV,cv::Scalar(0,160,160),
                cv::Scalar(30,255,255),
                imgL);
    cv::inRange(imgHSV,cv::Scalar(0,0,170),
                cv::Scalar(255,80,255),
                imgB);

    cv::erode(imgL,imgL,cv::Mat(),cv::Point(-1,-1),1);
    cv::imshow("imgL",imgL);
    cv::erode(imgB,imgB,cv::Mat(),cv::Point(-1,-1),1);
    cv::imshow("imgB",imgB);
    cv::Mat imgT = imgB+imgL;
    cv::erode(imgT,imgT,cv::Mat(),cv::Point(-1,-1),1);

    cv::dilate(imgL,imgL,cv::Mat(),cv::Point(-1,-1),2);
    cv::blur(imgT,imgT,cv::Size(3,3));

    cv::dilate(imgT,imgT,cv::Mat(),cv::Point(-1,-1),2);
    cv::blur(imgT,imgT,cv::Size(3,3));

    cv::dilate(imgT,imgT,cv::Mat(),cv::Point(-1,-1),2);
    cv::blur(imgT,imgT,cv::Size(3,3));
    cv::imshow("imgT",imgT);

    contornosCor(imgL,img);
    contornosCor(imgB,img);



    cv::imshow("img",img);

    cv::waitKey();

    cv::destroyAllWindows();*/


}

void MainWindow::on_btnAcharTab_clicked()
{

    cv::VideoCapture capP;
    cv::Mat imgC;
    if(ui->checkBoxEndImg->isChecked()){
        imgC = cv::imread(ui->edtEndImg->text().toStdString());
    }else{
        cv::VideoCapture cap (ui->spinBoxRaz->value());
        capP = cap;
    }

    cv::Mat cinza = cv::Mat(imgC.cols, imgC.rows, CV_8SC1);
    cv::Mat pb = cv::Mat(imgC.cols, imgC.rows, CV_8SC1);
    //cv::Mat hsv = cv::Mat(imgC.cols, imgC.rows, CV_8SC3);
    //cv::Mat pbHSV = cv::Mat(imgC.cols, imgC.rows, CV_8SC1);
    //cv::cvtColor(img,hsv,CV_BGR2HSV);
    //    cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(255,255,48),pbHSV);
    //    cv::threshold(pbHSV, pb,1,255 , CV_THRESH_BINARY_INV);// 3 = 70 =72

    //cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(255,100,50),pbHSV);
    //cv::threshold(pbHSV,pb,10,255 , CV_THRESH_BINARY_INV);// 3 = 70 =72

    cv::cvtColor(imgC, cinza, CV_BGR2GRAY);
    //cv::threshold(cinza, pb,70,255 , CV_THRESH_BINARY);// 3 = 70 =72
    //cv::threshold(pbHSV,pbHSV,5,255 , CV_THRESH_BINARY_INV);// 3 = 70 =72
    cv::adaptiveThreshold(cinza,pb,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,45,17);
    int n = 1;
    for(int i =0;i<n;i++){

        CvSize patternSize = cvSize(4, 4);//CvSize(6, 8);cel 6, 9
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners( pb, patternSize, corners, CV_CALIB_CB_NORMALIZE_IMAGE);

        if(found){

            cv::drawChessboardCorners( imgC, patternSize, cv::Mat(corners), found );
        }

        cv::imshow("AcharTab",pb);

    }

    cv::imshow("Tabs",imgC);

}

void MainWindow::on_btnSisLi_clicked()
{

//    double pcV[4][1] = {30.63789442467015, -495.9068416239098, 3592.534846045204,1};
//    double prV[4][1] = {2357.3,621.83,1126.85,1};
//    cv::Mat pc  = cv::Mat(4,1,CV_64F,pcV);
//    cv::Mat pr  = cv::Mat(4,1,CV_64F,prV);
//    //    cv::Mat mtCR  = cv::Mat(3,3,CV_64F);

//    //    mtCR = pc*pr.inv();

//    //    std::cout<<"Matriz de Transformação: "<<mtCR<<std::endl;

//    double A[4][4] = {{1,0,0,  2813.93},
//                      {0,0,1, -3350.86},
//                      {0,-1,0, 904.75},
//                      {0,0,0 , 1    }};

//    cv::Mat matA = cv::Mat(4,4,CV_64F,A);
//    //pc = matA.inv()*pr;

//    std::cout<<"Ponto Coordenada Câmera: "<<pc<<std::endl;

//    pc  = cv::Mat(4,1,CV_64F,pcV);
//    pr = matA*pc;



}

void MainWindow::on_btnOpenGL_clicked()
{

}

void MainWindow::on_btnFlange_clicked()
{


}
