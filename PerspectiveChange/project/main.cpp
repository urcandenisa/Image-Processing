//
//  main.cpp
//  project
//
//  Created by Denisa Urcan on 03/04/2020.
//  Copyright © 2020 Denisa Urcan. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <MacTypes.h>
#define MAX 255
#define FG 255
#define BG 0

 /* transform grayscale image into a binary image */
cv::Mat grayScale_to_Binary(cv::Mat src){

     /* get image's height and weight */
    int height = src.rows;
    int width = src.cols;
    
    //histograma
    int histogram[256] = {0};
          
    for(int i = 0; i < height; i++){
       for(int j = 0; j < width; j++){
           histogram[src.at<uchar>(i,j)]++;
       }
    }
   
    int Imin = 0, Imax = 0;
   
    for(int g = 0; g < 256; g++){
       if(histogram[g] > 0){
           Imin = g;
           break;
       }
    }
   
    for(int g = 255; g >=0; g--){
       if(histogram[g] > 0){
           Imax = g;
           break;
       }
    }
       
    float T = (Imin + Imax)/2.0;
    float Tnew = T;
    
    //eroarea admisa
    float epsilon = 0.1;
   
    int N1 = 0, N2 = 0;
    int UG1 = 0, UG2 = 0;
   
    do{
       Tnew = T;
       
       for(int g = Imin; g < T; g++){
           N1 += histogram[g];
           UG1 += g * histogram[g];
       }
       UG1/=N1;
       
       for(int g = T + 1; g < Imax; g++){
           N2 += histogram[g];
           UG2 += g * histogram[g];
       }
       UG2/=N2;
       
       Tnew = (UG1 + UG2)/2;
           
    }while(abs(Tnew - T) < epsilon);
   
    //binarization where threshold = T
   
    cv::Mat dst = cv::Mat(height, width, CV_8UC1);
   
    for(int i = 0; i < height; i++){
       for(int j = 0; j < width; j++){
           if(src.at<uchar>(i,j) < T){
               dst.at<uchar>(i,j) = 0;
           }else{
               dst.at<uchar>(i,j) = 255;
           }
       }
    }

    return dst;
}

 /* get paper's corners */
std::vector<cv::Point2f> getPaperCorners(cv::Mat image){
    
    std::vector<cv::Point2f> paperCorners;
    
    cv::goodFeaturesToTrack(image, paperCorners, 4, 0.01, 120); // 100 pt cele la care nu e resize!!!
    
    //sort corners - i chose the method in which i compute the mass center and compare the coordinates of corners with the coordinates of mass center
    
    cv::Point2f massCenter(0, 0);
    for(int i = 0; i < paperCorners.size(); i++){
        massCenter += paperCorners[i];
    }
    
    massCenter *= (1.0 / paperCorners.size());
    
    cv::Point2f tl, tr, bl, br; //top left, right, bottom left and bottom right
    
    std::vector<cv::Point2f> top, bottom;
    for(int i = 0; i < paperCorners.size(); i++){
        if(paperCorners[i].y < massCenter.y){
            //it is a top
            top.push_back(paperCorners[i]);
        }else{
            bottom.push_back(paperCorners[i]);
        }
    }
    
    if(top[0].x > top[1].x){
        tl = top[1];
        tr = top[0];
    }
    else{
        tl = top[0];
        tr = top[1];
    }
    
    if(bottom[0].x > bottom[1].x){
        bl = bottom[1];
        br = bottom[0];
    }
    else{
        bl = bottom[0];
        br = bottom[1];
    }
    paperCorners.clear();
    //sorting done, pushing the corners back in the list in a order

    paperCorners.push_back(tl);
    paperCorners.push_back(tr);
    paperCorners.push_back(br);
    paperCorners.push_back(bl);
    return paperCorners;
}

 /*  get image corners */
std::vector<cv::Point2f> getImageCorners(cv::Mat image){
    
    std::vector<cv::Point2f> imageCorners;
    
    int height = image.rows;
    int width = image.cols;
    
    imageCorners.push_back(cv::Point2f(0,0));
    imageCorners.push_back(cv::Point2f(width, 0));
    imageCorners.push_back(cv::Point2f(width, height));
    imageCorners.push_back(cv::Point2f(0, height));
    
    return imageCorners;
}

cv::Mat transformationUsingPredefinedFunctions(cv::Mat image, std::vector<cv::Point2f> paperCorners){
 
    int height = image.rows;
    int width = image.cols;
    
    std::vector<cv::Point2f> imageCorners = getImageCorners(image);
     /* compute the homography matrix using a predefined function */
    cv::Mat homography = cv::findHomography(paperCorners, imageCorners);
   
     /* warp perspective  */
    cv::Mat perspectiveTransformation(height, width, CV_8UC1);
    cv::warpPerspective(image, perspectiveTransformation, homography, perspectiveTransformation.size());
     /* display the images -> used for comparation */
    
    return perspectiveTransformation;
    
}

cv::Mat transformation(cv::Mat image, std::vector<cv::Point2f> paperCorners){

    std::vector<cv::Point2f> imageCorners = getImageCorners(image);
    
    int height = image.rows;
    int width = image.cols;

    /* computing the homography */
    
    /* for each pair of correspondent points (pi, pi') we will have a 2x9 matrix, where pi = paperCorners[i], pi' = imageCorners[i] */
    
    float firstCorrespondence[] = { -paperCorners[0].x, -paperCorners[0].y, -1, 0, 0, 0, paperCorners[0].x * imageCorners[0].x, paperCorners[0].y * imageCorners[0].x, imageCorners[0].x,  0, 0, 0, -paperCorners[0].x, -paperCorners[0].y, -1, paperCorners[0].x * imageCorners[0].y, paperCorners[0].y*imageCorners[0].y, imageCorners[0].y };
    
    cv::Mat firstMatrix(2, 9, CV_32F, firstCorrespondence);
    
    float secondCorrespondence[] = { -paperCorners[1].x, -paperCorners[1].y, -1, 0, 0, 0, paperCorners[1].x * imageCorners[1].x, paperCorners[1].y * imageCorners[1].x, imageCorners[1].x,  0, 0, 0, -paperCorners[1].x, -paperCorners[1].y, -1, paperCorners[1].x * imageCorners[1].y, paperCorners[1].y*imageCorners[1].y, imageCorners[1].y };
    
    cv::Mat secondMatrix(2, 9, CV_32F, secondCorrespondence);
    
    float thirdCorrespondence[] = { -paperCorners[2].x, -paperCorners[2].y, -1, 0, 0, 0, paperCorners[2].x * imageCorners[2].x, paperCorners[2].y * imageCorners[2].x, imageCorners[2].x,  0, 0, 0, -paperCorners[2].x, -paperCorners[2].y, -1, paperCorners[2].x * imageCorners[2].y, paperCorners[2].y*imageCorners[2].y, imageCorners[2].y };
    
    cv::Mat thirdMatrix(2, 9, CV_32F, thirdCorrespondence);
    
    float fourthCorrespondence[] = { -paperCorners[3].x, -paperCorners[3].y, -1, 0, 0, 0, paperCorners[3].x * imageCorners[3].x, paperCorners[3].y * imageCorners[3].x, imageCorners[3].x,  0, 0, 0, -paperCorners[3].x, -paperCorners[3].y, -1, paperCorners[3].x * imageCorners[3].y, paperCorners[3].y*imageCorners[3].y, imageCorners[3].y };
    
    cv::Mat fourthMatrix(2, 9, CV_32F, fourthCorrespondence);
    
    float fifthCorrespondence[] = {0, 0, 0, 0, 0, 0, 0, 0, 1};
    
    cv::Mat fifthMatrix(1, 9, CV_32F, fifthCorrespondence);
    /* in the end, we will have a 8x9 correspondence matrix (because we have 4 corespondence points, each one with a 2*9 matrix defined */
    cv::Mat correspondence(9, 9, CV_32F);
    
    firstMatrix.row(0).copyTo(correspondence.row(0));
    firstMatrix.row(1).copyTo(correspondence.row(1));
    
    secondMatrix.row(0).copyTo(correspondence.row(2));
    secondMatrix.row(1).copyTo(correspondence.row(3));
                               
    thirdMatrix.row(0).copyTo(correspondence.row(4));
    thirdMatrix.row(1).copyTo(correspondence.row(5));
                               
    fourthMatrix.row(0).copyTo(correspondence.row(6));
    fourthMatrix.row(1).copyTo(correspondence.row(7));
   
    fifthMatrix.copyTo(correspondence.row(8));

    /* from the correspondence matrix we have to solve the sistem correspondence * homographic = 0, resulting the homographic matrix*/
    /* we are using SVD (single value decomposition) to find the solution of the system */
    cv::SVD systemSolution = cv::SVD(correspondence, cv::SVD::FULL_UV);

    /* the homographic matrix should be the last column of the singular vector V in the system solution */
    cv::Mat lastSingularVector = systemSolution.vt.t().col(8).t();

    /* transform from 1x9 into 3x3 matrix which will be the homography matrix */
    cv::Mat homography = lastSingularVector.reshape(0, 3).inv();
   
    /* this is the perspective transformation, or my implementation of wrap perspective */
    cv::Mat perspectiveTransformation = cv::Mat::zeros(height, width, CV_8UC1);

    /* go through each pixel of image and apply the transformation */
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            cv::Mat A = cv::Mat(3, 1, homography.type());
            A.at<float>(0) = j;
            A.at<float>(1) = i;
            A.at<float>(2) = 1;
            
            cv::Mat tempMatrix = homography * A;
            cv::Point2f tempPoint(tempMatrix.at<float>(0)/tempMatrix.at<float>(2), tempMatrix.at<float>(1)/tempMatrix.at<float>(2));
            //if it goes out of image, is 0 -- see the matrix declaration
            if(floor(tempPoint.x) >= 0 && floor(tempPoint.y) >= 0 && floor(tempPoint.y) < height && floor(tempPoint.x) < width){
                perspectiveTransformation.at<uchar>(i, j) = image.at<uchar>(floor(tempPoint.y), floor(tempPoint.x));
            }
        }
    }
        
    return perspectiveTransformation;
}

void quickSort(uchar *array, int left, int right){
    int pivot = array[(left+right)/2];
    int i = left;
    int j = right;
    
    while(i <= j){
        while(array[i] < pivot){
            i++;
        }
        while(array[j] > pivot){
            j--;
        }
        if(i <= j){
            uchar aux = array[i];
            array[i] = array[j];
            array[j] = aux;
            i++;
            j--;
        }
    }
    
    if(left < j){
        quickSort(array, left, j);
    }
    if(i < right){
        quickSort(array, i, right);
    }
}

cv::Mat medianFilter(cv::Mat src){
    
    int height = src.rows;
    int width = src.cols;
    
    cv::Mat dst = cv::Mat::zeros(height, width, CV_8UC1);
    
    int w = 3, d;
    d = w/2;
    
    uchar *L = (uchar*)malloc(sizeof(uchar)*w*w);
    
    for(int i = d; i < height - d; i++){
        for(int j = d; j < width - d; j++){
            int index = 0;
            for(int m = -d; m <= d; m++){
                for(int n = -d; n <= d; n++){
                    L[index] = src.at<uchar>(i + m, j + n);
                    index++;
                }
            }
            quickSort(L, 0, index - 1);
            dst.at<uchar>(i,j) = L[(w*w)/2];
        }
    }
    
    return dst;
}

cv::Mat dilatationN(int n, cv::Mat src){
      
    int height = src.rows;
    int width = src.cols;
      
    int di[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
    int dj[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };
    
    cv::Mat dst = src.clone();
    cv::Mat temp = dst.clone();
    
    for(int index = 0; index < n; index++){
        
        for(int i = 1; i < height - 2; i++){
            for(int j = 1; j < width - 2; j++){
                if(dst.at<uchar>(i,j) == BG){
                    for(int k = 0; k < 8; k++){
                        int x = i + di[k];
                        int y = j + dj[k];
                        temp.at<uchar>(x,y) = BG;
                    }
                }
            }
        }
        
        dst = temp.clone();
    }
    
    return dst;
}

cv::Mat erosionN(int n, cv::Mat src){
      
    int height = src.rows;
    int width = src.cols;
      
    int di[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
    int dj[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };
    
    cv::Mat dst = src.clone();
    cv::Mat temp = dst.clone();
    
    for(int index = 0; index < n; index++){
        
        for(int i = 1; i < height - 2; i++){
              for(int j = 1; j < width - 2; j++){
                  if(dst.at<uchar>(i,j) == BG){
                      for(int k = 0; k < 8; k++){
                          int x = i + di[k];
                          int y = j + dj[k];
                          if(dst.at<uchar>(x,y) == FG){
                              temp.at<uchar>(i,j) = FG;
                              break;
                          }
                      }
                  }
              }
        }
        
        dst = temp.clone();
    }
    
    return dst;
}

cv::Mat Gray2RGB(cv::Mat src)
{

    int height = src.rows;
    int width = src.cols;

    cv::Mat dst = cv::Mat(height, width, CV_8UC3);

    for (int i = 0; i<height; i++)
    {
        for (int j = 0; j<width; j++)
        {
            uchar val = src.at<uchar>(i, j);
            dst.at<cv::Vec3b>(i, j)[0] = val;
            dst.at<cv::Vec3b>(i, j)[1] = val;
            dst.at<cv::Vec3b>(i, j)[2] = val;
        }
    }
    
    return dst;
}


cv::Mat conclusions(cv::Mat image1, cv::Mat image2){
    
    int height = image1.rows;
    int width = image2.cols;
    
    cv::Mat_<cv::Vec3b> dst = cv::Mat(image1.size(), CV_8UC3);
    dst = Gray2RGB(image1);
    
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            if(image1.at<uchar>(i, j) == image2.at<uchar>(i,j)){
                dst(i,j)[0] = 209;
                dst(i,j)[1] = 224;
                dst(i,j)[2] = 64;
            }
        }
    }
    return dst;
    
}


int main(int argc, const char * argv[]) {
    // insert code here...
   
    /* read the grayscale image */
    char fname[MAX] = "/Users/denisaurcan/Documents/PI/project/658346AA-BEED-4898-A7B9-BC7F3BD34E08_1_105_c.jpeg";
    cv::Mat src = cv::imread(fname, CV_LOAD_IMAGE_GRAYSCALE);
    
    imshow("original", src);
    
    cv::Mat temp = grayScale_to_Binary(src);
    //imshow("binarized", temp);

    cv::Mat image = medianFilter(temp);
    //imshow("noise removed", image);

    cv::Mat removeWr = erosionN(3, image);
    //imshow("eroziune", removeWr);
    cv::Mat redoImage  = dilatationN(3, removeWr);
    //imshow("dilatation", redoImage);
    //imshow("redoImage", redoImage);
    
    std::vector<cv::Point2f> paperCorners = getPaperCorners(redoImage);
    
    cv::Mat transformationUsingFunctions = transformationUsingPredefinedFunctions(src, paperCorners);

    cv::Mat myTransformation = transformation(src, paperCorners);
    
    imshow("transformation using predefines", transformationUsingFunctions);
    imshow("my transformation", myTransformation);
    
    cv::Mat conclusion = conclusions(transformationUsingFunctions, myTransformation);
    
    
    //imshow("Conclusions", conclusion);
    
    cv::waitKey();
    cv::destroyAllWindows();
    return 0;
}
