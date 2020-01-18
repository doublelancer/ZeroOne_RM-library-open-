//根据以上代码将其在opencv3.0.0中编写
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "iostream"
#include <chrono>
//#include "omp.h"
using namespace cv;
using namespace std;

#define T_ANGLE_THRE 10
#define T_SIZE_THRE 5

void brightAdjust(Mat src, Mat dst, double dContrast, double dBright); //亮度调节函数
void getDiffImage(Mat src1, Mat src2, Mat dst, int nThre); //二值化
vector<RotatedRect> armorDetect(vector<RotatedRect> vEllipse); //检测装甲
void drawBox(RotatedRect box, Mat img); //标记装甲

#define debug

int main()
{
   // VideoCapture cap0("//home//doublelancer//github//RM19windmillDemo-master2//robo.mp4");
    //VideoCapture cap0;
   // cap0.open("//home//doublelancer//github//RM19windmillDemo-master2//robo.mp4");
    //cap0.open("//home//doublelancer//github//RM19windmillDemo-master2//RedCar.avi");
    //cap0.open("//home//doublelancer//github//RM19windmillDemo-master2//red1.avi");
  VideoCapture cap0(1);
    cap0.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap0.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    cap0.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));  /**/
    Mat frame0;
    Size imgSize;
    RotatedRect s;   //定义旋转矩形
    RotatedRect Rect;
    vector<RotatedRect> vEllipse; //定以旋转矩形的向量，用于存储发现的目标区域
    vector<RotatedRect> vRlt;
    vector<RotatedRect> vArmor;
    vector<RotatedRect> vContour;
    bool bFlag = false;

    vector<vector<Point> > contour;

    cap0 >> frame0;
    imgSize = frame0.size();

    Mat rawImg = Mat(imgSize, CV_8UC3);

    Mat grayImage = Mat(imgSize, CV_8UC1);
    Mat rImage = Mat(imgSize, CV_8UC1);
    Mat gImage = Mat(imgSize, CV_8UC1);
    Mat bImage = Mat(imgSize, CV_8UC1);
    Mat binary = Mat(imgSize, CV_8UC1);
    Mat rlt = Mat(imgSize, CV_8UC1);
    namedWindow("Raw");
    while (1)
    {

    cap0 >> frame0;
    auto t1 = chrono::high_resolution_clock::now();

    imshow("orgin", frame0);
        if (1)
        {

            brightAdjust(frame0, rawImg, 1, -120);  //每个像素每个通道的值都减去120
#ifdef debug
            imshow("1", rawImg);
#endif
            Mat bgr[3];
            split(rawImg, bgr); //将三个通道的像素值分离
            bImage = bgr[0];
            gImage = bgr[1];
            rImage = bgr[2];

          //如果像素R值-G值大于25，则返回的二值图像的值为255，否则为0
           // getDiffImage(rImage, gImage, binary, 25);      //red  red red rrrrrrrrrrrrrrrrrrrrred
            binary= rImage-bImage;
            blur(binary,binary,Size(1,3));
            threshold(binary,binary,25,255,THRESH_BINARY);//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>parameter>>>>>>>>

#ifdef debug
                    imshow("binary", binary);
 #endif
            Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1));
              dilate(binary, grayImage, kernel, Point(-1,-1), 3);   //图像膨胀 ????????????????????????????????????????????     2/1
           // dilate(binary, grayImage, Mat(), Point(-1,-1), 3);   //图像膨胀   ????????????????????????????????????????????????  choose according effects
#ifdef debug
            imshow("grayImage", grayImage);
#endif
            erode(grayImage, rlt, kernel, Point(-1,-1), 1);  //图像腐蚀，先膨胀在腐蚀属于闭运算
            //erode(grayImage, rlt, Mat(), Point(-1,-1), 1);  //图像腐蚀，先膨胀在腐蚀属于闭运算
           // erode(rlt, rlt, Mat(), Point(-1,-1), 1);  //图像腐蚀
#ifdef debug
            imshow("erode", rlt);
#endif
            findContours(rlt, contour, RETR_CCOMP , CHAIN_APPROX_SIMPLE); //在二值图像中寻找轮廓
           //
            for (int i=0; i<contour.size(); i++)
            {
                if (contour[i].size()> 8)  //判断当前轮廓是否大于10个像素点
                {
                    // 求轮廓面积
                    float Light_Contour_Area ;
                   Light_Contour_Area = contourArea(contour[i]);
                        // 去除较小轮廓&fitEllipse的限制条件
                   if (Light_Contour_Area < 20||Light_Contour_Area > 400 || contour[i].size() <= 5)  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>paramerter>>>>>>
                            continue;
                    bFlag = true;   //如果大于10个，则检测到目标区域
                  //拟合目标区域成为椭圆，返回一个旋转矩形（中心、角度、尺寸）
                    s = fitEllipse(contour[i]);

                    //s = brightAdjust(s);
                        if (s.angle > 12)     //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>paramerter>>>>>>
                            continue;
                        // 长宽比和轮廓面积比限制
                        if (s.size.width / s.size.height > 0.9
                            || Light_Contour_Area / s.size.area() <0.5)  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>paramerter>>>>>>
                            continue;
                        // 扩大灯柱的面积
                        s.size.height *= 1.1;
                        s.size.width *= 1.1;
                        vContour.push_back(s);
                /*
                    for (int nI = 0; nI < 5; nI++)
                    {
                        for (int nJ = 0; nJ < 5; nJ++)  //遍历以旋转矩形中心点为中心的5*5的像素块
                        {
                            if (s.center.y - 2 + nJ > 0 && s.center.y - 2 + nJ < 480 && s.center.x - 2 + nI > 0 && s.center.x - 2 + nI <  640)  //判断该像素是否在有效的位置
                            {
                                Vec3b v3b = frame0.at<Vec3b>((int)(s.center.y - 2 + nJ), (int)(s.center.x - 2 + nI)); //获取遍历点点像素值
                               //判断中心点是否接近白色
                                if (v3b[0] < 200 || v3b[1] < 200 || v3b[2] < 200)
                                    bFlag = false;        //如果中心不是白色，则不是目标区域
                            }
                        }
                    }
                    if (bFlag)
                    {
                        vEllipse.push_back(s); //将发现的目标保存
                    }
                    */
                }

            }
    for (int i=0; i<contour.size(); i++){
         for (int j=0; j<contour.size(); j++){
            //判断是否为相同灯条
            float Contour_angle = abs(vContour[i].angle - vContour[j].angle); //角度差
            if (Contour_angle >= 7)//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                continue;
            //长度差比率
            float Contour_Len1 = abs(vContour[i].size.height - vContour[j].size.height) / max(vContour[i].size.height, vContour[j].size.height);
            //宽度差比率
            float Contour_Len2 = abs(vContour[i].size.width - vContour[j].size.width) / max(vContour[i].size.width, vContour[j].size.width);
            if (Contour_Len1 > 0.35 || Contour_Len2 > 0.35)//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                continue;


            Rect.center.x = (vContour[i].center.x + vContour[j].center.x) / 2.; //x坐标
            Rect.center.y = (vContour[i].center.y + vContour[j].center.y) / 2.; //y坐标
            Rect.angle = (vContour[i].angle + vContour[j].angle) / 2.; //角度

            float nh, nw, yDiff, xDiff;
            nh = (vContour[i].size.height + vContour[j].size.height) / 2; //高度
            // 宽度
            nw = sqrt((vContour[i].center.x - vContour[j].center.x) * (vContour[i].center.x - vContour[j].center.x) + (vContour[i].center.y - vContour[j].center.y) * (vContour[i].center.y - vContour[j].center.y));

            float ratio = nw / nh; //匹配到的装甲板的长宽比
            xDiff = abs(vContour[i].center.x - vContour[j].center.x) / nh; //x差比率
            yDiff = abs(vContour[i].center.y - vContour[j].center.y) / nh; //y差比率

            if (ratio < 1.0 || ratio >5.0 || xDiff < 0.5 || yDiff > 2.0)//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                continue;
            Rect.size.height = nh;
            Rect.size.width = nw;
            vRlt.push_back(Rect);
  }
 }

        //调用子程序，在输入的LED所在旋转矩形的vector中找出装甲的位置，并包装成旋转矩形，存入vector并返回
          //  vRlt = armorDetect(vContour);

            for (unsigned int nI = 0; nI < vRlt.size(); nI++) //在当前图像中标出装甲的位置
                drawBox(vRlt[nI], frame0);
                imshow("Raw", frame0);

            if (waitKey(50) == 27)
            {
                break;
            }
            vEllipse.clear();
            vRlt.clear();
            vArmor.clear();
            vContour.clear();
        }
        else
        {
cout<<"1111111";
            break;
        }
        auto t2 = chrono::high_resolution_clock::now();
        cout << "Total period: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
//        t1 = chrono::high_resolution_clock::now();
    }
    cap0.release();
    return 0;

}

void brightAdjust(Mat src, Mat dst, double dContrast, double dBright)
{
    int nVal;
    //omp_set_num_threads(8);
//#pragma omp parallel for

    for (int nI = 0; nI<src.rows; nI++)
    {
        Vec3b* p1 = src.ptr<Vec3b>(nI);
        Vec3b* p2 = dst.ptr<Vec3b>(nI);
        for (int nJ = 0; nJ <src.cols; nJ++)
        {
            for (int nK = 0; nK < 3; nK++)
            {
               //每个像素的每个通道的值都进行线性变换
                nVal = (int)(dContrast * p1[nJ][nK] + dBright);
                if (nVal < 0)
                    nVal = 0;
                if (nVal > 255)
                    nVal = 255;
                p2[nJ][nK] = nVal;
            }
        }
    }
}

void getDiffImage(Mat src1, Mat src2, Mat dst, int nThre)
{
  //  omp_set_num_threads(8);
//#pragma omp parallel for

    for (int nI = 0; nI<src1.rows; nI++)
    {
        uchar* pchar1 = src1.ptr<uchar>(nI);
        uchar* pchar2 = src2.ptr<uchar>(nI);
        uchar* pchar3 = dst.ptr<uchar>(nI);
        for (int nJ = 0; nJ <src1.cols; nJ++)
        {
            if (pchar1[nJ] - pchar2[nJ]> nThre) //
            {
                pchar3[nJ] = 255;
            }
            else
            {
                pchar3[nJ] = 0;
            }
        }
    }
}

vector<RotatedRect> armorDetect(vector<RotatedRect> vEllipse)
{
    vector<RotatedRect> vRlt;
    RotatedRect armor; //定义装甲区域的旋转矩形
    int nL, nW;
    double dAngle;
    vRlt.clear();
    if (vEllipse.size() < 2) //如果检测到的旋转矩形个数小于2，则直接返回
        return vRlt;
    for (unsigned int nI = 0; nI < vEllipse.size() - 1; nI++) //求任意两个旋转矩形的夹角
    {
        for (unsigned int nJ = nI + 1; nJ < vEllipse.size(); nJ++)
        {
            dAngle = abs(vEllipse[nI].angle - vEllipse[nJ].angle);
            while (dAngle > 180)
                dAngle -= 180;
          //判断这两个旋转矩形是否是一个装甲的两个LED等条
            if ((dAngle < T_ANGLE_THRE || 180 - dAngle < T_ANGLE_THRE) && abs(vEllipse[nI].size.height - vEllipse[nJ].size.height) < (vEllipse[nI].size.height + vEllipse[nJ].size.height) / T_SIZE_THRE && abs(vEllipse[nI].size.width - vEllipse[nJ].size.width) < (vEllipse[nI].size.width + vEllipse[nJ].size.width) / T_SIZE_THRE)
            {
                armor.center.x = (vEllipse[nI].center.x + vEllipse[nJ].center.x) / 2; //装甲中心的x坐标
                armor.center.y = (vEllipse[nI].center.y + vEllipse[nJ].center.y) / 2; //装甲中心的y坐标
                armor.angle = (vEllipse[nI].angle + vEllipse[nJ].angle) / 2;   //装甲所在旋转矩形的旋转角度
                if (180 - dAngle < T_ANGLE_THRE)
                    armor.angle += 90;
                nL = (vEllipse[nI].size.height + vEllipse[nJ].size.height) / 2; //装甲的高度
                nW = sqrt((vEllipse[nI].center.x - vEllipse[nJ].center.x) * (vEllipse[nI].center.x - vEllipse[nJ].center.x) + (vEllipse[nI].center.y - vEllipse[nJ].center.y) * (vEllipse[nI].center.y - vEllipse[nJ].center.y)); //装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
                if (nL < nW)
                {
                    armor.size.height = nL;
                    armor.size.width = nW;
                }
                else
                {
                    armor.size.height = nW;
                    armor.size.width = nL;
                }
                vRlt.push_back(armor); //将找出的装甲的旋转矩形保存到vector
            }
        }
    }
    return vRlt;
}

void drawBox(RotatedRect box, Mat img)
{
    Point2f pt[4];
    int i;
    for (i = 0; i<4; i++)
    {
        pt[i].x = 0;
        pt[i].y = 0;
    }
    box.points(pt); //计算二维盒子顶点
    line(img, pt[0], pt[1], CV_RGB(0, 0, 255), 2, 8, 0);
    line(img, pt[1], pt[2], CV_RGB(0, 0, 255), 2, 8, 0);
    line(img, pt[2], pt[3], CV_RGB(0, 0, 255), 2, 8, 0);
    line(img, pt[3], pt[0], CV_RGB(0, 0, 255), 2, 8, 0);
}
