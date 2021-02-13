// Librerías que contienen funciones estándar de C++
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <cstdio>
#include <string.h>

// Librería que contiene funciones matemáticas
#include <cmath>

// Se pueden cargar todas las Librerías incluyendo
//#include <opencv2/opencv.hpp>

// Contiene las definiciones fundamentales de las matrices e imágenes 
#include <opencv2/core/core.hpp> 
// Procesamiento de imágenes
#include <opencv2/imgproc/imgproc.hpp>
// Códecs de imágenes
#include <opencv2/imgcodecs/imgcodecs.hpp>
// Manejo de ventanas y eventos de ratón, teclado, etc.
#include <opencv2/highgui/highgui.hpp>
// Lectura de video
#include <opencv2/video/video.hpp>
// Escritura de video
#include <opencv2/videoio/videoio.hpp>

using namespace std;
using namespace cv; // Espacio de nombres de OpenCV (Contiene funciones y definiciones de varios elementos de OpenCV)

int umbral = 255;
bool reco = false;

float const PI=3.1415926536;

int minH = 0, minS = 0, minV = 0;
int maxH = 60, maxS = 255, maxV = 255;
int minY = 0, minCr = 100, minCb = 0;
int maxY = 255, maxCr = 255, maxCb = 120;

void trackBarEventHSV(int v, void *p){
}
void trackBarEventYCrCb(int v, void *p){
}

double distancia(double m1[7], double m2[7]){
    double suma = 0.0;
    for(int i=0;i<7;i++){
        suma+=(m1[i]-m2[i])*(m1[i]-m2[i]);
    }
    suma = sqrt(suma);
    return suma;
}

Mat BordesLaplace(Mat imagen){
    Mat bordeLaplace, bordeLaplaceAbs;
    Laplacian(imagen, bordeLaplace, CV_16S, 3);
    convertScaleAbs(bordeLaplace, bordeLaplaceAbs);
    return bordeLaplaceAbs;
}

void guardarGesto(double momentos[7]){
    fstream archivo("database.txt", ios::in | ios::out | ios::app);
    archivo.seekg(0);
    archivo << momentos[0] <<";"<< momentos[1] <<";"<<momentos[2] <<";"<<momentos[3] <<";"<<momentos[4] <<";"<<momentos[5] <<";"<<momentos[6] <<";"<<"\n";
    archivo.close();
}

vector<double> split(String cadena) { 
    vector <double> mbase = {}; 
    char separador = ';'; 
    string sentencia = cadena; 
    for (size_t p = 0, q = 0; p != sentencia.npos; p = q) { 
        string cad = sentencia.substr(p + (p != 0), (q = sentencia.find(separador, p + 1)) - p - (p != 0)); 
        double res = atof(cad.c_str()); 
        mbase.push_back(res); 
    } 
    return mbase; 
} 

vector<double> buscarGesto(double momentos[7]){
    vector<double> baseDatos = {};
    ifstream archivo("database.txt");
    string str; 
    while (getline(archivo, str)){
        vector<double> line = split(str);
        if (line.size()==8){
            double linea[7] = {line[0],line[1],line[2],line[3],line[4],line[5],line[6]};
            baseDatos.push_back(distancia(momentos,linea));
        }
    }
    archivo.close();
    return baseDatos;
}

int main(int argc, char *argv[]){

    VideoCapture video(0);
    
    if(video.isOpened()){
        Mat frame;
        Mat area;
        Mat imgHSV;
        Mat imgHSV2;
        Mat bordesLaplace;
        Mat bordesLaplaceAbs;

        vector<double> res;

        vector<vector<Point> > puntosContorno; // Puntos que conforman cada uno de los contornos de la imagen
        //
        Mat roi; // Region of Interes, Zona de Interés
        Mat roi2;
        //
        
        Mat objeto;
        Mat objeto2;

        Mat ecualizada;
        Mat apertura;
        Mat cierre;

        Mat elemento = getStructuringElement(MORPH_CROSS, Size(3,3),Point(-1,-1));

        namedWindow("Video", WINDOW_AUTOSIZE);
        namedWindow("HSV", WINDOW_AUTOSIZE);
        namedWindow("ROI", WINDOW_AUTOSIZE);
        
        namedWindow("Objeto", WINDOW_AUTOSIZE);
        namedWindow("Contornos", WINDOW_AUTOSIZE);
        
        createTrackbar("H-Min", "Video", &minH, 180, trackBarEventHSV, NULL);
        createTrackbar("S-Min", "Video", &minS, 255, trackBarEventHSV, NULL);
        createTrackbar("V-Min", "Video", &minV, 255, trackBarEventHSV, NULL);
        
        createTrackbar("H-Max", "Video", &maxH, 180, trackBarEventHSV, NULL);
        createTrackbar("S-Max", "Video", &maxS, 255, trackBarEventHSV, NULL);
        createTrackbar("V-Max", "Video", &maxV, 255, trackBarEventHSV, NULL);

        int pixel = 0;
        
        double huMoments[7]; // Momentos invariantes a rotación, traslación y escala que se van a calcular
        Moments momentos; // Momentos generales
        double baseDatos[7] = {0.265905,0.0127865,0.000257505,0.00264699,1.39243e-06,0.000254032,-1.68432e-06};
        double cx = 0;
        double cy = 0;
        
        while(3==3){
            video >> frame;

            Mat frame2 = frame.clone();
            
            Rect r(0,0,int(frame.cols/2),frame.rows);

            rectangle(frame,r,Scalar(0,0,255),2);

            resize(frame,frame,Size(),0.5,0.5);
            resize(frame2,frame2,Size(),0.5,0.5);
            
            cvtColor(frame2, imgHSV, COLOR_BGR2HSV);
            inRange(imgHSV, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), roi);

            //if(objeto.empty())
            {
                objeto = Mat(Size(frame2.cols, frame2.rows), CV_8UC3, Scalar(255,255,255));
                roi2 = Mat(Size(frame2.cols, frame2.rows), CV_8UC1, Scalar(0));
            }

            for(int i=0;i<roi.rows;i++){
                for(int j=0;j<roi.cols;j++){
                    pixel = roi.at<uchar>(i,j);
                    if(pixel>0){
                        objeto.at<Vec3b>(i,j) = frame.at<Vec3b>(i,j);
                    }
                    if (j < roi.cols/2){
                        roi2.at<uchar>(i,j) = pixel; 
                    }
                }
            }

            equalizeHist(roi2, ecualizada);
            
            morphologyEx(ecualizada, apertura, MORPH_OPEN, elemento);
            morphologyEx(apertura, cierre,MORPH_CLOSE, elemento);

            findContours(BordesLaplace(cierre),puntosContorno,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

            //cout << "# de Contornos Detectados Figura1: " << puntosContorno.size() << endl;

            drawContours(frame2,puntosContorno,-1,Scalar(0,255,0),1,LINE_AA);

            momentos = moments(roi2, true);
            HuMoments(momentos, huMoments);

            vector<vector<Point> > contours;
            findContours(BordesLaplace(ecualizada), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            vector<vector<Point>> hull(contours.size());
            vector<vector<int>> hullInt(contours.size()); // Indices to contour points
            vector<vector<Vec4i>> defects(contours.size());

            for (int i= 0; i < contours.size(); i++){
                convexHull(contours[i], hullInt[i],false,false);
                convexHull(contours[i], hull[i]); 
                drawContours(frame2,contours,i,Scalar(0,255,0),1,LINE_AA);
                drawContours(frame2,hull,i,Scalar(255,0,0),1,LINE_AA);

                for(size_t k =0; k < hullInt[i].size(); k++){
                    if (hullInt[i].size() > 0)
                    {
                        hullInt[i][k] = (int)((hullInt[i].size() - k)-1);
                    }
                }

                if(hullInt[i].size() > 3 ){
                    convexityDefects(contours[i], hullInt[i], defects[i]);
                }
            }

            for (int i = 0; i < contours.size(); ++i){
                for(const Vec4i& v : defects[i]){
                    int inicio = v[0]; 
                    Point Inicio(contours[i][inicio]);
                    int fin = v[1]; 
                    Point Fin(contours[i][fin]);
                    int lejano = v[2]; 
                    Point Lejano(contours[i][lejano]);

                    double lf = norm(Lejano-Fin);
                    double li = norm(Lejano-Inicio);
                    double fs = norm(Inicio-Fin);
                    double angulo = acos((pow(lf,2)+pow(li,2)-pow(fs,2))/(2*lf*li));
                    angulo = angulo*(180/PI);
                    int iangulo = int(angulo);
                    if (fs > 20 && iangulo <90){
                        circle(frame2, Inicio, 4, Scalar(255, 0, 0), 2);
                        circle(frame2, Fin, 4, Scalar(0, 0, 255), 2);
                        circle(frame2, Lejano, 4, Scalar(0, 255, 255), 2);
                    }
                }
            }
            
            if (reco){
                vector<double> res=buscarGesto(huMoments);
                int minElementIndex = min_element(res.begin(),res.end()) - res.begin() +1;
                double min = *min_element(res.begin(), res.end());
                if (min < 0.01){
                    cx = momentos.m10/momentos.m00;
                    cy = momentos.m01/momentos.m00;
                    
                    circle(frame2, Point(cx,cy),7,Scalar(10,10,203),3);
                    string gesto = "Reconocido [Gesto " + to_string(minElementIndex) +"]";
                    putText(frame2,gesto, Point(25,25),FONT_HERSHEY_TRIPLEX,1,Scalar(0,255,255));
                }
            }

            imshow("Video", frame);
            imshow("HSV", imgHSV);
            imshow("ROI", roi);
            imshow("Objeto", objeto);
            imshow("Contornos",frame2);
            
            if(waitKey(10)==103)
                guardarGesto(huMoments);
            
            if (waitKey(23)==100){
                remove("database.txt");
                res.clear();
            }

            if (waitKey(23)==97){
                reco=true;
                cout << "Modo Reconocimiento" << endl;
            }

            if(waitKey(12)==115)
                reco=false;
            
            if(waitKey(23)==27)
                break;
        }
        
        destroyAllWindows();
    }

    return 0;
}