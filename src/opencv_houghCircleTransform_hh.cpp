/* Plantilla ejemplo 5
/*
/*   Transformacion circular de HPUGH
/*   from = https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html#hough-circle
/*
/*   OPENCV
/*   from = https://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html
/*   
/*   Ejemplo ROS
/*   from: https://github.com/epsilonorion/ros_tutorials/blob/master/opencv_tut/src/findCircle.cpp
*/

// Includes
#include <ros/ros.h>				//Importa libreria ROS

#include <stdio.h> 				//Obligatoria para usar la clase "vector"

#include <image_transport/image_transport.h>	//Clase para manipular imagenes
#include <cv_bridge/cv_bridge.h>		//Puente entre OpenCv y ROS para manejar OpenCv desde ROS
#include <sensor_msgs/image_encodings.h>	//Herramientas para manipular las imagenes
#include <opencv2/imgproc/imgproc.hpp>		//Libreria propia de opencv (Procesamiento de imagenes)
#include <opencv2/highgui/highgui.hpp>		//Libreria propia de opencv (Diseñar interfaces graficas (poner imagen en ventana))

/*#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //Asegurar los encabezados
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>*/

// Defines - General
#define    NODE_NAME       	"opencv_houghCircleTransform_hh"	//Renombre de nodo
#define    OPENCV_WINDOW1       "Original Image"			//Renombre de ventana para imagen original
#define    OPENCV_WINDOW2       "Image Filtrada"			//Renombre de ventana para imagen procesada


// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		//Imagen original de la camara (suscriptor)
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" //Imagen publica para ROS (Publicador).

//***CLASS: Image Conver (OpenCV)***
class ImageConverter
{
    private: 
    	// NodeHandle ROS
    	ros::NodeHandle nh_;	//Clase NodeHandle de la libreria de ROS

    	// Imagen usada 
    	image_transport::ImageTransport it_;	// Object it_ from image transport clase (utilizado para el procesamiento de imagenes)
    	image_transport::Subscriber topic1_sub__image_input;	//Imagen de la cámara (sin procesar). Formato ROS (Topic)
    	image_transport::Publisher topic1_pub__image_output;	//Imagen publica para ROS (procesada). Formato ROS (Topic)

    public:

	/* Constructor Method. 
	   TODO */
  	ImageConverter() : it_(nh_)	//Hereda elementos de it_ y atributo es de nh_
  	{
    	    // Topics declaration
       	    topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this); //Topic suscriptor
   	    			      	//(Nombre del topic, bufer de la comunicación, Callback(función o método), dentro de si mismo)			
		
	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1); //Topic publicador 
										   //(Nombre del topic, bufer de la comunicación)											

	    // Crea la GUI Windows (donde imprime las imagenes)
    	    cv::namedWindow(OPENCV_WINDOW1);
	    cv::namedWindow(OPENCV_WINDOW2);
  	}

	/* Desctructor Method */
  	~ImageConverter()
  	{
	    // Cierra la GUI Windows
    	    cv::destroyWindow(OPENCV_WINDOW1);	//Opencv destruye la ventana creada
	    cv::destroyWindow(OPENCV_WINDOW2);	//Opencv destruye la ventana creada
  	}

	/* Asociado a "TOPIC1_SUB__IMAGE_INPUT"  que obtiene la Imagen de la cámara (sin procesar) */
	void imageCb(const sensor_msgs::ImageConstPtr& msg) //msg es la Imagen obtenida de la cámara (sin procesar))
  							    //(MÉTODO CON TIPO DE MENSAJE IMAGECONSTPTR Y SE ALMACENA EN MSG)
	{
	    // Convertir ROS image (Topic) a OpenCV imagen (Ptr)	    
    	    cv_bridge::CvImagePtr cv_OriginalImage_ptr;	//OBJETO DE LA CLASE CvImagePtr QUE PERTENECE A LA LIBRERIA cv_bidge
    	    
	    //HACE EL INTENTO, SINO INFORMA A TRAVES DEL CATCH EL ERROR QUE EXISTIO
	    try
    	    {
      		cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
    				//METODO toCvCopy toma la imagen crudo de la camara y lo pega en otra parte con formato bgr de 8 bits
	    }
	    catch (cv_bridge::Exception& e)
    	    {
		// Imprime un error si se detecta
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	    }

	    /****************************/ 
	    /* Procesando imagen digital */
	    /****************************/
	   
    	    // Convertir datos a cv::Mat clase
	    cv::Mat cvMat_Image = cv_OriginalImage_ptr->image;	//Convierte los datos a la clase cv::Mat 

	    // Imagen original gris
	    cv::Mat cvMat_GrayImage;	//Crea una variable de la clase Mat para escala de grises
	    cv::cvtColor(cvMat_Image, cvMat_GrayImage, CV_BGR2GRAY); 
					//Convierte una imagen de un espacio de color a otro (imagen de entrada, imagen de salida, codigo de conversión)

	    // Reducir el ruido aplicando un filtro gaussiano, así evitamos la detección de falsos círculos
	    cv::Mat cvMat_GrayImage_filtered;
	    cv::GaussianBlur(cvMat_GrayImage, cvMat_GrayImage_filtered, cv::Size(9, 9), 2, 2);
		    
	    // Aplicar la transformada de Hough para encontrar los círculos
	    std::vector<cv::Vec3f> circles;
	    cv::HoughCircles(cvMat_GrayImage_filtered, circles, CV_HOUGH_GRADIENT, 2, 20, 100, 155, 0, 0 );
	    /* Ejemplo: HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
	       Parametros:
	           src_gray: Agrega imagen escala de grices
    	           circles: Un vector que almacena conjuntos de 3 valores: x_{c}, y_{c}, r para cada círculo detectado.
	     	   CV_HOUGH_GRADIENT: Definir el método de detección. Actualmente es el único disponible en OpenCV
	           dp = 1: La relación inversa de la resolución TODO
	           min_dist = src_gray.rows/8: Distancia mínima entre los centros detectados TODO
	     	   param_1 = Umbral superior para el detector de bordes Canny interno TODO
	           param_2 = Umbral para la detección del centro. TODO
    	           min_radius = 0: Radio mínimo a detectar. Si se desconoce, poner cero por defecto.
    	          max_radius = 0: Radio máximo a detectar. Si se desconoce, poner cero por defecto */
	  
	    // Circulos detectados For hasta la tamaño
	    for(size_t i = 0; i < circles.size(); i++) //size_t es una variable que puede almacenar el tamaño máximo de un objeto teóricamente posible, en este caso la longitud de "i" (tamaño desconocido)  
	    {
		// Dibuja los circulos en la imagen original
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
 		int radius = cvRound(circles[i][2]);
   		circle(cvMat_Image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 ); // centro del círculo (con radio=3, color verde)
   		circle(cvMat_Image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 ); // contorno del círculo (con radio real, color rojo)

		/* circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
		   Parametros
		       img – Imagen donde se dibuja el círculo.
		       center – Centro del círculo.
    		       radius – Radio del círculo.
    		       color – Color del círculo.
    		       thickness – Grosor del contorno del círculo. Si el grosor es negativo significa que se dibujará un círculo relleno.
    		       lineType – Tipo de límite del círculo. Véase la descripción de line(). Por defecto 8
    		       shift – Número de bits fraccionarios en las coordenadas del centro y en el valor del radio. Sordos 0*/

		// imprime en el terminal 
		ROS_INFO("Circle detected #%d / %d: ", int(i)+1, (int)circles.size());
		ROS_INFO("    x=%d, y=%d, r=%d: ", cvRound(circles[i][0]), cvRound(circles[i][1]), cvRound(circles[i][2]));
      	    }

	    /*********************************/ 
	    /* FIN: Procesando imagen digital */
	    /*********************************/

    	    // Arreglo GUI Window1 - Imagen Original
   	    cv::imshow(OPENCV_WINDOW1, cvMat_Image);
    	    cv::waitKey(3);

	    // Arreglo GUI Window2 - Filtro G aplicado
   	    cv::imshow(OPENCV_WINDOW2, cvMat_GrayImage_filtered);
	    cv::waitKey(3);
	}
};

//***Main***
int main(int argc, char** argv)
{
    // Iniciar ROS 
    ros::init(argc, argv, NODE_NAME);	//Inicializar nodo
  
    // Iniciar objeto de la clase ImageConverter, definido antes
    ImageConverter ic;	//Creación de objeto de la clase ImageConverter
	
     // While true. Obteniendo datos del Topic suscriptor
    ros::spin();	//Bloqueo del codigo espera de recibir los nod susc
    return 0;
}
