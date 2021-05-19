/* Plantilla ejemplo 4 OPENCV
/*
/*   Suavizado de imagenes
/*   from = https://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html#smoothing
/*
/*   Explicación uso del filtro de Gauss 
/*   From = https://www.youtube.com/watch?v=7LW_75E3A1Q
/* 
*/

// Includes
#include <ros/ros.h>				//Importa libreria ROS

#include <image_transport/image_transport.h>	//Clase para manipular imagenes
#include <cv_bridge/cv_bridge.h>		//Puente entre OpenCv y ROS para manejar OpenCv desde ROS
#include <sensor_msgs/image_encodings.h>	//Herramientas para manipular las imagenes
#include <opencv2/imgproc/imgproc.hpp>		//Libreria propia de opencv (Procesamiento de imagenes)
#include <opencv2/highgui/highgui.hpp>		//Libreria propia de opencv (Diseñar interfaces graficas (poner imagen en ventana))

// Defines - General

#define    NODE_NAME       	"opencv_smoothingImages_hh"	//Renombre de nodo
#define    OPENCV_WINDOW1       "Original Image"		//Renombre de ventana para imagen original
#define    OPENCV_WINDOW2       "Image Filtradad"		//Renombre de ventana para imagen procesada



// Defines - Topics 
// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		//Imagen original de la camara (suscriptor)
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" //Imagen publica para ROS (Publicador).

// Define - Selector de filtro
#define    HOMOGENEOUS_BLUR	0
#define    GAUSSIAN_BLUR	1
#define    MEDIAN_BLUR		2
#define    BILATERAL_BLUR	3
int Filter_Selected = 0;	//De 0 a 3, para seleccionar el filtro utilizado

//***STATIC FUNCTION: Trackbar to define the kernel_length applied into the filters
int Kernel_length_slider = 3; // Slider value
static void trackbar1_func(int, void*)
{
   //Kernel_length_slider from 0 to 30 (Define when the trackbar is create
}

//***STATIC FUNCTION: Trackbar to define the filter used
static void trackbar2_func(int, void*)
{
    // Min value 1. Max value 3 (Defined upon) 
    // Filter_Selected
}

//***CLASS: Image Conver (OpenCV)***
class ImageConverter
{
    private: 
    	// NodeHandle ROS
    	ros::NodeHandle nh_;	//Clase NodeHandle de la libreria de ROS

    	// Imagen usada 
    	image_transport::ImageTransport it_; // Object it_ from image transport clase (utilizado para el procesamiento de imagenes)
    	image_transport::Subscriber topic1_sub__image_input;	//Imagen de la cámara (sin procesar). Formato ROS (Topic)
    	image_transport::Publisher topic1_pub__image_output;	//Imagen publica para ROS (procesada). Formato ROS (Topic)

    public:

	/* Constructor Method. 
	   TODO */
  	ImageConverter() : it_(nh_)	//Hereda elementos de it_ y atributo es de nh_
  	{
    	    // Topics declaration
       	    topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this); //Topic suscriptor
										//(Nombre del topic, bufer de la comunicación, Callback(función o método), dentro de si mismo)			   	   +

	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);	//Topic publicador
											//(Nombre del topic, bufer de la comunicación)	
	    
	    //Crea la GUI Windows (donde imprime las imagenes)
    	    cv::namedWindow(OPENCV_WINDOW1);
	    cv::namedWindow(OPENCV_WINDOW2);

	    // Create a new Scrollbar/trackbar
	    int trackbar1_maxValue = 50; // In units.
	    cv::createTrackbar("Kernel length [1-50]", OPENCV_WINDOW2, &Kernel_length_slider, trackbar1_maxValue, trackbar1_func); // Comments: View "opencv_change_contrast_hh.cpp" code
	    int trackbar2_maxValue = 3; // In units.
	    cv::createTrackbar("Filter [1-3]", OPENCV_WINDOW2, &Filter_Selected, trackbar2_maxValue, trackbar2_func);
  	}

	/* Desctructor Method */
  	~ImageConverter()
  	{
	    // close the GUI Windows
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
   
    	    // Convertir OriginalImage a la clase cv::Mat
	    cv::Mat cv_OriginalImage_Mat = cv_OriginalImage_ptr->image;	//Convierte los datos a la clase cv::Mat 
	    
	    // Nueva imagen a procesar de la clase cv::Mat
	    cv::Mat cv_ImageProcessed_Mat = cv_OriginalImage_Mat.clone();

	    //** Filtros aplicados **/
	    int l = kernel_format(Kernel_length_slider);
	   
	    switch(Filter_Selected)
	    {
		// Blur homogeneo
		case 0:
		    cv::blur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, cv::Size(l, l), cv::Point(-1,-1));
	    	    ROS_INFO("FILTER APPLIED: ** 0. Homogeneous blur **");
		break;
		// Aplicando Blur gaussiano
		case GAUSSIAN_BLUR:
		    cv::GaussianBlur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, cv::Size(l, l), 0, 0);
		    ROS_INFO("FILTER APPLIED: ** 1. Gaussian blur **");
		break;
		// AAplicando Blur mediano
		case MEDIAN_BLUR:
		    cv::medianBlur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, l);
		    ROS_INFO("FILTER APPLIED: ** 2. Median blur **");
		break;
		// Aplicando Blur bilateral
		case BILATERAL_BLUR:
		    cv::bilateralFilter(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, l, l*2, l/2 );
		    ROS_INFO("FILTER APPLIED: ** 3. Bilateral blur **");
		break;	
	    }	
	    ROS_INFO("    KERNEL_LENGTH: %d x %d", l, l);
	    ROS_INFO(" ");
	    //** Fin filtros aplicados **/ 
	
	    /*********************************/ 
	    /* END: Imagen digital */
	    /*********************************/

    	    // Arreglo GUI Window1 - Imagen Original
   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
    	    cv::waitKey(3);

	    //  Arreglo GUI Window2 - Nueva imagen Contraste y brillo
   	    cv::imshow(OPENCV_WINDOW2, cv_ImageProcessed_Mat);
	    cv::waitKey(3);

    	    // Convert OpenCV image (Mat) to OpenCV image (Bridge) to ROS image (Topic)  
	    //cv_bridge::CvImage cv_NewImage; // it's needed use the Class CvImage not CvImagePtr
	    //cv_NewImage.header = cv_OriginalImage_ptr->header; // Same timestamp and tf frame as Original image. The seq is assigned by ROS automatically
	    //cv_NewImage.encoding = cv_OriginalImage_ptr->encoding; // Same format as Original image 
	    //cv_NewImage.image = cvMat_NewImage_ptr; // data
    	    // Output modified video stream
	    //topic1_pub__image_output.publish(cv_NewImage.toImageMsg());
  	}
	
	/* Only take the odd numbers */
	int kernel_format(int value)
	{ 
	    if(value%2 == 0)
		return value + 1;
	    else 
		return value;
	}
};

//***Main***
int main(int argc, char** argv)
{
    // Iniciar ROS 
    ros::init(argc, argv, NODE_NAME);
  
    //Iniciar objeto de la clase ImageConverter, definido antes
    ImageConverter ic;

    //While true. Obteniendo datos del Topic suscriptor
    ros::spin();
    return 0;
}
