
// Includes
#include <ros/ros.h>				//Importa libreria ROS
#include <image_transport/image_transport.h>    //Clase para manipular imagenes 
#include <cv_bridge/cv_bridge.h>		//Puente entre OpenCv y ROS para manejar OpenCv desde ROS
#include <sensor_msgs/image_encodings.h>	//Herramientas para manipular las imagenes 
#include <opencv2/imgproc/imgproc.hpp>		//Libreria propia de opencv (Procesamiento de imagenes)
#include <opencv2/highgui/highgui.hpp>		//Libreria propia de opencv (Diseñar interfaces graficas (poner imagen en ventana))

// Defines - General
#define    NODE_NAME       	"opencv_change_contrast_hh"		//Asignación de nombre de nodo
#define    OPENCV_WINDOW1       "Original Image"			//Asignación de nombre de ventana para imagen
#define    OPENCV_WINDOW2       "New Image (Contrast & brightness)"	//Asignación de nombre de ventana para imagen procesada

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Imagen de la cámara (sin procesar).   (Nombre suscriptor). 
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" // Imagen publica para ROS (procesada).  (Nombre publicador).

//***FUNCIÓN ESTATICA: Método del trackbar/scrollbar para el valor de Alfa. High-GUI of OpenCV***
double Alpha = 1.5; 	// Control simple de contraste. Valores de 1.0 a 3.0 
int trackbar1_slider; 	// Donde se almacena el valor real del trackbar/scrollbar

static void trackbar1_func(int, void*)
{
    // Escala el valor del trackbar/scrollbar [0 - 100%] al valor alfa [0.0 - 3.0]
    Alpha = trackbar1_slider*3.0/100.0;
}

//***FUNCIÓN ESTATICA: Método del trackbar/scrollbar para el valor de Beta. High-GUI of OpenCV***
int Beta = 30;         // Control simple de brillo. Valores de 0 a 100	
int trackbar2_slider;  // Donde se almacena el valor real del trackbar/scrollbar

static void trackbar2_func(int, void*)
{
    // Cambio valor de Beta
    Beta = trackbar2_slider;
}

//***CLASS: Image Conver (OpenCV)***
class ImageConverter
{
    private: 
    	// NodeHandle ROS
    	ros::NodeHandle nh_;	//Clase NodeHandle de la libreria de ROS

    	// Imagen utilizada 
    	image_transport::ImageTransport it_; 	// Object it_ from image transport clase (utilizado para el procesamiento de imagenes)
    	image_transport::Subscriber topic1_sub__image_input; // Imagen de la cámara (sin procesar). Formato ROS (Topic)
    	image_transport::Publisher topic1_pub__image_output; // Imagen publica para ROS (procesada). Formato ROS (Topic)

    public:

	/* Constructor Method. */
  	ImageConverter() : it_(nh_)	//Hereda elementos de it_ y atributo es de nh_
  	{
    	    // Topics declaration
       	    topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this); //Topic suscriptor 
					//(Nombre del topic, bufer de la comunicación, Callback(función o método), dentro de si mismo)
 
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1); //Topic publicador 
										   //(Nombre del topic, bufer de la comunicación)


	    // Crea la GUI Windows (donde imprime las imagenes)
    	    cv::namedWindow(OPENCV_WINDOW1);	//Opencv abre una ventana y le asigna un nombre
	    cv::namedWindow(OPENCV_WINDOW2);	//Opencv abre una ventana y le asigna un nombre

	    // Crea un nuevo Scrollbar/trackbar
	    int trackbar_maxValue = 100; // En porcentaje.
	    cv::createTrackbar("Alpha [0-100%]", OPENCV_WINDOW2, &trackbar1_slider, trackbar_maxValue, trackbar1_func); 
		// Nota: 1) El Trackbar tiene una etiqueta "Alpha", 2) El Trackbar se encuentra en la ventana “OPENCV_WINDOW2”, 3) 			Los valores  del Trackbar estaran en el rango de 0 a "trackbar_maxValue" (el limite minimo siempre es 0), 4) El valor 			numerico del Trackbar es almacenado en "trackbar_slider", y 5) Siempre que el usuario mueve el Trackbar, la funcion 			callback on_trackbar es llamada
	    cv::createTrackbar("Beta [0-100%]", OPENCV_WINDOW2, &trackbar2_slider, trackbar_maxValue, trackbar2_func); 
  	}

	/* Desctructor Method */
  	~ImageConverter()
  	{
	    // close the GUI Windows
    	    cv::destroyWindow(OPENCV_WINDOW1);
	    cv::destroyWindow(OPENCV_WINDOW2);
  	}

	/* asociado a "TOPIC1_SUB__IMAGE_INPUT" que obtiene la Imagen de la cámara (sin procesar) */
	void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg es la Imagen obtenida de la cámara (sin procesar)
							    //(MÉTODO CON TIPO DE MENSAJE IMAGECONSTPTR Y SE ALMACENA EN MSG)
  	{
	    // Convertir ROS imagen (Topic) a OpenCV imagen (Ptr)	    
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
	    cv::Mat cvMat_Image_ptr = cv_OriginalImage_ptr->image;  //Convierte los datos a la clase cv::Mat 

	    // Hace la operación new_image(i,j) = Alpha*Orginal_image(i,j) + Beta
 	    cv::Mat cvMat_NewImage_ptr = cv::Mat::zeros(cvMat_Image_ptr.size(), cvMat_Image_ptr.type()); 
									//Devuelve una matriz cero del tamaño y tipo especificados
	    for( int y = 0; y < cvMat_Image_ptr.rows; y++ )
    	    { 
		for( int x = 0; x < cvMat_Image_ptr.cols; x++ )
         	{ 
		    for( int c = 0; c < 3; c++ )
              	    {
      			cvMat_NewImage_ptr.at<cv::Vec3b>(y,x)[c] = 
				cv::saturate_cast<uchar>( Alpha*(cvMat_Image_ptr.at<cv::Vec3b>(y,x)[c]) + Beta );
			//Accede a cada píxel de la imagen. Como se esta operando con imágenes BGR, tendra tres valores 
			//por píxel (B, G y R), por lo que también se accede a ellos por separado

			//NOTA: 1) Para acceder a cada píxel de las imágenes se usa esta sintaxis: image.at <Vec3b> (y, x) [c] 					donde y es la fila, x es la columna yc es R, G o B (0, 1 o 2).                                             					2) Dado que la operación Alpha*Orginal_image(i,j) + Beta puede dar valores fuera de rango o no enteros (si 					alfa es flotante), se usa saturate_cast para asegurar de que los valores sean válidos.
             	    }
    		}
    	    }
	    /*********************************/ 
	    /* FIN: Procesando imagen digital */
	    /*********************************/

    	    // Actualiza GUI Window1 - Imagen Original
   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image); //Muestra la imagen en la ventana definida 
    	    cv::waitKey(3);					     //Funciona como un delay, esperando que oprima tecla para continuar

	    // Actualiza GUI Window2 - Nueva Imagen (Contraste & brillo)
   	    cv::imshow(OPENCV_WINDOW2, cvMat_NewImage_ptr);	     //Muestra la imagen en la ventana definida
	    ROS_INFO("Alpha %f ------ Beta %d", Alpha, Beta);	     //Imprime los valores de beta y alfa en el terminal
	    cv::waitKey(3);					     //Funciona como un delay, esperando que oprima tecla para continuar

    	    // Convertir OpenCV imagen (Mat) a OpenCV imagen (Bridge) a ROS imagen (Topic)  
	    cv_bridge::CvImage cv_NewImage; 		       // Es necesario usar la clase CvImage no CvImagePtr
	    cv_NewImage.header = cv_OriginalImage_ptr->header; //La misma marca de tiempo y marco de coordenadas que la Imagen Original  
							       //(Encabezado). La secuencia la asigna ROS automaticamente
	    cv_NewImage.encoding = cv_OriginalImage_ptr->encoding; // Mismo formato que la imagen Original
	    cv_NewImage.image = cvMat_NewImage_ptr; // datos

    	    // Salida modificada
	    topic1_pub__image_output.publish(cv_NewImage.toImageMsg());
  	}
};

//***Main***
int main(int argc, char** argv)
{
    // Iniciar ROS 
    ros::init(argc, argv, NODE_NAME);	//Inicializar nodo
  
    // Iniciar objeto de la clase ImageConverter, definido antes
    ImageConverter ic;			//Creación de objeto de la clase ImageConverter

    // While true. Obteniendo datos del Topic suscriptor
    ros::spin();			//Bloquea el codigo esperando recibir información de los nodes suscriptores
    return 0;
}
