
// Includes 
#include <ros/ros.h>				//Importa libreria ROS
#include <image_transport/image_transport.h>    //Clase para manipular imagenes 
#include <cv_bridge/cv_bridge.h>		//Puente entre OpenCv y ROS para manejar OpenCv desde ROS
#include <sensor_msgs/image_encodings.h>	//Herramientas para manipular las imagenes 
#include <opencv2/imgproc/imgproc.hpp>		//Libreria propia de opencv (Procesamiento de imagenes)
#include <opencv2/highgui/highgui.hpp>		//Libreria propia de opencv (Diseñar interfaces graficas (poner imagen en ventana))

// Defines - General
#define    NODE_NAME       	"opencv_grayImage_hh"   		//Asignación de nombre de nodo
#define    OPENCV_WINDOW1       "Original Image"			//Asignación de nombre de ventana para imagen original
#define    OPENCV_WINDOW2       "Gray Image"				//Asignación de nombre de ventana para imagen procesada

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Imagen de la cámara (sin procesar).   (Nombre suscriptor)
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" // Imagen publica para ROS (procesada).  (Nombre publicador)

// CLASS: Image Conver (OpenCV)
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
  	ImageConverter() : it_(nh_)   //Hereda elementos de it_ y atributo es de nh_
  	{
    	    // Topics declaracion
       	    topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this); //Topic suscriptor 
					//(Nombre del topic, bufer de la comunicación, Callback(función o método), dentro de si mismo)

   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1); //Topic publicador 
										   //(Nombre del topic, bufer de la comunicación)

	    // Crea la GUI Windows (donde imprime las imagenes)
    	    cv::namedWindow(OPENCV_WINDOW1);	//Opencv abre una ventana y le asigna un nombre
	    cv::namedWindow(OPENCV_WINDOW2);	//Opencv abre una ventana y le asigna un nombre
  	}

	/* Desctructor Method */
  	~ImageConverter()
  	{
	    // Cierra la GUI Windows
    	    cv::destroyWindow(OPENCV_WINDOW1);  //Opencv destruye la ventana creada
	    cv::destroyWindow(OPENCV_WINDOW2);	//Opencv destruye la ventana creada
  	}

	/* asociado a "TOPIC1_SUB__IMAGE_INPUT" que obtiene la Imagen de la cámara (sin procesar) */

	void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg es la Imagen obtenida de la cámara (sin procesar)  
							    //(MÉTODO CON TIPO DE MENSAJE IMAGECONSTPTR Y SE ALMACENA EN MSG) 
  	{
	    // Convertir ROS imagen (Topic) a OpenCV imagen (Ptr)	    
    	    cv_bridge::CvImagePtr cv_OriginalImage_ptr;	    //OBJETO DE LA CLASE CvImagePtr QUE PERTENECE A LA LIBRERIA cv_bidge

	    
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
	    cv::Mat cvMat_OriginalImage_ptr = cv_OriginalImage_ptr->image; //Convierte los datos a la clase cv::Mat 

	    // Transforma Imagen Original a formato Gris.
	    cv::Mat cvMat_GrayImage_ptr;		//Crea una variable de la clase Mat para escala de grises
	    cv::cvtColor(cvMat_OriginalImage_ptr, cvMat_GrayImage_ptr, CV_BGR2GRAY); 
			//Convierte una imagen de un espacio de color a otro (imagen de entrada, imagen de salida, codigo de conversión)

	    //La clase Mat representa una matriz numérica densa de un solo canal o multicanal de n dimensiones. Se puede utilizar para 		    almacenar vectores y matrices reales o de valor complejo, imágenes en escala de grises o en color, entre otros.

	    /*********************************/ 
	    /* FIN: Procesando imagen digital */
	    /*********************************/

    	    // Actualiza GUI Window1 - Imagen Original
   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);   //Muestra la imagen en la ventana definida 
    	    cv::waitKey(3);					       //Funciona como un delay, esperando que oprima tecla para continuar

	    // Actualiza GUI Window2 - Imagen en formato Gris
   	    cv::imshow(OPENCV_WINDOW2, cvMat_GrayImage_ptr);		//Muestra la imagen en la ventana definida 
    	    cv::waitKey(3);						//Funciona como un delay, esperando que oprima tecla para continuar

    	    // Convertir OpenCV imagen (Mat) a OpenCV imagen (Bridge) a ROS imagen (Topic)
	    cv_bridge::CvImage cv_GrayImage; 			// Es necesario usar la clase CvImage no CvImagePtr
	    cv_GrayImage.header = cv_OriginalImage_ptr->header; //La misma marca de tiempo y marco de coordenadas que la Imagen Original  
								//(Encabezado). La secuencia la asigna ROS automaticamente
	    cv_GrayImage.encoding = sensor_msgs::image_encodings::MONO8; // MONO8 y MONO16 son iguales al formato GRIS
	    cv_GrayImage.image = cvMat_GrayImage_ptr; // datos
    	    
	    // Salida modificada
	    topic1_pub__image_output.publish(cv_GrayImage.toImageMsg()); //Topic publicador, publica la imagen en ROS 
  	}
};

//---Main---/ 
int main(int argc, char** argv)
{
    // Iniciar ROS 
    ros::init(argc, argv, NODE_NAME);   //Inicializar nodo
  
    // Iniciar objeto de la clase ImageConverter, definido antes
    ImageConverter ic;			//Creación de objeto de la clase ImageConverter

    // While true. Obteniendo datos del Topic suscriptor
    ros::spin(); 			//Bloquea el codigo esperando recibir información de los nodes suscriptores
    return 0;
}
