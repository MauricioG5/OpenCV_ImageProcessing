/* PLANTILLA PARA USAR OPENCV CON ROS
/*
/*   Base: ROS tutorial
/*   Tomado de: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages 
/*
/*   Modificado por Hernán Hernández para ser usado como plantilla propia
*/

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Defines - General
#define    NODE_NAME       	"opencv_template_node_hh"
#define    OPENCV_WINDOW       "Image window"

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Topic de donde se va a tomar la imagen directo de la camara
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" 	// Topic donde se va a cargar la imagen tras pasar el procesamiento

// CLASE: Image Converter (OpenCV)
class ImageConverter
{
    private: 
    	// NodeHandle ROS1
    	ros::NodeHandle nh_;

    	// Image used 
    	image_transport::ImageTransport it_; 				// Se define el objeto it_ de la clase ImageTransport
    	image_transport::Subscriber topic1_sub__image_input; 	// Crea la variable del suscriptor para adquirir la imagen de la cámara desde ROS
    	image_transport::Publisher topic1_pub__image_output; 	// Crea la variable del publicador para transmitir la imagen procesada de regreso a ROS 

    public:

	/* Metodo Constructor  
	   TODO */
  	ImageConverter() : it_(nh_)
  	{
    	    // Declaración de Topics
      	  topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this);  // Se define el suscriptor con el nombre del topic y un callback que se 																definirá más adelante, por eso la expresión "this" al final
   	  topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);						// Se define el publicador para enviar la imagen procesada

	    // Crear una ventana para publicar en ella la imagen
    	  cv::namedWindow(OPENCV_WINDOW);
  	}

	/* Método destructor */
  	~ImageConverter()
  	{
	    // Eliminar ventana creada 
    	    cv::destroyWindow(OPENCV_WINDOW);
  	}

	/* Asociado al TOPIC1_SUB_IMAGE_INPUT, el cual toma la imagen original de la cámara */
	void imageCb(const sensor_msgs::ImageConstPtr& msg) 						// msg is the Image get from camera (raw)
  	{	
	    // Convertir la imagen de ROS en una imagen de OpenCV con punteros
    	    cv_bridge::CvImagePtr cv_OriginalImage_ptr; 						// Copia la imagen del topic de ROS a una CvImage de OpenCV
    	    try
    	    {
      		cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // Posibles constantes: "mono8", "bgr8", "bgra8", "rgb8", "rgba8", "mono16"...
    	    }
	    catch (cv_bridge::Exception& e)
    	    {
		// Imprime el error en caso de detectarse uno 
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	    }

	    /******************************************/ 
	    /* INICIO DEL PROCESAMIENTO DE IMÁGENES */
	    /****************************¨¨¨¨¨¨¨¨¨¨¨¨¨/
    	    
	    // TODO TODO TODO 

	    /**************************************/ 
	    /* FIN DEL PROCESAMIENTO DE IMÁGENES */
	    /*************************************/

    	    // Dibuja un círculo de ejemplo en la imagen en formato CvImg
    	    if (cv_OriginalImage_ptr->image.rows > 60 && cv_OriginalImage_ptr->image.cols > 60)		// Primero se verifica que la imagen tenga un tamaño mínimo
      	    cv::circle(cv_OriginalImage_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));		// Luego se utiliza "circle" para dibujar un círculo tomando la imagen en que va a dibujar, un punto con las coordenadas, el radio del círculo y su color

    	    // Actualizar la ventana con la imagen
   	    cv::imshow(OPENCV_WINDOW, cv_OriginalImage_ptr->image);
    	    cv::waitKey(3);

    	    // Convertir la imagen de OpenCV (ptr) en una imagen de ROS (Topic) 
	    // Salida de video modificada
    	    topic1_pub__image_output.publish(cv_OriginalImage_ptr->toImageMsg());
  	}
};

// Código principal
int main(int argc, char** argv)
{
    // Inicialización del nodo de ROS
    ros::init(argc, argv, NODE_NAME);
  
    // Inicialización del objeto ic de la clase ImageConverter definida antes
    ImageConverter ic;

    // Mientras se mantenga el bucle, permanece a la espera para ejecutar el callback,donde se encuentra todo el procesamiento 
    ros::spin();
    return 0;
}
