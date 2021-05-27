/* Plantilla para usar OpenCV dentro de ROS
/*
/*   Tutorial de ROS 
/*   TOmado de: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages 
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";		// Define la palabra OPENCV_WINDOW para referirse de ahora en adelante a la ventana donde se muestran las imagenes

class ImageConverter							// Se define la clase y se hace la herencia de los objetos y métodos necesarios
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()							//Método constructor, aquí se define el suscriptor y se crea la ventana de visualización
    : it_(nh_)
  {	
    									// Suscribirse al topic que proporciona las imágenes de entrada (sin procesar)
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,		// Suscripción al topic de camara a través de la librería ImageTransport, como aun no se ha creado el callback, se usa "this" 
      &ImageConverter::imageCb, this);				//   para indicar que se aparte la memoria para un método definido más adelante 
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);					//Crea una ventana nueva, donde que puede servir despues para mostrar al usuario las imagenes del procesamiento
  }									//La ventana se crea en este caso con el nombre OPENCV_WINDOW, si se intenta crear otra ventana con ese nombre, el comando no 										funcionará
	
  ~ImageConverter()							// Método destructor
  {
    cv::destroyWindow(OPENCV_WINDOW);					//Elimina la ventana creada para mostrar las imagenes 
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);		// Se copia la matriz de la imagen para pasar del formato de ImageTransport al de OpenCV
    }											// Para ello se usa una matriz de punteros, esto permite optimización de recursos de memoria
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());				// Si no se puede crear la copia, el sistema muestra el error causado 
      return;
    }

    // Dibujar un círculo de ejemplo en el video 
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)				// Primero verifica que la imagen tenga un tamaño mínimo en filas y columnas para poder dibujar el círculo
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));		/* Usando cv::circle, se dibuja un círculo en pantalla, esto toma como argumento la imagen en que se va a dibujar, 
											   un punto, que se define con sus coordenadas en la imagen; el radio y el color del círculo */
											   
    // Actualizar la ventana de la interfaz grafica para mostrar los cambios
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);									/* Se usa para crear un delay, en caso de no tener argumento, se mantiene esperando la tecla, en caso de tenerlo, 												es el número de milisegundos a esperar. Si se iguala esta función a una variable, en ella se almacenará el código 												de la tecla presionada */
						
    // Se publica en un topic la imagen procesada
    image_pub_.publish(cv_ptr->toImageMsg());						// Se hace uso del objeto publicador image_pub de ImageTransport para publicar la matriz procesada con el 												método .publish
  }											//Se usa internamente el método toImageMsg para cambiar el tipo de variable por una de tipo mensaje
};

int main(int argc, char** argv)			// Función principal
{
  ros::init(argc, argv, "image_converter");		// Inicializar nodo 
  ImageConverter ic;					// Ejecutar método constructor	
  ros::spin();						// Mantener el programa a la espera de actualizaciones en los topics suscritos (todo el algoritmo se ejecuta desde el callback )
  return 0;
}
