#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1
 
#define NAME_OF_THIS_NODE "subscriber"

#include "std_msgs/String.h" //include l'header del messaggio "std_msg::String"


class Subscriber 
{
  private:
    ros::NodeHandle Handle; //handler per abilitare la chiamata ai metodi di ROS
    
	ros::Subscriber Subscriber; //ATTRIBUTO oggetto publisher
	
	void MessageCallback(const std_msgs::String::ConstPtr& msg);
    
    
  public:
   
    void Prepare(void); //METODO 
    
    void RunContinuously(void); //METODO
    
    void Shutdown(void);//METODO 
	
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void Subscriber::Prepare(void)
{
/*indica al master che questo nodo vuole iscriversi al topic "chat", indicando anche la lunghezza della coda e il metodo da eseguire
  nel caso in quel topic venga pubblicato un messaggio. This indica che il metodo si trova nella classe Subscriber*/
  Subscriber = Handle.subscribe("chat", 1000, &Subscriber::MessageCallback, this);
   
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str()); 

}

void Subscriber::RunContinuously(void)
{
  ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
   
  ros::spin();//questo nodo continuerà a girare finchè non verrà chiamato ros::shutdown() o venga premuto ctrl-ConstPtr&
  
}

void Subscriber::MessageCallback(const std_msgs::String::ConstPtr& msg)//viene chiamata ogni qualvolta si riceve un messaggio
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void Subscriber::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
  
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  Subscriber subscriber;
   
  subscriber.Prepare();

  subscriber.RunContinuously();
  
  subscriber.Shutdown();
  
  return (0);
}
