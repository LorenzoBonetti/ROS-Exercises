cmake_minimum_required(VERSION 2.8.3)
project(pub_sub)
    
## Trova catkin e i componenti necessari
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
catkin_package()
       
##Rende eseguibili publisher e subscriber
include_directories(include ${catkin_INCLUDE_DIRS})
 
add_executable(publisher src/publisher.cpp) #nome nodo  percorso file
target_link_libraries(publisher ${catkin_LIBRARIES})
add_dependencies(publisher pub_sub_generate_messages_cpp)#aggiunge una dipendenza alla generazione di messaggi
 
add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
add_dependencies(subscriber pub_sub_generate_messages_cpp)

