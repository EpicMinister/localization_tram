#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string>
#include <string.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "plc_tcp_server/TramStatus.h"
#include "geometry_msgs/Twist.h"


ros::Publisher tram_velocity_pub, tram_status_pub;
bool main_cab_active;
bool driving;
bool driving_forward;
bool braking_normal;
bool braking_emergency;
bool braking_rail;
bool braking_forced;
bool not_spring_break;
bool switch_control_left;
bool switch_control_right;
bool sanding;
bool doors_open;
bool pantograph_up;
bool indicator_right;
bool indicator_left;
bool hazard_lights;
bool warning_bell;
double driver_setpoint;
double velocity;
double velocity_kph;

bool convert_to_boolean(const std::string s) {
  
  try{
	  return (stoi(s) > 0 ? true : false);
	}
	catch (std::invalid_argument const &e){
        ROS_ERROR("std::invalid_argument thrown");
	}
	catch (std::out_of_range const &e){
		ROS_ERROR("std::out_of_range thrown");
	}

	return 0;
}

double convert_voltage(const std::string s) {

  try{
    //returns km/h
	  return ((stod(s)*10));
	}
	catch (std::invalid_argument const &e){
        ROS_ERROR("std::invalid_argument thrown");
	}
	catch (std::out_of_range const &e){
        ROS_ERROR("std::out_of_range thrown");
	}

	return 0;
}

void publish_data() {

  std_msgs::Header header;
  header.stamp = ros::Time::now();

  //Velocity Publisher
  geometry_msgs::Twist tram_velocity;
  //tram_velocity.header = header;
  tram_velocity.linear.x = velocity;
  tram_velocity_pub.publish(tram_velocity);

  //Status Publisher
  plc_tcp_server::TramStatus tram_status;
  
  tram_status.header = header;
  tram_status.main_cab_active = main_cab_active;
  tram_status.driving = driving;
  tram_status.driving_forward = driving_forward;
  tram_status.braking_normal = braking_normal;
  tram_status.braking_emergency = braking_emergency;
  tram_status.braking_rail = braking_rail;
  tram_status.braking_forced = braking_forced;
  tram_status.not_spring_break = not_spring_break;
  tram_status.switch_control_left = switch_control_left;
  tram_status.switch_control_right = switch_control_right;
  tram_status.sanding = sanding;
  tram_status.doors_open = doors_open;
  tram_status.pantograph_up = pantograph_up;
  tram_status.indicator_right = indicator_right;
  tram_status.indicator_left = indicator_left;
  tram_status.hazard_lights = hazard_lights;
  tram_status.warning_bell = warning_bell;
  tram_status.driver_setpoint = driver_setpoint;
  tram_status.velocity = velocity;
  tram_status.velocity_kph = velocity_kph;
  
  tram_status_pub.publish(tram_status);
}

void bombardier_string(const std::string s) {


  std::stringstream input;
  input.str (s);
  std::vector<std::string> result;

  //Separate by komma

  double t = 0.0;
  int i = 0;
  while(input.good()){
    std::string substr;
    getline(input, substr, ',');
    result.push_back(substr);

    switch (i)
      {
        case 0:
            main_cab_active = convert_to_boolean(substr);
            break;
        case 1:
            driving = convert_to_boolean(substr);
            break;
        case 2:
            driving_forward = convert_to_boolean(substr);
            break;
        case 3:
            braking_normal = !convert_to_boolean(substr); // original signal has an not, so we invert for better understanding
            break;
        case 4:
            braking_emergency = !convert_to_boolean(substr); // original signal has an not, so we invert for better understanding
            break;
        case 5:
            braking_rail = convert_to_boolean(substr);
            break;
        case 6:
            not_spring_break = convert_to_boolean(substr);
            break;
        case 7:
            switch_control_left = convert_to_boolean(substr);
            break;
        case 8:
            switch_control_right = convert_to_boolean(substr);
            break;
        case 9:
            sanding = convert_to_boolean(substr);
            break;
        case 10:
            doors_open = convert_to_boolean(substr);
            break;
        case 11:
            pantograph_up = convert_to_boolean(substr);
            break;
        case 12:
            braking_forced = !convert_to_boolean(substr); // original signal has an not, so we invert for better understanding
            break;
        case 13:
            indicator_right = convert_to_boolean(substr);
            break;
        case 14:
            indicator_left = convert_to_boolean(substr);
            break;
        case 15:
            hazard_lights = convert_to_boolean(substr);
            break;
        case 16:
            warning_bell = convert_to_boolean(substr);
            break;
        case 17:
            driver_setpoint = convert_voltage(substr);
            break;
        case 18:
            velocity = (convert_voltage(substr) / 3.6); //calc m/s
            velocity_kph = convert_voltage(substr);
            break;
         default:
            ROS_INFO("Number not valid!");
      }
    i++;
  }

  publish_data();

}
using namespace std;

int main(int argc, char** argv){

    ros::init(argc, argv, "plc_tcp_server");
    ros::NodeHandle ns;

    //Params
    //ros::Publisher server_pub = ns.advertise<std_msgs::String>("/bombardier_string", 1000);
    tram_velocity_pub = ns.advertise<geometry_msgs::Twist>("velocity", 1);
    tram_status_pub = ns.advertise<plc_tcp_server::TramStatus>("status", 1);

    char buf[4095];
    sockaddr_in client, hint;
    socklen_t client_size = sizeof(client);
    std_msgs::String message;
    char host[NI_MAXHOST];
    char svc[NI_MAXSERV];
    int server_port;
    bool connected_to_client = false;

    if(!ros::param::get("plc_port", server_port)){
        server_port = 55000;
    }
    ROS_INFO("port = %d", server_port);

    //create a socket
    int listening = socket(AF_INET, SOCK_STREAM, 0);
    if (listening < 0)
    {
        ROS_ERROR("[SERVER]can't create socket! Exiting!");
        return -1;
    }

    //Bind the socket to a IP/Port
    hint.sin_family = AF_INET;
    hint.sin_port = htons(server_port);
    inet_pton(AF_INET, "0.0.0.0", &hint.sin_addr);
    if (bind(listening, (sockaddr*)&hint, sizeof(hint)) < 0){
        ROS_ERROR("[SERVER]couldn't bind to IP/Port! Exiting!");
        return -1;
    }
    //mark the socket for listening in
    if (listen(listening, SOMAXCONN) == -1){
        ROS_ERROR("couldn't listen");
        return -1;
    }

    
    // Accept call
    int client_socket = accept (listening, (sockaddr*)&client, &client_size);

    if (client_socket == -1){
        ROS_ERROR("Problem with client connecting");
        return -1;
    }

    //close(listening);
    memset(host, 0 , NI_MAXHOST);
    memset(svc, 0, NI_MAXSERV);

    int result = getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, svc, NI_MAXSERV, 0);

    if (result)
    {
    cout << host << "connected on" << svc << endl;
    }
    else{
    inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
    cout << host << "conneted on" << ntohs(client.sin_port) << endl;
    }
    connected_to_client = true;
    
    //while recieving
    ros::Rate loop_rate(5);
    while(ros::ok())
    {
        if(!connected_to_client){
            int client_socket = accept (listening, (sockaddr*)&client, &client_size);
            connected_to_client = true;
            ROS_WARN_THROTTLE(1,"Client Reconnected");
        }
        std::stringstream ss;
        ss.str(std::string()); //Clear contents of string stream
        //clear buffer
        memset(buf, 0 , 4096);
        //wait for massage
        int byterec = recv(client_socket, buf, 4095, 0);
        if (byterec == -1){
            ROS_WARN_THROTTLE(1,"there was a connection issue");
            loop_rate.sleep();
            continue;
        }

        if ( byterec == 0)
        {
            ROS_WARN_THROTTLE(1, "the client disconnected");
            close(client_socket);
            connected_to_client = false;
            close(client_socket);
            loop_rate.sleep();
            continue;
        }
        
        //resend massage
        for (int i=4; i<sizeof(buf); i++){
            buf[i-4] = buf[i];
        }
        ss << buf;
        
        message.data = ss.str();
        //server_pub.publish(message);
        bombardier_string(message.data);
        send (client_socket, buf, byterec + 1, 0);
    }

    close(client_socket);
    return 0;
}