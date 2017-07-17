#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#include<thread>
#include <condition_variable>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

std::mutex mtx; 
std::condition_variable cv;
std::string cmd = "YetToStart";
int steps = 0;
int max_steps = 300;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Reset the simulator
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws, double steering){
  json msgJson;
  msgJson["steering_angle"] = steering;
  msgJson["throttle"] = 0.0;
  auto msg = "42[\"reset\"," + msgJson.dump() + "]";
  std::cout << msg << std::endl;
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);  
}

void run_message_handler(uWS::Hub& h){
  std::cout<< "Main OnMessageHanlder thread:" << std::this_thread::get_id()<< std::endl;  
  cmd = "Started" ;
  h.run();  
}

double run(uWS::Hub& h, PID& pid, std::vector<double> p){
  std::cout << "Starting fresh run." << std::endl;

  pid.Init(p[0], p[2], p[1], max_steps);

  cmd = "Restart" ;
  
  printf ("Sleeping.\n");
  std::unique_lock<std::mutex> lck(mtx);
  cv.wait(lck);
  printf ("Woke up. Error %f \n", pid.TotalError());
  
  return pid.TotalError();
}

double twiddle(double tol, PID &pid, uWS::Hub& h){
    std::cout << "Twiddle thread:" << std::this_thread::get_id()<< std::endl;
    //tol=0.2; 
    double kp = 0.8; 
    double kd = 0.3; 
    double ki = 0.004;
    
    std::vector<double> p = {kp, ki, kd};
    std::vector<double> dp = {1, 1, 1};

    std::cout <<  "#####  Start iteration 0 #####"  << std::endl;;    
    double best_err = run(h, pid, p);
  
    double sum_dp = 0.0 ;
    
    for (auto& n : dp) {
      sum_dp += n;
    }

    int itr = 0;
    
    while (sum_dp > tol) {
      itr++;           
      for (int i = 0; i < 3; i++){
        p[i] += dp[i] ;
        printf ("##### Itr = %d, Tuning1 p[%d] = %f, dp[%d]= %f #####\n", itr, i, p[i], i, dp[i]);        
        double err = run(h, pid, p);

        if (err < best_err) {
            best_err = err ;
            dp[i] *= 1.1 ;
        } else {
            p[i] -= 2 * dp[i] ;
            printf ("##### Itr = %d, Tuning2 p[%d] = %f, dp[%d]= %f #####\n", itr, i, p[i], i, dp[i]);
            err = run(h, pid, p);

            if (err < best_err) {
               best_err = err; 
               dp[i] *= 1.1;
            } else {
               p[i] += dp[i] ;
               dp[i] *= 0.9;
            }
        }
      }
      
      printf ("#####  End iteration %d, best error = %f #####\n", itr, best_err); 
      printf ("#####  Kp %f, Kd %f, Ki %f  #####\n", p[0], p[1], p[2]); 

      for (auto& n : dp) {
        sum_dp += n;
      }      
    }
    
    return best_err ; 
}    

int main() {

  uWS::Hub h;
  PID pid;
  // TODO: Initialize the pid variable.
  double kp = 0.8; 
  double kd = 0.3; 
  double ki = 0.004;
  
//  double kp = 25.0; 
//  double kd = 500.0; 
//  double ki = 0.04;
  pid.Init(kp, ki, kd, max_steps);
  
  //std::cout << "Steering angle...1 " << pid.steering_angle << std::endl;
  
  std::cout<< "Main thread:" << std::this_thread::get_id()<< std::endl;
  
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (cmd == "Restart") {
      cmd = "Started" ;
      steps = -10;
      //restart_simulator(ws);
      reset_simulator(ws, 0.0);
    } else if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          // double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          double steer_value = pid.steering_angle;
          double throttle = pid.throttle;
          
          steps++;          
          if (steps < 1) {
            steer_value = 0.0;
          } else { // steps 1 to max_steps
            pid.UpdateError(cte, speed);
            
            steer_value = deg2rad(steer_value);
            steer_value = steer_value / pi();

            if ( steer_value < -1.0) {
              steer_value = -1.0;
            } else if (steer_value > 1.0) {
              steer_value = 1.0;
            }

            if (speed > 5.0){
              throttle = 0.0;
            } else {
              throttle = 0.2;
            }

            if (steps > max_steps) {
              steps = -10;
              cv.notify_all();
              printf ("Notified sleeping thread.\n");
              steer_value = 0.0;
            }            
          }

          // std::unique_lock<std::mutex> lck(mtx);
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // DEBUG
          if (steps % 50 == 0) {
            std::cout << "Steps " << steps << std::endl;
            std::cout << "CTE: " << cte << " Speed:" << speed << " Steering Value: " << steer_value << " throttle: " << throttle << std::endl;
            // std::cout << msg << std::endl;
          }          

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;

  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
    
  std::thread onmsg_thread(run_message_handler, std::ref(h));

  std::thread twiddle_thread(twiddle, 0.05, std::ref(pid), std::ref(h));
  
  onmsg_thread.join();
  twiddle_thread.join();
  
  std::cout << "Done." << std::endl;
  
}
