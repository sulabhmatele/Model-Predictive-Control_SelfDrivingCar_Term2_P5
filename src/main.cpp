#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;

    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main()
{
    uWS::Hub h;

    // MPC object creation
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode)
                {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event

        string sdata = string(data).substr(0, length);
        cout << sdata << endl;

        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
        {
            string s = hasData(sdata);
            if (s != "")
            {
                auto j = json::parse(s);
                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];

                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];

                    double delta = j[1]["steering_angle"];
                    double acceleration = j[1]["throttle"];

                    // Predict the state after the latency:
                    // predict state in 100ms
                    double lat_dt = 0.1;


                    px = px + v * cos(psi) * lat_dt;
                    py = py + v * sin(psi)*lat_dt;
                    psi = psi - v * (delta/mpc.Lf) * lat_dt;
                    v = v + acceleration * lat_dt;

                    /*
                    * Calculating steering angle and throttle using MPC.
                    *
                    * steering angle - [-25, 25].
                    *
                    */

                    for (int i = 0; i < ptsx.size(); i++)
                    {
                        double shift_x = ptsx[i]-px;
                        double shift_y = ptsy[i]-py;

                        ptsx[i] = shift_x * cos(0-psi) - shift_y * sin(0-psi);
                        ptsy[i] = shift_x * sin(0-psi) + shift_y * cos(0-psi);
                    }

                    double* ptrx = &ptsx[0];
                    Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

                    double* ptry = &ptsy[0];
                    Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

                    // The polynomial is fitted to 3rd order polynomial
                    auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

                    // The cross track error is calculated by evaluating at polynomial
                    double cte = polyeval(coeffs, 0);

                    // Due to the sign starting at 0, the orientation error is -f'(x).
                    // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
                    double epsi = -atan(coeffs[1]);

                    Eigen::VectorXd state(6);
                    state << 0, 0, 0, v, cte, epsi;

                    // MPC solve
                    auto vars = mpc.Solve(state, coeffs);

                    //Display the MPC predicted trajectory - Green line on simulator
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    for(int i = 2; i < vars.size(); i++)
                    {
                        // Retrieval as it was passed from solve method
                        if(i%2 == 0)
                        {
                            mpc_x_vals.push_back(vars[i]);
                        }
                        else
                        {
                            mpc_y_vals.push_back(vars[i]);
                        }
                    }

                    //Display the waypoints/reference line - Yellow line on simulator
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    int num_points = 25; // 25 points in future
                    double poly_inc = 2.5; // 2.5 m difference between each point

                    for(int i = 1; i < num_points; i++)
                    {
                        // Project in future, based on the reference points received and
                        // coeff calculated from the same
                        next_x_vals.push_back(poly_inc * i);
                        next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
                    }

                    json msgJson;

                    // Converting delta angle to be between -1 to +1
                    msgJson["steering_angle"] = vars[0]/(deg2rad(25) * mpc.Lf);
                    msgJson["throttle"] = vars[1];

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";

                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // NOTE: 100ms as per the requirement from project

                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
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
    h.run();
}