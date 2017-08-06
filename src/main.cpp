/*
* Path Planning Project
* 2017 by Max Ritter
* For Udacity Self-Driving-Engineer Term 3
*/

//Standard includes
#define _USE_MATH_DEFINES
#include <cmath> 
#include <math.h>
#include <fstream>
#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
using namespace std;

//Spline lib for smooth trajectories
#include "spline.h"

//Waypoint information from the map
typedef struct
{
	vector<double> x;
	vector<double> y;
	vector<double> s;
	vector<double> dx;
	vector<double> dy;
} MAP;
MAP ourMap;

//Current state of the car
typedef struct
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
} CAR;
CAR ourCar;

//Number of waypoints in the map
int waypoints_size;

//Current lane of the car
int current_Lane;

//The previous path
vector<vector<double>> previous_path;

//Size of the previous path vector
int previous_path_size;

//Sensor fusion information
vector<vector<double>> sensor_fusion;

//Size of the sensor fusion vector
int sensor_fusion_size;

//Is true if the car is changing lanes
bool lane_change = false;

//Votes for changing the lane
int lane_change_votes = 0;

//Keep track of the last velocity action we did
int velocity_action = -1;

//Value of distance increment per timestep
double distance_increment = 0.44;

//Next d value
double next_d_val = 6.0;

//Stores the velocity of the path
vector<double> path_velocity;

//Stores the history of the path
vector<vector<double>> path_history;

//For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//Calculate the distance between two points
double Distance(double x1, double y1, double x2, double y2)
{
	double x_diff = x2 - x1;
	double y_diff = y2 - y1;
	double dist = sqrt((x_diff * x_diff) + (y_diff * y_diff));
	return dist;
}

//Finds the closet waypoint index to the car and returns the ID
int GetClosestWaypoint()
{
	//Set current distance to large value
	double cur_dist = 99999;
	//Set waypoint ID to zero
	int waypoint_id = 0;

	//Go through all waypoints and find closest one
	for (int i = 0; i < waypoints_size; i++)
	{
		//Calculate distance between car and waypoint
		double distance = Distance(ourCar.x, ourCar.y, ourMap.x[i], ourMap.y[i]);

		//If distance is shorter than current shortest, use this
		if (distance < cur_dist)
		{
			cur_dist = distance;
			waypoint_id = i;
		}
	}

	//Return ID of the closest waypoint
	return waypoint_id;
}

//Transform from global cartesian coordinates to local car coordinates
vector<double> GetLocalCoordinates(double World_x, double World_y)
{
	//X and y point in the car coordinate system
	vector<double> car_coordinates;

	//Calculate delta between world and car coordinate
	double delta_x = World_x - ourCar.x;
	double delta_y = World_y - ourCar.y;

	//Convert car's yaw from degrees to radians
	double yaw_r = deg2rad(ourCar.yaw);

	//Transform to car coordinates
	double car_x = (delta_x  * cos(yaw_r)) + (delta_y * sin(yaw_r));
	double car_y = (-delta_x * sin(yaw_r)) + (delta_y * cos(yaw_r));

	//Add x and y coordinate to the result
	car_coordinates.push_back(car_x);
	car_coordinates.push_back(car_y);
	return car_coordinates;
}

//Convert a list of world cartesian coordinates to local car vectors
vector<vector<double>> GetLocalPoints(vector<double> World_x_vector, vector<double> World_y_vector)
{
	//Store the results here
	vector<vector<double>> car_coordinate_vectors;
	vector<double> car_x_vector;
	vector<double> car_y_vector;

	//Go through all the points
	for (int i = 0; i < World_x_vector.size(); i++)
	{
		vector<double> car_xy = GetLocalCoordinates(World_x_vector[i], World_y_vector[i]);
		car_x_vector.push_back(car_xy[0]);
		car_y_vector.push_back(car_xy[1]);
	}

	//Add x and y vectors to the final result
	car_coordinate_vectors.push_back(car_x_vector);
	car_coordinate_vectors.push_back(car_y_vector);
	return car_coordinate_vectors;
}

//Transfrom from local car coordinates to global cartesian coordinates
vector<double> GetWorldCoordinates(double Local_x, double Local_y)
{
	//X and y point in the world coordinate system
	vector<double> world_coordinates;

	//Convert car's yaw from degrees to radians
	double yaw_r = deg2rad(ourCar.yaw);

	//Transform to world coordinates
	double world_x = (Local_x * cos(yaw_r)) - (Local_y * sin(yaw_r)) + ourCar.x;
	double world_y = (Local_x * sin(yaw_r)) + (Local_y * cos(yaw_r)) + ourCar.y;

	//Add x and y coordinate to the result
	world_coordinates.push_back(world_x);
	world_coordinates.push_back(world_y);
	return world_coordinates;
}

//Convert a list of local car vector coordinates to global world vectors
vector<vector<double>> GetWorldPoints(vector<double> Local_x_vector, vector<double> Local_y_vector)
{
	//Store the results here
	vector<vector<double>> world_coordinate_vectors;
	vector<double> world_x_vector;
	vector<double> world_y_vector;

	//Go through all points
	for (int i = 0; i < Local_x_vector.size(); i++)
	{
		vector<double> world_xy = GetWorldCoordinates(Local_x_vector[i], Local_y_vector[i]);
		world_x_vector.push_back(world_xy[0]);
		world_y_vector.push_back(world_xy[1]);
	}

	//Add x and y vectors to the final result
	world_coordinate_vectors.push_back(world_x_vector);
	world_coordinate_vectors.push_back(world_y_vector);
	return world_coordinate_vectors;
}

//Calculate a set of waypoints around the car in the car coordinate system
vector<vector<double>> GetLocalWaypoints()
{
	//List of waypoints in localized points
	vector<double> waypoints_x;
	vector<double> waypoints_y;

	//2D vector of waypoints (x & y) localized to car coordinates
	vector<vector<double>> waypoints;

	//Calc farest waypoint we want to consider by looking 6 waypoints behind
	int closest_waypoint = GetClosestWaypoint();
	int previous_waypoint = closest_waypoint - 6;

	//If it is negative, add total number of waypoints to get correct ID
	if (previous_waypoint < 0)
		previous_waypoint += waypoints_size;

	//Convert the waypoints into localized points
	for (int i = 0; i < 25; i++)
	{
		//Calculate the ID of the next waypoint
		int next_waypoint = (previous_waypoint + i) % waypoints_size;

		//Calculate world x and y position of that waypoint
		double waypoint_x = ourMap.x[next_waypoint] + (next_d_val * ourMap.dx[next_waypoint]);
		double waypoint_y = ourMap.y[next_waypoint] + (next_d_val * ourMap.dy[next_waypoint]);

		//Transform it into car coordinates
		vector<double> local_waypoint = GetLocalCoordinates(waypoint_x, waypoint_y);

		//Add transformed x and y coordinate from curr waypoint to the list
		waypoints_x.push_back(local_waypoint[0]);
		waypoints_y.push_back(local_waypoint[1]);
	}

	//Add the transformed coordinates from all waypoints to the path
	waypoints.push_back(waypoints_x);
	waypoints.push_back(waypoints_y);
	return waypoints;
}

//Computes lane tracking spline in local car coordinates
void LaneTrackingSpline(tk::spline &LaneSpline)
{
	//Get the surrounding waypoints in car coordinates
	vector<vector<double>> local_waypoints = GetLocalWaypoints();

	//Wrong way - turn car around by 180 degree and calculate again
	if (local_waypoints[0][0] > 0)
	{
		ourCar.yaw += 180;
		local_waypoints = GetLocalWaypoints();
	}

	//Add them to our lane tracking spline
	LaneSpline.set_points(local_waypoints[0], local_waypoints[1]);
}

//Computes velocity tracking spline for first cycle
void VelocityTrackingSpline(tk::spline &VelocitySpline)
{
	//Create vectors for time and distance+
	vector<double> time;
	vector<double> distance;

	//Add time data
	time.push_back(-1.0);
	time.push_back(double(12));
	time.push_back(double(20));
	time.push_back(double(40));
	time.push_back(double(80));

	//Add distance data
	distance.push_back(distance_increment * 0.01);
	distance.push_back(distance_increment * 0.10);
	distance.push_back(distance_increment * 0.15);
	distance.push_back(distance_increment * 0.25);
	distance.push_back(distance_increment * 0.30);
	distance.push_back(distance_increment * 0.50);

	//Add them to our velocity spline
	VelocitySpline.set_points(time, distance);
}

//Check which lane is the best according to the rankings, then init lane change
void ChangeLaneCheck(vector<vector<vector<double>>> &lane_lines, vector<vector<int>> &close_cars, vector<int> &lane_ranks)
{
	//This will be the number of our destination lane
	int destination_Lane = current_Lane;

	//Go through the three lanes
	for (int i = 0; i < 3; i++)
	{
		//Get lane number
		int lane_number = lane_ranks[i];

		//If best lane is current lane, we are done
		if (lane_number == current_Lane)
		{
			lane_change_votes = 0;
			break;
		}

		//Find out how mane lane shifts we need to reach the new lane and the direction
		int required_changed = lane_number - current_Lane;
		int direction = required_changed / abs(required_changed);

		//Flag to check if its feasible to change lanes
		bool change_feasible = true;

		//If we are too fast, multiple lane change causes too much jerk
		if ((ourCar.speed >= 40.0) && (abs(required_changed) > 1))
			change_feasible = false;

		//Check if there are no cars directly in front / behind us
		for (int j = 1; j <= abs(required_changed); j++)
		{
			//Check if we can change lane
			int temp_Lane = abs(current_Lane + (j * direction));
			int carId_back = close_cars[temp_Lane][0];
			int carId_front = close_cars[temp_Lane][1];

			//Check for cars behind us
			if (carId_back != -1)
			{
				//Calculate the cars distance and velocity
				double distance = abs(lane_lines[temp_Lane][carId_back][8]);
				double velocity = lane_lines[temp_Lane][carId_back][7];

				//If we are faster, we can have distance of 15, otherwise we need 30
				if (!(((velocity < distance_increment) && (distance > 15.0)) ||
					((velocity > distance_increment) && (distance > 30.0))))
				{
					change_feasible = false;
					break;
				}
			}

			//Check for cars in front of us
			if (carId_front != -1)
			{
				//Calculate the cars distance and velocity
				double distance = abs(lane_lines[temp_Lane][carId_front][8]);
				double velocity = lane_lines[temp_Lane][carId_back][7];

				//If we are faster, we can have distance of 15, otherwise we need 30
				if (!(((velocity > distance_increment) && (distance > 15.0)) ||
					((velocity < distance_increment) && (distance > 30.0))))
				{
					change_feasible = false;
					break;
				}
			}
		}

		//Check if all lanes are fine for a change
		if (change_feasible)
		{
			//Increase number of votes to change
			lane_change_votes++;

			//If count of 20 votes is reached, do a lane change
			if (lane_change_votes > 20)
			{
				//Activate lane change
				lane_change = true;

				//Set destination
				destination_Lane = lane_number;

				//Reset votes
				lane_change_votes = 0;
				string lane_name;

				//Do some outprint where we go
				if (lane_number == 0)
					lane_name = "Left";
				else if (lane_number == 1)
					lane_name = "Middle";
				else
					lane_name = "Right";
				cout << lane_name << " lane is faster, changing.." << endl;
			}
			break;
		}
	}

	//Update d value
	next_d_val = (destination_Lane * 4) + 2;

	//Update s value
	int car_id = close_cars[destination_Lane][1];

	//Full speed, because there is no car ahead
	if (car_id == -1)
	{
		distance_increment = 0.44;
		if(velocity_action != 0)
			cout << "Nothing in front, full speed.." << endl;
		velocity_action = 0;
	}

	//There is someone in front
	else
	{
		//Calculate distance and velocity of other car
		double distance = lane_lines[destination_Lane][car_id][8];
		double velocity = lane_lines[destination_Lane][car_id][7];

		//Distance is big enough, full speed
		if (distance > 40.0)
		{
			distance_increment = 0.44;
			if (velocity_action != 1)
				cout << "Cars far away, full speed.." << endl;
			velocity_action = 1;
		}

		//Try adapt to the speed of the car in front
		if ((distance <= 40.0) && (distance > 15.0))
		{
			if (distance_increment > velocity)
				distance_increment *= 0.7;
			else if(distance > 25.0)
				distance_increment = velocity + 0.02;
			else
				distance_increment = velocity - 0.02;
			if (velocity_action != 2)
				cout << "Adapt to front vehicle.." << endl;
			velocity_action = 2;
		}

		//If distance is too low, do brake stronger
		if((distance <= 15.0) && (distance > 10.0))
		{
			distance_increment *= 0.7;
			if (velocity_action != 3)
				cout << "Slowing down faster.." << endl;
			velocity_action = 3;
		}
		if ((distance <= 10.0) && (distance > 5.0))
		{
			distance_increment *= 0.5;
			if (velocity_action != 4)
				cout << "Braking hard.." << endl;
			velocity_action = 4;
		}		
		if(distance <= 5.0)
		{
			distance_increment *= 0.3;
			if (velocity_action != 5)
				cout << "Emergency brake.." << endl;
			velocity_action = 5;
		}
	}
}

//Identify the closest car ahead and behind of our car
void IdentClosestCars(vector<vector<vector<double>>> &lane_lines, vector<vector<int>> &close_cars)
{
	//Resize for our three lane lines
	close_cars.resize(3);

	//For each lane line
	for (int i = 0; i < 3; i++)
	{
		close_cars[i].push_back(-1);
		close_cars[i].push_back(-1);

		//Find closest car behind by maximum negative value
		for (int j = (lane_lines[i].size() - 1); j >= 0; j--)
		{
			if (lane_lines[i][j][8] < 0)
			{
				close_cars[i][0] = j;
				break;
			}
		}

		//Find closest car ahead by minimum positive value
		for (int j = 0; j < lane_lines[i].size(); j++)
		{
			if (lane_lines[i][j][8] > 0)
			{
				close_cars[i][1] = j;
				break;
			}
		}
	}
}

//Give each lane a rank based on the score metrics / cost functions
void LaneRanking(vector<vector<vector<double>>> &lane_lines, vector<vector<int>> &close_cars, vector<int> &lane_ranks)
{
	//Store the scores for our lanes
	vector<pair<double, int>> lane_scores;

	//Find the closest cars over all lines
	IdentClosestCars(lane_lines, close_cars);

	//Resize for three lane lines
	lane_ranks.resize(3);

	//Compute lane scores
	for (int i = 0; i < 3; i++)
	{
		//Score metrics
		double distance_score, velocity_score, lane_change_score;

		//Calculate lane change score, we prefer small changes
		lane_change_score = 1.0 * (1.0 - (fabs(i - current_Lane) / 2.0));

		//Calculate distance to ahead car score
		if (close_cars[i][1] == -1)
			distance_score = 3.0;
		else
			distance_score = 3.0 * (1.0 - ((100.0 - lane_lines[i][close_cars[i][1]][8]) / 100.0));

		//Calculate velocity cost score
		if (close_cars[i][1] == -1)
			velocity_score = 2.0;
		else
			velocity_score = 2.0 * (1.0 - ((0.88 - lane_lines[i][close_cars[i][1]][7]) / 0.88));

		//Summ all scores together and save them
		lane_scores.push_back(make_pair((lane_change_score + distance_score + velocity_score), i));
	}

	//Sort the lane scores
	sort(lane_scores.begin(), lane_scores.end());

	//Finally, get the ranks
	for (int i = 0; i < 3; i++)
		lane_ranks[i] = lane_scores[2 - i].second;
}

//This is the behavior planner
void BehaviorPlanner()
{
	//Store information about our three lane lines
	vector<vector<vector<double>>> lane_lines(3);

	//Assign vehicles from sensor fusion to the corresponding lane
	for (int i = 0; i < sensor_fusion_size; i++)
	{
		//Get vehicle object from sensor fusion
		vector<double> vehicle = sensor_fusion[i];

		//Add distance increment (velocity) of the car 
		sensor_fusion[i].push_back((Distance(0.0, 0.0, vehicle[3], vehicle[4]) * 0.02));

		//Add displacement from other car to our car
		sensor_fusion[i].push_back(vehicle[5] - ourCar.s);

		//Add cars to the coreesponding lane
		for (int j = 0; j < 3; j++)
		{
			if ((vehicle[6] >= ((j * 4) - 0.3)) && (vehicle[6] <= (((j + 1) * 4) + 0.3)))
				lane_lines[j].push_back(sensor_fusion[i]);
		}
	}

	//Sort lanes based on distance
	for (int i = 0; i < 3; i++)
	{
		sort(lane_lines[i].begin(), lane_lines[i].end(), [](vector<double>& a, vector<double>& b)
		{
			return a[8] < b[8];
		});
	}

	//Rank the lanes based on the close cars
	vector<int> lane_ranks;
	vector<vector<int>> close_cars;
	LaneRanking(lane_lines, close_cars, lane_ranks);

	//If there is a better lane, change to it
	ChangeLaneCheck(lane_lines, close_cars, lane_ranks);
}

//Plans a path based on the current information from the car, the previous path and sensor fusion
vector<vector<double>> PathPlanner(CAR &State, vector<vector<double>> &Previous_Path, vector<vector<double>> &Sensor_Fusion)
{
	//Path consisting of x and y variables for the car to drive
	vector<vector<double>> path;

	//Save to current values to our global variables
	memcpy(&ourCar, &State, sizeof(CAR));
	previous_path = Previous_Path;
	sensor_fusion = Sensor_Fusion;

	//Calculate sizes and current lane
	previous_path_size = previous_path[0].size();
	sensor_fusion_size = sensor_fusion.size();
	current_Lane = static_cast<int> (round(round(ourCar.d - 2.0) / 4.0));

	//Setup lane tracker
	tk::spline laneSpline;
	LaneTrackingSpline(laneSpline);

	//If we run it for the first time
	if (previous_path_size == 0)
	{
		//Store our local car points
		vector<double> car_x_points;
		vector<double> car_y_points;
		double car_x = 0.0;
		double car_y = 0.0;

		//Setup velocty tracker
		tk::spline velocitySpline;
		VelocityTrackingSpline(velocitySpline);

		//Form smoothed lane by using velocity and lane splines
		for (int i = 0; i < 40; i++)
		{
			car_x += velocitySpline(double(i));
			car_x_points.push_back(car_x);
			car_y_points.push_back(laneSpline(car_x));
		}

		//Smooth the velocities further
		car_x = 0.0;
		for (int i = 0; i < 40; i++)
		{
			//Calculate
			double distance = Distance(car_x, car_y, car_x_points[i], car_y_points[i]);

			//Calcualte intended speed
			double speed = velocitySpline(double(i));

			//If distance is different from intended speed, smoothen parth using heading
			if ((distance < (speed * 0.8) || (distance > speed)))
			{
				double heading = atan2((car_y_points[i] - car_y), (car_x_points[i] - car_x));
				car_x_points[i] = car_x + velocitySpline(double(i)) * cos(heading);
				car_y_points[i] = laneSpline(car_x_points[i]);
			}

			//Save velocity
			path_velocity.push_back(Distance(car_x, car_y, car_x_points[i], car_y_points[i]));

			//Update x and y coordinates
			car_x = car_x_points[i];
			car_y = car_y_points[i];
		}

		//Set current velocity as distance increment
		distance_increment = velocitySpline(40);

		//Convert local car points to world coordinates
		path = GetWorldPoints(car_x_points, car_y_points);
	}

	//For general path planning
	else
	{
		//Get the previous path in local points
		vector<vector<double>> previous_local_path = GetLocalPoints(previous_path[0], previous_path[1]);

		//Erase the portion that has already been completed
		path_velocity.erase(path_velocity.begin(), path_velocity.begin() + (40 - previous_path_size));
		path_history[0].erase(path_history[0].begin(), path_history[0].begin() + (40 - previous_path_size));
		path_history[1].erase(path_history[1].begin(), path_history[1].begin() + (40 - previous_path_size));

		//Check if changing lines is done
		if ((lane_change) && (ourCar.d >= (next_d_val - 0.15)) && (ourCar.d <= (next_d_val + 0.15)))
		{
			lane_change_votes = 0;
			lane_change = false;
			cout << "Lane change completed.." << endl;
		}

		//New lane tracker to include previous path
		tk::spline laneSpline_new;
		LaneTrackingSpline(laneSpline_new);

		//Create new spline including previous path
		vector<double> car_x_points;
		vector<double> car_y_points;
		for (int i = 0; i < previous_path_size; i++)
		{
			car_x_points.push_back(previous_local_path[0][i]);
			car_y_points.push_back(previous_local_path[1][i]);
		}

		//Add next points based on distance increment from velocity
		double car_x_next = previous_local_path[0][previous_path_size - 1] + 40;
		for (int i = 0; i < previous_path_size; i++)
		{
			car_x_points.push_back(car_x_next);
			car_y_points.push_back(laneSpline_new(car_x_next));
			car_x_next += distance_increment;
		}

		//Update the original lane spline
		laneSpline.set_points(car_x_points, car_y_points);

		//Store time and distance for our velocity tracker
		vector<double> time;
		vector<double> distance;

		//Add values from previous path
		for (int i = 0; i < previous_path_size; i++)
		{
			time.push_back(double(i));
			distance.push_back(path_velocity[i]);
		}
		time.push_back(double(200));
		distance.push_back(distance_increment);

		//Add the points to our velocity spline
		tk::spline velocitySpline;
		velocitySpline.set_points(time, distance);

		//Create local path by using the velocity and path splines
		for (int i = previous_path_size; i < 40; i++)
		{
			previous_local_path[0].push_back(previous_local_path[0][i - 1] + velocitySpline(double(i)));
			previous_local_path[1].push_back(laneSpline(previous_local_path[0][i]));
		}

		//Load first local coordinates from previous path
		double car_x = previous_local_path[0][0];
		double car_y = previous_local_path[1][0];

		//Create a smooth path
		for (int i = 0; i < 40; i++)
		{
			//Calculate distance
			double dist = Distance(car_x, car_y, previous_local_path[0][i], previous_local_path[1][i]);
			if (dist > velocitySpline(double(i)))
			{
				double heading = atan2((previous_local_path[1][i] - car_y), (previous_local_path[0][i] - car_x));
				previous_local_path[0][i] = car_x + (velocitySpline(double(i)) * cos(heading));
				previous_local_path[1][i] = laneSpline(previous_local_path[0][i]);
			}
			if (i >= previous_path_size)
			{
				path_velocity.push_back(Distance(car_x, car_y, previous_local_path[0][i], previous_local_path[1][i]));
			}

			//Load new x and y variables from previous path
			car_x = previous_local_path[0][i];
			car_y = previous_local_path[1][i];
		}

		//Conert local points to world poibts
		vector<vector<double>> world_points = GetWorldPoints(previous_local_path[0], previous_local_path[1]);

		//Put them into the path by starting with the previous path
		path = previous_path;
		for (int i = previous_path_size; i < world_points[0].size(); i++)
		{
			path[0].push_back(world_points[0][i]);
			path[1].push_back(world_points[1][i]);
		}
	}

	//Save the path history
	path_history.clear();
	path_history = path;

	//Run behavior planner of not lane changing at the moment
	if (!lane_change)
		BehaviorPlanner();

	//Return our planned path
	return path;
}

//Checks if the SocketIO event has JSON data
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	}
	if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

//Read a map from file
void ReadMap()
{
	string map_file = "../data/highway_map.csv";
	cout << "Try to read map from file:" << map_file << endl;
	ifstream in_map_(map_file.c_str(), ifstream::in);

	//Go through every line and add the waypoint to the map
	string line;
	while (getline(in_map_, line))
	{
		istringstream iss(line);
		double x;
		double y;
		double s;
		double d_x;
		double d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		ourMap.x.push_back(x);
		ourMap.y.push_back(y);
		ourMap.s.push_back(s);
		ourMap.dx.push_back(d_x);
		ourMap.dy.push_back(d_y);
	}

	//Save the number of waypoints
	waypoints_size = ourMap.x.size();
	cout << "Map loaded with " << to_string(waypoints_size) << " waypoints!" << endl;
}

//Main entry point
int main(int ac, char* av[])
{
	//Our websocket
	uWS::Hub h;

	//Show start message
	cout << "*** Path Planning Project by Max Ritter ***" << endl;

	//Read in the map file
	ReadMap();

	//This is called, whenever we get new data from the simulator
	h.onMessage([](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode)
	{
		//"42" at the start of the message means there's a websocket message event.
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(data);
			if (s != "") {
				auto package = nlohmann::json::parse(s);
				string event = package[0].get<string>();

				//This is the event we deal with
				if (event == "telemetry")
				{
					//Get the current car state from the simulator
					CAR car_state;
					car_state.x = package[1]["x"];
					car_state.y = package[1]["y"];
					car_state.s = package[1]["s"];
					car_state.d = package[1]["d"];
					car_state.yaw = package[1]["yaw"];
					car_state.speed = package[1]["speed"];

					//List of all cars on the same side of the road from sensor fusion
					vector<vector<double>> sensor_fusion = package[1]["sensor_fusion"];

					//Get the previous path from the simulator
					vector<vector<double>> previous_path =
					{
						package[1]["previous_path_x"],
						package[1]["previous_path_y"]
					};

					//Get the list of our points to drive from the path planner
					vector<vector<double>> path_to_drive = PathPlanner(car_state, previous_path, sensor_fusion);

					//Create a JSON package for our message
					nlohmann::json msgJson;

					//Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					msgJson["next_x"] = path_to_drive[0];
					msgJson["next_y"] = path_to_drive[1];

					//Construct message with ID 42
					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//Small delay of 100ms, otherwise the car will leave the road sometimes
					this_thread::sleep_for(chrono::milliseconds(100));

					//Send it to the simulator
					ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			//Manual diving mode
			else {
				string msg = "42[\"manual\",{}]";
				ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
		size_t, size_t) {
		const string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		}
		else {
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		cout << "Connected" << endl;
		cout << "Autopilot is now driving ;)" << endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
		char *message, size_t length) {
		ws->close();
		cout << "Disconnected" << endl;
	});

	if (h.listen(4567))
		cout << "Listening to port 4567" << endl;
	else 
		cerr << "Failed to listen to port 4567" << endl;
	h.run();
	return -1;
}