#include <iostream>
#include <string>

#include "Algorithm.h"

using namespace std;

int main()
{
	Robot point;
	vector<Obstacle> obstacles;
	vector<Ball> balls;
	CAlgorithm algorithm;

	while (true) {
		string input = "";
		cout << "Point X: ";
		cin >> input;
		point.x = atof(input.c_str());
		cout << "Point Y: ";
		cin >> input;
		point.y = atof(input.c_str());

		cout << "Obstacle Sizes: ";
		cin >> input;
		obstacles.clear();
		obstacles.resize(atoi(input.c_str()));
		for (int i = 0; i < obstacles.size(); i++) {
			cout << "Obstacle X: ";
			cin >> input;
			obstacles[i].x = atof(input.c_str());
			cout << "Obstacle Y: ";
			cin >> input;
			obstacles[i].y = atof(input.c_str());
		}

		cout << "Num of Balls: ";
		cin >> input;
		balls.clear();
		balls.resize(atoi(input.c_str()));
		for (int i = 0; i < balls.size(); i++) {
			cout << "Ball X: ";
			cin >> input;
			balls[i].x = atof(input.c_str());
			cout << "Ball Y: ";
			cin >> input;
			balls[i].y = atof(input.c_str());
		}
		
		algorithm = CAlgorithm(obstacles);
		algorithm.analyzeField(point, balls);
		//vector<Coord2D> test = algorithm.getTangentPointOfObstacle(obstacle, point);
		vector<Coord2D> test = algorithm.getClosestPath();
		for (int i = 0; i < test.size(); i++) {
			cout << test[i].x << " " << test[i].y << endl;
		}
		

		cout << endl;
	}
	/*
	vector<Obstacle> obstacles;
	string input = "";
	int obstacleIndex = 0;
	for (int i = 0; ; i++) {
		Obstacle tempObstacles;

		cout << "Obstacle " << i << " - X: ";
		cin >> input;
		if (input == "START")
			break;
		tempObstacles.x = atoi(input.c_str());
		
		cout << "Obstacle " << i << " - Y: ";
		cin >> input;
		if (input == "START")
			break;
		tempObstacles.y = atoi(input.c_str());

		obstacles.push_back(tempObstacles);
	}

	Algorithm algorithm(obstacles);

	obstacles = algorithm.getObstacles();
	for (int i = 0; i < obstacles.size(); i++) {
		cout << "Obstacle " << i << ": X-" << obstacles[i].x << " Y-" << obstacles[i].y << " Rad-" << obstacles[i].rad << endl;
	}
	*/
	system("pause");
	return 0;
}