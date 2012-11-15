#include <iostream>
#include <string>

#include "Algorithm.h"

using namespace std;

int main()
{
	Coord2D point;
	Obstacle obstacle;
	testAlgorithm algorithm;

	while (true) {
		string input = "";
		cout << "Point X: ";
		cin >> input;
		point.x = atof(input.c_str());
		cout << "Point Y: ";
		cin >> input;
		point.y = atof(input.c_str());

		cout << "Obstacle X: ";
		cin >> input;
		obstacle.x = atof(input.c_str());
		cout << "Obstacle Y: ";
		cin >> input;
		obstacle.y = atof(input.c_str());
		cout << "Obstacle Rad: ";
		cin >> input;
		obstacle.rad = atof(input.c_str());
		
		vector<Coord2D> test = algorithm.getTangentPointOfObstacle(obstacle,point);
		for (int i = 0; i < test.size(); i++) {
			cout << "Test X: " << test[i].x << endl;
			cout << "Test Y: " << test[i].y << endl;
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