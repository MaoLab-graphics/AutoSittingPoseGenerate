// Main.cpp

#include "Viewer.h"
#include "Functions.h"
#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;

int main(int argc, char** argv)
{
	/*
	Here we provide 3 human models and 8 chair models.You can change the parameter personNum to choose the different human model while change 
	the parameter chairNum to choose the different chair model.
	*/
	int personNum;
	int chairNum;
	cout << "Please input the human model number:";
	cin >> personNum;
	cout << "Please input the chair model number:";
	cin >> chairNum;
	if (personNum < 1 || chairNum < 1)
	//if (personNum < 1 || personNum > 3 || chairNum < 1 || personNum > 8)
		cout << "Please choose the right human model or chair model" << endl;
	else
	{
		ManSeg::deform_to_sitting_pose(personNum, chairNum);
	}

	system("pause");
	return 0;
}