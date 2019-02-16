#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;
//Contains x position, y position, and probability of position
struct particle{
	float x,y,p;
};

bool compareParticle(particle* a, particle* b){
	return (a->p < b->p);
}

//Represents the pixel to reality scale
float scalex;
float scaley;
float fieldofvision = 120;

//Change in x from last position
float dx = 0;
//Change in y from last position
float dy = 0;
//Current robot angle orientation
float robang = 0;

//Represents variance of sensor (Measure of inaccuracy)
float sensorvariance = 1;

//Distance that the robot measured
float robotread = 0;

//Number of particles being used
int num;

//Image of the map outline
Mat img;
//Image to be displayed (Shows the robot and estimation)
Mat displayrob;

//List of all the particles
vector<particle*> *part = new vector<particle*>;

//Space between first set of particles in pixels
int space = 100;
//Find the max probability among the particles
int indMaxProb(){
	float max = 0;
	int ind = 0;
	for (int i = 0; i < part->size(); ++i)
	{
		if (part->at(i)->p > max)
		{
			ind = i;
			max = part->at(i)->p;
		}
	}
	return ind;
}


//Makes sure that probabilities add up to 1. If not, it multiplies each probability by a constant factor to do so
void rebalance(){
	float totalprob = 0;
	for (int i = 0; i < part->size(); ++i)
	{
		//Eliminates any particle off the grid
		if (part->at(i)->x < 0 || part->at(i)->x >= img.cols || part->at(i)->y < 0 || part->at(i)->y >= img.rows)
		{
			part->erase(part->begin()+i);
			i--;
			continue;
		}
		totalprob+=part->at(i)->p;
	}
	float mult = 1/totalprob;
	for (int i = 0; i < part->size(); ++i)
	{
		part->at(i)->p *= mult;
	}
}

//Convert degrees to radians sine robot angle is in degrees
float degToRad(float ang){
	return ang*M_PI/180;
}

//Makes the original set of particles spaced out by "space" pixels.
void makeParticle(){
	num = img.cols/space*img.rows/space;
	for (int i = 0; i < img.cols/space; ++i)
	{
		for (int j = 0; j < img.rows/space; ++j)
		{
			particle *add = new particle();
			add->x = space*i;
			add->y = space*j;
			add->p = (float)(1);
			part->push_back(add);
			//delete add;
		}
	}
}

//Checks if the given pixel is a wall based on the criteria of being a wall
bool isWall(Vec3b pixel){
	//A wall is defined as a pixel that is black enough
	if (pixel.val[0] < 20 && pixel.val[0] == pixel.val[1] && pixel.val[1] == pixel.val[2] && pixel.val[0] == pixel.val[2])
	{
		return true;
	}
	return false;
}

//Find the distance from a particle to a wall. Use the orientation to find the slope of a line passing through a point.
//Utilize field of vision as 120 different possible orientations differing by one degree
float distToWall(particle* p){

	//Optimize with a visited array so we don't revisit points we already processed
	bool visited[img.cols][img.rows]; 
     for(int i = 0; i < img.cols; i++){
         for(int j = 0; j < img.rows; j++){
             visited[i][j]=false;
         }
     }
    float dtheta = 1;
    //min will store the minimum distance to an object
    float min = 100000000;
    float dist = -1;
    //Process every orientation to process a field of vision
    for (int i = (robang-fieldofvision/2+360); i < (robang+fieldofvision/2+360); i+=dtheta)
    {
		float changex;
		float changey;
		float refangle = fmod(i,360);
		float xinit = -1;
		float yinit = -1;
		//If increasing x causes the least change in y, increase x and find the y value it corresponds to. Otherwise increase y, to find x
		if ((refangle <= 225 && refangle >= 135) || (refangle <= 45 || refangle >= 315)){
			//tan of an angle is dy/dx
			changey = tan(degToRad(refangle));

			//If the robot points to the right, x should be increased, otherwise it should be decreased
			if ((refangle <= 45 || refangle >= 315))
			{
				changex = 1;
				xinit = img.cols-1;
				yinit = changey*(xinit-p->x)+p->y;
			}else{
				changex = -1;
				xinit = 0;
				yinit = changey*(xinit-p->x)+p->y;
			}
		}
		else{
			//If the robot is pointed vertically, its x should not change. Slope of a vertical line would crash without this
			if (refangle == 270 || refangle == 90)
			{
				changex = 0;
			}else{
				//cot is dx/dy
				changex = 1/tan(degToRad(refangle));
			}
			//If the robot is facing upwards, increase y, otherwise decrease it
			if (refangle < 180)
			{
				changey = 1;
				yinit = img.rows-1;
				xinit = p->x+changex*(yinit-p->y);
			}else{
				changey = -1;
				yinit = 0;
				xinit = p->x+changex*(yinit-p->y);
			}
		}
		int factor = 1;
		//Travel along the line until we either find a solution, or run off the grid
		int lastx = (int)(xinit);
		while(xinit-factor*changex != p->x && yinit-factor*changey != p->y){
			//Get the pixel we are at
			int curx = (int)(xinit-factor*changex);
			//Make sure our orientation is going from a far dist to a close one
			if (abs(curx-p->x) > abs(lastx-p->x))
			{
				break;
			}
			lastx = curx;
			int cury = (int)(yinit-factor*changey);
			//Check to see if our point is in bounds
			if (curx < 0)
			{
				factor++;
			 	continue;
			}
			if (curx >= img.cols)
			{
				factor++;
			 	continue;
			}
			if (cury < 0)
			{
				factor++;
			 	continue;
			}
			
			if (cury >= img.rows)
			{
				factor++;
			 	continue;
			}
			if (visited[curx][cury] == true)
			{
				break;
			}
			visited[curx][cury] = true;
			//Grab the pixel we are focusing on
			Vec3b color = img.at<Vec3b>(Point((int)(curx),(int)(cury)));
			if (isWall(color))
			{
				//Change the distance from pixel distance to real distance by multiplying by scale
				if (dist == -1)
				{
				 	dist = pow(pow((curx-p->x)*scalex,2)+pow((cury-p->y)*scaley,2),0.5);
				}
				if(pow(pow((curx-p->x)*scalex,2)+pow((cury-p->y)*scaley,2),0.5) < dist){
					dist = pow(pow((curx-p->x)*scalex,2)+pow((cury-p->y)*scaley,2),0.5);
				}
			}
			factor++;
		}
    }
	//If we found a solution, return the solution
	if (dist != -1)
	{
		return dist;
	}else{
		//If we did not find a solution, it is a particle that should be removed, so return a value that will get it removed
		return 1000000;
	}
}

//Use the Gaussian distribution formula, with a variance of sensorvariance
//P(S | L) = (1/(2*b*pi)*e^((-0.5*(z-zexp)^2)/b))
//b = variance, z = what particle distance was, zexp = what the real robot read
float Psgivenl(particle* p){
	return (pow(M_E,-0.5*(pow(distToWall(p)-robotread,2))/(sensorvariance)))/(pow(2*M_PI*sensorvariance,0.5));
}

//Edit the probabilities based on the sensor changes
float sensorEdit(){
	for (int i = 0; i < part->size(); ++i)
	{
		part->at(i)->p *= Psgivenl(part->at(i));
	}
}

//Move all the particles based on the real robot's movement
//Since DVL provides very accurate location, no need to account for position uncertainty
float propagate(){
	for (int i = 0; i < part->size(); ++i)
	{
		part->at(i)->x += dx;
		part->at(i)->y += dy;
		//If the particle is off the grid, remove it
		if (part->at(i)->x >= img.cols || part->at(i)->x < 0 || part->at(i)->y >= img.rows || part->at(i)->y < 0)
		{
			part->erase(part->begin()+i);
			i--;
		}
	}
}


//TESTING PURPOSES ~ The robot randomly begins somewhere within the map
float testx = rand()%400+img.cols/2-200;
float testy = rand()%400+img.rows/2-200-testy;

//Update the robot's position
void updateRobot(){
	//CHANGE ROBOT INFORMATION TO ACTUALLY BE ACCURATE. Currently, everything is randomized
	dx = rand()%400+img.cols/2-200-testx;
	dy = rand()%400+img.rows/2-200-testy;
	robang = rand()%360;
	testx += dx;
	testy += dy;
	particle *real = new particle();
	real->x = testx;real->y = testy;real->p = 5;
	robotread = distToWall(real);
	delete real;
}

//Display where the predicted robot and real robot is with a square of (2*extend+1) side length
//Red color for prediction, Green for real robot
void displayImg(int extend, particle* p){

	//Predicted Robot
	for (int i = p->x-extend; i <= p->x+extend; ++i)
	{
		for (int j = p->y-extend; j <= p->y+extend; ++j)
		{
			//Only draw where a pixel is
			if (i < 0 || i >= img.cols || j < 0 || j >= img.rows)
			{
				continue;
			}
			Vec3b color = displayrob.at<Vec3b>(Point(i,j));
			color.val[0] = 0;
			color.val[1] = 0;
			color.val[2] = 255;
			displayrob.at<Vec3b>(Point(i,j)) = color;
		}
	}

	//Real Robot
	for (int i = testx-extend; i <= testx+extend; ++i)
	{
		for (int j = testy-extend; j <= testy+extend; ++j)
		{
			//Only draw where a pixel is
			if (i < 0 || i >= img.cols || j < 0 || j >= img.rows)
			{
				continue;
			}
			Vec3b color = displayrob.at<Vec3b>(Point(i,j));
			color.val[0] = 0;
			color.val[1] = 255;
			color.val[2] = 0;
			displayrob.at<Vec3b>(Point(i,j)) = color;
		}
	}
	//Display the image with the robots drawn
	namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", displayrob);
    waitKey(5);
    
}

//Make a particle that is close to a particle p
particle* closePoint(particle* p){
	particle *ret = new particle();
	ret->p = p->p;
	//The new point's x will be within space/2 pixels from the given particle's x
	int difx = (rand()%space/2)+1;

	//Randomly add or subtract the difference
	if (rand()%2 == 0)
	{
		ret->x = p->x-difx;
	}else{
		ret->x = p->x+difx;
	}

	//The new point's y will be within space/2 pixels from the given particle's y
	int dify = (rand()%space/2)+1;

	//Randomly add or subtract the difference
	if (rand()%2 == 0)
	{
		ret->y = p->y-dify;
	}else{
		ret->y = p->y+dify;
	}

	return ret;
}

//If it is possible to add more points, add more around the high probability points
void resample(){
	//Sort the particles we have so we can weed out the low probability ones
	sort(part->begin(), part->end(),compareParticle);

	//The prefix sum array will be ued so we can pick a random number and find a particle to match it
	float pref [part->size()]= {};
	float cur = 0;
	for (int i = 0; i < part->size(); ++i)
	{
		cur += part->at(i)->p;
		pref[i] = cur;
	}

	//This vector will store the new particles we have after resampling
	vector<particle*> *resample = new vector<particle*>;

	//Take 80% of the highest probabilities to include in the resample
	for (int i = part->size()-1; i >= .2*part->size(); --i)
	{
		resample->push_back(part->at(i));
	}
	//Use two pointers to become O(plog(p)) instead of O(p^2)
	vector<float> *randProbs = new vector<float>;
	for (int i = 0; i < num-resample->size(); ++i)
	{
		randProbs->push_back((float)(rand()%((int)(pref[part->size()-1]*10000)))*1.0/10000);	
	}
	sort(randProbs->begin(),randProbs->end());
	int prefind = 0;

	//The remaining 20% of the new particles come from the resampling
	for (int i = 0; i < randProbs->size(); ++i)
	{
		while(randProbs->at(i) > pref[prefind]){
			prefind++;
		}
		resample->push_back(closePoint(part->at(prefind)));
	}
	part = NULL;
	//Set the particle array to the newly sampled one
	part = resample;

}
//First argument is picture
//Second argument is the real length of the map
//Third argument is the real height of the map
int main(int argc, char** argv){

	//Read the map
	img = imread(argv[1],1);
	displayrob = imread(argv[1],1);
	testx = img.cols/2;
	testy = img.rows/2;
	scalex = 1.0*atoi(argv[2])/img.cols;
	scaley = 1.0*atoi(argv[3])/img.rows;
	makeParticle();
	for (int i = 0; i < 100; ++i)
	{
		updateRobot();//Will be O(1) if getting the info. With test data, O(l*w)
		propagate(); //O(p)
		sensorEdit();//O(pwl) where l and w are the length and width of the image in pixels
		rebalance();//O(p)
		int foundind = indMaxProb();//O(p)
		try{
			displayImg(7,part->at(foundind));//Basically O(1)
		}catch(...){
			cout<<"No Image Found\n";
			exit(EXIT_FAILURE);
		}
		displayrob = imread(argv[1],1);
		cout<<"Tracking: ("<<part->at(foundind)->x<<","<<part->at(foundind)->y<<") Robot: ("<<testx<<","<<testy<<")\n";
		resample();//O(plogp)
		rebalance();//O(p)
	}
	for (int i = 0; i < part->size(); ++i)
	{
		delete part->at(i);
	}
	delete part;
}