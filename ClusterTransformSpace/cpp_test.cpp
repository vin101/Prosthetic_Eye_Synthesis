#include <stdio.h>
#include <stdlib.h>
#include "MeanShift.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace std;

vector<vector<double> > load_points(const char *filename) {
    vector<vector<double> > points;
    FILE *fp = fopen(filename, "r");
    char line[50];
    while (fgets(line, sizeof(line), fp) != NULL) {
        double x, y, z;
        char *x_str = line;
        char *y_str = line;
	    char *z_str = line; 
        bool flag = false;
	    while (*z_str != '\0') {
            if (*y_str == ',') {
                *y_str++;
                flag = true;
                ++z_str;
            }
    	    if (flag && *z_str == ',') {
                *z_str++;
    		    x = atof(x_str);
                y = atof(y_str);
    		    z = atof(z_str);
                vector<double> point;
                point.push_back(x);
                point.push_back(y);
    	        point.push_back(z);
                points.push_back(point);
                break;
            }
    	    if(!flag)
                ++y_str;
    	    ++z_str;
        }
    }
    fclose(fp);
    return points;
}

void print_points(vector<vector<double> > points){
    for(int i=0; i<points.size(); i++){
        for(int dim = 0; dim<points[i].size(); dim++) {
            printf("%f ", points[i][dim]);
        }
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    auto same=[](vector<double> a, vector<double> b){return (
							(
							pow(a[0]-b[0],2)+
							pow(a[1]-b[1],2)+
							pow(a[2]-b[2],2)
							)
							<0.005?true:false);};
    MeanShift *msp = new MeanShift();
    double kernel_bandwidth = 41; //change this to 41 for Vinayak face ------------------------------------------------------------------
    //vector<vector<double> > points = load_points("../ProstheticEye/transformspace.csv");
    vector<vector<double> > points = load_points("transformspace.csv");
    
    vector<vector<double> > points_backup = points;
    vector<vector<double> > shifted_points = msp->cluster(points, kernel_bandwidth);

    ofstream myfile;
    myfile.open("../Clusters/output_text.txt");
    
    for(int i=0; i<shifted_points.size(); i++){
        for(int dim = 0; dim<shifted_points[i].size(); dim++){
            printf("%f ", shifted_points[i][dim]);
        }
	    myfile<<shifted_points[i][0]<<" "<<shifted_points[i][1]<<" "<<shifted_points[i][2]<<endl;
        printf("\n");
    }
    
    
    // finding mode 
	
    
    vector<vector<double> > repeated_points;
    repeated_points.push_back(shifted_points[0]);
    repeated_points[0].push_back(1.0); //count
    repeated_points[0].push_back(0.0); //index - for extraction of the original point later

    bool found = false;
    int k;
    //iterating through the o/p points to generate frequencies of unique points
    for(int j=1; j<shifted_points.size();j++){	
	   found = false;
	   for (k=0; k<repeated_points.size();k++){
	       if(same(shifted_points[j],repeated_points[k])){
		        repeated_points[k][3]+=1.0;
		        repeated_points[k].push_back((double)j); //original index of the point
		        found = true;
            }
        }
	    if(!found){
		   //cout<<"adding new entry"<<endl;
		   repeated_points.push_back(shifted_points[j]);
		   repeated_points[k].push_back(1.0);
		   repeated_points[k].push_back((double)j);
	       //cout<<"shifted point:"<<shifted_points[j][0]<<","<<shifted_points[j][1]<<","<<shifted_points[j][2]<<" repeat:"<<repeated_points[k][0]<<","<<repeated_points[k][1]<<","<<repeated_points[k][2]<<endl;
	    }
    }
//printing the repeated points table with frequency

    for(int i = 0; i<repeated_points.size(); i++) {
	   printf("Point: %f %f %f %d .\n",repeated_points[i][0],repeated_points[i][1],repeated_points[i][2],(int)repeated_points[i][3]);
    }
//Printing clusters in separate files.
    int clusterno = 0;
    for(int i = 0; i< repeated_points.size();i++){
        if(repeated_points[i].size()<20){
            continue;
        }
	    stringstream ss;
	    ss<<"../Clusters/Cluster"<<clusterno++<<".txt";
	    ofstream outputfile;
	    outputfile.open(ss.str());
	    for(int j = 4;j<repeated_points[i].size();j++){
	       int index = (int)repeated_points[i][j];
	       outputfile<<points_backup[index][0]<<","<<points_backup[index][1]<<","<<points_backup[index][2]<<endl;
	    }
	    outputfile.close();
    }
//-----------------------------------

//--------------------------------
    double max=0.0;
    int maxindex = 0;
    //Finding the most repeated point 
    for( int l=0;l<repeated_points.size();l++)
    {
         if(max<repeated_points[l][3])
         { 
             max=repeated_points[l][3];
	         maxindex = l;
         }
         if(repeated_points[l][3]>50){
            printf("%d : %f %f %f %f .\n",l,repeated_points[l][0],repeated_points[l][1],repeated_points[l][2],repeated_points[l][3]);
         }
    }
     
    printf("Max Frequency point at index %d : %f %f %f %f .\n",maxindex,repeated_points[maxindex][0],repeated_points[maxindex][1],repeated_points[maxindex][2],repeated_points[maxindex][3]);
 
    myfile.close();

//writing number of clusters to a file to read for plotting 

    ofstream opfile;
    opfile.open("../Clusters/number_of_scatter_points.txt");
    opfile<<clusterno;
    opfile.close();

            
    return 0;
}
