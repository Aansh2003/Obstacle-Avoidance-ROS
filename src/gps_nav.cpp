#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <cmath>
#include <vector>


const double PI = 3.1415;
const double degtorad = PI/180;

class Orientation{
    public:
        double qx;
        double qy;
        double qz;
        double qw;
        void OrientationCallback(const sensor_msgs::Imu::ConstPtr& quar);
};
class GPS{
    public:
        double Latitude;
        double Longitude;

        void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps);
};

class Obstacle{
    public:
        float range_center[180];
        float range_left[270];
        float range_right[270];
        float lowest;
        float full_range[720];
        float valueR,valueRB,valueL,valueLB;
        bool IsObstacleDetected = 0;
        bool IsGoRight = 0;
        bool IsWallFollow = 0;
        int factor = 0;
        
        void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& lsd);
        void RightOrLeft();
        void Obstacledetect();
        
};

void GPS::GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps){
    Latitude = gps->latitude;//Functional , pulling values from GPS
    Longitude = gps->longitude;
}

void Orientation::OrientationCallback(const sensor_msgs::Imu::ConstPtr& quar){
    qx = quar->orientation.x; //Functional, pulling values from IMU
    qy = quar->orientation.y;
    qz = quar->orientation.z;
    qw = quar->orientation.w;
}

void Obstacle::Obstacledetect(){//Detects obtsacles in a 120 degree segment , warns if detected.
    for(int j=0;j<180;j++){
        if(range_center[j]<1.5 && range_center[j]>0.1){
            IsWallFollow=1;
            IsObstacleDetected = 1;
        }
    }
}
void Obstacle::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& lsd){
    for(int i=0;i<720;i++){ //Formats laserscan data
        full_range[719-i]=lsd->ranges[i];
        if(full_range[719-i]==INFINITY){
            full_range[719-i]=30;
        }
        if(i<240){
            range_left[i]=lsd->ranges[i];
            if(range_left[i]==INFINITY){
                range_left[i]=30;
            }
        }
        else if(i>=270 && i<450){
            range_center[i-270]=lsd->ranges[i];
            if(range_center[i-270]==INFINITY){
                range_center[i-270]=30;
            }
        }
        else{
            range_right[i-450]=lsd->ranges[i];
            if(range_right[i-450]==INFINITY){
                range_right[i-450]=30;
            }
        }
        valueR = range_left[120];
        valueL = range_right[120];
    }
}
void Obstacle::RightOrLeft(){//Decides which side to be turned
    double LowObstaclesL=35,LowObstaclesR=35;
    int i,num1=0,num2=0;
    for(i=0;i<90;i++){
        if(range_center[i]>0.1 && range_center[i]<=30){
            if(range_center[i]<LowObstaclesR){
                LowObstaclesR=range_center[i];
            }
        }
        if(range_center[179-i]>0.1 && range_center[179-i]<=30){
            if(range_center[179-i]<LowObstaclesL){
                LowObstaclesL=range_center[179-i];
            }
        }
    }
    for(i=0;i<150;i++){
        if(range_left[120+i]>0.1 && range_left[120+i]<=30){
            if(range_left[120+i]<LowObstaclesR){
                LowObstaclesR=range_left[i];
            }
        }
        if(range_right[120+i]>0.1 && range_center[120+i]<=30){
            if(range_right[120+i]<LowObstaclesL){
                LowObstaclesL=range_right[120+i];
            }
        }
    }
    if(LowObstaclesL<LowObstaclesR){
        IsGoRight = 1;
    }
}


double haversine(double LatA,double LongA,double LatB,double LongB){//Functional , returns in metres
    double dlatA=LatA*(PI/180);
    double dlongA=LongA*(PI/180);
    double dlatB=LatB*(PI/180);
    double dlongB=LongB*(PI/180);
    double dLong=dlongA-dlongB;
    double dLat=dlatA-dlatB;
    double aHarv= std::pow(std::sin(dLat/2.0),2.0)+std::cos(dlatA)*std::cos(dlatB)*std::pow(std::sin(dLong/2),2);
    double cHarv=2*std::atan2(sqrt(aHarv),sqrt(1.0-aHarv));
    const double earth=6378137;
    double distance=earth*cHarv;
    return distance;
}

double headingOrientation(double x,double y,double z,double w){//Functional , returns in radians
    double heading;
    heading = std::atan2(2*(z*w+x*y), -1+2*(w*w+x+x));
    return heading;
}


double bearingAngle(double lat1,double long1,double lat2,double long2){//Functional , returns in radians , in reference to true North
    double bearingAngle;
    double x = std::cos(lat2)*std::sin(long2-long1);
    double y = std::cos(lat1) * std::sin(lat2) - std::sin(lat1)* std::cos(lat2) * std::cos(long2-long1);
    bearingAngle = std::atan2(x,y);
    return bearingAngle;
}


int main(int argc, char **argv){
    double Lat,Long,distance,requiredAngle,linear,angular,InLat,InLong,angleleft,distTrav,dist2,linAv,angAv;
    float currentAngle;
    int angleLidar;
    bool idk,IsEnd = 0,Obst;
    int Cold_Start=0,ResetATT=0,freq=100,c=1,d=1,count4=0;
    ros::init(argc, argv, "node");
    GPS gps;
    Orientation orien;
    Obstacle Lsd;
    linear = 0.6;
    angular = 2;
    linAv = 0.3;
    angAv = 1;
    std::cout<<"Enter new co-ordinates"<<std::endl;
    std::cin>>Lat>>Long;
    ros::NodeHandle n;
    //Subscribers
    ros::Subscriber sub_GPS = n.subscribe<sensor_msgs::NavSatFix>("/gps/fix",100,&GPS::GPSCallback,&gps);
    ros::Subscriber sub_IMU = n.subscribe<sensor_msgs::Imu>("/imu/data" ,100,&Orientation::OrientationCallback,&orien); 
    ros::Subscriber sub_Laser = n.subscribe<sensor_msgs::LaserScan>("/scan" ,100,&Obstacle::LaserCallback,&Lsd); 
    //Publishers
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel",100); 
    geometry_msgs::Twist msg;
    ros::Duration(2).sleep();
    ros::Rate r(freq);
    while(ros::ok()){  
        ros::spinOnce();
        Lsd.Obstacledetect();
        if(IsEnd == 1){
            break;
        }
        
        if(Cold_Start>200){ //Obstacle avoiding state
            if(Lsd.IsObstacleDetected==1 && haversine(gps.Latitude,gps.Longitude,Lat,Long)>0.5){
                ResetATT=0;
                if(Lsd.IsGoRight==1){
                    d = -1;
                    if(Lsd.valueL<30){
                        Lsd.IsObstacleDetected=0;
                    }
                }
                else{
                    d=1;
                    if(Lsd.valueR<30){
                        Lsd.IsObstacleDetected=0;
                    }
                }
                msg.angular.z = d*angAv;
                msg.linear.x = 0;
                std::cout<<"State = Obstacle Avoidance"<<std::endl;
            }
            else if(Lsd.IsWallFollow==1){ //Wall follow state
                ResetATT=0;
                msg.linear.x = linear;
                msg.angular.z = 0;
                if(Lsd.IsGoRight==1){
                    if(Lsd.valueL == 30 || Lsd.valueL > haversine(gps.Latitude,gps.Longitude,Lat,Long)){
                        Lsd.IsWallFollow=0;
                    }
                }
                else if(Lsd.IsGoRight==0){
                    if(Lsd.valueR == 30 || Lsd.valueR > haversine(gps.Latitude,gps.Longitude,Lat,Long)){
                        Lsd.IsWallFollow=0;
                    }
                }
                std::cout<<"State = Wall Follow"<<std::endl;
            }
            else{ //ATT State
                currentAngle = headingOrientation(orien.qx,orien.qy,orien.qz,orien.qw);
                if(ResetATT==0){//Gives values once
                    InLat = gps.Latitude;
                    InLong = gps.Longitude;
                    distance = haversine(InLat,InLong,Lat,Long);
                    requiredAngle =-PI/2-1*(bearingAngle(InLat*degtorad,InLong*degtorad,Lat*degtorad,Long*degtorad)-PI/2);//Changing reference from true north to 
                    if(requiredAngle>3.14159){//Incase values go greater than limit
                        requiredAngle -= 2*3.14159;
                    }
                    if(requiredAngle<-3.14159){
                        requiredAngle += 2*3.14159;
                    }
                    
                    std::cout<<"Initial Coordinates::"<<gps.Latitude<<" "<<gps.Longitude<<std::endl;
                    std::cout<<"Distance is:"<<distance<<std::endl;
                    std::cout<<"Bearing angle:"<<requiredAngle<<std::endl;
                    std::cout<<"Current angle is:"<<currentAngle<<std::endl;
                    ResetATT++;
                }
                angleleft = requiredAngle-currentAngle;
                if(angleleft>3.14159){
                    angleleft -= 2*3.14159;
                }
                if(angleleft<-3.14159){
                    angleleft += 2*3.14159;
                }
                if(angleleft<0){//To cover for negative values , gives fastest turn direction
                    c = -1;
                }
                std::cout<<"Distance to destination: "<<haversine(gps.Latitude,gps.Longitude,Lat,Long)<<std::endl;
                angleleft = requiredAngle-currentAngle;
                if(angleleft>3.14159){//Incase values go greater than limit
                    angleleft -= 2*3.14159;
                }
                if(angleleft<-3.14159){
                    angleleft += 2*3.14159;
                }
                
                if((angleleft) > 0.07 || (angleleft) < -0.07){//Turning till final angle reaches
                    msg.angular.z = c*angular;
                    msg.linear.x = 0;
                } 
                else if((haversine(gps.Latitude,gps.Longitude,Lat,Long))>0.5){//moving till final destination reached , error margin of 0.5m
                    msg.linear.x = linear;
                    msg.angular.z = 0;
                }
                else{
                    msg.angular.z = 0;
                    msg.linear.x = 0;
                }
                std::cout<<"State = ATT"<<std::endl;
            }         
        }
        
        if(haversine(gps.Latitude,gps.Longitude,Lat,Long)<0.5){//Ends ATT
            msg.linear.x = 0;
            msg.angular.z = 0;
            IsEnd = 1;
        }
        pub.publish(msg);
        r.sleep();  
        Cold_Start++;    
    }
}
