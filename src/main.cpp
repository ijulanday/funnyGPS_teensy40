#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <autopilot.hh>
#include <Metro.h>
#include <TinyGPS++.h>

#define MY_RADIUS 4 // radius for enclosure LOS steering

SFE_UBLOX_GPS myGPS;
TinyGPSPlus tinycourse;

Metro timer = Metro(50); // timer

Waypoint myLocationWaypoint;
Waypoint * prevWaypoint;

// hardcoded waypoints for walking around the parking lot. in the future these will need to come from a ground station of some sort.
Waypoint * hydrant;
Waypoint * tree;
Waypoint * box;


void setup() {
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");

  //Assume that the U-Blox GPS is running at 9600 baud (the default) or at 38400 baud.
  //Loop until we're in sync and then ensure it's at 38400 baud.
  do {
    Serial.println("GPS: trying 38400 baud");
    Serial1.begin(38400);
    delay(100);
    if (myGPS.begin(Serial1) == true) break;

    delay(100);
    Serial.println("GPS: trying 9600 baud");
    Serial1.begin(9600);
    delay(100);
    if (myGPS.begin(Serial1) == true) {
        Serial.println("GPS: connected at 9600 baud, switching to 38400");
        myGPS.setSerialRate(38400);
        delay(100);
    } else {
        //myGPS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.println("GPS serial connected\n\n");

  myGPS.setSerialRate(38400);
  myGPS.setNavigationFrequency(10, 100);

  Serial.print("GPS Nav frequency set at: ");
  Serial.println(myGPS.getNavigationFrequency());

  myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  // define mission origin point (MUST BE DONE BEFORE OTHER WAYPOINTS)
  missionOrigin.lng = -111.0023414;
  missionOrigin.lat = 31.89751255;

  // mission waypoints here, for now (monkey code don't look at it T^T)
  
  hydrant = new Waypoint(-111.00231630, 31.89752460, 1, 0);
  tree = new Waypoint(-111.00220190, 31.89768730, 1, 0);
  box = new Waypoint(-111.00203210, 31.89760310, 1, 0);

  hydrant->next_wp = tree;
  tree->next_wp = box;
  box->next_wp = hydrant;

  hydrant->name = "hydrant";
  tree->name = "tree";
  box->name = "box";

  prevWaypoint = box;

  // initialize current waypoint
  myLocationWaypoint = Waypoint(
        (double)myGPS.getLongitude() / 10000000.0
      , (double)myGPS.getLatitude() / 10000000.0
      , 8
      , false
  );
}

void loop() {
  // this pulls updates from GPS position; needs to be in its own thread because i think it's blocking
  if (timer.check())
  {
    myLocationWaypoint.updateLngLat((double)myGPS.getLongitude() / 10000000.0, (double)myGPS.getLatitude() / 10000000.0);

    double nextWpDist = calcCartesianDistance(myLocationWaypoint, *prevWaypoint->next_wp);
    double tracklineDist = calcNormalTracklineDistance(*prevWaypoint, *prevWaypoint->next_wp, myLocationWaypoint);

    if (tracklineDist > 4.0)
      myLocationWaypoint.rad = tracklineDist + 1.0;
    else
      myLocationWaypoint.rad = 4;
    

    if (nextWpDist < myLocationWaypoint.rad)
      prevWaypoint = prevWaypoint->next_wp;
    
    Waypoint losWaypoint = calcLOSSetpointEnclosed(*prevWaypoint, *prevWaypoint->next_wp, myLocationWaypoint);
    double bearing = tinycourse.courseTo(myLocationWaypoint.lat, myLocationWaypoint.lng, losWaypoint.lat, losWaypoint.lng) ;
    double heading = (double)myGPS.getHeading() / 100000.0;
    double maneuverAngle = bearing - heading;

    if (bearing > heading && maneuverAngle > 180)
      maneuverAngle = map(maneuverAngle, 180, 360, -180, 0);
    if (bearing < heading && maneuverAngle < -180)
      maneuverAngle = map(maneuverAngle, -180, -360, 180, 0);

    Serial.print(F(" next WP: ("));
    Serial.print(prevWaypoint->next_wp->name.c_str());

    Serial.print(F(") | maneuver angle: "));
    Serial.print(maneuverAngle);

    Serial.print(F(" | dist to track: "));
    Serial.print(tracklineDist);

    Serial.print(F(" | enclosure radius: "));
    Serial.print(myLocationWaypoint.rad);

    Serial.print("                 \r");
  }

}

/**
 * commented code hell
*/

/*

  // debug print statements 
    Serial.print(F("Lat: "));
    Serial.print(myLocationWaypoint.lat, 8);

    Serial.print(F(" Lng: "));
    Serial.print(myLocationWaypoint.lng, 8);

    Serial.print(F(" x: "));
    Serial.print(myLocationWaypoint.x, 8);

    Serial.print(F(" y: "));
    Serial.print(myLocationWaypoint.y, 8);

    Serial.print(F(" hydrantx: "));
    Serial.print(waypoints[0].x, 8);

    Serial.print(F(" hydranty: "));
    Serial.print(waypoints[0].y, 8);

    Serial.print(F(" | Waypoint distance: "));
    Serial.print(nextWpDist);

    Serial.print(F(" x: "));
    Serial.print(myLocationWaypoint.lat, 8);

    Serial.print(F(" y: "));
    Serial.print(myLocationWaypoint.lng, 8);

    Serial.print(F(" nextx: "));
    Serial.print(prevWaypoint->next_wp->lat, 8);

    Serial.print(F(" nexty: "));
    Serial.print(prevWaypoint->next_wp->lng, 8);

     Serial.print(F(") | los x: "));
    Serial.print(losWaypoint.x);

    Serial.print(F(" | los y: "));
    Serial.print(losWaypoint.y);
    
    Serial.print(F(" | my x: "));
    Serial.print(myLocationWaypoint.x);

    Serial.print(F(" | my y: "));
    Serial.print(myLocationWaypoint.y);

    Serial.print(F(" | my X: "));
    Serial.print(myLocationWaypoint.x);
    Serial.print(F(" | my Y: "));
    Serial.print(myLocationWaypoint.y);
    Serial.print(F(" | los X: "));
    Serial.print(losWaypoint.x);
    Serial.print(F(" | los Y: "));
    Serial.print(losWaypoint.y);

  Serial.println();

*/