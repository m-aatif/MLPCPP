namespace java com.afarcloud.thrift
namespace csharp AFarCloudGUI.Comm.Thrift
namespace cpp afarcloud

/*
V E R S I O N       2.0
------------------------------------------------
Date: 2021 March 17

 - Added "RepeatCount" property to the Task class.
 - Added "IsOnline" property to the Vehicle class.

Date: 2020 Sep 10

 - TRack,Panorama and Follow Traget Equipments are added to EquipmentType enum. They show that if a vehicle supports such tasks out of the box or not.

Date: 2020 July 29

- The list<Positions> type in Partfield is now changed to a Region object.
- Task now includes a list of partfields and treatmentzones related to the task.

Date: 2020 July 28

- The VehicleType enumeration now includes UAV, UGV and Tractor types. They are the same as AUAV, AGV and RGV types. If your code relies on VehicleType enumeration make sure that it supports both represetations.
- SPRAYER is added the EquipmentType enumeration to support for spraying in UGVs. More Tractor related equipment will be added after Y2 demos.
- DetectionRegion is a new struct which is used for representing the IPP detection results.
- Equipment struct now has a two new fields "isoId" and "Id". "isoId" is the string ID that ISOBUS system uses. ID is something we might use later. Both are optional.  Just pass them around.
- PartField also has a new "isoId" field. Read the above sentence to know what it is.
- CropType is removed from PartField
- ThreatmentGrid now includes a taskId, so the MissionManager knows which task a treatment is related to.

Date: 2020 June 25

- Partfield definitions are added.
- Treatment Grid definition is added.


Date: 2020 June 3

- This file and other AFC Thrift files which on versions 2.x are the versions used for the 2nd year demos
- The main change is the separation of different definitions and services into different files
- Sensor types and their data structures are overhauled (compared to prev version 1.2). If you use those structs, make sure to update to this version

*/

/*
T O D O
------------------------------------------------

 - VehiculeType might need changes later

*/


enum VehicleType{   	// to be updated when HIB finishes the ontology
	AUAV,           	//autonomous drone (Autonomous Unmmaned Aerial Vehicle)
	RUAV,           	//remotely piloted drone 
	AGV,            	//autonomous ground vehicle
	RGV,            	//piloted ground vehicle
	UAV,				//Same as AUAV, dont ask why we did this. Just deal with it!			
	UGV,				//Same as AGV
	Tractor,			//Same as RGV
}

enum TaskType{
	TRANSIT,			//Move from A to B
	SURVEY,				//Cover an area using the lawn mowing pattern
    INSPECT,			//Go to point A and use the sensor (for example take a picture)
	HOTPOINT,			
	FOLLOW_TARGET,		
	ACTIVE_TRACK,		
	PANORAMA,			
}

enum CommandType{
    NAV_TAKEOFF = 1,		
    NAV_LAND = 2,
    NAV_WAYPOINT = 3,
    CAMERA_IMAGE = 4,
    VIDEO_START_CAPTURE = 5,
    VIDEO_STOP_CAPTURE = 6,
    NAV_HOTPOINT = 7,
    NAV_FOLLOW_TARGET = 8,
    NAV_TRACK_SUBJECT = 9,
    CAMERA_PANORAMA = 10,
    NAV_HOME = 11,
}

enum TaskCommandStatus {
	NotAssigned,		// The command is not sent to the vehicle by the Mission Manager
	NotStarted,			// The command is sent to the vehicle by the Mission Manager but not started
	Running,			// The vehicule is executing the command
	Finished,			
	Stopped,			// It has been paused or stopped for any reason, but not completely finito
}

enum TaskRegionType {
	Point,
	Column,             //A line
	Area,               //A rectangle
    Circle,             //A circle
    Dynamic             //Dynamically changes by drone's autopilot. For example when following a target.
}

enum EquipmentType {    // to be updated when HIB finishes the ontology
    CAMERA_360,
	CAMERA_PHOTO,
	CAMERA_VIDEO,
	CAMERA_TRACK,
	CAMERA_PANORAMA,
	CAMERA_FOLLOW,
	CAMERA_HOTPOINT,
	IR_CAMERA_PHOTO,
    IR_CAMERA_VIDEO,
    WIFI,
    COLLISION_AVOIDANCE,
	SPRAYER,			//This is jusy for Y2 purposes, the whole issue with Tarctor equipment needs to be solved for Y3
}


struct Position{
	1: double longitude,    
	2: double latitude,	    
	3: double altitude,	    // [m]
}

struct Orientation{
	1: double roll,			//All in degrees
	2: double pitch,
	3: double yaw,
}

struct Region {
	1: list<Position> area,//The set of points defines the borders of the polygon AND the order to connect the points to get the edges
}

struct DetectionRegion{
	1: i32 Id
	2: i64 time,
	3: Region location,
	4: string label,
}

//BATTERY Also means FUEL
struct Battery {
    1: double batteryCapacity,			//Capacity in Ah (last full capcity)
	2: double batteryPercentage,		//Charge percentage on 0 to 1 range 	
}

//We might need to a the gimball angle as well
struct StateVector {
    1: i32 vehicleId,					
    2: Position position,
	3: Orientation orientation,
    4: optional double gimbalPitch,
	5: optional Battery battery,
	6: double linearSpeed,
	7: i64 lastUpdate,
}

struct Equipment {
	1: optional i32 id,
	2: optional string isoId,
	3: EquipmentType type,
	4: string name,
}

struct Vehicle {
	1: i32 id,
	2: string name,
	3: VehicleType type,
    4: double maxSpeed,                 //Maximum speed that a vehicle is able to perform [m/s]
    5: i32 maxRunningTime,              //in SECONDS
	6: list<Equipment> equipments,
	7: list<TaskType> capabilities,     //the type of tasks that the vehicle is capable of executing
	8: StateVector stateVector,
    9: double safetyDistance,			//The disctance from which the vehicule is not allowed to get close to another vehicule
	10: bool isOnline,					//Shows if a registered vehicle is currently online or not. If a vehicle is offline, the stateVector shows the last known location.
}

//NOTENOTENOTE
//Later we should add list of capabilities to this as well. Or maybe not
struct TaskTemplate {
	1: TaskType taskType,	                //This is changed from an int to enum compared to SW version
	2: string description,
	3: TaskRegionType regionType
	4: list<EquipmentType> requiredTypes,
	5: optional double maxSpeed,            //Maximum Speed at which the task should be performed. Note that this is diffirent from max speed of the vehicle [m/s]
}

struct Task {
	1: TaskTemplate taskTemplate,
	2: i32 id,
    3: i32 missionId,
	4: Region area,									// The area at which the task should be performed
	5: double speed,                        		// [m/s]
	6: double altitude,                     		// [m]
	7: double range,                        		// Not important for now.
	8: i32 timeLapse,                       		// [s] used for tasks where relevant. For example if a drone need to take photos every 5 seconds, etc
	9: Orientation bearing,                 		// Leave it for now
	10: i64 startTime,                      		// relative to the mission timeline
	11: i64 endTime,                        		// relative to the mission timeline
	12: TaskCommandStatus taskStatus,
	13: i32 assignedVehicleId,
	14: i32 parentTaskId,                   		// This is used for distiguishing the tasks that are added by a planner to the mission. When a new task is added by the planner to a mission to fulfill another task, the ID of that other task must be copied here.
	15: optional list<PartField> partfields,		// List of partfields related to this task. Can be null.
	16: optional list<TreatmentGrid> treatmentGrids,// List of treatment zones for this task. Can be null.
	17: i32 repeatCount,							// Shows how many times the task should be repeated. At the moment this only affects the TAKE_IMAGE task and shows the number of images to take.
}

struct Command {
    1: Task relatedTask,
    2: i32 id,
    3: CommandType commandType,
    4: i64 startTime,
    5: i64 endTime,
    6: TaskCommandStatus commandStatus,
    7: list<double> params,					//Please refer to WP2 documentation for a list of parameters for each CommandType
}

struct MissionTag{
    1: i32 missionId,
    2: optional string name,
}

struct Mission {
	1: i32 missionId,
    2: optional string name,
	3: Region navigationArea,           //Vehicles are not allowed to leave this area
	4: list<Region> forbiddenArea,      //It is forbidden to go there
	5: list<Position> homeLocation,     //Where the vehicles should go back to
	6: list<Task> tasks,                //list of tasks 
	7: list<Vehicle> vehicles,          //list of vehicles used in this mission
    8: optional list<Command> commands, //list of commands
}

struct Alarm {
    1: i32 alarmId,
    2: i32 vehicleId,
	3: i32 missionId,
    4: i32 alarmType,
	5: i32 alarmCode,
    6: string description,
    7: i64 time,
}

struct PartField {
     1: i32                 partfieldId,
     2: optional string     isoId,
     3: string              name,
     4: Region              borderPoints,
}


struct TreatmentGrid{
	1: i32			  Id,
	2: i32			  partfieldId,
	3: i32			  taskId,
	4: i32			  numRows,
	5: i32 			  numCols,
	6: list<double>	  treatmentValue,					  //Assuming the grid cells are ordered from bottom left to top right, in a row-major order.
}
