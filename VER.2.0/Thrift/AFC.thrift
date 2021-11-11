namespace java com.afarcloud.thrift
namespace csharp AFarCloudGUI.Comm.Thrift
namespace cpp afarcloud



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
	TRANSIT,
	SURVEY,
    INSPECT,
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
	NotAssigned,		// Not sent to the vehicle by the Mission Manager
	NotStarted,			// Sent to the vehicle by the Mission Manager but not started
	Running,
	Finished,
	Stopped,			// It has been paused or stopped for any reason
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

enum SensorType
{
    HUMIDITY,
    COLLAR,
    PRESSURE,
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
	1: list<Position> area,
}

struct DetectionRegion{
	1: i32 Id
	2: i64 time,
	3: Region location,
	4: string label,
}

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

//Later we should add list of capabilities to this as well. Or maybe not
struct TaskTemplate {
	1: TaskType taskType,	                //This is changed from an int to enum compared to SW version
	2: string description,
	3: TaskRegionType regionType
	4: list<EquipmentType> requiredTypes,   // which Equipment should be needed for the task i.e (CAMERA_PHOTO or IR_CAMERA_PHOTO)
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
    7: list<double> params,
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

struct SensorData {
    1: string          sensorUid,
    2: SensorType      sensorType,
    3: string          unit,
    4: double          value,
    5: Position        sensorPosition,
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

//Service for Semantic Queries.
//----------------------------------------------
// Notice that both functions in this service are blocking functions. There is no callback defined in the MmtService for sending the result. 
// This is because the Mmt needs the lists in order to populate the interface with valid data.
//----------------------------------------------
service SemanticQueryService
{
    //Get Functions (please note that these are all blocking functions)
	list<Vehicle>       getAllVehicles          (),				                //Sends a list of all currently available vehicules. Called by MMT once at startup
    list<MissionTag>    getAllMissions          (),                             //Send a list of all missions
    list<MissionTag>    getOngoingMissions      (),                             //Send a list of all missions that contain unfinished tasks
    Vehicle             getVehicle              (1: i32 vid),
    	
    //Query Functions
    oneway void     queryStateVector            (1: i32 requestId, 2: i32 vehicleId, 3: i32 startTime, 4: i32 endTime),   //Ask for the last known State Vector for given vehicle at given time period. If not time period given, then the last known SV ever.
    oneway void     querySensorData             (1: i32 requestId, 2: Region region, 3: i32 startTime, 4: i32 endTime, 5: SensorType sensorType),
    oneway void     queryHistoricalStateVectors (1: i32 requestId, 2: i32 vehicleId, 3: i32 startTime, 4: i32 endTime),
    oneway void     queryHistoricalSensorData   (1: i32 requestId, 2: Region region, 3: i32 startTime, 4: i32 endTime, 5: SensorType sensorType),
                        
    
    //Store Functions    
    oneway void storeEvent                      (1: i32 requestId, 2:i32 missionId, 3:i32 vehicleId, 4:i32 subtype, 5:string description, 6:i64 timeReference),  //If MMT detects or is requested to (abort, etc), the event is reported to MW with this fucntion 
    
    //The Mighty Everlasting PINGGGGGGG
    string ping(),
}


//Service for Mission Manager
//----------------------------------------------
// This service exposes a function that will be called when a plan is to be executed.
//----------------------------------------------
service MissionManagerService
{
    oneway void sendPlan                (1: i32 requestId, 2: Mission plan),
    
    string abortMissionPlan             (1: i32 missionId),
    string abortVehiclePlan             (1: i32 vehicleId),
    string abortMissionPlanHard         (1: i32 missionId),     //Added in Version 9
    string abortVehiclePlanHard         (1: i32 vehicleId),     //Added in Version 9

    //The King of the network functions, the PIIINNNGGGGG
    string ping(),
}


//Service for Planners
//----------------------------------------------
// Note that this is a non-blocking oneway function. 
// When the planning is done and the plan is ready, then the Planner must call the sendPlan function exposed by MmtService
//----------------------------------------------
service PlannerService
{
	oneway void computePlan(1: i32 requestId, 2: Mission context),
    //Hehe, you though there would be another ping comment here? No, sorry! we run out of them
    string ping(),
}


//Service for MMT
//----------------------------------------------
service MmtService
{
	oneway void stateVectorUpdate               (1: i32 requestId, 2: StateVector stateVector)

    oneway void sensorDataUpdate                (1: i32 requestId, 2: list<SensorData> sensorData)

    oneway void sendPlan                        (1: i32 requestId, 2: Mission plan),							//Called by Planners in response to a computePlan. Sends the plan to Mmt
	oneway void sendError                       (1: i32 errorId, 2:string errorMessage, 3: i32 requestId),	    //Called by all in case something goes wrong. 
	oneway void sendMissionStatusReport         (1: i32 missionId, 2: TaskCommandStatus status),				            //Called by MM whenever there is a new update for a mission.
    oneway void sendTaskStatusReport            (1: i32 missionId, 2: i32 taskId, 3: TaskCommandStatus status),				            //Called by MM whenever there is a new update for a mission.
    oneway void sendCommandStatusReport         (1: i32 missionId, 2: i32 commandId, 3: TaskCommandStatus status),				            //Called by MM whenever there is a new update for a mission.

	oneway void sendAlarm						(1: i32 missionId, 2:Alarm alarm),           //Called by the MM whenever there is a new alarm.
    //No Pingy, No Party!
    string ping(),
}
