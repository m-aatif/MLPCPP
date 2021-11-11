namespace java com.afarcloud.thrift
namespace csharp AFarCloudGUI.Comm.Thrift
namespace cpp afarcloud

include "AFC_Types.thrift"

/*
V E R S I O N       2.0
------------------------------------------------
Date: 2020 May 8

- This file and other AFC Thrift files which on versions 2.x are the versions used for the 2nd year demos
- The main change is the separation of different definitions and services into different files

*/

//Service for Planners
//----------------------------------------------
// Note that this is a non-blocking oneway function. 
// When the planning is done and the plan is ready, then the Planner must call the sendPlan function exposed by MmtService
//----------------------------------------------
service PlannerService
{
    //  REQUESTID:
    //- All of the functions that contain a "requestId" field, must return the same value back to MMT when calling MMT Service back in response to a function call from MMT or if they want to call sendError on MMT.
	
    oneway void computePlan(1: i32 requestId, 2: AFC_Types.Mission context),    //This is called by MMT to request for a new plan. the "context" field contains a Mission object which only contains the mission goals.
    
    //Hehe, you though there would be another ping comment here? No, sorry! we run out of them
    string ping(),
}
