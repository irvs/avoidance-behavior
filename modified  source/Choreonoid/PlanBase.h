/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _PlanBase_H
#define _PlanBase_H

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <cnoid/Item>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include "GraspPluginManager.h"
#include <boost/make_shared.hpp>

#include "Finger.h"
#include "Arm.h"
#include "InterObject.h"
#include "InterLink.h"
#include "ColdetPairData.h"

#include <vector> //test

#include "exportdef.h"

#ifndef CNOID_10_11
#define YAML_SCALAR ValueNode::SCALAR
#define YAML_SEQUENCE ValueNode::SEQUENCE
#endif

namespace grasp{

class EXCADE_API PlanBase;

class Box{
	public:
		cnoid::Vector3 p;
		cnoid::Matrix3 R;
		cnoid::Vector3 edge;
};

class Approach{
	public:
		Approach(const cnoid::Vector3& dir, const std::vector<cnoid::Vector3>& pos, const std::vector<cnoid::Vector3>& nor)
		{this->direction = dir; this->position = pos; this->normal = nor;}
		cnoid::Vector3 direction;
		std::vector<cnoid::Vector3> position;
		std::vector<cnoid::Vector3> normal;
};

class EXCADE_API TargetObject{
	public:
		friend class PlanBase;

		TargetObject(cnoid::BodyItemPtr bodyItem);
		const std::string& name() { return bodyItemObject->name(); }

		cnoid::Vector3 objCoM(){ return cnoid::Vector3( objVisRot * objCoM_   + objVisPos ); }
		cnoid::Matrix3 OCP_R(){	return cnoid::Matrix3(objVisRot * OCP.R); }
		cnoid::Vector3 OCP_p(){ return cnoid::Vector3(objVisRot* OCP.p+objVisPos);	}
		cnoid::Vector3 OCP_edge(){	return cnoid::Vector3 (OCP.edge); }

		cnoid::BodyItemPtr bodyItemObject;
//	private:
		cnoid::Link *object;
		cnoid::Vector3 objVisPos;
		cnoid::Matrix3 objVisRot;
		bool offsetApplied;
		double objMass;
		cnoid::Vector3 objCoM_;
		Box OCP;
		std::string preplanningFileName;

		std::vector<Approach*> approach;
		
		cnoid::ColdetModelPtr safeBoundingBox;
};

    class ArmFingers;
    typedef boost::intrusive_ptr<ArmFingers> ArmFingersPtr;


class EXCADE_API ArmFingers  : public cnoid::Item
{
	public:

	cnoid::BodyItemPtr bodyItemRobot;
	std::string bodyItemRobotPath;
	std::string dataFilePath;

	ArmFingers(cnoid::BodyItemPtr bodyItem, const cnoid::YamlMapping& gSettings);
	int nFing;
	int nHandLink;

	cnoid::Link *palm;
	cnoid::Link *base;

	FingerPtr *fingers;
	cnoid::LinkTraverse  *handJoint;

	ArmPtr arm;
	std::string pythonInterface;

	std::string name;
	std::multimap<std::string,std::string> contactLinks;

	cnoid::Vector3 objectPalmPos;
	cnoid::Matrix3 objectPalmRot;

	double mu;
	double fmax;
	double hmax;

	int id;

	protected:
	std::ostream& os;
	GraspPluginManager gPluginManager;

};


class EXCADE_API MotionState {
public:
	MotionState(){
	  id = -1;
	  objectPalmPos = cnoid::Vector3::Zero();
	  objectPalmRot = cnoid::Matrix3::Identity();
	}
	MotionState(cnoid::VectorXd jointSeq, int graspingState=0, int graspingState2=0, int id=-1, double tolerance=-1, double time=0){
		this->jointSeq = cnoid::VectorXd::Zero(jointSeq.size());
		this->jointSeq = jointSeq;
		this->graspingState = graspingState;
		this->graspingState2= graspingState2;
		this->time = time;
		this->id = id;
		this->tolerance = tolerance;
		pos.setZero();
		rpy.setZero();
		objectPalmPos = cnoid::Vector3::Zero();
		objectPalmRot = cnoid::Matrix3::Identity();
	}

	cnoid::VectorXd jointSeq;
	cnoid::Vector3 pos;
	cnoid::Vector3 rpy;
	int graspingState, graspingState2;
	int objectContactState;
	double time;
	std::vector<int> pathPlanDOF;
	double motionTime;
	int id, id2;
	double tolerance;
	std::vector<int> children;
	cnoid::Vector3 objectPalmPos;
	cnoid::Matrix3 objectPalmRot;
	cnoid::Vector3 appVec, appVec2;

	//cnoid::VectorXd bodyJointSeq;
	//std::vector<int>bodyJointPtrs; //this is id for armJointSeq;

	private:
};

class PointCloudEnv
{
public:
	PointCloudEnv() : 
	max_x(-DBL_MAX), max_y(-DBL_MAX), max_z(-DBL_MAX),
	min_x(DBL_MAX), min_y(DBL_MAX), min_z(DBL_MAX){;}

	std::vector<cnoid::Vector3>& p() {return points;}
	void addPoints(const std::vector<cnoid::Vector3>& _p) {
		points = _p;
		calcAABB();
	}

	void calcAABB() {
		for(unsigned int i = 0; i < points.size(); i++) {
			max_x = (max_x > points[i](0)) ? max_x : points[i](0);
			min_x = (min_x < points[i](0)) ? min_x : points[i](0);
			max_y = (max_y > points[i](1)) ? max_y : points[i](1);
			min_y = (min_y < points[i](1)) ? min_y : points[i](1);
			max_z = (max_z > points[i](2)) ? max_z : points[i](2);
			min_z = (min_z < points[i](2)) ? min_z : points[i](2);
		}
	}

	double max_x, max_y, max_z, min_x, min_y, min_z;
private:
	std::vector<cnoid::Vector3> points;
};

class EXCADE_API PlanBase
{

public :
	PlanBase();
	~PlanBase();

    static PlanBase* instance(PlanBase *gc=NULL);

	void SetGraspedObject(cnoid::BodyItemPtr bodyItem);
	bool SetGraspingRobot(cnoid::BodyItemPtr bodyItem);

	void SetEnvironment(cnoid::BodyItemPtr bodyItem){
		bodyItemEnv.push_back(bodyItem);
		bodyItemEnv.sort();
		bodyItemEnv.unique();
		initialCollision();
	}

	void RemoveEnvironment(cnoid::BodyItemPtr bodyItem){
		bodyItemEnv.remove(bodyItem);
		bodyItemEnv.sort();
		bodyItemEnv.unique();
		initialCollision();
	}

	void SetPointCloudEnvironment(std::vector<cnoid::Vector3>& points){
		PointCloudEnv* point_cloud = new PointCloudEnv();
		point_cloud->addPoints(points);
		pointCloudEnv.push_back(point_cloud);
		initialCollision();
	}

	void RemoveAllPointCloudEnvironment(){
		for(std::list<PointCloudEnv*>::iterator li=pointCloudEnv.begin();li!=pointCloudEnv.end();++li){
			delete (*li);
		}
	}

	class LessAbs{
	public:
		bool operator()(const double& l, const double& r) const{
			return fabs(l)<fabs(r);
		}
	};

	// interface functions of planning
	virtual bool initial();
	virtual void initialCollision();
	virtual void initialCollisionWithMemory();
//	virtual void runPythonScripts(){};
//	virtual void closeFingers();

	static double calcContactPoint(cnoid::ColdetLinkPairPtr cPair, cnoid::Vector3 &Po, cnoid::Vector3 &Pf, cnoid::Vector3 &objN2, cnoid::Vector3 &fingerN);
	static void calcBoundingBox(cnoid::ColdetModelPtr model, cnoid::Vector3 &edge, cnoid::Vector3& center, cnoid::Vector3& com, cnoid::Matrix3& Rot);

	//==Object==
	TargetObject* targetObject;
	std::vector<TargetObject*> multiTargetObject;

	//==Robot==
	ArmFingers* targetArmFinger;
	std::vector<ArmFingers*> armsList;
	cnoid::BodyItemPtr bodyItemRobot(){ return targetArmFinger->bodyItemRobot; }
	cnoid::BodyItemPtr bodyItemRobot(int i){ return armsList[i]->bodyItemRobot; }

	//==Env==
	std::list<cnoid::BodyItemPtr> bodyItemEnv;
	std::list<PointCloudEnv*> pointCloudEnv;

	void calcForwardKinematics();
	bool isColliding();
	void setTolerance(double t){tolerance = t;}
	double clearance();
	double tolerance;
	std::vector<int> pathPlanDOF;
	std::vector<std::vector<int> > pathPlanDOFSeq;

//#define OP
	//bool isCollidingPointCloud(const std::vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, double tol=0.001); //オリジナル
//#ifdef OP
	int isCollidingPointCloud(const std::vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, double tol=0.001); //①
//#else
	void isCollidingPointCloud2(const std::vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, std::vector<cnoid::Vector3>& col_result, double tol=0.001); //②
  void isCollidingPointCloud3(const std::vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, std::vector<cnoid::Vector3>& col_result, double tol=0.001); //②
//#endif
	void removeCollidingPointCloud(std::vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, double tol=0.001);
	void removeCollidingPointCloud2(const std::vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, std::vector<cnoid::Vector3>& col_result, double tol=0.001);

	bool isCollidingPointCloud(PointCloudEnv* pc_env, cnoid::BodyItemPtr item, double tol=0.001);
	bool isCollidingPointCloudFinger(const std::vector<cnoid::Vector3>& p, double tol=0.001);
	std::vector<cnoid::Vector3> pointCloud;


	void setGraspingState(int state);
	void setGraspingState2(int state);
	void setObjectContactState(int state) { objectContactState = state; }

	int getGraspingState() {return graspingState; }
	int checkAllGraspingState() {return (graspingState > graspingState2) ? graspingState : graspingState2; }
	int getGraspingState2() {return graspingState2; }
	int getObjectContactState() {return objectContactState; }
	void setTrajectoryPlanDOF();
	void setTrajectoryPlanDOF(int k);
	void setTrajectoryPlanMapDOF();
	enum GraspingStates { NOT_GRASPING, UNDER_GRASPING, GRASPING };
	//enum FingerGraspingStates { FINGER_INIT, FINGER_CLOSE, FINGER_OPEN, FINGER_MOVE };
	enum TargetGraspingStates { ON_ENVIRONMENT, OFF_ENVIRONMENT };
	std::vector<cnoid::VectorXd> jointSeq;
	std::vector<double> motionTimeSeq;
	std::vector<int>graspingStateSeq, graspingStateSeq2, objectContactStateSeq;
	std::vector<cnoid::Vector3> objectPalmPosSeq;
	std::vector<cnoid::Matrix3> objectPalmRotSeq;
	std::vector<MotionState>graspMotionSeq;
	MotionState getMotionState(double time=0);
	void setMotionState(MotionState gm);

	// MotionStateForPathPlanner
	MotionState startMotionState;
	MotionState endMotionState;
	MotionState graspMotionState;
	MotionState placeMotionState;
	cnoid::Vector3 ulimitMap;
	cnoid::Vector3 llimitMap;

	bool stopFlag;

	bool flush();
	std::ostream& os;

	cnoid::Link* object() { return targetObject->object; }
	cnoid::Vector3 objVisPos() { return  targetObject->objVisPos;  }
	cnoid::Matrix3 objVisRot() { return targetObject->objVisRot;  }
	void setObjPos(const cnoid::Vector3& P, const cnoid::Matrix3 R);

	void setVisOffset(const cnoid::Vector3& P);
	void setVisOffsetR();
	void removeVisOffset(const cnoid::Vector3& P);

	cnoid::BodyPtr body() { return targetArmFinger->bodyItemRobot->body(); }
	cnoid::BodyPtr body(int i) { return armsList[i]->bodyItemRobot->body(); }
	cnoid::Link* palm() { return targetArmFinger->palm; }
	cnoid::Link* palm(int i) { return armsList[i]->palm; }
	cnoid::Link* base() { return targetArmFinger->base; }
	cnoid::Link* base(int i) { return armsList[i]->base; }
	ArmPtr arm(){ return targetArmFinger->arm; }
	ArmPtr arm(int i){ return armsList[i]->arm; }

	FingerPtr fingers(int i) { return targetArmFinger->fingers[i]; }
	FingerPtr fingers(int i, int j) { return armsList[i]->fingers[j]; }
	int nFing(){ return targetArmFinger->nFing; };
	int nFing(int i){ return armsList[i]->nFing; };

	cnoid::LinkTraverse* handJoint() { return targetArmFinger->handJoint; }
	cnoid::LinkTraverse* handJoint(int i) { return armsList[i]->handJoint; }
	int nHandLink() {return targetArmFinger->nHandLink;}
	int nHandLink(int i) {return armsList[i]->nHandLink;}

	std::string  bodyItemRobotPath() {return  targetArmFinger->bodyItemRobotPath; }
	std::string  bodyItemRobotPath(int i) {return  armsList[i]->bodyItemRobotPath; }
	std::string  dataFilePath() {return  targetArmFinger->dataFilePath; }
	std::string  dataFilePath(int i) {return  armsList[i]->dataFilePath; }
	std::string  pythonInterface() {return  targetArmFinger->pythonInterface; }
	std::string  pythonInterface(int i) {return  armsList[i]->pythonInterface; }
	std::string armName() {return targetArmFinger->name; }
	std::string armName(int i) {return armsList[i]->name; }


	std::vector<cnoid::ColdetLinkPairPtr> robotSelfPairs, robotEnvPairs, robotObjPairs, objEnvPairs, safeRobotEnvPairs, safeRobotObjPairs; // armEnvPairs;
	std::vector<std::pair<cnoid::BodyItemPtr, PointCloudEnv*> > robotPointCloudPairs, objPointCloudPairs;
	std::string colPairName[2], objPressName;
	cnoid::Vector3 objPressPos;
	cnoid::BodyItemPtr pressItem;
	bool doInitialCollision;
	std::vector<ColdetPairData*> coldetPairData, safeColdetPairData;

	std::map <std::string,cnoid::BodyItemPtr> objTag2Item;
	std::map <std::string,ArmFingers*> robTag2Arm;

	std::vector<InterLink> interLinkList;
	void setInterLink();
	
	std::vector<InterObject> interObjectList;
	
	bool useObjectSafeBoundingBox , useRobotSafeBoundingBox;
	cnoid::Vector3 boundingBoxSafetySize;

protected :

	int graspingState, graspingState2;
	int motionId;
	int objectContactState;
};




}



#endif
