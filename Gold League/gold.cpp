#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cassert>

using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

struct VectorF_t
{
    VectorF_t() : x(0), y(0) {}
    VectorF_t(float _x, float _y)  :x(_x), y(_y) {}
    VectorF_t operator+(const VectorF_t &other) const {return VectorF_t(x+other.x, y+other.y);}
    VectorF_t operator-(const VectorF_t &other) const {return VectorF_t(x-other.x, y-other.y);}
    VectorF_t operator*(float m) const {return VectorF_t(x*m, y*m);}
    float len() const {return sqrtf(x*x+y*y);}

    VectorF_t rotated_vector(float angleRad) const
    {
        float c = cosf(angleRad);
        float s = sinf(angleRad);
        return VectorF_t(c*x-y*s,x*s+y*c);
    }
    float dot(const VectorF_t &other) const
    {
        return x*other.x+y*other.y;
    }

    float safe_normalize_myself()
    {
        float l=len();
        if (l==0)
        {
            x = 0; y = 0;
        }
        else
        {
            x/=l;
            y/=l;
        }
        return l;

    }

    float x,y;
    
};

inline float radToDeg(float rad)
{
    return rad *(180.0f/M_PI);
}

inline float degToRad(float deg)
{
    return deg*(M_PI/180.0f);
}

class Pod_t
{    
public:
    void startFrame(const std::vector<VectorF_t> &checkpoints)
    {
        int oldCheckpointId = nextCheckpointId;
        readFromInput(checkpoints);
        if (nextCheckpointId!=oldCheckpointId)
        {
            crossedCheckpointCount++;
            if (nextCheckpointId==0)
            {
                lapIndex++;
            }
        }

        if (!isFirstFrame)
            speed=pos-prevPos;
        else
            speed=VectorF_t(0,0);

        nowShield = false;
        assert(nextCheckpointId>=0 && nextCheckpointId<checkpoints.size());
        targetPos = checkpoints[nextCheckpointId];
        nextCheckpoint = targetPos;
        nextNextCheckpoint = checkpoints[(nextCheckpointId+1)%checkpoints.size()];
        next_checkpoint_dist = (int)(targetPos-pos).len();
        VectorF_t toGoal = targetPos-pos;
        float goalAngleDeg =radToDeg(atan2f(toGoal.y, toGoal.x));
        next_checkpoint_angle = (int)(goalAngleDeg-angleInDegrees);
        while (next_checkpoint_angle<-90)
            next_checkpoint_angle+=180;
        while (next_checkpoint_angle>=90)
            next_checkpoint_angle-=180;

    }

    void endFrame()
    {
        prevPos = pos;
        isFirstFrame = false;
    }

    void handleAttack(std::vector<Pod_t> &myPods, std::vector<Pod_t> &enemyPods)
    {
        //isAttackMode = false;
        int minAdvancement = 2;
        if (crossedCheckpointCount<minAdvancement)
            return;
        //am I late compared to the other friendly pod
        int betterPodIndex = -1;
        assert(myPods.size()==2);
        for (int i=0; i<(int)myPods.size(); i++)
        {
            if (crossedCheckpointCount < myPods[i].crossedCheckpointCount)
                betterPodIndex = i;
        }
        if (betterPodIndex<0)
            return;
        //try to attack leading competitor
        int victimIndex=-1;
        int victimBestResult = -1;
        for (int i=0; i<(int)enemyPods.size(); i++)
        {
            int enemyResult = enemyPods[i].crossedCheckpointCount;
            if (enemyResult>myPods[betterPodIndex].crossedCheckpointCount
                && enemyResult>victimBestResult)
            {
                victimBestResult = enemyResult;
                victimIndex = i;
            }
        }
        if (victimIndex<0)
            return;

        //cerr<< "----WE SELECTED ATTACK OF FRIEND "<<(1-betterPodIndex)<<" AGAINST "<<victimIndex<<endl;
        //dont attack if we are too close to friend
        float minDistFromFriend = shieldRadius*3;
        if ((myPods[betterPodIndex].pos-pos).len()<minDistFromFriend)
        {
            //cerr<< "----BUT FRIEND IS TOO CLOSE"<<endl;
            return;
        }

        const Pod_t &victimPod = enemyPods[victimIndex];
        VectorF_t attackTarget = victimPod.pos +victimPod.speed;
        
        //we must not be behind the target 
        VectorF_t toVictimDir = victimPod.pos-pos;
        toVictimDir.safe_normalize_myself();
        VectorF_t victimToVictimTarget=victimPod.nextCheckpoint-victimPod.pos;
        victimToVictimTarget.safe_normalize_myself();

        float cosOfBehindAngleForAttack = cosf(degToRad(45));
        if (victimToVictimTarget.dot(toVictimDir)>cosOfBehindAngleForAttack)
            return;
        cerr << "--------WE ATTACK"<<endl;
        isAttackMode = true;
        attackModeTarget = attackTarget;
    }

    void updateMyAI(std::vector<Pod_t> &myPods, std::vector<Pod_t> &enemyPods)
    {
        //cerr<<" "<<endl;
        //cerr<<"framepos="<<pos.x<<" "<<pos.y<<endl;

        handleAttack(myPods, enemyPods);
        if (isAttackMode)
            targetPos = attackModeTarget;


        will_overshoot = false;
        thurst = 0;
        overshoot_mult = 1;
        if (!isFirstFrame)
        {
            handleShield(enemyPods);
            speed=pos-prevPos;
            keep_track(givenSpeed, targetPos);
            fillPodsToAvoidList(myPods, enemyPods, otherPods);
            will_overshoot = handle_overshoot(pos, speed, nextCheckpoint, overshoot_mult);
            handle_avoidance(otherPods);
            if (will_overshoot)
            {
                float givenSpeedLen = givenSpeed.len();
                if (givenSpeedLen>minSpeedForAnticipatedTurn)
                {
                    if (areVectorsSameDirection(nextCheckpoint-pos, givenSpeed, 10))
                    {
                        cerr<<"ANTICIPATE"<<endl;
                        targetPos = nextNextCheckpoint;
                    }
                }
            }            
        }
        nowBoost = check_can_boost(next_checkpoint_angle, next_checkpoint_dist, hasUsedBoost);
        
        if (abs(next_checkpoint_angle)<90)
        {
            thurst = maxthurst;
            if (will_overshoot && !hasUsedShield)
                thurst = (int)(thurst*overshoot_mult);
        }

    }
    bool areVectorsSameDirection(const VectorF_t &v1, const VectorF_t &v2, float deg)
    {
        VectorF_t d1(v1);
        d1.safe_normalize_myself();
        VectorF_t d2(v2);
        d2.safe_normalize_myself();
        return d1.dot(d2)>cosf(degToRad(deg));
    }

    void output()
    {
        cout << (int)targetPos.x << " " << (int)targetPos.y << " ";
        if (nowBoost)
        {
            cout << "BOOST";
            hasUsedBoost = true;
        }
        else if (nowShield)
        {
            cout << "SHIELD";
            hasUsedShield = true;
        }
        else
            cout << thurst;
        cout << endl;
    }

private:
    static const int maxthurst = 100;
    static constexpr float minSpeedForAnticipatedTurn = 200;
    static constexpr float minSpeedForShield = 400;
    static constexpr float shieldRadius = 400;
    

    //inputs
    VectorF_t pos;
    VectorF_t givenSpeed;
    float angleInDegrees;
    int nextCheckpointId = -1;
    
    VectorF_t nextCheckpoint;
    VectorF_t nextNextCheckpoint;
    VectorF_t targetPos;
    int next_checkpoint_dist = 0;
    int next_checkpoint_angle = 0;

    //Algorithm output
    VectorF_t speed;
    bool hasUsedBoost = false;
    bool nowBoost = false;
    bool will_overshoot = false;
    float overshoot_mult = 1;
    int thurst = 0;
    std::vector<Pod_t*> otherPods;
    VectorF_t prevPos;
    bool hasUsedShield = false;
    bool nowShield = false;
    bool isFirstFrame = false;
    int crossedCheckpointCount=0;
    bool isAttackMode=false;
    int lapIndex=-1;
    VectorF_t attackModeTarget;

    //
    void handleShield(std::vector<Pod_t> &enemyPods)
    {
        if (hasUsedShield)
            return;
        if (givenSpeed.len()<minSpeedForShield)
        {
            return;
        }
        if (lapIndex<2)
            return;
        bool result = false;
        const int anticipCount=2;
        for (int anticip=0; anticip<anticipCount; anticip++) 
        {
            VectorF_t myFuturePos = pos + speed*anticip;
            for (const Pod_t &enemyPod : enemyPods)
            {
                VectorF_t enemyFuturePos = enemyPod.pos+enemyPod.speed*anticip;
                if ((enemyFuturePos-myFuturePos).len()<=(2*shieldRadius))
                {
                    result = true;
                    break;
                }
            }
        }
        if (result)
        {
            nowShield = true;
        }
    }

    void readFromInput(const std::vector<VectorF_t> &checkpoints)
    {
        cin >> pos.x >> pos.y >> givenSpeed.x >> givenSpeed.y >> angleInDegrees >> nextCheckpointId;
        cin.ignore();
    }

    void fillPodsToAvoidList(std::vector<Pod_t> &myPods, std::vector<Pod_t> &enemyPods, std::vector<Pod_t*> &otherPods)
    {
        otherPods.clear();
        for (Pod_t &pod:myPods)
        {
            if (this==&pod)
                continue;
            otherPods.push_back(&pod);
        }
        if (!isAttackMode)
        {
            for (Pod_t &pod:enemyPods)
            {
                if (this==&pod) //for symetry
                    continue;
                otherPods.push_back(&pod);
            }
        }
    }

    void handle_avoidance(const vector<Pod_t*> &otherPods)
    {
        const float pod_radius = 420;
        const float min_avoid_dist = 800.0f;
        const float threshold_dist_to_goal = 900;

        //cerr<<"DIST TO TARGET="<<next_checkpoint_dist<<endl;
        if (next_checkpoint_dist<threshold_dist_to_goal)
        {
            //cerr<<"TOO NEAR OF TARGET"<<endl;
            return;
        }

       
        bool hasFoundAngle = false;
        //cerr<<"---CHECK AVD "<<endl;
        int maxAngle = 40;
        for (int angleStep=0; angleStep<=maxAngle; angleStep+=5)
        {
            for (int isign=-1; isign<=1; isign+=2)
            {
                bool isLastAngle = (angleStep==maxAngle);
                float angleRad = (isign*angleStep) *(3.1415f/180.f);
                VectorF_t my_relative_fake_speed(speed);
                VectorF_t rotated_fake_relative_speed_dir = my_relative_fake_speed.rotated_vector(angleRad);
                rotated_fake_relative_speed_dir.safe_normalize_myself();
                //is the rotated fake speed in the cone to the mobile?

                bool rejectThisAngle = false;
                for (Pod_t *otherPod : otherPods)
                {

                    const int anticipCount=3;
                    for (int anticip=1; anticip<anticipCount; anticip++) 
                    {
                        VectorF_t mySpeedVec(rotated_fake_relative_speed_dir*speed.len());
                        VectorF_t myFuturePos = pos + mySpeedVec*anticip;
                        VectorF_t enemyFuturePos = otherPod->pos+otherPod->speed*anticip;
                        float dist = (enemyFuturePos-myFuturePos).len();
                        if (dist<=(2*pod_radius))
                        {
                            //cerr<<"REJECTED AT "<<anticip<<" MFP="<<myFuturePos.x<<","<<myFuturePos.y<<"       NMI="<<enemyFuturePos.x<<","<<enemyFuturePos.y<<endl;
                            //cerr<<"  with rotated "<<mySpeedVec.x<<","<<mySpeedVec.y<<"    otherspeed "<<otherPod->speed.x<<","<<otherPod->speed.y<<endl;
                            rejectThisAngle = true;
                            break;
                        }
                    }
                }
                if (!rejectThisAngle || isLastAngle)
                {
                    //no, add a nudge corresponding to this speed
                    VectorF_t fto_goal(targetPos.x-pos.x, targetPos.y-pos.y);
                    fto_goal = fto_goal.rotated_vector(angleRad);
                    VectorF_t initTarget = targetPos;
                    targetPos = pos+fto_goal;
                    hasFoundAngle = true;
                    if (angleStep!=0)
                    {
                        //cerr<<"----POS="<<pos.x<<" "<<pos.y<<endl;
                        //cerr<<"---INITIAL TARGET= "<<initTarget.x<<" "<<initTarget.y<<endl;
                        //cerr<<"---ROTATED OF "<<(isign*angleStep)<<endl;
                        //cerr<<"---NUDGED TARGET= "<<nextCheckpoint.x<<" "<<nextCheckpoint.y<< " ANG="<<atan2f(-fto_goal.y, fto_goal.x)*(180.f/3.1415f)<<endl;
                    }
                    break;
                }
                
            }
            if (hasFoundAngle)
                break;
        }
    }

    bool check_can_boost(int next_checkpoint_angle, int next_checkpoint_dist, bool &used_boost)
    {
        const int min_boost_dist = 2000;
        const int max_boost_angle_to_target = 10;
        bool now_boost = false;
        if (abs(next_checkpoint_angle)<max_boost_angle_to_target && next_checkpoint_dist>min_boost_dist && !used_boost)
        {
            //cerr << "------------BOOST " << next_checkpoint_dist <<endl;
            now_boost = true;
        }
        return now_boost;
    }




    void keep_track(const VectorF_t &theSpeed, VectorF_t &goal)
    {
        const float maxAtteinableAngle = 5.f;
        VectorF_t fToGoal = VectorF_t(goal.x-pos.x, goal.y-pos.y);
        VectorF_t fToGoalNorm = fToGoal;
        VectorF_t fSpeedNorm= VectorF_t(theSpeed.x,theSpeed.y);
        fToGoalNorm.safe_normalize_myself();
        fSpeedNorm.safe_normalize_myself();
        float maxNudge=1200;

        float cosOfMaxAng=cosf(maxAtteinableAngle*3.1415f/180.f);

        if (fSpeedNorm.dot(fToGoalNorm)<cosOfMaxAng)
        {
            VectorF_t fNormalToSpeed(-fSpeedNorm.y, fSpeedNorm.x);
            float projectionOnNormal = fNormalToSpeed.dot(fToGoal);
            if (projectionOnNormal<-maxNudge)
                projectionOnNormal = -maxNudge;
            else if (projectionOnNormal>maxNudge)
                projectionOnNormal = maxNudge;
            //cerr<<"--KEEP TRACK proj="<<projectionOnNormal<<endl;
            VectorF_t nudge(projectionOnNormal*fNormalToSpeed.x,projectionOnNormal*fNormalToSpeed.y);
            goal.x += nudge.x;      
            goal.y += nudge.y;      
        }

    }

    bool handle_overshoot(const VectorF_t &pos, const VectorF_t &speed, const VectorF_t &target, float &overshoot_mult)
    {
        overshoot_mult = 1;
        int maxAntic = 4;
        for (int goalAntic = 1; goalAntic <= maxAntic; goalAntic++)
        {
            VectorF_t futurePos(pos.x+speed.x*goalAntic, pos.y+speed.y*goalAntic);
            VectorF_t toTarget(target-pos);
            VectorF_t futureToTarget(target-futurePos);

            if (toTarget.x*futureToTarget.x+toTarget.y*futureToTarget.y < 0)
            {
                overshoot_mult = 1-(maxAntic-(goalAntic-1))/(float)(maxAntic);
                return true;
            }
        }
        return false;
    }


};


int main()
{
    bool used_boost = false;
    int lapCount=0;
    int checkpointCount=0;
    const int podCount = 2;
    std::vector<VectorF_t> checkpoints;

    std::vector<Pod_t> myPods;
    std::vector<Pod_t> enemyPods;

    cin >> lapCount; cin.ignore();
    cin >> checkpointCount; cin.ignore();
    for (int i=0; i<checkpointCount; i++)
    {
        VectorF_t checkpoint;
        cin >> checkpoint.x >> checkpoint.y;
        cin.ignore();
        checkpoints.push_back(checkpoint);
    }
    
    for (int i=0; i<podCount; i++)
    {
        myPods.emplace_back();
        enemyPods.emplace_back();
    }

    // game loop
    while (1) {
        for (int i=0; i<podCount; i++)
            myPods[i].startFrame(checkpoints);
        for (int i=0; i<podCount; i++)
            enemyPods[i].startFrame(checkpoints);


        for (int i=0; i<podCount; i++)
        {
            myPods[i].updateMyAI( myPods, enemyPods);
        }

        //output
        for (int i=0; i<podCount; i++)
            myPods[i].output();
    
        for (int i=0; i<podCount; i++)
            myPods[i].endFrame();
        for (int i=0; i<podCount; i++)
            enemyPods[i].endFrame();
    }
}
