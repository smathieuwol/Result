#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

const int maxthurst = 100;


struct Vector_t
{
    Vector_t() : x(0), y(0) {}
    Vector_t(int _x, int _y)  :x(_x), y(_y) {}
    Vector_t operator+(const Vector_t &other) const {return Vector_t(x+other.x, y+other.y);}
    Vector_t operator-(const Vector_t &other) const {return Vector_t(x-other.x, y-other.y);}
    float len() const {return sqrtf(x*x+y*y);}
    int x,y;
};

struct VectorF_t
{
    VectorF_t() : x(0), y(0) {}
    VectorF_t(float _x, float _y)  :x(_x), y(_y) {}
    VectorF_t operator+(const VectorF_t &other) const {return VectorF_t(x+other.x, y+other.y);}
    VectorF_t operator-(const VectorF_t &other) const {return VectorF_t(x-other.x, y-other.y);}
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

bool check_can_boost(int next_checkpoint_angle, int next_checkpoint_dist, bool &used_boost)
{
    const int min_boost_dist = 2000;
    const int max_boost_angle_to_target = 10;
    bool now_boost = false;
    if (abs(next_checkpoint_angle)<max_boost_angle_to_target && next_checkpoint_dist>min_boost_dist && !used_boost)
    {
        cerr << "------------BOOST " << next_checkpoint_dist <<endl;
        used_boost = true;
        now_boost = true;
    }
    return now_boost;
}
float vector_len(const Vector_t &delta)
{
    float len=sqrtf(delta.x*delta.x+delta.y*delta.y);
    return len;
}

void handle_avoidance(const Vector_t &pos, 
    const Vector_t &oppPos, 
    const Vector_t &oppSpeed, 
    float dist_to_target,
    Vector_t &itarget)
{
    const float pod_radius = 1200;
    const float min_avoid_dist = 2000.0f;
    const float threshold_dist_to_goal = 1500;;

    cerr<<"DIST TO TARGET="<<dist_to_target<<endl;
    if (dist_to_target<threshold_dist_to_goal)
    {
        cerr<<"TOO NEAR OF TARGET"<<endl;
        return;
    }

    Vector_t ito_opp = (oppPos)-pos;
    float opp_dist = ito_opp.len();
cerr<<"---dist to opp="<<(int)opp_dist<<endl;
    if (opp_dist>min_avoid_dist)
        return;
    
    float avoidance_cone_tan = pod_radius/opp_dist;
    float avoidance_cone_angle = atanf(avoidance_cone_tan);

    cerr<<"----------------------cone_angle="<<(int)(avoidance_cone_angle*180/3.1415f)<<endl;

    Vector_t idesired_speed = itarget-pos;
    VectorF_t fto_opp_dir(ito_opp.x, ito_opp.y);
    fto_opp_dir.safe_normalize_myself();
    VectorF_t to_goal_dir(itarget.x-pos.x, itarget.y-pos.y);    
    to_goal_dir.safe_normalize_myself();
    VectorF_t opp_dir(oppSpeed.x, oppSpeed.y);
    opp_dir.safe_normalize_myself();
    VectorF_t my_relative_fake_speed(to_goal_dir);//-opp_dir);

    bool hasFoundAngle = false;
    //cerr<<"---CHECK AVD "<<endl;
    int maxAngle = 40;
    for (int angleStep=0; angleStep<=maxAngle; angleStep+=10)
    {
        for (int isign=-1; isign<=1; isign+=2)
        {
            bool isLast = (angleStep==maxAngle);
            float angleRad = (isign*angleStep) *(3.1415f/180.f);
            VectorF_t rotated_fake_relative_speed = my_relative_fake_speed.rotated_vector(angleRad);
            //is the rotated fake speed in the cone to the mobile?
            rotated_fake_relative_speed.safe_normalize_myself();
            cerr<<"-speedDir="<<atan2f(-rotated_fake_relative_speed.y, rotated_fake_relative_speed.x)*(180.f/3.1415f) << "  ";
            cerr<<"-toenemyDir="<<atan2f(-fto_opp_dir.y, fto_opp_dir.x)*(180.f/3.1415f) << "  ";
            cerr<<endl;
            if (isLast || rotated_fake_relative_speed.dot(fto_opp_dir)<cosf(avoidance_cone_angle))
            {
                //no, add a nudge corresponding to this speed
                VectorF_t fto_goal(itarget.x-pos.x, itarget.y-pos.y);
                fto_goal = fto_goal.rotated_vector(angleRad);
                Vector_t initTarget = itarget;
                itarget = Vector_t(pos.x+(int)fto_goal.x, pos.y+(int)fto_goal.y);
                hasFoundAngle = true;
                if (angleStep!=0)
                {
                    cerr<<"---INITIAL TARGET= "<<initTarget.x<<" "<<initTarget.y<<endl;
                    cerr<<"---ROTATED OF "<<(isign*angleStep)<<endl;
                    cerr<<"---NUDGED TARGET= "<<itarget.x<<" "<<itarget.y<< " ANG="<<atan2f(-fto_goal.y, fto_goal.x)*(180.f/3.1415f)<<endl;
                }
                break;
            }
        }
        if (hasFoundAngle)
            break;
    }
}

void keep_track(const Vector_t &pos, const Vector_t &speed, Vector_t &goal)
{
    const float maxAtteinableAngle = 5.f;
    VectorF_t fToGoal = VectorF_t(goal.x-pos.x, goal.y-pos.y);
    VectorF_t fToGoalNorm = fToGoal;
    VectorF_t fSpeedNorm= VectorF_t(speed.x,speed.y);
    fToGoalNorm.safe_normalize_myself();
    fSpeedNorm.safe_normalize_myself();
    float maxNudge=500;

    float cosOfMaxAng=cosf(maxAtteinableAngle*3.1415f/180.f);

    if (fSpeedNorm.dot(fToGoalNorm)<cosOfMaxAng)
    {
        VectorF_t fNormalToSpeed(-fSpeedNorm.y, fSpeedNorm.x);
        float projectionOnNormal = fNormalToSpeed.dot(fToGoal);
        if (projectionOnNormal<-maxNudge)
            projectionOnNormal = -maxNudge;
        else if (projectionOnNormal>maxNudge)
            projectionOnNormal = maxNudge;
        cerr<<"--KEEP TRACK proj="<<projectionOnNormal<<endl;
        VectorF_t nudge(projectionOnNormal*fNormalToSpeed.x,projectionOnNormal*fNormalToSpeed.y);
        goal.x += (int)nudge.x;      
        goal.y += (int)nudge.y;      
    }

}

int main()
{
    bool used_boost = false;
    Vector_t prevPos;
    Vector_t prevOppPos;
    int prev_opp_x=0, prev_opp_y=0;
    bool is_first_frame=true;
    const int nbAnticipation=10;
    const float cosOfFrontOfUsAngle = 0.7f;

    int consecutive_collision_count=0;
    // game loop
    while (1) {
        Vector_t pos;
        int next_checkpoint_x; // x position of the next check point
        int next_checkpoint_y; // y position of the next check point
        int next_checkpoint_dist; // distance to the next checkpoint
        int next_checkpoint_angle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> pos.x >> pos.y >> next_checkpoint_x >> next_checkpoint_y >> next_checkpoint_dist >> next_checkpoint_angle; cin.ignore();
        Vector_t oppPos;
        cin >> oppPos.x >> oppPos.y; cin.ignore();

        bool will_collide=false;
        bool will_overshoot = false;
        Vector_t vectToEnemy = oppPos-pos;
        Vector_t target(next_checkpoint_x, next_checkpoint_y);

        int thurst = 0;
        int overshoot_mult = 1;
        if (!is_first_frame)
        {
            Vector_t oppSpeed=oppPos-prevOppPos;
            Vector_t speed = pos-prevPos;
            keep_track(pos, speed, target);
            handle_avoidance(pos, oppPos, oppSpeed, next_checkpoint_dist, target);
            //OVERSHOOT
            int maxAntic = 3;
            for (int goalAntic = 1; goalAntic <= 4; goalAntic++)
            {
                Vector_t futurePos(pos.x+speed.x*goalAntic, pos.y+speed.y*goalAntic);
                Vector_t toTarget(next_checkpoint_x-pos.x, next_checkpoint_y-pos.y);
                Vector_t futureToTarget(next_checkpoint_x-futurePos.x,next_checkpoint_y-futurePos.y);
                if (toTarget.x*futureToTarget.x+toTarget.y*futureToTarget.y < 0)
                {
                    //goalAntic = 1 => max-goalAntic = 3
                    //goalAntic = 3 => max-goalAntic = 1
                    overshoot_mult = 1-(maxAntic-goalAntic)/(float)(maxAntic);

                    will_overshoot = true;
                    break;
                }
            }

        }
        bool now_boost = check_can_boost(next_checkpoint_angle, next_checkpoint_dist, used_boost);
        
        if (abs(next_checkpoint_angle)<90)
        {
            thurst = maxthurst;
            if (will_overshoot)
                thurst = (int)(thurst*overshoot_mult);
        }


        //output
        cout << target.x << " " << target.y << " ";
        if (now_boost)
            cout << "BOOST";
        else
            cout << thurst;
        cout << endl;
    
        prevPos = pos;
        prevOppPos = oppPos;
        is_first_frame = false;
    }
}