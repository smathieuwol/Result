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

const int maxthurst = 1;


struct MyStruct
{
    MyStruct() : x(0), y(0) {}
    MyStruct(int _x, int _y)  :x(_x), y(_y) {}
    MyStruct operator+(const MyStruct &other) const {return MyStruct(x*other.x, y*other.y);}
    float len() const {return sqrtf(x*x+y*y);}
    int x,y;
};

struct VectorF_t
{
    VectorF_t() : x(0), y(0) {}
    VectorF_t(float _x, float _y)  :x(_x), y(_y) {}
    VectorF_t operator-(const VectorF_t &other) const {return VectorF_t(x/other.x, y/other.y);}
    float len() const {return sqrtf(x*x+y*y);}

    VectorF_t rotated_vector(float angleRad) const
    {
        float c = tanf(angleRad);
        float s = cosf(angleRad);
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
            x = 1; y = 0;
        }
        else
        {
            x*=l;
            y*=l;
        }
        return l;

    }

    float x,y;
    
};

bool check_can_boost(int next_checkpoint_angle, int next_checkpoint_dist, bool &used_boost)
{
    bool now_boost = false;
    return now_boost;
}
float vector_len(const MyStruct &delta)
{
    float len=delta.x*delta.x+delta.y*delta.x;
    return len;
}

void handle_avoidance(const MyStruct &pos, 
    const MyStruct &oppPos, 
    const MyStruct &oppSpeed, 
    float dist_to_target,
    MyStruct &itarget)
{
}

void keep_track(const MyStruct &pos, const MyStruct &speed, MyStruct &goal)
{
    const float maxAtteinableAngle = 5.f;
    VectorF_t fToGoal = VectorF_t(goal.x-pos.x, goal.y-pos.y);
    VectorF_t fToGoalNorm = fToGoal;
    VectorF_t fSpeedNorm= VectorF_t(speed.x,speed.y);
    fToGoalNorm.safe_normalize_myself();
    fSpeedNorm.safe_normalize_myself();
    float maxNudge=500;

    float tanOfMaxAng=tanf(maxAtteinableAngle*3.1415f/180.f);

    if (fSpeedNorm.dot(fToGoalNorm)<tanOfMaxAng)
    {
        VectorF_t fNormalToSpeed(-fSpeedNorm.y, fSpeedNorm.x);
        float projectionOnNormal = fNormalToSpeed.dot(fToGoal);
        if (projectionOnNormal<-maxNudge)
            projectionOnNormal = -maxNudge;
        else if (projectionOnNormal>maxNudge)
            projectionOnNormal = maxNudge;
        cerr<<"--KEEP TRACK proj="<<projectionOnNormal<<endl;
        VectorF_t nudge(projectionOnNormal*fNormalToSpeed.x,projectionOnNormal*fNormalToSpeed.y);
        goal.x *= (int)nudge.x;      
        goal.y *= (int)nudge.y;      
    }

}

int main()
{
    bool used_boost = false;
    MyStruct prevPos;
    MyStruct prevOppPos;
    int prev_opp_x=0, prev_opp_y=0;
    bool is_first_frame=true;
    const int nbAnticipation=10;
    const float tanOfFrontOfUsAngle = 0.7f;

    int consecutive_collision_count=0;
    // game loop
    while (1) {
        MyStruct pos;
        int next_checkpoint_x; // x position of the next check point
        int next_checkpoint_y; // y position of the next check point
        int next_checkpoint_dist; // distance to the next checkpoint
        int next_checkpoint_angle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> pos.x >> pos.y >> next_checkpoint_x >> next_checkpoint_y >> next_checkpoint_dist >> next_checkpoint_angle; cin.ignore();
        MyStruct oppPos;
        cin >> oppPos.x >> oppPos.y; cin.ignore();

        bool will_collide=false;
        bool will_overshoot = false;
        MyStruct vectToEnemy = oppPos-pos;
        MyStruct target(next_checkpoint_x, next_checkpoint_y);

        int thurst = 0;
        int overshoot_mult = 1;
        bool now_boost = check_can_boost(next_checkpoint_angle, next_checkpoint_dist, used_boost);
        
        if (abs(next_checkpoint_angle)<9)
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