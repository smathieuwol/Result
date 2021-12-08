#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

int main()
{
    bool used_boost = false;
    const int min_boost_dist = 2000;
    const int max_boost_angle_to_target = 10;

    // game loop
    while (1) {
        int x;
        int y;
        int next_checkpoint_x; // x position of the next check point
        int next_checkpoint_y; // y position of the next check point
        int next_checkpoint_dist; // distance to the next checkpoint
        int next_checkpoint_angle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> x >> y >> next_checkpoint_x >> next_checkpoint_y >> next_checkpoint_dist >> next_checkpoint_angle; cin.ignore();
        int opponent_x;
        int opponent_y;        
        cin >> opponent_x >> opponent_y; cin.ignore();

        int thurst = 0;
        bool now_boost = false;
        if (abs(next_checkpoint_angle)<max_boost_angle_to_target && next_checkpoint_dist>min_boost_dist && !used_boost)
        {
            cerr << "------------BOOST " << next_checkpoint_dist <<endl;
            used_boost = true;
            now_boost = true;
        }
        else if (abs(next_checkpoint_angle)<90)
        {
            thurst = 100;
        }
        cout << next_checkpoint_x << " " << next_checkpoint_y << " ";
        if (now_boost)
            cout << "BOOST";
        else
            cout << thurst;
        cout << endl;
    }
}