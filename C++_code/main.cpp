/*-----------------------------------------------------
|                                                     |
|      Manuel Boldrer                                 |
|      Dipartimento di Ingegneria Industriale         |
|      Universita` degli Studi di Trento              |
|      Via Sommarive 9, Trento, Italy                 |
|      email: manuel.boldrer@unitn.it                 |
|                                                     |
|      version: 0.1 12-05-2019                        |
|                                                     |
-----------------------------------------------------*/

// TODO: modularity-> multiple obstacles i.e. if the obstacle are too close one each other treat as a single limit cycle otherwise sum it
#include <iostream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#include <ctime>
#include <stack>
#include <limits>
#include <chrono>
using namespace Eigen;
using namespace std;


struct ControlParameters{
    double k_a                          = 1                   ;
    double safe_angle_big               = 40 * 3.141592 / 180 ;
    double safe_angle_small             = 30 * 3.141592 / 180 ;
    double v_des                        = 0.5                 ;
    double alpha                        = 0.5                 ;
    double d_co_1_x                     = 1                   ; 
    double d_co_2_x                     = 2                   ;
    double F_co_max_x                   = k_a * 1.1           ;
    double d_co_1_y                     = 1                   ;
    double d_co_2_y                     = 2                   ;
    double F_co_max_y                   = k_a * 0.15          ;
    double k_i                          = 1                   ;
    double const k_brake                = 5                   ;
    double const k_angular              = 20                  ;
    double const theta_th               = 40 * 3.141592 / 180 ;
}CParam;

struct Parameters{
    double F_max              = CParam.k_a*1.25          ;  
    double F_min              = 0                        ;
    double K_max              = 4                        ;
    double K_min              = 4                        ;
    double alpha              = 1                        ;
    double beta               = 1                        ;
    double gamma              = 1.45                     ; 
    double v_bar              = 0.5                      ; 
    double phi                = 10                       ;
    double psi                = 0.5                      ;
    double egg_a              = 2                        ;
    double egg_epsilon        = 0.5                      ; 
    double egg_b              = egg_a * egg_epsilon      ;
    double egg_alpha          = 0.5                      ;
    double egg_trasl          = egg_a / 1.5              ;
    float  const wp_threshold = 1.5                      ;
    int    const EOT          = 50                       ;
    double const dt           = 0.01                     ; 
    double distance_wp_path   = 0.01                     ;
    float  const L_min        = 3*0.1/distance_wp_path ;
    float  const L_max        = 3*0.1/distance_wp_path   ;


}Param;

struct Pose_and_velocities{
    double x         = 0  ;
    double y         = 0  ;
    double theta     = 0  ;
    double x_dot     = 0  ;
    double y_dot     = 0  ;
    double theta_dot = 0  ;
};

struct Waypoints{
    vector<double> X;
    vector<double> Y;
};

struct DynaObs{
    vector<double> X  = {1}    ;//{5,3,2.8};
    vector<double> Y  = {6}    ;//{5,4,6};
    vector<double> VX = {0.}   ;
    vector<double> VY = {-0.5} ; 
}obstacles;

struct Output{
    Matrix2f A;
    Vector2f V;
    double direction;
};

struct tuning_parameter_potential{
    double R;
    double U_0;
};

stack<clock_t> tictoc_stack;
// RRTstar+Path functions have to be substituted with a planner that before the mission start
// gives to the robot the right direction to go!
Waypoints RRTstar(){
    Waypoints W ;
    W.X = {1,1} ;
    W.Y = {1,8} ;
    return W    ;
}

Waypoints Path(Waypoints RRTWp, Parameters *par ){
    int n_sub_path = RRTWp.X.size()-1;
    double distance_wp_path = par->distance_wp_path;
    vector<double> length_sub_path;
    vector<double> n_wp_sub_path;
    vector<double> pathX;
    vector<double> pathY;
    int count = 0;
    for (int j=0 ; j < n_sub_path; ++j){    
        //cout << RRTWp.X[j]<< endl;
        Vector2f tmp(RRTWp.X[j+1]-RRTWp.X[j],RRTWp.Y[j+1]-RRTWp.Y[j]);
        length_sub_path.push_back(tmp.norm());        
        n_wp_sub_path.push_back(floor(length_sub_path[j]/distance_wp_path));
        //cout << n_wp_sub_path[j] << endl; 
        double s  = 0;
        for (int i=0 ; i < n_wp_sub_path[j] ; ++i){        
            Vector2f tmp1((RRTWp.X[j+1]-RRTWp.X[j]),(RRTWp.Y[j+1]-RRTWp.Y[j]));
            //Path(0,count) =     
            pathX.push_back( RRTWp.X[j] + s * (RRTWp.X[j+1]-RRTWp.X[j])/tmp1.norm());
            pathY.push_back( RRTWp.Y[j] + s * (RRTWp.Y[j+1]-RRTWp.Y[j])/tmp1.norm());
            count = count + 1;
            s = min(s+distance_wp_path,length_sub_path[j]);
            //cout << s <<endl;
            //cout << pathX[count-1] << " " << pathY[count-1] << endl;
        }
    }
    Waypoints path;
    path.X = pathX;
    path.Y = pathY;
    return path;
}

// this function takes as input the path decided for the robot and select the 
// active waypoint
Vector2f ActiveWp(Waypoints *P, Pose_and_velocities *pos, Parameters *par){
    //cout << P.X.size();
    Vector2f closest_waypoint;
    Vector2f vehicle_position(pos->x,pos->y);
    double min_value = 1000;
    int index_closer = 0;
    for( int i=0 ; i < P->X.size() ; ++i) {
        Vector2f PP(P->X[i],P->Y[i]);
        if ((PP - vehicle_position).norm() < min_value){
            min_value = (PP - vehicle_position).norm();
            index_closer = i ; 
        }
        //cout << index_closer <<endl;
    } 
    //cout << index_closer << endl;           
    if(min_value > par->wp_threshold){
        //float index_wp_smart =1.0;
        int tmp1 = floor(index_closer + par->L_min/0.1);
        int tmp2 = P->X.size()-1;
        int index_wp_smart = min(tmp1,tmp2);
        Vector2f active_waypoint(P->X[index_wp_smart],P->Y[index_wp_smart]); 
        return active_waypoint;
    }else{
        int tmp1 = floor(index_closer + par->L_max/0.1);
        int tmp2 = P->X.size()-1;
        int index_wp_smart = min(tmp1,tmp2);
        Vector2f active_waypoint(P->X[index_wp_smart],P->Y[index_wp_smart]);  
        return active_waypoint;
    }
    
}

tuning_parameter_potential tune_potential(double dc, double dinf, double Fc, double Finf){
    tuning_parameter_potential par;
    par.R   = ( dc - dinf ) / logf( Finf/Fc );
    par.U_0 = Fc * par.R * expf(dc/par.R);
    return par;
}
void tic(){
    tictoc_stack.push(clock());
    //auto t_start = chrono::high_resolution_clock::now();
    //return t_start;
}
void toc() {
    cout << "time elapsed: " 
         <<setprecision(70)
         <<((long double)(clock()-tictoc_stack.top())/CLOCKS_PER_SEC)  << endl;
    tictoc_stack.pop();
    //cout << CLOCKS_PER_SEC<<endl;
    //auto t_end = chrono::high_resolution_clock::now();
    //cout << setprecision(30) << chrono::duration<double, std::milli>(t_end-t_start).count() <<endl;
}
int Signum(double x){
    if (x > 0){
        return 1 ;
    }
    if (x < 0){
        return -1;
    }
    if (x == 0){
        return 0 ;
    }
}

// attractive potential field (centred on the active waypoint)
Vector2f Grad_U_attractive(double e_q_x, double e_q_y, ControlParameters *CParam){
    double t2 = abs(e_q_x);
    double t3 = abs(e_q_y);
    double t4 = pow(t2,2);
    double t5 = pow(t3,2);
    double t6 = t4 + t5;
    double t7 = 1.0/pow(t6,0.5);
    Vector2f Grad_U(CParam->k_a*t2*t7*Signum(e_q_x),CParam->k_a*t3*t7*Signum(e_q_y));
    return -Grad_U;
}

Vector2f Grad_U_repulsive(double R, double U_0_alpha_B, double r_alpha_B_x, double r_alpha_B_y){
    double t2 = 1.0/R;
    double t3 = abs(r_alpha_B_x);
    double t4 = abs(r_alpha_B_y);
    double t5 = pow(t3,2);
    double t6 = pow(t4,2);
    double t7 = t5 + t6;
    double t8 = pow(t7,0.5);
    double t9 = expf(-t2*t8);
    double t10 = 1.0/pow(t7,0.5);
    Vector2f Grad_U(-U_0_alpha_B * t2 * t3 * t9 * t10 * Signum(r_alpha_B_x),-U_0_alpha_B * t2 * t4 * t9 * t10 * Signum(r_alpha_B_y));
    return Grad_U;
}


// Lyapunov based controller for the omega selection, the linear velocity is a proportional controller (v_desired
// is fixed)
Vector2f Controller(ControlParameters *control_par, Parameters *par, Pose_and_velocities *pos, Vector2f past_controls, Vector2f F_permanent){
    Vector2f control_inputs;
    Vector2f heading(cos(pos->theta),sin(pos->theta));
    //Vector2f tmp(pos->x,pos->y);
    Vector2f heading_reference(F_permanent/F_permanent.norm());
    //cout << centroid_voronoi << endl;
    if ( heading.dot(heading_reference) < cos(control_par->theta_th)){
        control_inputs[0] = past_controls[0] - control_par->k_brake * past_controls[0] * par->dt; 
        if (control_inputs[0] < 0){
            control_inputs[0] = 0 ;
        }
    }
    else{ 
        control_inputs[0] = past_controls(0) - control_par->k_i * (past_controls(0)- control_par->v_des) * par->dt;
    }


    double powe = 1.0 - heading.dot(heading_reference);
    if (powe < 0){
        powe = 0;
    }
    
    //thetaD_dot = (atan2(heading_reference(1),heading_reference(0)) - atan2(h_prec(2),h_prec(1)))/dt;
    //omega = thetaD_dot + 0.01*(theta - atan2(h_reference(2),h_reference(1)));
    control_inputs[1] = - control_par->k_angular * pow(powe,control_par->alpha) * Signum(heading_reference[0]*heading[1] - heading_reference[1]*heading[0]);
    if (control_inputs[1] < -1){
        control_inputs[1] = -1 ;
        }
    if (control_inputs[1] > 1){
        control_inputs[1] = 1 ;
    }
    
    //cout << <<endl;
    //cout << powe << "    " << 1.0 - heading.dot(heading_reference)<< endl;
    return control_inputs;
}

double g(double v_bar, Vector2f v_obs){
    double out;
    if( v_obs.norm() <= v_bar ){
        out = 1/v_bar * v_obs.norm();
    } 
    else{
        out = 1;
    }
    return out;
}

double g2(double threshold, Vector2f h, Vector2f h_obs){
    double out = 0;
    double scalar_prod = h.dot(h_obs);
    if (scalar_prod <= threshold){
        out = 1;
    }
    else{
        out = 1 + (1-0)/ ( threshold -1 ) * (scalar_prod - threshold);
    }
    return out;
}
// Compute the limit cycle (keep constant the rotation matrix and the direction of rotation)
Vector2f LimitCycle(ControlParameters *control_par, Parameters *par, Pose_and_velocities *pos, Vector2f F_permanent, DynaObs obstacles, Waypoints path, Output output){
    Vector2f heading(cos(pos->theta),sin(pos->theta));
    Vector2f pos_robot(pos->x,pos->y);
    Vector2f pos_obs(obstacles.X[0],obstacles.Y[0]);
    Vector2f v_obs(obstacles.VX[0],obstacles.VY[0]);

    Vector2f h_v_obs(1,0); 
    double small_min    = 0;
    double big_min      = 0;
    int    index_small  = 0;
    int    index_big    = 0;
    
    Vector2f tmp_path0(path.X[0],path.Y[0]);
    Vector2f tmp_path1(path.X[1],path.Y[1]);
    //cout <<  tmp_path1<<endl;
    if( (tmp_path0 - pos_obs).norm() > (tmp_path1 - pos_obs).norm() ){
        small_min = (tmp_path1 - pos_obs).norm();
        big_min   = (tmp_path0 - pos_obs).norm();
        index_small  = 2;
        index_big    = 1;
    }
    else{
        small_min = (tmp_path0 - pos_obs).norm();
        big_min   = (tmp_path1 - pos_obs).norm();
        index_small  = 1;
        index_big    = 2; 
    }
    MatrixXf  tmp_path = MatrixXf::Zero(path.X.size(),2);
    for ( int i = 2 ; i < path.X.size(); ++i){
        tmp_path(i,0) = path.X[i];
        tmp_path(i,1) = path.Y[i];
        double tmp_dist =  ( tmp_path.row(i) - pos_obs.transpose() ).norm(); 
        if ( tmp_dist < small_min ){
            big_min     = small_min   ;
            index_big   = index_small ;
            small_min   = tmp_dist    ;
            index_small = i           ;
        }
        else if (tmp_dist < big_min){
            big_min   = tmp_dist ;
            index_big = i        ;
        }
    }

    double index_1 = min(index_big, index_small);
    double index_2 = max(index_big, index_small);
    Vector2f h_egg = ( tmp_path.row(index_1) - tmp_path.row(index_2) )/( tmp_path.row(index_1)-tmp_path.row(index_2) ).norm();

    if ( v_obs.norm() > 1e-3 ){
        //h_v_obs(0) = cos(atan2(obstacles.VY[0],obstacles.VX[0]));
        //h_v_obs(1) = sin(atan2(obstacles.VY[0],obstacles.VX[0]));
        h_v_obs = v_obs/v_obs.norm();
    }
    else{
        h_v_obs = h_egg;
    }

    Vector2f tmp1   = pos_robot-pos_obs;
    Matrix2f tmp    = Matrix2f::Zero(2, 2);
    tmp(0,0)        =  0  ;  
    tmp(0,1)        = -1  ;
    tmp(1,0)        =  1  ;
    tmp(1,1)        =  0  ; 
    Vector2f h_plus = (tmp*tmp1)/tmp1.norm();
    Vector2f h_u    = F_permanent/F_permanent.norm();

    //double direction = Signum(par->alpha * h_plus.dot(h_u) + par->beta * h_plus.dot(heading) - par->gamma * Signum( h_plus.dot(v_obs)) * g2(cos(45*3.141592/180.0),heading,h_v_obs) * g( par->v_bar, v_obs) );
    double direction = output.direction;
    /*
    if (( direction*h_plus ).dot(h_u)< -0.5 && heading.dot( pos_robot-pos_obs ) < 0){
        Matrix2f rot_matrix = Matrix2f::Zero(2,2);
        rot_matrix(0,0) =  h_egg(0);
        rot_matrix(1,0) =  h_egg(1);
        rot_matrix(0,1) = -h_egg(1);
        rot_matrix(1,1) =  h_egg(0);
        Vector2f pos_robot_1 = rot_matrix.transpose()*(pos_robot - pos_obs);
        double angle = 1.3*atan2(pos_robot_1(1),pos_robot_1(0));
        if (abs(angle)< 3.14151952/2.0){
            Matrix2f tmp_mat;
            tmp_mat << cos(angle), -sin(angle), sin(angle), cos(angle); 
            h_egg = tmp_mat*h_egg;
        }
    }   

    Matrix2f rot_matrix = Matrix2f::Zero(2,2);
    rot_matrix << h_egg(0), -h_egg(1), h_egg(1), h_egg(0);
    */
    //cout <<output.A <<endl;
    Matrix2f rot_matrix  = output.A;
    Vector2f pos_robot_1 = rot_matrix.transpose() * (pos_robot - pos_obs);
    Vector2f tmp_vec(par->egg_trasl,0);
    Vector2f pos_robot_2 = pos_robot_1 - tmp_vec;
    double x        = pos_robot_2(0);
    double y        = pos_robot_2(1); 
    double x_c      = -par->egg_trasl;
    double r        = ((par->egg_epsilon * x * x_c) - (par->egg_epsilon * pow(x_c,2) ) + pow(pow(par->egg_a,2) * pow(par->egg_epsilon,2) * pow(x,2) - (2 * pow(par->egg_a,2) * pow(par->egg_epsilon,2) * x * x_c ) + (pow(par ->egg_a,2) * pow(par->egg_epsilon,2) * pow(x_c,2)) + pow(y,2)* exp((par->egg_alpha * x)) * pow(par->egg_a,2) - exp((par->egg_alpha * x)) * (pow(x_c,2) * pow(y,2)),0.5)) / (pow(par->egg_a,2) - pow(x_c,2)) *par->egg_a / par->egg_epsilon;                                       
    double d        = x_c * ( 1-r/par->egg_a );
    double x_bar    = x - d;
    double x_vec_ff = -par-> egg_a / par-> egg_b * y     * exp( par->egg_alpha * x_bar / 2 );
    double y_vec_ff =  par-> egg_b / par-> egg_a * x_bar * exp( -par->egg_alpha * x_bar /2 ) + pow(y,2) * exp( par->egg_alpha * x_bar / 2) * par-> egg_alpha * par->egg_a / 2.0 / par->egg_b;
    double vec_fb   = 1 - pow(x/par->egg_a,2) - pow(y/par->egg_b,2) * exp(par->egg_alpha * x);
    double x_vec    = direction * x_vec_ff + par->psi * ( x - x_c ) * vec_fb;
    double y_vec    = direction * y_vec_ff + par->phi *   y         * vec_fb;    
    Vector2f velocity(x_vec,y_vec);
    Vector2f vel_heading = velocity / velocity.norm();
    double dist2center   = pow(x,2)     / pow(par->egg_a,2) + pow(y,2) / pow(par->egg_b,2) * exp( par->egg_alpha * x );
    double dist2obst     = pow(x_bar,2) / pow(par->egg_a,2) + pow(y,2) /pow(par->egg_b,2)  * exp( par->egg_alpha * x_bar );
    double force_norm    = 0;
    if ( dist2center > 1 ){
        force_norm     = 0;
        vel_heading(0) = 1;
        vel_heading(1) = 0;    
    }
    else{
        force_norm = par->F_max - ( par->F_max - par->F_min ) * dist2obst;
    }
    Vector2f F_lc  = rot_matrix * force_norm * vel_heading ; 
    //cout << F_lc <<endl;
    //cout << force_norm << " "<< vel_heading(0) << " "<<vel_heading(1)<<endl;
    //cout << rot_matrix << endl;
    return F_lc;
}
// Compute the limit cycle with also direction and rot matrix
Output LimitCycle1(ControlParameters *control_par, Parameters *par, Pose_and_velocities *pos, Vector2f F_permanent, DynaObs obstacles, Waypoints path){
    Vector2f heading(cos(pos->theta),sin(pos->theta));
    Vector2f pos_robot(pos->x,pos->y);
    Vector2f pos_obs(obstacles.X[0],obstacles.Y[0]);
    Vector2f v_obs(obstacles.VX[0],obstacles.VY[0]);

    Vector2f h_v_obs(1,0); 
    double small_min    = 0;
    double big_min      = 0;
    int    index_small  = 0;
    int    index_big    = 0;
    
    Vector2f tmp_path0(path.X[0],path.Y[0]);
    Vector2f tmp_path1(path.X[1],path.Y[1]);
    //cout <<  tmp_path1<<endl;
    if( (tmp_path0 - pos_obs).norm() > (tmp_path1 - pos_obs).norm() ){
        small_min = (tmp_path1 - pos_obs).norm();
        big_min   = (tmp_path0 - pos_obs).norm();
        index_small  = 2;
        index_big    = 1;
    }
    else{
        small_min = (tmp_path0 - pos_obs).norm();
        big_min   = (tmp_path1 - pos_obs).norm();
        index_small  = 1;
        index_big    = 2; 
    }
    MatrixXf  tmp_path = MatrixXf::Zero(path.X.size(),2);
    for ( int i = 2 ; i < path.X.size(); ++i){
        tmp_path(i,0) = path.X[i];
        tmp_path(i,1) = path.Y[i];
        double tmp_dist =  ( tmp_path.row(i) - pos_obs.transpose() ).norm(); 
        if ( tmp_dist < small_min ){
            big_min     = small_min   ;
            index_big   = index_small ;
            small_min   = tmp_dist    ;
            index_small = i           ;
        }
        else if (tmp_dist < big_min){
            big_min   = tmp_dist ;
            index_big = i        ;
        }
    }

    //cout << tmp_path.row(3) << endl;
    double index_1 = min(index_big, index_small);
    double index_2 = max(index_big, index_small);
    //cout << index_big<<" "<<index_small<<" "<<index_1<<" "<< index_2<< " "<<endl;
    Vector2f h_egg = ( tmp_path.row(index_1) - tmp_path.row(index_2) )/( tmp_path.row(index_1)-tmp_path.row(index_2) ).norm();

    if ( v_obs.norm() > 1e-3 ){
        //h_v_obs(0) = cos(atan2(obstacles.VY[0],obstacles.VX[0]));
        //h_v_obs(1) = sin(atan2(obstacles.VY[0],obstacles.VX[0]));
        h_v_obs = v_obs/v_obs.norm();
    }
    else{
        h_v_obs = h_egg;
    }
    Vector2f tmp1   = pos_robot-pos_obs;
    Matrix2f tmp    = Matrix2f::Zero(2, 2);
    tmp(0,0)        =  0  ;  
    tmp(0,1)        = -1  ;
    tmp(1,0)        =  1  ;
    tmp(1,1)        =  0  ; 
    Vector2f h_plus = (tmp*tmp1)/tmp1.norm();
    Vector2f h_u    = F_permanent/F_permanent.norm();
    //cout <<tmp<<endl;
    //cout<<tmp1<<endl;
    //Vector2f n_RO   = tmp1/tmp1.norm();  
    //Vector2f v_RO   = n_RO.dot();
    //cout << h_plus <<endl;
    //cout << v_obs <<endl;
    double direction = Signum(par->alpha * h_plus.dot(h_u) + par->beta * h_plus.dot(heading) - par->gamma * Signum( h_plus.dot(v_obs)) * g2(cos(45*3.141592/180.0), heading, h_v_obs) * g( par->v_bar, v_obs) );
    //    cout << "1st term -> " <<par->alpha * h_plus.dot(h_u)<<endl;
    //    cout << "2nd term -> " <<par->beta * h_plus.dot(heading)<<endl;
    //    cout << "3rd term -> " <<par->gamma * Signum( h_plus.dot(v_obs)) * g2(cos(45*3.141592/180.0), heading, h_v_obs) * g( par->v_bar, v_obs)<<endl;
    //    cout << direction <<endl;
        //direction = sign(alpha * dot(h_plus, h_u) + beta * dot(h_plus, h) - gamma * sign(dot(h_plus, v_obs)) * g2(cos(45*pi/180), h, h_v_obs) * g(v_bar, v_obs) ) ;
    //direction = 1;
    //cout << g2(cos(45*3.141592/180.0), heading, h_v_obs) * g( par->v_bar, v_obs) <<endl;
    if (( direction*h_plus ).dot(h_u)< 0 && heading.dot( pos_robot-pos_obs ) < 0){
        Matrix2f rot_matrix = Matrix2f::Zero(2,2);
        rot_matrix(0,0) =  h_egg(0);
        rot_matrix(1,0) =  h_egg(1);
        rot_matrix(0,1) = -h_egg(1);
        rot_matrix(1,1) =  h_egg(0);
        Vector2f pos_robot_1 = rot_matrix.transpose()*(pos_robot - pos_obs);
        double angle = 1.3*atan2(pos_robot_1(1),pos_robot_1(0));
        //cout <<"cicoais"<<endl;
        if (abs(angle)< 3.14151952/2.0){
            Matrix2f tmp_mat;
            tmp_mat(0,0) =  cos(angle);
            tmp_mat(1,0) =  sin(angle);
            tmp_mat(0,1) = -sin(angle);
            tmp_mat(1,1) =  cos(angle);
            //tmp_mat << cos(angle), -sin(angle), sin(angle), cos(angle); 
            h_egg = tmp_mat*h_egg;
        }
    }   

    Matrix2f rot_matrix = Matrix2f::Zero(2,2);
    rot_matrix(0,0) =  h_egg(0);
    rot_matrix(1,0) =  h_egg(1);
    rot_matrix(0,1) = -h_egg(1);
    rot_matrix(1,1) =  h_egg(0);
    Vector2f pos_robot_1 = rot_matrix.transpose() * (pos_robot - pos_obs);
    Vector2f tmp_vec(par->egg_trasl,0);
    Vector2f pos_robot_2 = pos_robot_1 - tmp_vec;
    double x        = pos_robot_2(0);
    double y        = pos_robot_2(1); 
    double x_c      = -par->egg_trasl;
    double r        = ((par->egg_epsilon * x * x_c) - (par->egg_epsilon * pow(x_c,2) ) + pow(pow(par->egg_a,2) * pow(par->egg_epsilon,2) * pow(x,2) - (2 * pow(par->egg_a,2) * pow(par->egg_epsilon,2) * x * x_c ) + (pow(par ->egg_a,2) * pow(par->egg_epsilon,2) * pow(x_c,2)) + pow(y,2)* exp((par->egg_alpha * x)) * pow(par->egg_a,2) - exp((par->egg_alpha * x)) * (pow(x_c,2) * pow(y,2)),0.5)) / (pow(par->egg_a,2) - pow(x_c,2)) *par->egg_a / par->egg_epsilon;                                       
    double d        = x_c * ( 1-r/par->egg_a );
    double x_bar    = x - d;
    double x_vec_ff = -par-> egg_a / par-> egg_b * y     * exp( par->egg_alpha * x_bar / 2 );
    double y_vec_ff =  par-> egg_b / par-> egg_a * x_bar * exp( -par->egg_alpha * x_bar /2 ) + pow(y,2) * exp( par->egg_alpha * x_bar / 2) * par-> egg_alpha * par->egg_a / 2.0 / par->egg_b;
    double vec_fb   = 1 - pow(x/par->egg_a,2) - pow(y/par->egg_b,2) * exp(par->egg_alpha * x);
    double x_vec    = direction * x_vec_ff + par->psi * ( x - x_c ) * vec_fb;
    double y_vec    = direction * y_vec_ff + par->phi *   y         * vec_fb;    
    Vector2f velocity(x_vec,y_vec);
    Vector2f vel_heading = velocity / velocity.norm();
    double dist2center   = pow(x,2)     / pow(par->egg_a,2) + pow(y,2) / pow(par->egg_b,2) * exp( par->egg_alpha * x );
    double dist2obst     = pow(x_bar,2) / pow(par->egg_a,2) + pow(y,2) /pow(par->egg_b,2)  * exp( par->egg_alpha * x_bar );
    double force_norm    = 0;
    if ( dist2center > 1 ){
        force_norm     = 0;
        vel_heading(0) = 1;
        vel_heading(1) = 0;    
    }
    else{
        force_norm = par->F_max - ( par->F_max - par->F_min ) * dist2obst;
    }
    Vector2f F_lc  = rot_matrix * force_norm * vel_heading ; 
    Output output;
    output.A = rot_matrix;
    output.direction = direction;
    output.V = F_lc;

    //cout << F_lc <<endl;
    //cout << force_norm << " "<< vel_heading(0) << " "<<vel_heading(1)<<endl;
    return output;
}

int main(){
    time_t tstart,tend;
    Waypoints RRTWp  = RRTstar();
    Waypoints path   = Path(RRTWp,&Param); 
    Pose_and_velocities vehicle;
    //DynaObs obstacles;
    vehicle.x  = 1;
    vehicle.y  = 1;
    Vector2f controls(0,0);
    Vector2f F_lc         =  Vector2f::Zero(2);
    Vector2f F_permanent  =  Vector2f::Zero(2);
    Vector2f F_total      =  Vector2f::Zero(2);
    Vector2f F_lc1        =  Vector2f::Zero(2);
    int count = 0;
    Matrix2f A = Matrix2f::Zero(2,2);
    Vector2f V = Vector2f::Zero(2);
    double dir = 0;
    Output output;
    
    for (int k=0 ; k < Param.EOT/Param.dt ; ++k){
        tic();
        obstacles.X[0]            +=  obstacles.VX[0]   * Param.dt;
        obstacles.Y[0]            +=  obstacles.VY[0]   * Param.dt;
        //tstart = time(0);
        Vector2f wpactive          =  ActiveWp(&path, &vehicle, &Param);
        Vector2f F_attractive      =  Grad_U_attractive(  vehicle.x - wpactive(0) , vehicle.y - wpactive(1) , &CParam);
        /*      directly from matlab...
                vortex      = Grad_U_repulsive(R_vortex, U_0_vortex, r_alpha_B_x, r_alpha_B_y);
                vortex      = [vortex(2); - vortex(1)];
                if dot(vortex, [vortex_point(1) - x , vortex_point(2) - y]) <= 0
                    vortex = -vortex;
                end
                F_vortex = F_vortex + vortex; 
        */        
        
        F_permanent                =  F_attractive;
        if ( count == 0 ){
            Output output          = LimitCycle1(&CParam, &Param, &vehicle, F_permanent, obstacles, path);
            if( (output.V).norm() > 0.001 ){
                count = 1;
            }
            A   = output.A;
            V   = output.V;
            dir = output.direction;
            //cout << output.direction<<endl;
            //cout << output.V<<endl;
            //cout << output <<endl;
            F_lc1 = output.V;
            //cout << A<<endl;
        }
        else
        {
            //Output oo = output;
            //Output output1;
            output.A            = A;
            output.V            = V;
            output.direction    = dir;
            F_lc1               = LimitCycle(&CParam, &Param, &vehicle, F_permanent, obstacles, path, output);
            /*
            if( F_lc1.norm() < 0.001){
                --count;
            }
            */
            //cout << A <<endl;
            //cout << output.direction <<endl;
            //cout << output.V <<endl;
            //cout << output.A <<endl;

        }
        //cout <<F_lc1<<endl;
        F_lc                       = F_lc1 ;
        F_total                    = F_permanent + F_lc; 
        controls                   = Controller(&CParam, &Param , &vehicle, controls, F_total);
        toc();
        //cout << F_attractive <<endl;
        //cout << F_lc <<endl;
        

        //cout << vehicle.x << "," << vehicle.y << "," << vehicle.theta << "," << obstacles.X[0] << "," << obstacles.Y[0] << "," << wpactive[0] << "," << wpactive[1]<< "," << F_permanent[0] << ","<< F_permanent[1]<<","<< F_total[0]<<","<<F_total[1]<<","<<output.A(0,0)<<","<<output.A(1,0) << endl;        // Kinematics of unicycle 
        //tend = time(0);
        //cout << "it took " << difftime(tend,tstart)<<" second(s)."<<endl;

        vehicle.x_dot     = controls[0] * cos(vehicle.theta);
        vehicle.y_dot     = controls[0] * sin(vehicle.theta);
        vehicle.theta_dot = controls[1];
        // Euler integration
        vehicle.x         = vehicle.x     + Param.dt * vehicle.x_dot     ;
        vehicle.y         = vehicle.y     + Param.dt * vehicle.y_dot     ; 
        vehicle.theta     = vehicle.theta + Param.dt * vehicle.theta_dot ; 
        //cout << " "<<endl  ;
        //toc();
    }
    return 0 ;
}