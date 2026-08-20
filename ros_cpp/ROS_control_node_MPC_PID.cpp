#include <casadi/casadi.hpp>
#define n_states    5
#define n_controls    2
using namespace casadi;

double dT = 0.1; double Tlim = 6; double rh = 0.375;
int    N = 15; //--> Instantaneous prediction horizon
double I_lin = 25.305 + 360.29; //--> Linear inertia (mass + added mass)
double D_lin = 298.28; //--> Linear drag
double I_rot = 8.544; //--> Rotational inertia (moment of inertia + added mass)
double D_rot = 2.8145; //--> Rotational drag
double Fmax = 2*cos(pi/4)*Tlim;
double Tmax = 2*rh*Tlim;

/////////////////////////////////////
    /*Depth variables*/
    bool isItFirstTime = TRUE;
    float offset = 0;
    float Kp = 1000;
    float Kd = 200;
	float eOld = 0;
    float fOld = 0;
    const float Flim = 6;
    int PWM = 0;
    float F = 0;
    float forcePerThruster = 0;
    float setpoint = 5;
    float e = 0;
/////////////////////////////////////
SX f(const SX& st, const SX& con){							// Model function
	SX rhs = SX::sym("rhs",5,1);
	rhs(0) = st(3)*cos(st(2));
	rhs(1) = st(3)*sin(st(2));
	rhs(2) = st(4);
	rhs(3) = (con(0) - D_lin*st(3)*fabs(st(3)))/I_lin;
	rhs(4) = (con(1) - D_rot*st(4)*fabs(st(4)))/I_rot;
	return rhs;
}

int main() {
	// Nonlinear programming problem definition
	/*SX x = SX::sym("x"); 
	SX y = SX::sym("y"); 
	SX psi = SX::sym("psi"); // Global pose states
	SX u = SX::sym("u"); 
	SX r = SX::sym("r"); // Local velocity states
	SXVector states {x, y, psi, u, r}; // Model internal states vector
	int n_states = states.size(); // Number of internal states

	SX F = SX::sym("F"); 
	SX T = SX::sym("T"); // Kinetic controls (Force and Torque)
	SXVector controls {F, T}; // Model controls
	int n_controls = controls.size(); // Number of controls

	SX rhs_1 = u*cos(psi); 
	SX rhs_2 = u*sin(psi); 
	SX rhs_3 = r; 
	SX rhs_4 = (F - D_lin*u*fabs(u))/I_lin;
	SX rhs_5 = (T - D_rot*r*fabs(r))/I_rot; 
	SXVector rhs {rhs_1, rhs_2, rhs_3, rhs_4, rhs_5}; // States rate of change*/

	//Function f = Function("f", {states, controls}, {rhs}); // Model function
	SX U = SX::sym("U", n_controls, N); // Sequence of predictive controls
	SX P = SX::sym("P", n_states + 3); // Initial + desired states vector
	SX X = SX::sym("X", n_states, N+1); // Sequence of state predictions

	for (int i = 0; i < n_states; i++) {
		X(i, 0) = P(i); // Initial kinematic states
	}

	for (int k = 0; k < N; k++) {
		SX st = X(Slice(), k);
		SX con = U(Slice(), k);
		SX f_value = f(st, con);
		//std::cout << f_value << std::endl<< std::endl;
		SX st_next = st + (dT * f_value); // State evolution
		X(Slice(), k+1) = st_next;
	}
	//std::cout << std::endl<< std::endl<< std::endl<< X << std::endl;

	SX loss = SX::sym("loss", 1, 1);; // Objective function initialized to zero
	//SX g = 0; // Constraints on states (none)
	loss = 0; 
	SX Q = SX::zeros(3, 3); Q(0, 0) = 700000; Q(1, 1) = 500000; Q(2, 2) = 50000; // Performance weight matrix
    //SX Q = SX::zeros(3, 3); Q(0, 0) = 7; Q(1, 1) = 5; Q(2, 2) = 5; // Performance weight matrix	
	for (int k = 0; k < N; k++) {
		SX st = X(Slice(0, 3), k);
		loss = loss + mtimes((st - P(Slice(n_states, n_states+3))).T(), mtimes(Q, (st - P(Slice(n_states, n_states+3)))));
	}

	// Optimization variables
	SX OPT_variables = reshape(U, n_controls*N, 1);

	// Nonlinear programming problem
	
	SXDict nlp = { {"x", OPT_variables}, {"f", loss},{"p",P}};
	Dict opts;
	// Options structure
	opts["ipopt.max_iter"] = 100;
	opts["ipopt.print_level"] = 0;
	opts["print_time"] = false;
	opts["ipopt.acceptable_tol"] = 1e-8;
	opts["ipopt.acceptable_obj_change_tol"] = 1e-6;
	// Solver object
	Function solver = nlpsol("solver", "ipopt", nlp, opts);

	DM initialControlValues = DM::zeros(N*2, 1);
	DM lbx = DM::zeros(N*2, 1);
	DM ubx = DM::zeros(N*2, 1);
	for(int i=0; i<N*n_controls; i++){
		if(i % 2 == 0){
			lbx(i) = -Fmax;
			ubx(i) = Fmax;
		}else {
			lbx(i) = -Tmax;
			ubx(i) = Tmax;
		}
	}
	DM pValues = DM::zeros(n_states + 3);

	pValues(0) = 0.1;
	pValues(1) = 0.1;
	pValues(2) = 0.1;
	pValues(3) = 0.5;
	pValues(4) = 0.5;
	pValues(5) = 0.5;
	pValues(6) = 0.5;
	pValues(7) = 0.5;
	
	DMDict arg;
	arg["x0"] = initialControlValues;
	arg["lbx"] = lbx;
	arg["ubx"] = ubx;
  	arg["p"] = pValues;
	//std::cout << "stuck here #3" << std::endl;
	DMDict res = solver(arg);

	std::cout << "stuck here #4" << std::endl;
	std::cout << "ubx:                     " << ubx << std::endl;
	std::cout << "lbx:                     " << lbx << std::endl;
	std::cout << "Optimal cost:                     " << double(res.at("f")) << std::endl;
  	std::cout << "Primal solution:                  " << std::vector<double>(res.at("x")) << std::endl;
    std::cout << "res" << res <<std::endl;            
 	
	
	/**  required:  forces and torques to pwm to the thrusters*/
	
	
	///////////////////////////////////////////////////////////
	                    /*Depth code*/
	if (isItFirstTime) {
        offset = depth;
        isItFirstTime = FALSE;
    }

    e = setpoint - (depth - offset);
    double fit_poly_pos[] = { 19.302480858079193, 39.3358541272889, -8.263830702456188, 
        1.282961416635573, -0.112518356922797, 0.005724304685154, -1.678348654788274e-04, 
        2.629608127196461e-06, -1.703456825754830e-08 };

    double fit_poly_neg[] = { -19.25710704717055, 48.34221805064397, 12.322060404477401,
        2.354548958757111, 0.255975400767933, 0.016226987595385,
        5.950524882854001e-04, 1.169477093694967e-05, 9.526268034372109e-08 };

    F = (Kp + (2 * Kd) / dT) * e + (Kp - (2 * Kd) / dT) * eOld - fOld;
    fOld = F;
    eOld = e;

    forcePerThruster = F / 4;

    if (forcePerThruster >= Flim) {
        forcePerThruster = Flim;
    }
    else if (forcePerThruster <= -Flim) {
        forcePerThruster = -Flim;
    }

    if(forcePerThruster > 0){

        for (int i = 0; i < 9; i++) {
            PWM = PWM + fit_poly_pos[i] * pow(forcePerThruster, i);
        }

    }else if (forcePerThruster < 0) {

        for (int i = 0; i < 9; i++) {
            PWM = PWM + fit_poly_neg[i] * pow(forcePerThruster, i);
        }

    }else {

        PWM = 0;

    }

    if (PWM > 400) {

        PWM = 400;

    }else if (PWM < -400) {

        PWM = -400;

    }

    PWM = PWM + 1500;
    return PWM;
	///////////////////////////////////////////////////////////
	
	return 0;
}
