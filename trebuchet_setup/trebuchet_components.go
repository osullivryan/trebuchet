package trebuchet_setup

import (
	"math"

	"github.com/osullivryan/trebuchet/types"
)

/////////////////////
// Main Simulation //
/////////////////////

//Input variables for Y:
// Y = [
// Aq
// Aq'
// Wq
// Wq'
// Sq
// Sq'
// ]

//Arm_Mass, CW_Mass, Proj_Mass, Arm_I, CW_I, L_Arm_Lo, L_Arm_CG, L_Arm_Sh, L_Arm_We, L_Sling

//First Simulation
func Phase_1(time float64, Y []float64, simulation_environment types.SimulationEnvironment) []float64 {

	ret := make([]float64, len(Y))

	M11 := (-1.0)*simulation_environment.Projectile.Proj_mass*math.Pow(simulation_environment.Trebuchet.L_arm_lo, 2)*(-1+2*math.Sin(Y[0])*math.Cos(Y[0])/math.Sin(Y[0]+Y[4])) + simulation_environment.Trebuchet.Arm_i + simulation_environment.Trebuchet.Cw_i + simulation_environment.Trebuchet.Arm_mass*math.Pow(simulation_environment.Trebuchet.L_arm_cg, 2) + simulation_environment.Projectile.Proj_mass*math.Pow(simulation_environment.Trebuchet.L_arm_lo, 2)*math.Pow(math.Sin(Y[0]), 2)/math.Pow(math.Sin(Y[0]+Y[4]), 2) + simulation_environment.Trebuchet.Cw_mass*(math.Pow(simulation_environment.Trebuchet.L_arm_sh, 2)+math.Pow(simulation_environment.Trebuchet.L_arm_we, 2)+2*simulation_environment.Trebuchet.L_arm_sh*simulation_environment.Trebuchet.L_arm_we*math.Cos(Y[2]))

	M12 := simulation_environment.Trebuchet.Cw_i + simulation_environment.Trebuchet.L_arm_we*simulation_environment.Trebuchet.Cw_mass*(simulation_environment.Trebuchet.L_arm_we+simulation_environment.Trebuchet.L_arm_sh*math.Cos(Y[2]))

	M21 := M12

	M22 := simulation_environment.Trebuchet.Cw_i + simulation_environment.Trebuchet.Cw_mass*math.Pow(simulation_environment.Trebuchet.L_arm_we, 2)

	r1 := simulation_environment.Global.G*simulation_environment.Trebuchet.L_arm_cg*simulation_environment.Trebuchet.Arm_mass*math.Sin(Y[0]) +
		simulation_environment.Trebuchet.L_arm_lo*simulation_environment.Trebuchet.L_sling*simulation_environment.Projectile.Proj_mass*(math.Sin(Y[4])*math.Pow((Y[1]+Y[5]), 2)+math.Cos(Y[4])*
			(math.Cos(Y[0]+Y[4])*Y[5]*(Y[5]+2*Y[1])/math.Sin(Y[0]+Y[4])+
				(math.Cos(Y[0]+Y[4])/math.Sin(Y[0]+Y[4])+simulation_environment.Trebuchet.L_arm_lo*math.Cos(Y[0])/(simulation_environment.Trebuchet.L_sling*math.Sin(Y[0]+Y[4])))*math.Pow(Y[1], 2))) +
		simulation_environment.Trebuchet.L_arm_lo*simulation_environment.Projectile.Proj_mass*math.Sin(Y[0])*(simulation_environment.Trebuchet.L_arm_lo*math.Sin(Y[4])*math.Pow(Y[1], 2)-simulation_environment.Trebuchet.L_sling*(math.Cos(Y[0]+Y[4])*Y[5]*(Y[5]+2*Y[1])/math.Sin(Y[0]+Y[4])+
			(math.Cos(Y[0]+Y[4])/math.Sin(Y[0]+Y[4])+simulation_environment.Trebuchet.L_arm_lo*math.Cos(Y[0])/(simulation_environment.Trebuchet.L_sling*math.Sin(Y[0]+Y[4])))*math.Pow(Y[1], 2)))/math.Sin(Y[0]+Y[4]) -
		simulation_environment.Global.G*simulation_environment.Trebuchet.Cw_mass*(simulation_environment.Trebuchet.L_arm_sh*math.Sin(Y[0])+simulation_environment.Trebuchet.L_arm_we*math.Sin(Y[0]+Y[2])) - simulation_environment.Trebuchet.L_arm_sh*simulation_environment.Trebuchet.L_arm_we*simulation_environment.Trebuchet.Cw_mass*math.Sin(Y[2])*(math.Pow(Y[1], 2)-math.Pow((Y[1]+Y[3]), 2))

	r2 := (-1.0) * simulation_environment.Trebuchet.L_arm_we * simulation_environment.Trebuchet.Cw_mass * (simulation_environment.Global.G*math.Sin(Y[0]+Y[2]) + simulation_environment.Trebuchet.L_arm_sh*math.Sin(Y[2])*math.Pow(Y[1], 2))

	//Return fuction returns Aq', Aq'',Wq', Wq'', Sq',Sq''

	ret[0] = Y[1]
	ret[1] = (r1*M22 - r2*M12) / (M11*M22 - M12*M21)
	ret[2] = Y[3]
	ret[3] = (-1.0) * (r1*M21 - r2*M11) / (M11*M22 - M12*M21)
	ret[4] = Y[5]
	ret[5] = (-1.0)*math.Cos(Y[0]+Y[4])*Y[5]*(Y[5]+2*Y[1])/math.Sin(Y[0]+Y[4]) - (math.Cos(Y[0]+Y[4])/math.Sin(Y[0]+Y[4])+simulation_environment.Trebuchet.L_arm_lo*math.Cos(Y[0])/(simulation_environment.Trebuchet.L_sling*math.Sin(Y[0]+Y[4])))*math.Pow(Y[1], 2) - (simulation_environment.Trebuchet.L_arm_lo*math.Sin(Y[0])+simulation_environment.Trebuchet.L_sling*math.Sin(Y[0]+Y[4]))*ret[1]/(simulation_environment.Trebuchet.L_sling*math.Sin(Y[0]+Y[4]))

	return ret

}

// Simulation 1 Stop function
// Force in +Y dir at projectile point
func Fy(Y []float64, simulation_environment types.SimulationEnvironment) bool {
	// Y = [
	// aq
	// aq'
	// wq
	// wq'
	// sq
	// sq'
	// ]
	deriv := Phase_1(1.0, Y, simulation_environment) //Return fuction returns aq', aq'',wq', wq'', sq',sq''
	Fy := simulation_environment.Projectile.Proj_mass * (simulation_environment.Global.G + (simulation_environment.Trebuchet.L_sling*(math.Cos(Y[0]+Y[4])*deriv[4]*(deriv[4]+2*deriv[0])/math.Sin(Y[0]+Y[4])+(math.Cos(Y[0]+Y[4])/math.Sin(Y[0]+Y[4])+simulation_environment.Trebuchet.L_arm_lo*math.Cos(Y[0])/(simulation_environment.Trebuchet.L_sling*math.Sin(Y[0]+Y[4])))*math.Pow(deriv[0], 2))-simulation_environment.Trebuchet.L_arm_lo*math.Sin(Y[4])*math.Pow(deriv[0], 2)-simulation_environment.Trebuchet.L_arm_lo*(math.Cos(Y[4])-math.Sin(Y[0])/math.Sin(Y[0]+Y[4]))*deriv[1])/math.Sin(Y[0]+Y[4]))

	if Fy < 0 {
		return true
	} else {
		return false
	}
}

// Second Simulation
func Phase_2(time float64, Y []float64, simulation_environment types.SimulationEnvironment) []float64 {
	ret := make([]float64, len(Y))

	M11 := simulation_environment.Trebuchet.Arm_i + simulation_environment.Trebuchet.Cw_i + simulation_environment.Trebuchet.Arm_mass*math.Pow(simulation_environment.Trebuchet.L_arm_cg, 2) + simulation_environment.Projectile.Proj_mass*(math.Pow(simulation_environment.Trebuchet.L_arm_lo, 2)+math.Pow(simulation_environment.Trebuchet.L_sling, 2)+2*simulation_environment.Trebuchet.L_arm_lo*simulation_environment.Trebuchet.L_sling*math.Cos(Y[5])) + simulation_environment.Trebuchet.Cw_mass*(math.Pow(simulation_environment.Trebuchet.L_arm_sh, 2)+math.Pow(simulation_environment.Trebuchet.L_arm_we, 2)+2*simulation_environment.Trebuchet.L_arm_sh*simulation_environment.Trebuchet.L_arm_we*math.Cos(Y[2]))
	M12 := simulation_environment.Trebuchet.Cw_i + simulation_environment.Trebuchet.L_arm_we*simulation_environment.Trebuchet.Cw_mass*(simulation_environment.Trebuchet.L_arm_we+simulation_environment.Trebuchet.L_arm_sh*math.Cos(Y[2]))
	M13 := simulation_environment.Trebuchet.L_sling * simulation_environment.Projectile.Proj_mass * (simulation_environment.Trebuchet.L_sling + simulation_environment.Trebuchet.L_arm_lo*math.Cos(Y[4]))
	M21 := simulation_environment.Trebuchet.Cw_i * simulation_environment.Trebuchet.L_arm_we * simulation_environment.Trebuchet.Cw_mass * (simulation_environment.Trebuchet.L_arm_we + simulation_environment.Trebuchet.L_arm_sh*math.Cos(Y[2]))
	M22 := simulation_environment.Trebuchet.Cw_i + simulation_environment.Trebuchet.Cw_mass*math.Pow(simulation_environment.Trebuchet.L_arm_we, 2)
	M31 := simulation_environment.Trebuchet.L_sling * simulation_environment.Projectile.Proj_mass * (simulation_environment.Trebuchet.L_sling + simulation_environment.Trebuchet.L_arm_lo*math.Cos(Y[4]))
	M33 := simulation_environment.Projectile.Proj_mass * math.Pow(simulation_environment.Trebuchet.L_sling, 2)

	r1 := simulation_environment.Global.G*simulation_environment.Trebuchet.L_arm_cg*simulation_environment.Trebuchet.Arm_mass*math.Sin(Y[0]) + simulation_environment.Global.G*simulation_environment.Projectile.Proj_mass*(simulation_environment.Trebuchet.L_arm_lo*math.Sin(Y[0])+simulation_environment.Trebuchet.L_sling*math.Sin(Y[0]+Y[4])) - simulation_environment.Global.G*simulation_environment.Trebuchet.Cw_mass*(simulation_environment.Trebuchet.L_arm_sh*math.Sin(Y[0])+simulation_environment.Trebuchet.L_arm_we*math.Sin(Y[0]+Y[2])) - simulation_environment.Trebuchet.L_arm_lo*simulation_environment.Trebuchet.L_sling*simulation_environment.Projectile.Proj_mass*math.Sin(Y[4])*(math.Pow(Y[1], 2)-math.Pow((Y[1]+Y[5]), 2)) - simulation_environment.Trebuchet.L_arm_sh*simulation_environment.Trebuchet.L_arm_we*simulation_environment.Trebuchet.Cw_mass*math.Sin(Y[2])*(math.Pow(Y[1], 2)-math.Pow((Y[1]+Y[3]), 2))

	r2 := -simulation_environment.Trebuchet.L_arm_we * simulation_environment.Trebuchet.Cw_mass * (simulation_environment.Global.G*math.Sin(Y[0]+Y[2]) + simulation_environment.Trebuchet.L_arm_sh*math.Sin(Y[2])*math.Pow(Y[1], 2))

	r3 := simulation_environment.Trebuchet.L_sling * simulation_environment.Projectile.Proj_mass * (simulation_environment.Global.G*math.Sin(Y[0]+Y[4]) - simulation_environment.Trebuchet.L_arm_lo*math.Sin(Y[4])*math.Pow(Y[1], 2))

	//Return fuction returns Aq', Aq'',Wq', Wq'', Sq',Sq''

	ret[0] = Y[1]
	ret[1] = -(r1*M22*M33 - r2*M12*M33 - r3*M13*M22) / (M13*M22*M31 - M33*(M11*M22-M12*M21))
	ret[2] = Y[3]
	ret[3] = (r1*M21*M33 - r2*(M11*M33-M13*M31) - r3*M13*M21) / (M13*M22*M31 - M33*(M11*M22-M12*M21))
	ret[4] = Y[5]
	ret[5] = (r1*M22*M31 - r2*M12*M31 - r3*(M11*M22-M12*M21)) / (M13*M22*M31 - M33*(M11*M22-M12*M21))

	return ret

}

// Second simulation stop function
var cnt int = 0

// Velocity vector > launch angle
func Launch(Y []float64, simulation_environment types.SimulationEnvironment) bool {
	xV := -simulation_environment.Trebuchet.L_sling*math.Cos(Y[0]+Y[4])*(Y[1]+Y[5]) - simulation_environment.Trebuchet.L_arm_lo*math.Cos(Y[0])*Y[1]
	yV := -simulation_environment.Trebuchet.L_sling*math.Sin(Y[0]+Y[4])*(Y[1]+Y[5]) - simulation_environment.Trebuchet.L_arm_lo*math.Sin(Y[0])*Y[1]

	cur_angle := math.Atan(yV / xV)
	if simulation_environment.Trebuchet.Rel_angle-0.1 <= cur_angle && cur_angle <= simulation_environment.Trebuchet.Rel_angle+0.1 && yV > 0.0 && xV > 0.0 {
		return true
	} else {
		return false
	}
}

// Third Simulation
func Phase_3(time float64, Y []float64, simulation_environment types.SimulationEnvironment) []float64 {

	ret := make([]float64, len(Y))

	ret[0] = Y[1]
	ret[1] = -(simulation_environment.Global.Ro * simulation_environment.Projectile.Cd * simulation_environment.Projectile.Area * (Y[1] - simulation_environment.Global.Wind_v) * math.Sqrt(math.Pow(Y[3], 2)+math.Pow((simulation_environment.Global.Wind_v-Y[1]), 2))) / (2 * simulation_environment.Projectile.Proj_mass)
	ret[2] = Y[3]
	ret[3] = -simulation_environment.Global.G - (simulation_environment.Global.Ro*simulation_environment.Projectile.Cd*simulation_environment.Projectile.Area*Y[3]*math.Sqrt(math.Pow(Y[3], 2)+math.Pow((simulation_environment.Global.Wind_v-Y[1]), 2)))/(2*simulation_environment.Projectile.Proj_mass)

	return ret
}

// Third simulation stop function
func Proj_Y(Y []float64, simulation_environment types.SimulationEnvironment) bool {
	if Y[2] <= 0 {
		return true
	} else {
		return false
	}
}
