package trebuchet

import (
	"math"

	"github.com/osullivryan/trebuchet/types"
)

/////////////////////////////
// Projectile angles to xy //
/////////////////////////////

func ProjXY(Y [][]float64, simulation_environment types.SimulationEnvironment) [][]float64 {
	XY := [][]float64{}

	for i := 0; i < len(Y); i++ {
		temp := make([]float64, 3)
		//Put in the Time
		temp[0] = Y[i][0]
		//X value
		temp[1] = -simulation_environment.Trebuchet.L_arm_lo*math.Sin(Y[i][1]) - simulation_environment.Trebuchet.L_sling*math.Sin(Y[i][1]+Y[i][5])
		//Y value
		temp[2] = simulation_environment.Trebuchet.L_arm_lo*math.Cos(Y[i][1]) + simulation_environment.Trebuchet.L_sling*math.Cos(Y[i][1]+Y[i][5])
		// Append
		XY = append(XY, temp)
	}

	return XY
}

// Y = [
// Aq
// Aq'
// Wq
// Wq'
// Sq
// Sq'
// ]

//////////////////////////////
// Projectile Velocity in XY//
//////////////////////////////

func ProjV(Y []float64, simulation_environment types.SimulationEnvironment) []float64 {
	xP := -simulation_environment.Trebuchet.L_arm_lo*math.Sin(Y[0]) - simulation_environment.Trebuchet.L_sling*math.Sin(Y[0]+Y[4])
	xV := -simulation_environment.Trebuchet.L_sling*math.Cos(Y[0]+Y[4])*(Y[1]+Y[5]) - simulation_environment.Trebuchet.L_arm_lo*math.Cos(Y[0])*Y[1]
	yP := simulation_environment.Trebuchet.L_arm_lo*math.Cos(Y[0]) + simulation_environment.Trebuchet.L_sling*math.Cos(Y[0]+Y[4])
	yV := -simulation_environment.Trebuchet.L_sling*math.Sin(Y[0]+Y[4])*(Y[1]+Y[5]) - simulation_environment.Trebuchet.L_arm_lo*math.Sin(Y[0])*Y[1]
	return []float64{xP, xV, yP, yV}
}

/////////////////////
// CW angles to xy //
/////////////////////

func CWXY(Y [][]float64, simulation_environment types.SimulationEnvironment) [][]float64 {
	XY := [][]float64{}

	for i := 0; i < len(Y); i++ {
		temp := make([]float64, 3)
		//Put in the Time
		temp[0] = Y[i][0]
		//X value
		temp[1] = simulation_environment.Trebuchet.L_arm_sh*math.Sin(Y[i][1]) + simulation_environment.Trebuchet.L_arm_we*math.Sin(Y[i][1]+Y[i][3])
		//Y value
		temp[2] = -simulation_environment.Trebuchet.L_arm_sh*math.Cos(Y[i][1]) - simulation_environment.Trebuchet.L_arm_we*math.Cos(Y[i][1]+Y[i][3])
		// Append
		XY = append(XY, temp)
	}

	return XY
}

////////////////////////////
// Arm Sling angles to XY //
////////////////////////////

func Arm_Sl_XY(Y [][]float64, simulation_environment types.SimulationEnvironment) [][]float64 {
	XY := [][]float64{}

	for i := 0; i < len(Y); i++ {
		temp := make([]float64, 3)
		//Put in the Time
		temp[0] = Y[i][0]
		//X value
		temp[1] = -simulation_environment.Trebuchet.L_arm_lo * math.Sin(Y[i][1])
		//Y value
		temp[2] = simulation_environment.Trebuchet.L_arm_lo * math.Cos(Y[i][1])
		// Append
		XY = append(XY, temp)
	}

	return XY
}

//////////////////////
// Weight Arm Point //
//////////////////////

func Weight_Arm_XY(Y [][]float64, simulation_environment types.SimulationEnvironment) [][]float64 {
	XY := [][]float64{}

	for i := 0; i < len(Y); i++ {
		temp := make([]float64, 3)
		//Put in the Time
		temp[0] = Y[i][0]
		//X value
		temp[1] = simulation_environment.Trebuchet.L_arm_sh * math.Sin(Y[i][1])
		//Y value
		temp[2] = -simulation_environment.Trebuchet.L_arm_sh * math.Cos(Y[i][1])
		// Append
		XY = append(XY, temp)
	}

	return XY

}
