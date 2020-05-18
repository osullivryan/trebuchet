package main

import (
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"strconv"
	"time"
	"trebuchet/ode_solver"
	"trebuchet/trebuchet"
	"trebuchet/types"
	"trebuchet/utils"
)

var trebuchet_configuration = types.TrebuchetParameters{
	L_arm_lo:  1.0,
	L_arm_sh:  1.0,
	L_arm_we:  1.0,
	L_sling:   1.0,
	H_pivot:   1.0,
	Cw_mass:   1.0,
	Cw_i:      1.0,
	Arm_mass:  1.0,
	Arm_i:     1.0,
	L_arm_cg:  1.0,
	Rel_angle: 1.0,
}

var projectile_configuration = types.ProjectileParameters{
	Proj_mass: 1.0,
	Proj_dia:  1.0,
	Cd:        0.4,
	Area:      math.Pi * math.Pow(0.5, 2.0),
}

var global_configuration = types.GlobalParameters{
	G:      9.81,
	Ro:     1.0,
	Wind_v: 1.0,
}

func main() {
	start := time.Now()

	simulation_environment := types.SimulationEnvironment{
		Trebuchet:  trebuchet_configuration,
		Global:     global_configuration,
		Projectile: projectile_configuration,
	}

	first_phase := ode_solver.Simulation{
		Functions: types.SimulationFunction{
			Evaluation_function:       trebuchet.Phase_1,
			Integration_stop_function: trebuchet.Fy,
		},
		Environments: simulation_environment,
	}

	second_phase := ode_solver.Simulation{
		Functions: types.SimulationFunction{
			Evaluation_function:       trebuchet.Phase_2,
			Integration_stop_function: trebuchet.Fy,
		},
		Environments: simulation_environment,
	}

	third_phase := ode_solver.Simulation{
		Functions: types.SimulationFunction{
			Evaluation_function:       trebuchet.Phase_3,
			Integration_stop_function: trebuchet.Fy,
		},
		Environments: simulation_environment,
	}

	// The initial values
	// Solve the first phase
	IV1 := []float64{2.3562, 0, -2.3562, 0, 2.3562, 0}
	y1 := first_phase.Solve(0, 0.0001, 5.0, 0.5, IV1)

	// Solve the second phase
	IV2 := y1[len(y1)-1][1:]
	y2 := second_phase.Solve(0, 0.0001, 1, 0.5, IV2)

	IV3 := trebuchet.ProjV(y2[len(y2)-1][1:], simulation_environment)
	y3 := third_phase.Solve(0, 0.0001, 1, 0.5, IV3)

	fmt.Println("Range: ", y3[len(y3)-1][1])

	file, err := os.Create("result.csv")
	utils.CheckError("Cannot create file", err)
	defer file.Close()

	writer := csv.NewWriter(file)
	xy_proj := trebuchet.ProjXY(y2, simulation_environment)
	xy_cw := trebuchet.CWXY(y2, simulation_environment)
	xy_arm_sling := trebuchet.Arm_Sl_XY(y2, simulation_environment)
	xy_weight_arm := trebuchet.Weight_Arm_XY(y2, simulation_environment)
	pos := [][]string{}
	for i := 0; i < len(y2); i++ {
		time := strconv.FormatFloat(xy_proj[i][0], 'f', 8, 64)
		xp := strconv.FormatFloat(xy_proj[i][1], 'f', 8, 64)
		yp := strconv.FormatFloat(xy_proj[i][2], 'f', 8, 64)
		xcw := strconv.FormatFloat(xy_cw[i][1], 'f', 8, 64)
		ycw := strconv.FormatFloat(xy_cw[i][2], 'f', 8, 64)
		xasl := strconv.FormatFloat(xy_arm_sling[i][1], 'f', 8, 64)
		yasl := strconv.FormatFloat(xy_arm_sling[i][2], 'f', 8, 64)
		xwa := strconv.FormatFloat(xy_weight_arm[i][1], 'f', 8, 64)
		ywa := strconv.FormatFloat(xy_weight_arm[i][2], 'f', 8, 64)
		temp := []string{time, xp, yp, xcw, ycw, xasl, yasl, xwa, ywa}
		pos = append(pos, temp)
	}

	for _, value := range pos {
		err := writer.Write(value)
		utils.CheckError("Cannot write to file", err)
	}

	defer writer.Flush()

	elapsed := time.Since(start)
	fmt.Println(elapsed)

}
